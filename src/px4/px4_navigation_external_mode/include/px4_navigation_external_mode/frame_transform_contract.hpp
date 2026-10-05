#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

enum class FrameTransformDecision : std::uint8_t {
  kUnavailable = 0U,
  kApplied = 1U,
  kRecertifyRequired = 2U,
  kDegradeToL1 = 3U,
};

struct FrameTransformConfig final {
  // A certificate is expected to live for roughly two seconds.  The slow
  // filter is deliberately much slower than that certificate, as required by
  // INV-4.  The value is a contract input, not a claim about PX4 capability.
  double tau_s{10.0};
  double horizon_s{2.0};
  double frame_budget_m{0.227};

  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(tau_s) && std::isfinite(horizon_s) &&
        std::isfinite(frame_budget_m) && tau_s >= 10.0 && horizon_s > 0.0 &&
        frame_budget_m > 0.0;
  }
};

struct FrameTransformObservation final {
  std::int64_t source_stamp_ns{0};
  Eigen::Vector3d bias_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_var_m2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d innov_test_ratio{Eigen::Vector3d::Zero()};
  bool yaw_align{false};
  std::uint8_t heading_reset_counter{0U};
};

struct FrameTransformResult final {
  FrameTransformDecision decision{FrameTransformDecision::kUnavailable};
  std::optional<Eigen::Vector3d> applied_bias_m;
  double raw_applied_pressure_m{0.0};
  double applied_rate_mps{0.0};
  std::uint64_t version{0U};
};

// Pure, side-effect-free-of-ROS contract for the diagnostic T3 layer.  It is
// intentionally not a setpoint publisher or an execution authority.  A
// caller must route kRecertifyRequired/kDegradeToL1 through the existing
// planner/runtime ownership boundary before any product command changes.
class FrameTransformContract final {
 public:
  explicit FrameTransformContract(FrameTransformConfig config = {}) noexcept
  : config_(config) {}

  [[nodiscard]] const FrameTransformConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] const std::optional<Eigen::Vector3d>& appliedBias() const noexcept {
    return applied_bias_m_;
  }

  [[nodiscard]] std::uint64_t version() const noexcept {
    return version_;
  }

  void reset() noexcept {
    applied_bias_m_.reset();
    previous_bias_var_m2_.reset();
    last_source_stamp_ns_ = 0;
    heading_reset_counter_.reset();
    degraded_ = false;
    version_ = 0U;
  }

  [[nodiscard]] FrameTransformResult observe(
      const FrameTransformObservation& observation) noexcept {
    FrameTransformResult result;
    result.applied_bias_m = applied_bias_m_;
    result.version = version_;
    if (!config_.valid() || observation.source_stamp_ns <= 0 ||
        (last_source_stamp_ns_ > 0 &&
         observation.source_stamp_ns <= last_source_stamp_ns_)) {
      return result;
    }

    // Consume source ordering even for a malformed payload.  An older valid
    // payload must not be allowed to resurrect a newer invalid observation.
    last_source_stamp_ns_ = observation.source_stamp_ns;
    if (!validPayload(observation)) return result;

    if (degraded_ || !observation.yaw_align) {
      degraded_ = true;
      result.decision = FrameTransformDecision::kDegradeToL1;
      return result;
    }
    if (heading_reset_counter_.has_value() &&
        *heading_reset_counter_ != observation.heading_reset_counter) {
      degraded_ = true;
      result.decision = FrameTransformDecision::kDegradeToL1;
      return result;
    }
    if (anyInnovationRejected(observation.innov_test_ratio)) {
      result.decision = FrameTransformDecision::kRecertifyRequired;
      return result;
    }
    if (previous_bias_var_m2_.has_value() &&
        previous_bias_var_m2_->maxCoeff() > kVarianceEpsilon &&
        observation.bias_var_m2.maxCoeff() <= kVarianceEpsilon) {
      result.decision = FrameTransformDecision::kRecertifyRequired;
      return result;
    }

    if (!applied_bias_m_.has_value()) {
      applied_bias_m_ = observation.bias_m;
      previous_bias_var_m2_ = observation.bias_var_m2;
      heading_reset_counter_ = observation.heading_reset_counter;
      last_applied_source_stamp_ns_ = observation.source_stamp_ns;
      ++version_;
      result.decision = FrameTransformDecision::kApplied;
      result.applied_bias_m = applied_bias_m_;
      result.version = version_;
      return result;
    }

    const double pressure = (observation.bias_m - *applied_bias_m_).norm();
    result.raw_applied_pressure_m = pressure;
    if (!std::isfinite(pressure) || pressure > config_.frame_budget_m) {
      result.decision = FrameTransformDecision::kRecertifyRequired;
      return result;
    }

    const double dt_s = static_cast<double>(
        observation.source_stamp_ns - last_applied_source_stamp_ns_) * 1.0e-9;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      return result;
    }
    const double alpha = dt_s / (config_.tau_s + dt_s);
    const Eigen::Vector3d delta = alpha * (observation.bias_m - *applied_bias_m_);
    const double rate_mps = delta.norm() / dt_s;
    result.applied_rate_mps = rate_mps;
    if (!std::isfinite(alpha) || !std::isfinite(rate_mps) ||
        rate_mps * config_.horizon_s > config_.frame_budget_m) {
      result.decision = FrameTransformDecision::kRecertifyRequired;
      return result;
    }

    *applied_bias_m_ += delta;
    previous_bias_var_m2_ = observation.bias_var_m2;
    heading_reset_counter_ = observation.heading_reset_counter;
    last_applied_source_stamp_ns_ = observation.source_stamp_ns;
    ++version_;
    result.decision = FrameTransformDecision::kApplied;
    result.applied_bias_m = applied_bias_m_;
    result.version = version_;
    return result;
  }

 private:
  static constexpr double kVarianceEpsilon = 1.0e-12;

  [[nodiscard]] static bool validPayload(
      const FrameTransformObservation& observation) noexcept {
    return observation.bias_m.allFinite() && observation.bias_var_m2.allFinite() &&
        observation.innov_test_ratio.allFinite() &&
        (observation.bias_var_m2.array() >= 0.0).all() &&
        (observation.innov_test_ratio.array() >= 0.0).all();
  }

  [[nodiscard]] static bool anyInnovationRejected(
      const Eigen::Vector3d& test_ratio) noexcept {
    return (test_ratio.array() >= 1.0).any();
  }

  FrameTransformConfig config_{};
  std::optional<Eigen::Vector3d> applied_bias_m_;
  std::optional<Eigen::Vector3d> previous_bias_var_m2_;
  std::int64_t last_source_stamp_ns_{0};
  std::int64_t last_applied_source_stamp_ns_{0};
  std::optional<std::uint8_t> heading_reset_counter_;
  bool degraded_{false};
  std::uint64_t version_{0U};
};

}  // namespace px4_navigation_external_mode

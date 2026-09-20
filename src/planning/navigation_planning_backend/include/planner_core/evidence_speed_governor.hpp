#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <planner_core/backup_braking.hpp>

namespace navigation_planning_backend {

enum class EvidenceSpeedFailure : std::uint8_t {
  kNone,
  kInvalidInput,
  kOutsideRecoveryEnvelope,
  kInsufficientSupport,
  kStopSynthesisFailed,
  kBudgetExhausted,
};

struct EvidenceSpeedLimit final {
  double speed_mps{0.0};
  double support_m{0.0};
  bool sufficient{false};
  EvidenceSpeedFailure failure{EvidenceSpeedFailure::kInvalidInput};
};

namespace detail {

inline EvidenceSpeedFailure stopFailureToSpeedFailure(
    const StopFailureReason failure) noexcept {
  switch (failure) {
    case StopFailureReason::kNone:
      return EvidenceSpeedFailure::kNone;
    case StopFailureReason::kInvalidState:
    case StopFailureReason::kInvalidDynamics:
      return EvidenceSpeedFailure::kInvalidInput;
    case StopFailureReason::kOutsideRecoveryEnvelope:
      return EvidenceSpeedFailure::kOutsideRecoveryEnvelope;
    case StopFailureReason::kInsufficientSupport:
      return EvidenceSpeedFailure::kInsufficientSupport;
    case StopFailureReason::kSynthesisFailed:
      return EvidenceSpeedFailure::kStopSynthesisFailed;
  }
  return EvidenceSpeedFailure::kInvalidInput;
}

template <typename ShouldAbort>
[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimitImpl(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m,
    const double sample_traj_dt_s,
    ShouldAbort&& should_abort) noexcept {
  EvidenceSpeedLimit result;
  if (!state.allFinite() || !dynamics.valid() ||
      !std::isfinite(sample_traj_dt_s) || sample_traj_dt_s <= 0.0) {
    return result;
  }
  result.support_m = std::numeric_limits<double>::infinity();
  for (const double support : support_m) {
    if (!std::isfinite(support) || support <= 0.0) {
      result.support_m = 0.0;
      result.failure = EvidenceSpeedFailure::kInsufficientSupport;
      return result;
    }
    result.support_m = std::min(result.support_m, support);
  }

  // First certify the actual PVAJ boundary under the caller's physical
  // recovery model. Reducing desired cruise speed does not reduce current
  // measured velocity, acceleration, or jerk.
  const auto current_stop = evaluateStopReachability(
      state, dynamics, result.support_m, sample_traj_dt_s);
  if (!current_stop.feasible) {
    result.failure = stopFailureToSpeedFailure(current_stop.failure);
    return result;
  }

  const Eigen::Vector3d velocity = state.col(1);
  const Eigen::Vector3d direction = velocity.norm() > 1.0e-9
      ? velocity.normalized() : Eigen::Vector3d::UnitX();
  const double upper = dynamics.intent.requested_cruise_speed_mps;
  // Feasibility of this polynomial family is not assumed monotone in duration
  // or entry speed. Inspect a fixed descending grid instead of bisection; each
  // accepted value has its own concrete stop-polynomial support certificate.
  constexpr std::size_t kBoundedSpeedSamples = 16U;
  for (std::size_t sample = kBoundedSpeedSamples; sample > 0U; --sample) {
    bool abort = false;
    try {
      abort = static_cast<bool>(should_abort());
    } catch (...) {
      abort = true;
    }
    if (abort) {
      result.failure = EvidenceSpeedFailure::kBudgetExhausted;
      return result;
    }
    const double candidate_speed =
        upper * static_cast<double>(sample) /
        static_cast<double>(kBoundedSpeedSamples);
    navigation_math::StatePVAJ candidate_state;
    candidate_state.setZero();
    candidate_state.col(0) = state.col(0);
    candidate_state.col(1) = direction * candidate_speed;
    // Cruise is a future steady-state proposal. Actual non-zero boundary
    // derivatives were independently included in current_stop above.
    const auto candidate_stop = evaluateStopReachability(
        candidate_state, dynamics, result.support_m, sample_traj_dt_s);
    if (candidate_stop.feasible) {
      result.speed_mps = candidate_speed;
      result.sufficient = true;
      result.failure = EvidenceSpeedFailure::kNone;
      return result;
    }
    if (candidate_stop.failure == StopFailureReason::kOutsideRecoveryEnvelope) {
      result.failure = EvidenceSpeedFailure::kOutsideRecoveryEnvelope;
    } else if (candidate_stop.failure == StopFailureReason::kSynthesisFailed) {
      result.failure = EvidenceSpeedFailure::kStopSynthesisFailed;
    }
  }
  if (result.failure == EvidenceSpeedFailure::kInvalidInput) {
    result.failure = EvidenceSpeedFailure::kInsufficientSupport;
  }
  return result;
}

}  // namespace detail

template <typename ShouldAbort>
[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m,
    const double sample_traj_dt_s,
    ShouldAbort&& should_abort) noexcept {
  return detail::evidenceAwareSpeedLimitImpl(
      state, dynamics, support_m, sample_traj_dt_s,
      std::forward<ShouldAbort>(should_abort));
}

[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m) noexcept {
  return detail::evidenceAwareSpeedLimitImpl(
      state, dynamics, support_m, 0.05, [] { return false; });
}

[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const double configured_speed_mps, const double max_acceleration_mps2,
    const double max_jerk_mps3, const std::array<double, 4>& support_m) noexcept {
  EvidenceSpeedLimit result;
  if (!std::isfinite(configured_speed_mps) || configured_speed_mps <= 0.0 ||
      !std::isfinite(max_acceleration_mps2) || max_acceleration_mps2 <= 0.0 ||
      !std::isfinite(max_jerk_mps3) || max_jerk_mps3 <= 0.0) {
    return result;
  }
  // This convenience form has no measured boundary state; interpret the
  // configured value only as a prospective steady cruise speed.
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  navigation_planning::DynamicLimits dynamics;
  dynamics.vehicle.maximum_velocity_mps = configured_speed_mps;
  dynamics.vehicle.maximum_acceleration_mps2 = max_acceleration_mps2;
  dynamics.vehicle.maximum_jerk_mps3 = max_jerk_mps3;
  dynamics.intent.requested_cruise_speed_mps = configured_speed_mps;
  return evidenceAwareSpeedLimit(state, dynamics, support_m);
}

}  // namespace navigation_planning_backend

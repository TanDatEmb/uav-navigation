#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace navigation_planning {

/** A single polynomial segment in local time.
 *
 * The position polynomial is represented in the power basis:
 *
 *   p(t) = c[0] + c[1] t + c[2] t^2 + ...
 *
 * where each coefficient is a 3D vector and t is measured from the start of
 * this segment. Coefficients are not modified by the evaluator.
 */
struct PolynomialSegment {
  double duration_s{0.0};
  std::vector<Eigen::Vector3d> coefficients{};
};

struct PolynomialTrajectorySample {
  double time_s{0.0};
  double local_time_s{0.0};
  std::size_t segment_index{0U};

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool allFinite() const noexcept {
    return std::isfinite(time_s) && std::isfinite(local_time_s) &&
           position.allFinite() && velocity.allFinite() &&
           acceleration.allFinite() && jerk.allFinite() && snap.allFinite();
  }
};

enum class PolynomialTrajectoryValidationCode {
  Valid,
  EmptyTrajectory,
  NonFiniteDuration,
  NonPositiveDuration,
  NonMonotonicTime,
  EmptyCoefficients,
  NonFiniteCoefficient,
  NonFiniteTotalDuration,
};

struct PolynomialTrajectoryValidation {
  PolynomialTrajectoryValidationCode code{
      PolynomialTrajectoryValidationCode::EmptyTrajectory};
  std::size_t segment_index{std::numeric_limits<std::size_t>::max()};

  [[nodiscard]] bool valid() const noexcept {
    return code == PolynomialTrajectoryValidationCode::Valid;
  }
};

/** Evaluates a finite sequence of local-time polynomial segments.
 *
 * Segment intervals are [start, end), except that the final segment includes
 * the terminal time. An exact internal boundary is therefore evaluated using
 * the segment on its right. Input times outside the trajectory are clamped to
 * [0, duration]. Invalid trajectories and non-finite query times return
 * std::nullopt instead of producing an unsafe numerical result.
 */
class PiecewisePolynomialTrajectory final {
 public:
  PiecewisePolynomialTrajectory() = default;

  explicit PiecewisePolynomialTrajectory(std::vector<PolynomialSegment> segments) {
    (void)setSegments(std::move(segments));
  }

  [[nodiscard]] PolynomialTrajectoryValidation setSegments(
      std::vector<PolynomialSegment> segments) {
    segments_ = std::move(segments);
    rebuildCumulativeEndTimes();
    validation_ = computeValidation();
    return validation_;
  }

  [[nodiscard]] const std::vector<PolynomialSegment>& segments() const noexcept {
    return segments_;
  }

  [[nodiscard]] double duration() const noexcept {
    return validation_.valid() ? total_duration_s_ : 0.0;
  }

  [[nodiscard]] PolynomialTrajectoryValidation validate() const noexcept {
    return validation_;
  }

  [[nodiscard]] std::optional<PolynomialTrajectorySample> evaluate(
      double time_s) const noexcept {
    if (!validation_.valid() || !std::isfinite(time_s)) return std::nullopt;

    const double clamped_time_s = std::clamp(time_s, 0.0, total_duration_s_);
    const auto upper = std::upper_bound(cumulative_end_times_.begin(),
                                        cumulative_end_times_.end(),
                                        clamped_time_s);
    std::size_t segment_index = static_cast<std::size_t>(
        std::distance(cumulative_end_times_.begin(), upper));
    if (segment_index >= segments_.size()) segment_index = segments_.size() - 1U;

    const double segment_start_s =
        segment_index == 0U ? 0.0 : cumulative_end_times_[segment_index - 1U];
    double local_time_s = clamped_time_s - segment_start_s;
    local_time_s = std::clamp(local_time_s, 0.0,
                              segments_[segment_index].duration_s);

    PolynomialTrajectorySample sample;
    sample.time_s = clamped_time_s;
    sample.local_time_s = local_time_s;
    sample.segment_index = segment_index;
    const auto& coefficients = segments_[segment_index].coefficients;
    sample.position = evaluateDerivative(coefficients, local_time_s, 0U);
    sample.velocity = evaluateDerivative(coefficients, local_time_s, 1U);
    sample.acceleration = evaluateDerivative(coefficients, local_time_s, 2U);
    sample.jerk = evaluateDerivative(coefficients, local_time_s, 3U);
    sample.snap = evaluateDerivative(coefficients, local_time_s, 4U);
    if (!sample.allFinite()) return std::nullopt;
    return sample;
  }

 private:
  [[nodiscard]] PolynomialTrajectoryValidation computeValidation() const noexcept {
    if (segments_.empty()) {
      return {PolynomialTrajectoryValidationCode::EmptyTrajectory,
              std::numeric_limits<std::size_t>::max()};
    }

    double previous_end_s = 0.0;
    for (std::size_t index = 0U; index < segments_.size(); ++index) {
      const auto& segment = segments_[index];
      if (!std::isfinite(segment.duration_s)) {
        return {PolynomialTrajectoryValidationCode::NonFiniteDuration, index};
      }
      if (segment.duration_s <= 0.0) {
        return {PolynomialTrajectoryValidationCode::NonPositiveDuration, index};
      }
      if (segment.coefficients.empty()) {
        return {PolynomialTrajectoryValidationCode::EmptyCoefficients, index};
      }
      for (const auto& coefficient : segment.coefficients) {
        if (!coefficient.allFinite()) {
          return {PolynomialTrajectoryValidationCode::NonFiniteCoefficient, index};
        }
      }

      const double end_s = previous_end_s + segment.duration_s;
      if (!std::isfinite(end_s)) {
        return {PolynomialTrajectoryValidationCode::NonFiniteTotalDuration, index};
      }
      if (end_s <= previous_end_s) {
        return {PolynomialTrajectoryValidationCode::NonMonotonicTime, index};
      }
      previous_end_s = end_s;
    }

    return {PolynomialTrajectoryValidationCode::Valid,
            std::numeric_limits<std::size_t>::max()};
  }
  static Eigen::Vector3d evaluateDerivative(
      const std::vector<Eigen::Vector3d>& coefficients, double time_s,
      std::size_t derivative_order) noexcept {
    if (coefficients.size() <= derivative_order) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    for (std::size_t index = coefficients.size(); index-- > derivative_order;) {
      double scale = 1.0;
      for (std::size_t factor = 0U; factor < derivative_order; ++factor) {
        scale *= static_cast<double>(index - factor);
      }
      result = result * time_s + coefficients[index] * scale;
    }
    return result;
  }

  void rebuildCumulativeEndTimes() {
    cumulative_end_times_.clear();
    cumulative_end_times_.reserve(segments_.size());
    double end_s = 0.0;
    for (const auto& segment : segments_) {
      end_s += segment.duration_s;
      cumulative_end_times_.push_back(end_s);
    }
    total_duration_s_ = cumulative_end_times_.empty()
                            ? 0.0
                            : cumulative_end_times_.back();
  }

  std::vector<PolynomialSegment> segments_{};
  std::vector<double> cumulative_end_times_{};
  double total_duration_s_{0.0};
  PolynomialTrajectoryValidation validation_{};
};

}  // namespace navigation_planning

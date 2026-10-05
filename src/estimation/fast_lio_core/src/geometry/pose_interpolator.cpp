#include "fast_lio_core/geometry/pose_interpolator.hpp"

#include <cmath>

#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

Result<RigidTransform> PoseInterpolator::interpolate(const PoseStamped& lower,
                                                     const PoseStamped& upper,
                                                     const Timestamp& query_time) {
  if (lower.pose.targetFrame() != upper.pose.targetFrame() ||
      lower.pose.sourceFrame() != upper.pose.sourceFrame()) {
    return Status(StatusCode::kFrameMismatch,
                  "Cannot interpolate poses with different frame directions");
  }
  if (!lower.time.sameClockDomain(upper.time) || !lower.time.sameClockDomain(query_time)) {
    return Status(StatusCode::kClockDomainMismatch,
                  "Pose interpolation timestamps use different clock domains");
  }
  const std::int64_t lower_ns = lower.time.nanoseconds();
  const std::int64_t upper_ns = upper.time.nanoseconds();
  const std::int64_t query_ns = query_time.nanoseconds();
  if (upper_ns < lower_ns) {
    return Status(StatusCode::kTimestampRegression, "Pose interpolation interval is reversed");
  }
  if (query_ns < lower_ns || query_ns > upper_ns) {
    return Status(StatusCode::kOutOfRange, "Pose interpolation query is outside the interval");
  }
  if (upper_ns == lower_ns) {
    if (query_ns != lower_ns) {
      return Status(StatusCode::kOutOfRange, "Query does not match zero-duration pose interval");
    }
    return lower.pose;
  }

  const auto numerator = checkedDifference(query_time, lower.time);
  const auto denominator = checkedDifference(upper.time, lower.time);
  if (!numerator.ok()) {
    return numerator.status();
  }
  if (!denominator.ok()) {
    return denominator.status();
  }
  const double alpha = static_cast<double>(numerator.value().nanoseconds()) /
                       static_cast<double>(denominator.value().nanoseconds());
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    return Status(StatusCode::kNumericalFailure,
                  "Pose interpolation ratio is not finite");
  }
  Eigen::Quaterniond rotation = lower.pose.rotation().slerp(alpha, upper.pose.rotation());
  rotation.normalize();
  const Eigen::Vector3d translation =
      (1.0 - alpha) * lower.pose.translation() + alpha * upper.pose.translation();
  return RigidTransform(lower.pose.targetFrame(), lower.pose.sourceFrame(), rotation, translation);
}

}  // namespace uav::nav::lio

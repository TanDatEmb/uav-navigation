#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

inline double stableVectorNorm(const Eigen::Vector3d& vector) {
  return std::hypot(std::hypot(vector.x(), vector.y()), vector.z());
}

struct TrackingEnvelopeResult {
  bool valid{false};
  double longitudinal_error_m{0.0};
  double reverse_error_m{0.0};
  double lateral_error_m{0.0};
  double longitudinal_limit_m{0.0};
};

// A command is accepted only when its position remains inside the same finite
// geometric envelope in every direction. Tracking delay is an execution
// observation, not a permission to enlarge the command acceptance region.
inline TrackingEnvelopeResult evaluateTrackingEnvelope(
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& command_position,
    const Eigen::Vector3d& command_velocity,
    double geometric_limit_m) {
  TrackingEnvelopeResult result;
  if (!measured_position.allFinite() || !command_position.allFinite() ||
      !command_velocity.allFinite() || !std::isfinite(geometric_limit_m) ||
      geometric_limit_m <= 0.0) {
    return result;
  }

  const Eigen::Vector3d error = command_position - measured_position;
  if (!error.allFinite()) {
    return result;
  }
  const double speed = stableVectorNorm(command_velocity);
  if (!std::isfinite(speed)) {
    return result;
  }
  result.longitudinal_limit_m = geometric_limit_m;
  if (speed <= 1e-3) {
    result.longitudinal_error_m = stableVectorNorm(error);
    if (!std::isfinite(result.longitudinal_error_m)) {
      return result;
    }
    result.lateral_error_m = 0.0;
    result.valid = result.longitudinal_error_m <= geometric_limit_m;
    return result;
  }

  const Eigen::Vector3d tangent = command_velocity / speed;
  const double signed_longitudinal_error = error.dot(tangent);
  if (!std::isfinite(signed_longitudinal_error)) {
    return result;
  }
  result.longitudinal_error_m = std::max(0.0, signed_longitudinal_error);
  result.lateral_error_m =
      stableVectorNorm(error - signed_longitudinal_error * tangent);
  result.reverse_error_m = std::max(0.0, -signed_longitudinal_error);
  if (!std::isfinite(result.lateral_error_m) ||
      !std::isfinite(result.reverse_error_m)) {
    return result;
  }
  result.valid = result.longitudinal_error_m <= result.longitudinal_limit_m &&
                 result.reverse_error_m <= geometric_limit_m &&
                 result.lateral_error_m <= geometric_limit_m;
  return result;
}

}  // namespace px4_navigation_external_mode

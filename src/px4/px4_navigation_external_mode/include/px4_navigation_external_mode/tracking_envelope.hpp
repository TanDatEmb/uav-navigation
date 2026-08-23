#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

struct TrackingEnvelopeResult {
  bool valid{false};
  double longitudinal_error_m{0.0};
  double lateral_error_m{0.0};
  double longitudinal_limit_m{0.0};
};

// A moving position reference is expected to lead the measured vehicle by the
// closed-loop tracking delay.  Keep lateral and reverse/overshoot errors on the
// strict geometric limit, while allowing only forward lag proportional to the
// commanded speed.  A blocked vehicle therefore still trips the finite bound
// instead of letting the trajectory clock run away indefinitely.
inline TrackingEnvelopeResult evaluateTrackingEnvelope(
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& command_position,
    const Eigen::Vector3d& command_velocity,
    double geometric_limit_m,
    double tracking_lag_s) {
  TrackingEnvelopeResult result;
  if (!measured_position.allFinite() || !command_position.allFinite() ||
      !command_velocity.allFinite() || !std::isfinite(geometric_limit_m) ||
      geometric_limit_m <= 0.0 || !std::isfinite(tracking_lag_s) || tracking_lag_s < 0.0) {
    return result;
  }

  const Eigen::Vector3d error = command_position - measured_position;
  const double speed = command_velocity.norm();
  result.longitudinal_limit_m = geometric_limit_m + speed * tracking_lag_s;
  if (speed <= 1e-3) {
    result.longitudinal_error_m = error.norm();
    result.lateral_error_m = 0.0;
    result.valid = result.longitudinal_error_m <= geometric_limit_m;
    return result;
  }

  const Eigen::Vector3d tangent = command_velocity / speed;
  const double signed_longitudinal_error = error.dot(tangent);
  result.longitudinal_error_m = std::max(0.0, signed_longitudinal_error);
  result.lateral_error_m = (error - signed_longitudinal_error * tangent).norm();
  const double reverse_error_m = std::max(0.0, -signed_longitudinal_error);
  result.valid = result.longitudinal_error_m <= result.longitudinal_limit_m &&
                 reverse_error_m <= geometric_limit_m &&
                 result.lateral_error_m <= geometric_limit_m;
  return result;
}

}  // namespace px4_navigation_external_mode

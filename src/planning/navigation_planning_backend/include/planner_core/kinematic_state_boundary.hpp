#pragma once

#include <cmath>

#include <Eigen/Core>

namespace navigation_planning_backend {

// Propagated odometry currently derives acceleration and jerk from finite
// differences. Those values are useful diagnostics, but they are not measured
// command-boundary constraints. Keep measured position and velocity untouched
// while preventing an estimated high-order derivative from making the next
// certified MINCO boundary physically infeasible.
inline Eigen::Vector3d boundEstimatedDerivative(
    const Eigen::Vector3d& derivative, bool estimated, double maximum_norm) noexcept {
  if (!estimated || !derivative.allFinite() || !std::isfinite(maximum_norm) ||
      maximum_norm <= 0.0) {
    return derivative;
  }
  const double norm = derivative.norm();
  if (!std::isfinite(norm) || norm <= maximum_norm) return derivative;
  return derivative * (maximum_norm / norm);
}

}  // namespace navigation_planning_backend

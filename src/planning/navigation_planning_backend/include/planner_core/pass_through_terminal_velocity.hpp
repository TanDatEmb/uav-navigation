#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <Eigen/Core>

namespace navigation_planning_backend {

// Compute a bounded outgoing velocity for a pass-through waypoint. The
// direction comes from the mission-owned next target; the planner's dynamic
// certificates still validate the complete polynomial afterwards.
inline std::optional<Eigen::Vector3d> passThroughTerminalVelocity(
    const Eigen::Vector3d& endpoint,
    const Eigen::Vector3d& next_target,
    const double maximum_velocity,
    const double maximum_acceleration) noexcept {
  if (!endpoint.allFinite() || !next_target.allFinite() ||
      !std::isfinite(maximum_velocity) || maximum_velocity <= 0.0 ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d delta = next_target - endpoint;
  const double distance = delta.norm();
  if (!std::isfinite(distance) || distance <= 1.0e-6) return std::nullopt;
  const double speed = std::min(
      maximum_velocity, std::sqrt(maximum_acceleration * distance));
  if (!std::isfinite(speed) || speed <= 0.0) return std::nullopt;
  return delta * (speed / distance);
}

}  // namespace navigation_planning_backend

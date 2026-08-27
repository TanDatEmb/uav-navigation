#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <Eigen/Core>

namespace navigation_planning_backend {

inline double passThroughMaximumVelocityChange(
    const double transition_duration_s,
    const double maximum_acceleration,
    const double maximum_jerk) noexcept {
  if (!std::isfinite(transition_duration_s) || transition_duration_s <= 0.0 ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0 ||
      !std::isfinite(maximum_jerk) || maximum_jerk <= 0.0) {
    return 0.0;
  }
  const double acceleration_ramp_s = maximum_acceleration / maximum_jerk;
  if (!std::isfinite(acceleration_ramp_s) || acceleration_ramp_s <= 0.0) {
    return 0.0;
  }
  const double maximum_delta = transition_duration_s <= 2.0 * acceleration_ramp_s
      ? 0.25 * maximum_jerk * transition_duration_s * transition_duration_s
      : maximum_acceleration * transition_duration_s -
            maximum_acceleration * maximum_acceleration / maximum_jerk;
  return std::isfinite(maximum_delta) ? std::max(0.0, maximum_delta) : 0.0;
}

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

// Shape an outgoing pass-through velocity without asking a short local leg to
// rotate the velocity vector instantaneously. The blend is a conservative
// jerk/acceleration-limited S-curve bound; the final MINCO trajectory remains
// responsible for the exact continuous V/A/J certificate.
inline std::optional<Eigen::Vector3d> passThroughTerminalVelocity(
    const Eigen::Vector3d& endpoint,
    const Eigen::Vector3d& next_target,
    const Eigen::Vector3d& incoming_velocity,
    const double transition_duration_s,
    const double maximum_velocity,
    const double maximum_acceleration,
    const double maximum_jerk) noexcept {
  const auto desired = passThroughTerminalVelocity(
      endpoint, next_target, maximum_velocity, maximum_acceleration);
  if (!desired.has_value() || !incoming_velocity.allFinite()) {
    return desired;
  }
  const Eigen::Vector3d delta = *desired - incoming_velocity;
  const double delta_norm = delta.norm();
  if (!std::isfinite(delta_norm) || delta_norm <= 1.0e-9) {
    return desired;
  }
  const double maximum_delta = passThroughMaximumVelocityChange(
      transition_duration_s, maximum_acceleration, maximum_jerk);
  if (maximum_delta <= 0.0 || delta_norm <= maximum_delta) {
    return desired;
  }
  return incoming_velocity + delta * (maximum_delta / delta_norm);
}

// A guide endpoint that stops at a sensing/planning frontier is not a mission
// waypoint.  Keep its terminal velocity on the guide tangent so a hot replan
// can renew the command without manufacturing a stop before the next map
// update.  The dynamic envelope still certifies the complete trajectory; this
// helper only chooses a bounded continuity state for the optimizer.
inline std::optional<Eigen::Vector3d> frontierContinuationVelocity(
    const Eigen::Vector3d& endpoint,
    const Eigen::Vector3d& previous_guide_point,
    const Eigen::Vector3d& incoming_velocity,
    const double preferred_speed,
    const double maximum_velocity,
    const double maximum_acceleration,
    const double maximum_jerk,
    const double transition_duration_s) noexcept {
  if (!endpoint.allFinite() || !previous_guide_point.allFinite() ||
      !incoming_velocity.allFinite() ||
      !std::isfinite(preferred_speed) || preferred_speed < 0.0 ||
      !std::isfinite(maximum_velocity) || maximum_velocity <= 0.0 ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0 ||
      !std::isfinite(maximum_jerk) || maximum_jerk <= 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d guide_delta = endpoint - previous_guide_point;
  const double guide_length = guide_delta.norm();
  if (!std::isfinite(guide_length) || guide_length <= 1.0e-6) {
    return std::nullopt;
  }

  const double incoming_speed = incoming_velocity.norm();
  const double requested_speed = preferred_speed > 1.0e-6
      ? preferred_speed
      : (std::isfinite(incoming_speed) ? incoming_speed : 0.0);
  const double speed = std::min(maximum_velocity, requested_speed);
  if (!std::isfinite(speed) || speed <= 1.0e-6) {
    return std::nullopt;
  }

  const Eigen::Vector3d desired = guide_delta * (speed / guide_length);
  const Eigen::Vector3d delta = desired - incoming_velocity;
  const double delta_norm = delta.norm();
  if (!std::isfinite(delta_norm) || delta_norm <= 1.0e-9) {
    return desired;
  }
  const double maximum_delta = passThroughMaximumVelocityChange(
      transition_duration_s, maximum_acceleration, maximum_jerk);
  if (maximum_delta <= 0.0 || delta_norm <= maximum_delta) {
    return desired;
  }
  return incoming_velocity + delta * (maximum_delta / delta_norm);
}

}  // namespace navigation_planning_backend

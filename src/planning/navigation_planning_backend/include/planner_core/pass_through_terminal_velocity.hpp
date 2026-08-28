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

// Upper bound for a terminal speed that can be reached over a guide path
// with a fixed time seed when the incoming speed is projected on its tangent.
// This is a parameterization guard, not a dynamic-limit relaxation; the
// optimizer and continuous certificate remain authoritative.
inline double terminalSpeedCapForPath(
    const double path_length_m,
    const double duration_s,
    const double incoming_speed_along_path_mps,
    const double maximum_velocity_mps) noexcept {
  if (!std::isfinite(path_length_m) || path_length_m <= 0.0 ||
      !std::isfinite(duration_s) || duration_s <= 0.0 ||
      !std::isfinite(incoming_speed_along_path_mps) ||
      !std::isfinite(maximum_velocity_mps) || maximum_velocity_mps <= 0.0) {
    return 0.0;
  }
  const double cap = 2.0 * path_length_m / duration_s -
      incoming_speed_along_path_mps;
  return std::clamp(cap, 0.0, maximum_velocity_mps);
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

// A pass-through waypoint with a genuine heading change needs a bounded
// route-window endpoint.  Ending the incoming polynomial at the exact corner
// leaves no geometric room for the next leg to rotate the velocity vector.
// The endpoint remains inside the mission-owned acceptance ball; it is not a
// waypoint skip and it never changes the collision or dynamic certificates.
struct PassThroughRouteWindow {
  Eigen::Vector3d entry;
  Eigen::Vector3d outgoing_blend;
  Eigen::Vector3d endpoint;
};

inline std::optional<PassThroughRouteWindow> passThroughRouteWindow(
    const Eigen::Vector3d& waypoint,
    const Eigen::Vector3d& next_target,
    const Eigen::Vector3d& incoming_tangent,
    const double acceptance_radius_m,
    const double connection_tolerance_m) noexcept {
  if (!waypoint.allFinite() || !next_target.allFinite() ||
      !incoming_tangent.allFinite() ||
      !std::isfinite(acceptance_radius_m) || acceptance_radius_m <= 0.0 ||
      !std::isfinite(connection_tolerance_m) || connection_tolerance_m < 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d outgoing_delta = next_target - waypoint;
  const double outgoing_length = outgoing_delta.norm();
  const double incoming_length = incoming_tangent.norm();
  if (!std::isfinite(outgoing_length) || outgoing_length <= 1.0e-6 ||
      !std::isfinite(incoming_length) || incoming_length <= 1.0e-6) {
    return std::nullopt;
  }

  // Keep the existing corner classifier in one place.  A dot product above
  // this value is a shallow bend and does not need a route window.
  constexpr double kGenuineCornerTangentDot = 0.7;
  if (incoming_tangent.normalized().dot(outgoing_delta / outgoing_length) >
      kGenuineCornerTangentDot) {
    return std::nullopt;
  }

  // The fixed interior ratio is geometry, not a safety threshold: it leaves
  // enough acceptance margin that the endpoint is not re-snapped to the
  // exact goal by the smaller planner connection tolerance.
  constexpr double kAcceptanceInteriorRatio = 0.75;
  const double offset_m = acceptance_radius_m * kAcceptanceInteriorRatio;
  if (!std::isfinite(offset_m) ||
      offset_m <= connection_tolerance_m + 1.0e-6) {
    return std::nullopt;
  }
  const Eigen::Vector3d endpoint = waypoint +
      (outgoing_delta / outgoing_length) * offset_m;
  const Eigen::Vector3d entry = waypoint -
      incoming_tangent.normalized() * offset_m;
  const Eigen::Vector3d outgoing_blend = waypoint +
      (outgoing_delta / outgoing_length) * (0.5 * offset_m);
  if (!entry.allFinite() || !outgoing_blend.allFinite() || !endpoint.allFinite() ||
      (entry - waypoint).norm() > acceptance_radius_m + 1.0e-6 ||
      (outgoing_blend - waypoint).norm() > acceptance_radius_m + 1.0e-6 ||
      (endpoint - waypoint).norm() > acceptance_radius_m + 1.0e-6) {
    return std::nullopt;
  }
  return PassThroughRouteWindow{entry, outgoing_blend, endpoint};
}

inline std::optional<Eigen::Vector3d> passThroughRouteWindowEndpoint(
    const Eigen::Vector3d& waypoint,
    const Eigen::Vector3d& next_target,
    const Eigen::Vector3d& incoming_tangent,
    const double acceptance_radius_m,
    const double connection_tolerance_m) noexcept {
  const auto window = passThroughRouteWindow(
      waypoint, next_target, incoming_tangent, acceptance_radius_m,
      connection_tolerance_m);
  return window.has_value()
      ? std::optional<Eigen::Vector3d>{window->endpoint} : std::nullopt;
}

}  // namespace navigation_planning_backend

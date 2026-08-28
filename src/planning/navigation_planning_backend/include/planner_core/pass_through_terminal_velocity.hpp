#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <Eigen/Core>

#include <planner_core/backup_braking.hpp>

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

// A genuine pass-through corner has only the mission acceptance region as
// local geometric room for changing heading. Bound the terminal seed used
// until the measured waypoint handoff by the centripetal envelope implied by
// that room. This is a parameterization guard, not a relaxation of the
// trajectory certificate; the generated polynomial still has to pass the
// strict continuous V/A/J and world checks.
inline double passThroughCornerSpeedCap(
    const double acceptance_radius_m,
    const double maximum_acceleration_mps2,
    const double maximum_velocity_mps) noexcept {
  if (!std::isfinite(acceptance_radius_m) || acceptance_radius_m <= 0.0 ||
      !std::isfinite(maximum_acceleration_mps2) ||
      maximum_acceleration_mps2 <= 0.0 ||
      !std::isfinite(maximum_velocity_mps) || maximum_velocity_mps <= 0.0) {
    return 0.0;
  }
  const double cap = std::sqrt(acceptance_radius_m * maximum_acceleration_mps2);
  return std::isfinite(cap) ? std::min(cap, maximum_velocity_mps) : 0.0;
}

// The next route piece must remain executable while the planner renews the
// command after a measured waypoint crossing.  Derive its minimum length from
// the current stopping envelope, two replan-forward intervals, and the
// configured receding prefix.  This is a route-window geometry bound, not a
// safety-gate relaxation; the selected path and the final polynomial retain
// their existing world and dynamic certificates.
inline double passThroughRequiredLookaheadDistance(
    const double speed_mps,
    const double maximum_velocity_mps,
    const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3,
    const double replan_forward_dt_s,
    const double receding_distance_m) noexcept;

inline double passThroughLookaheadDistance(
    const double speed_mps,
    const double maximum_velocity_mps,
    const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3,
    const double replan_forward_dt_s,
    const double receding_distance_m,
    const double available_route_distance_m) noexcept {
  if (!std::isfinite(available_route_distance_m) || available_route_distance_m <= 0.0) {
    return 0.0;
  }
  const double required_distance = passThroughRequiredLookaheadDistance(
      speed_mps, maximum_velocity_mps, maximum_acceleration_mps2,
      maximum_jerk_mps3, replan_forward_dt_s, receding_distance_m);
  if (!std::isfinite(required_distance) || required_distance <= 0.0) return 0.0;
  return std::min(available_route_distance_m, required_distance);
}

// Required route-window length before considering the finite mission leg.  The
// planner records this separately from the certified prefix so a map-bounded
// short prefix cannot masquerade as a complete continuity window.
inline double passThroughRequiredLookaheadDistance(
    const double speed_mps,
    const double maximum_velocity_mps,
    const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3,
    const double replan_forward_dt_s,
    const double receding_distance_m) noexcept {
  if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
      !std::isfinite(maximum_velocity_mps) || maximum_velocity_mps <= 0.0 ||
      !std::isfinite(maximum_acceleration_mps2) || maximum_acceleration_mps2 <= 0.0 ||
      !std::isfinite(maximum_jerk_mps3) || maximum_jerk_mps3 <= 0.0 ||
      !std::isfinite(replan_forward_dt_s) || replan_forward_dt_s <= 0.0 ||
      !std::isfinite(receding_distance_m) || receding_distance_m <= 0.0) {
    return 0.0;
  }
  const double bounded_speed = std::min(speed_mps, maximum_velocity_mps);
  const double stop_distance = jerkLimitedStopDistance(
      bounded_speed, maximum_acceleration_mps2, maximum_jerk_mps3);
  const double required_distance = stop_distance +
      2.0 * bounded_speed * replan_forward_dt_s + receding_distance_m;
  if (!std::isfinite(required_distance) || required_distance <= 0.0) {
    return 0.0;
  }
  return required_distance;
}

inline bool passThroughLookaheadComplete(
    const double required_distance_m, const double certified_distance_m) noexcept {
  return std::isfinite(required_distance_m) && required_distance_m > 0.0 &&
         std::isfinite(certified_distance_m) && certified_distance_m >= 0.0 &&
         certified_distance_m + 1.0e-6 >= required_distance_m;
}

// Long outgoing lookahead is a route-continuity mechanism for every
// pass-through boundary, including genuine corners. Corner classification
// selects the hard route-boundary gate; it must not disable the geometry that
// gives the optimizer time and distance to execute the turn.
inline bool passThroughOutgoingLookaheadEligible(
    const double desired_lookahead_m,
    const double outgoing_distance_m,
    const double remaining_search_distance_m,
    const double map_resolution_m) noexcept {
  return std::isfinite(desired_lookahead_m) && desired_lookahead_m > 1.0e-6 &&
         std::isfinite(outgoing_distance_m) && outgoing_distance_m > 1.0e-6 &&
         std::isfinite(remaining_search_distance_m) &&
         std::isfinite(map_resolution_m) && map_resolution_m > 0.0 &&
         remaining_search_distance_m > 2.0 * map_resolution_m;
}

// Outgoing-route lookahead may start only after the current guide actually
// reaches the controller-owned mission waypoint. A visibility-bounded prefix
// is an internal receding-horizon endpoint, never a route boundary.
inline bool passThroughGuideReachesMissionBoundary(
    const Eigen::Vector3d& guide_endpoint,
    const Eigen::Vector3d& mission_waypoint,
    const double connection_tolerance_m) noexcept {
  if (!guide_endpoint.allFinite() || !mission_waypoint.allFinite() ||
      !std::isfinite(connection_tolerance_m) || connection_tolerance_m < 0.0) {
    return false;
  }
  const double error = (guide_endpoint - mission_waypoint).norm();
  return std::isfinite(error) && error <= connection_tolerance_m + 1.0e-6;
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

inline bool passThroughGenuineCorner(
    const Eigen::Vector3d& waypoint,
    const Eigen::Vector3d& next_target,
    const Eigen::Vector3d& incoming_tangent) noexcept {
  if (!waypoint.allFinite() || !next_target.allFinite() ||
      !incoming_tangent.allFinite()) {
    return false;
  }
  const Eigen::Vector3d outgoing_delta = next_target - waypoint;
  const double outgoing_length = outgoing_delta.norm();
  const double incoming_length = incoming_tangent.norm();
  if (!std::isfinite(outgoing_length) || outgoing_length <= 1.0e-6 ||
      !std::isfinite(incoming_length) || incoming_length <= 1.0e-6) {
    return false;
  }
  // A shallow bend can terminate at the measured waypoint with its outgoing
  // tangent preserved. Extending that leg past the waypoint gives MINCO an
  // unnecessary soft endpoint and can let the nominal curve bow outside the
  // mission acceptance ball before the measured handoff occurs.
  constexpr double kGenuineCornerTangentDot = 0.7;
  return incoming_tangent.normalized().dot(outgoing_delta / outgoing_length) <=
      kGenuineCornerTangentDot;
}

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

  if (!passThroughGenuineCorner(waypoint, next_target, incoming_tangent)) {
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

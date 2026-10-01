#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Core>

#include <navigation_mission/route_progress.hpp>
#include <navigation_planning/planning_request.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <planner_core/pass_through_terminal_velocity.hpp>

namespace navigation_planning_backend {

struct RouteBackboneTarget {
  bool valid{false};
  double start_arc_m{0.0};
  double target_arc_m{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  bool reaches_active_waypoint{false};
};

// Geometry only: an immutable same-goal MAIN head inside a shallow
// pass-through boundary may continue toward the outgoing waypoint. The active
// mission identity/progress is unchanged; fresh export/admission still require
// the active boundary witness and the complete certified command. Genuine
// corners retain the existing acceptance-ball shaping path.
inline std::optional<Eigen::Vector3d> outgoingGoalAtUnacceptedMainBoundary(
    const navigation_planning::PlanningRequest& request) noexcept {
  if (!request.valid() || request.key.start_mode !=
          navigation_planning::PlanningStartMode::kCommittedFutureState ||
      !request.anchor ||
      request.anchor->active_role != navigation_planning::CandidateRole::kMain ||
      request.anchor->state.role != navigation_planning::CandidateRole::kMain ||
      request.anchor->goal_epoch != request.key.goal_epoch ||
      request.anchor->request_id != request.key.request_id) return std::nullopt;
  const auto& route = request.route_snapshot;
  const auto index = route.active_waypoint_index;
  if (index == 0U || index + 1U >= route.waypoints.size()) return std::nullopt;
  const auto& active = route.waypoints[index];
  const auto& next = route.waypoints[index + 1U];
  if (active.behavior != navigation_mission::MissionWaypoint::Behavior::PassThrough ||
      (request.anchor->state.position_world - active.position_enu).norm() >
          active.acceptance_radius_m ||
      (next.position_enu - active.position_enu).norm() <=
          navigation_world_model::kGoalConnectionToleranceM) return std::nullopt;
  const auto incoming = std::find_if(route.segments.begin(), route.segments.end(),
      [index](const navigation_mission::RouteSegment& segment) {
        return segment.end_waypoint_index == index;
      });
  if (incoming == route.segments.end() ||
      passThroughGenuineCorner(active.position_enu, next.position_enu,
                               incoming->tangent)) return std::nullopt;
  return next.position_enu;
}

// Select a deterministic local-search target on the active mission leg.  The
// target advances from the larger of measured route progress and the planning
// start projection and never crosses the active waypoint boundary.  It is a
// guidance point, not an executable endpoint: when the active waypoint is
// remote, keep it beyond the local search horizon so A* can return REACH_HORIZON
// after a detour instead of being forced to reach a horizon-edge point exactly.
// The caller still truncates and certifies the executable prefix at one horizon.
inline RouteBackboneTarget selectRouteBackboneTarget(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const Eigen::Vector3d& planning_start,
    const double maximum_distance_m) noexcept {
  RouteBackboneTarget output;
  if (!route.valid() || !planning_start.allFinite() ||
      !std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0 ||
      route.active_waypoint_index == 0U) {
    return output;
  }

  const auto segment_iterator = std::find_if(
      route.segments.begin(), route.segments.end(),
      [&route](const navigation_mission::RouteSegment& segment) {
        return segment.end_waypoint_index == route.active_waypoint_index;
      });
  if (segment_iterator == route.segments.end() ||
      !segment_iterator->tangent.allFinite() ||
      !std::isfinite(segment_iterator->length_m) ||
      segment_iterator->length_m <= 0.0) {
    return output;
  }

  const auto& segment = *segment_iterator;
  const double active_boundary =
      route.waypoint_arc_lengths_m[route.active_waypoint_index];
  const double projected_distance = std::clamp(
      (planning_start - segment.start).dot(segment.tangent),
      0.0, segment.length_m);
  const double projected_arc = segment.start_arc_m + projected_distance;
  const double start_arc = std::clamp(
      std::max(route.measured_progress.progress_arc_m, projected_arc),
      segment.start_arc_m, active_boundary);
  const auto projected_point = route.pointAtArc(projected_arc);
  if (!projected_point.has_value()) return output;

  const double lateral_distance_squared =
      (planning_start - *projected_point).squaredNorm();
  constexpr double kGuidanceHorizonMultiplier = 2.0;
  const double guidance_distance_m =
      kGuidanceHorizonMultiplier * maximum_distance_m;
  const double radius_squared = guidance_distance_m * guidance_distance_m;
  if (!std::isfinite(lateral_distance_squared) ||
      lateral_distance_squared > radius_squared) {
    return output;
  }

  const double forward_support =
      std::sqrt(std::max(0.0, radius_squared - lateral_distance_squared));
  const double target_arc =
      std::min(active_boundary,
               std::max(start_arc, projected_arc + forward_support));
  const auto target_point = route.pointAtArc(target_arc);
  if (!target_point.has_value() || !target_point->allFinite() ||
      target_arc <= start_arc + 1.0e-6) {
    return output;
  }

  output.valid = true;
  output.start_arc_m = start_arc;
  output.target_arc_m = target_arc;
  output.point = *target_point;
  output.reaches_active_waypoint =
      target_arc >= active_boundary - 1.0e-6;
  return output;
}

}  // namespace navigation_planning_backend

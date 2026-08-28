#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include <navigation_mission/route_progress.hpp>

namespace navigation_planning_backend {

struct RouteBackboneTarget {
  bool valid{false};
  double start_arc_m{0.0};
  double target_arc_m{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  bool reaches_active_waypoint{false};
};

// Select a deterministic local-search target on the active mission leg.  The
// target advances from the larger of measured route progress and the planning
// start projection, never crosses the active waypoint boundary, and remains
// inside the local executable radius.  A* may deform the path around sensed
// obstacles, while repeated solves keep aiming back at the same route spine.
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
  const double radius_squared = maximum_distance_m * maximum_distance_m;
  if (!std::isfinite(lateral_distance_squared) ||
      lateral_distance_squared > radius_squared) {
    return output;
  }

  const double forward_support =
      std::sqrt(std::max(0.0, radius_squared - lateral_distance_squared));
  const double target_arc =
      std::min(active_boundary, projected_arc + forward_support);
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

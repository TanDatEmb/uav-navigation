#include "navigation_runtime/mission_goal.hpp"

#include <geometry_msgs/msg/point.hpp>

namespace navigation_runtime {
namespace {
geometry_msgs::msg::Point point(const Eigen::Vector3d& vector) {
  geometry_msgs::msg::Point result;
  result.x = vector.x();
  result.y = vector.y();
  result.z = vector.z();
  return result;
}
}  // namespace

std::optional<navigation_contracts::msg::NavigationGoal> makeMissionGoal(
    const MissionProgress& progress, const builtin_interfaces::msg::Time& stamp) {
  const auto route = progress.routeSnapshot();
  const auto gate = progress.currentGate();
  if (!route.valid() || gate.request_id == 0U ||
      route.active_waypoint_index != gate.waypoint_index ||
      route.request_id != gate.request_id ||
      gate.waypoint_index >= route.waypoints.size()) {
    return std::nullopt;
  }
  const auto& waypoint = route.waypoints[gate.waypoint_index];
  navigation_contracts::msg::NavigationGoal goal;
  goal.header.stamp = stamp;
  goal.header.frame_id = route.frame;
  goal.mission_id = route.mission_id;
  goal.waypoint_index = gate.waypoint_index;
  goal.request_id = gate.request_id;
  goal.target = point(waypoint.position_enu);
  goal.acceptance_radius_m = waypoint.acceptance_radius_m;
  goal.behavior = waypoint.behavior == navigation_mission::MissionWaypoint::Behavior::Stop
      ? navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
      : navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  if (gate.waypoint_index + 1U < route.waypoints.size()) {
    goal.has_next_target = true;
    goal.next_target = point(route.waypoints[gate.waypoint_index + 1U].position_enu);
  }
  goal.route.mission_id = route.mission_id;
  goal.route.frame_id = route.frame;
  goal.route.route_revision = route.route_revision;
  goal.route.request_id = route.request_id;
  goal.route.active_waypoint_index = gate.waypoint_index;
  goal.route.measured_progress_valid = route.measured_progress.valid;
  goal.route.measured_segment_index = route.segments.empty()
      ? 0U : static_cast<std::uint32_t>(route.measured_progress.projection.segment_index);
  goal.route.measured_progress_arc_m = route.measured_progress.progress_arc_m;
  goal.route.measured_projection_arc_m = route.measured_progress.projection.arc_length_m;
  goal.route.measured_lateral_error_m = route.measured_progress.projection.lateral_error_m;
  goal.route.waypoint_positions.reserve(route.waypoints.size());
  goal.route.waypoint_ids.reserve(route.waypoints.size());
  goal.route.waypoint_acceptance_radii_m.reserve(route.waypoints.size());
  goal.route.waypoint_behaviors.reserve(route.waypoints.size());
  for (const auto& member : route.waypoints) {
    goal.route.waypoint_positions.push_back(point(member.position_enu));
    goal.route.waypoint_ids.push_back(member.id);
    goal.route.waypoint_acceptance_radii_m.push_back(member.acceptance_radius_m);
    goal.route.waypoint_behaviors.push_back(
        member.behavior == navigation_mission::MissionWaypoint::Behavior::Stop
            ? navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP
            : navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH);
  }
  return goal;
}
}  // namespace navigation_runtime

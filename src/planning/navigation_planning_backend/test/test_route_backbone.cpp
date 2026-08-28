#include "planner_core/route_backbone.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

navigation_mission::ImmutableRouteSnapshot makeSnapshot(
    const std::vector<Eigen::Vector3d>& positions,
    const std::size_t active_index,
    const Eigen::Vector3d& measured_position) {
  navigation_mission::Mission mission;
  mission.id = "route-backbone";
  mission.frame = "lio_odom";
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    navigation_mission::MissionWaypoint waypoint;
    waypoint.id = "wp-" + std::to_string(index);
    waypoint.position_enu = positions[index];
    waypoint.acceptance_radius_m = 1.0;
    waypoint.behavior = index + 1U == positions.size()
        ? navigation_mission::MissionWaypoint::Behavior::Stop
        : navigation_mission::MissionWaypoint::Behavior::PassThrough;
    mission.waypoints.push_back(std::move(waypoint));
  }
  navigation_mission::RouteProgress progress(mission);
  (void)progress.update(measured_position);
  return progress.snapshot(mission.id, mission.frame, 1U, 1U, active_index);
}

}  // namespace

TEST(RouteBackbone, SelectsVisibilityBoundedPointOnStraightMissionLeg) {
  const auto route = makeSnapshot(
      {{0.0, 0.0, 3.0}, {140.0, 0.0, 3.0}}, 1U,
      {10.0, 0.0, 3.0});
  const auto target = navigation_planning_backend::selectRouteBackboneTarget(
      route, {10.0, 0.0, 3.0}, 23.0);
  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.start_arc_m, 10.0, 1.0e-9);
  EXPECT_NEAR(target.target_arc_m, 33.0, 1.0e-9);
  EXPECT_NEAR(target.point.x(), 33.0, 1.0e-9);
  EXPECT_NEAR(target.point.y(), 0.0, 1.0e-9);
  EXPECT_FALSE(target.reaches_active_waypoint);
}

TEST(RouteBackbone, AccountsForCrossTrackDistanceWithoutChangingRouteSpine) {
  const auto route = makeSnapshot(
      {{0.0, 0.0, 3.0}, {140.0, 0.0, 3.0}}, 1U,
      {10.0, 0.0, 3.0});
  const auto target = navigation_planning_backend::selectRouteBackboneTarget(
      route, {10.0, 8.0, 3.0}, 23.0);
  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.target_arc_m,
              10.0 + std::sqrt(23.0 * 23.0 - 8.0 * 8.0), 1.0e-9);
  EXPECT_DOUBLE_EQ(target.point.y(), 0.0);
}

TEST(RouteBackbone, NeverCrossesActiveWaypointBoundary) {
  const auto route = makeSnapshot(
      {{0.0, 0.0, 3.0}, {30.0, 0.0, 3.0}, {30.0, 40.0, 3.0}}, 1U,
      {22.0, 0.0, 3.0});
  const auto target = navigation_planning_backend::selectRouteBackboneTarget(
      route, {22.0, 0.0, 3.0}, 23.0);
  ASSERT_TRUE(target.valid);
  EXPECT_TRUE(target.reaches_active_waypoint);
  EXPECT_NEAR(target.target_arc_m, 30.0, 1.0e-9);
  EXPECT_NEAR(target.point.x(), 30.0, 1.0e-9);
  EXPECT_NEAR(target.point.y(), 0.0, 1.0e-9);
}

TEST(RouteBackbone, UsesPlanningStartProjectionButNeverRegressesHighWater) {
  auto route = makeSnapshot(
      {{0.0, 0.0, 3.0}, {140.0, 0.0, 3.0}}, 1U,
      {40.0, 0.0, 3.0});
  const auto behind = navigation_planning_backend::selectRouteBackboneTarget(
      route, {35.0, 0.0, 3.0}, 23.0);
  ASSERT_TRUE(behind.valid);
  EXPECT_NEAR(behind.start_arc_m, 40.0, 1.0e-9);
  EXPECT_NEAR(behind.target_arc_m, 58.0, 1.0e-9);
  EXPECT_LE((behind.point - Eigen::Vector3d{35.0, 0.0, 3.0}).norm(),
            23.0 + 1.0e-9);

  const auto ahead = navigation_planning_backend::selectRouteBackboneTarget(
      route, {46.0, 0.0, 3.0}, 23.0);
  ASSERT_TRUE(ahead.valid);
  EXPECT_NEAR(ahead.start_arc_m, 46.0, 1.0e-9);
}

TEST(RouteBackbone, RejectsUnsupportedOrNonForwardTargets) {
  const auto initial = makeSnapshot(
      {{0.0, 0.0, 3.0}, {140.0, 0.0, 3.0}}, 0U,
      {0.0, 0.0, 3.0});
  EXPECT_FALSE(navigation_planning_backend::selectRouteBackboneTarget(
      initial, {0.0, 0.0, 3.0}, 23.0).valid);

  const auto terminal = makeSnapshot(
      {{0.0, 0.0, 3.0}, {140.0, 0.0, 3.0}}, 1U,
      {140.0, 0.0, 3.0});
  EXPECT_FALSE(navigation_planning_backend::selectRouteBackboneTarget(
      terminal, {140.0, 0.0, 3.0}, 23.0).valid);
  EXPECT_FALSE(navigation_planning_backend::selectRouteBackboneTarget(
      terminal, {140.0, 0.0, 3.0},
      std::numeric_limits<double>::quiet_NaN()).valid);
}

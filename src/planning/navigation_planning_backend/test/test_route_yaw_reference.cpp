#include "planner_core/route_yaw_reference.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

navigation_mission::ImmutableRouteSnapshot makeSnapshot(
    const std::vector<Eigen::Vector3d>& positions,
    const std::size_t active_index = 0U) {
  navigation_mission::Mission mission;
  mission.id = "yaw-route";
  mission.frame = "lio_odom";
  for (std::size_t index = 0; index < positions.size(); ++index) {
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
  (void)progress.update(positions.front());
  return progress.snapshot(
      mission.id, mission.frame, 1U, 1U, active_index);
}

}  // namespace

TEST(RouteYawReference, FacesEveryHorizontalBearingWithShortestUnwrap) {
  const std::vector<double> bearings{
      -M_PI, -3.0 * M_PI_4, -M_PI_2, -M_PI_4,
      0.0, M_PI_4, M_PI_2, 3.0 * M_PI_4};
  for (const double bearing : bearings) {
    const Eigen::Vector3d end{
        20.0 * std::cos(bearing), 20.0 * std::sin(bearing), 3.0};
    const auto route = makeSnapshot({Eigen::Vector3d{0.0, 0.0, 3.0}, end});
    const auto reference = navigation_planning_backend::computeRouteYawReference(
        route, Eigen::Vector3d{0.0, 0.0, 3.0},
        Eigen::Vector3d{std::cos(bearing), std::sin(bearing), 0.0}, 3.0);
    ASSERT_TRUE(reference.valid);
    EXPECT_EQ(reference.source,
              navigation_planning_backend::RouteYawSource::kRouteLookahead);
    EXPECT_NEAR(std::remainder(reference.target_yaw_rad - bearing, 2.0 * M_PI),
                0.0, 1.0e-9);
    EXPECT_LE(std::abs(reference.target_yaw_rad - 3.0), M_PI + 1.0e-9);
  }
}

TEST(RouteYawReference, HoldsMeasuredYawAtStandstillAndPureVerticalMotion) {
  const auto horizontal = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0}, Eigen::Vector3d{20.0, 0.0, 3.0}});
  const auto stopped = navigation_planning_backend::computeRouteYawReference(
      horizontal, Eigen::Vector3d{0.0, 0.0, 3.0}, Eigen::Vector3d::Zero(), 1.2);
  EXPECT_TRUE(stopped.valid);
  EXPECT_EQ(stopped.source,
            navigation_planning_backend::RouteYawSource::kHoldLowSpeed);
  EXPECT_DOUBLE_EQ(stopped.target_yaw_rad, 1.2);

  const auto vertical = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0}, Eigen::Vector3d{0.0, 0.0, 20.0}});
  const auto climbing = navigation_planning_backend::computeRouteYawReference(
      vertical, Eigen::Vector3d{0.0, 0.0, 4.0},
      Eigen::Vector3d{0.0, 0.0, 2.0}, -0.8);
  EXPECT_TRUE(climbing.valid);
  EXPECT_EQ(climbing.source,
            navigation_planning_backend::RouteYawSource::kHoldLowSpeed);
  EXPECT_DOUBLE_EQ(climbing.target_yaw_rad, -0.8);
}

TEST(RouteYawReference, StraightLegTargetDoesNotDependOnTrajectoryGeneration) {
  const auto route = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0}, Eigen::Vector3d{50.0, 0.0, 3.0}});
  const auto first = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{8.0, 0.2, 3.0}, Eigen::Vector3d{5.0, 0.0, 0.0}, 0.1);
  const auto replan = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{8.0, 0.2, 3.0}, Eigen::Vector3d{5.0, 0.0, 0.0}, 0.1);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(replan.valid);
  EXPECT_DOUBLE_EQ(first.target_yaw_rad, replan.target_yaw_rad);
  EXPECT_DOUBLE_EQ(first.progress_arc_m, replan.progress_arc_m);
}

TEST(RouteYawReference, LooksIntoOutgoingCornerBeforeWaypointAcceptance) {
  auto route = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{10.0, 0.0, 3.0},
      Eigen::Vector3d{10.0, 20.0, 3.0}}, 1U);
  route.measured_progress.progress_arc_m = 7.0;
  route.measured_progress.projection.valid = true;
  route.measured_progress.projection.segment_index = 0U;
  route.measured_progress.projection.arc_length_m = 7.0;
  route.measured_progress.projection.point = Eigen::Vector3d{7.0, 0.0, 3.0};
  route.measured_progress.projection.tangent = Eigen::Vector3d::UnitX();
  const auto reference = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{8.0, 0.0, 3.0}, Eigen::Vector3d{5.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(reference.valid);
  EXPECT_EQ(reference.source,
            navigation_planning_backend::RouteYawSource::kRouteLookahead);
  EXPECT_GT(reference.target_yaw_rad, 0.0);
  EXPECT_LT(reference.target_yaw_rad, M_PI_2);
}

TEST(RouteYawReference, ReversalDoesNotTurnBeforeStopTurnGoBoundary) {
  auto route = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{10.0, 0.0, 3.0},
      Eigen::Vector3d{0.0, 0.0, 3.0}}, 1U);
  route.measured_progress.progress_arc_m = 8.0;
  route.measured_progress.projection.valid = true;
  route.measured_progress.projection.segment_index = 0U;
  route.measured_progress.projection.arc_length_m = 8.0;
  route.measured_progress.projection.point = Eigen::Vector3d{8.0, 0.0, 3.0};
  route.measured_progress.projection.tangent = Eigen::Vector3d::UnitX();
  const auto approaching = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{8.0, 0.0, 3.0}, Eigen::Vector3d{4.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(approaching.valid);
  EXPECT_NEAR(approaching.target_yaw_rad, 0.0, 1.0e-9);
  EXPECT_LE(approaching.target_point.x(), 10.0);
}

TEST(RouteYawReference, ReversalTurnsTowardOutgoingLegOnlyAfterTransition) {
  auto route = makeSnapshot({
      Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{10.0, 0.0, 3.0},
      Eigen::Vector3d{0.0, 0.0, 3.0}}, 2U);
  route.measured_progress.progress_arc_m = 10.0;
  route.measured_progress.projection.valid = true;
  route.measured_progress.projection.segment_index = 0U;
  route.measured_progress.projection.arc_length_m = 10.0;
  route.measured_progress.projection.point = Eigen::Vector3d{10.0, 0.0, 3.0};
  route.measured_progress.projection.tangent = Eigen::Vector3d::UnitX();

  const auto turning = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{10.0, 0.0, 3.0}, Eigen::Vector3d::Zero(), 0.0);
  ASSERT_TRUE(turning.valid);
  EXPECT_EQ(turning.source,
            navigation_planning_backend::RouteYawSource::kRouteTurnInPlace);
  EXPECT_NEAR(std::abs(turning.target_yaw_rad), M_PI, 1.0e-9);
}

TEST(RouteYawReference, CrossingGeometryCannotJumpBeyondActiveWaypoint) {
  auto route = makeSnapshot({
      Eigen::Vector3d{-10.0, 0.0, 3.0},
      Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{10.0, 0.0, 3.0},
      Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{0.0, 10.0, 3.0}}, 1U);
  route.measured_progress.progress_arc_m = 8.0;
  route.measured_progress.projection.valid = true;
  route.measured_progress.projection.segment_index = 0U;
  route.measured_progress.projection.arc_length_m = 8.0;
  route.measured_progress.projection.point = Eigen::Vector3d{-2.0, 0.0, 3.0};
  route.measured_progress.projection.tangent = Eigen::Vector3d::UnitX();

  const auto reference = navigation_planning_backend::computeRouteYawReference(
      route, Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{2.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(reference.valid);
  EXPECT_LE(reference.progress_arc_m,
            route.waypoint_arc_lengths_m[route.active_waypoint_index]);
}

#include <gtest/gtest.h>

#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/navigation_command_contract.hpp>

TEST(NavigationContracts, RegisteredScanCarriesAtomicIdentityAndPayload) {
  navigation_contracts::msg::RegisteredScan message;
  message.localization_epoch = 7U;
  message.scan_sequence = 11U;
  message.header.frame_id = "lio_odom";
  message.body_frame_id = "base_link";
  message.points.header.frame_id = message.header.frame_id;
  message.points.header.stamp = message.header.stamp;
  message.free_space_endpoints.header.frame_id = message.header.frame_id;
  message.free_space_endpoints.header.stamp = message.header.stamp;

  EXPECT_EQ(message.localization_epoch, 7U);
  EXPECT_EQ(message.scan_sequence, 11U);
  EXPECT_EQ(message.points.header.frame_id, "lio_odom");
  EXPECT_EQ(message.free_space_endpoints.header.frame_id, "lio_odom");
  EXPECT_EQ(message.body_frame_id, "base_link");
}

TEST(NavigationContracts, PropagatedOdometryCarriesEpochSequenceAndSourceState) {
  navigation_contracts::msg::PropagatedOdometry message;
  message.localization_epoch = 7U;
  message.sequence = 11U;
  message.odometry.header.frame_id = "lio_odom";
  message.odometry.child_frame_id = "base_link";
  message.odometry.header.stamp.sec = 12;
  message.odometry.header.stamp.nanosec = 34U;

  EXPECT_EQ(message.localization_epoch, 7U);
  EXPECT_EQ(message.sequence, 11U);
  EXPECT_EQ(message.odometry.header.frame_id, "lio_odom");
  EXPECT_EQ(message.odometry.child_frame_id, "base_link");
  EXPECT_EQ(message.odometry.header.stamp.sec, 12);
  EXPECT_EQ(message.odometry.header.stamp.nanosec, 34U);
}

TEST(NavigationContracts, NavigationGoalCarriesVersionedRouteAndMeasuredProgress) {
  navigation_contracts::msg::NavigationGoal message;
  message.header.frame_id = "lio_odom";
  message.mission_id = "three-pillars";
  message.waypoint_index = 1U;
  message.request_id = 7U;
  message.route.mission_id = message.mission_id;
  message.route.frame_id = message.header.frame_id;
  message.route.route_revision = 3U;
  message.route.request_id = message.request_id;
  message.route.active_waypoint_index = message.waypoint_index;
  message.route.measured_progress_valid = true;
  message.route.measured_segment_index = 0U;
  message.route.measured_progress_arc_m = 8.5;
  message.route.measured_projection_arc_m = 8.0;
  message.route.measured_lateral_error_m = 0.2;
  geometry_msgs::msg::Point point;
  point.x = 10.0;
  point.z = 2.0;
  message.route.waypoint_positions.push_back(point);
  message.route.waypoint_ids.push_back("wp-1");
  message.route.waypoint_acceptance_radii_m.push_back(1.0);
  message.route.waypoint_behaviors.push_back(
      navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH);

  EXPECT_EQ(message.route.mission_id, message.mission_id);
  EXPECT_EQ(message.route.frame_id, message.header.frame_id);
  EXPECT_EQ(message.route.route_revision, 3U);
  EXPECT_EQ(message.route.request_id, message.request_id);
  EXPECT_EQ(message.route.active_waypoint_index, message.waypoint_index);
  ASSERT_EQ(message.route.waypoint_positions.size(), 1U);
  EXPECT_EQ(message.route.waypoint_ids.front(), "wp-1");
  EXPECT_DOUBLE_EQ(message.route.measured_progress_arc_m, 8.5);
  EXPECT_DOUBLE_EQ(message.route.measured_projection_arc_m, 8.0);
}

TEST(NavigationContracts, EstimatorHealthUsesTypedStateAndIndependentFlags) {
  navigation_contracts::msg::EstimatorHealth message;
  message.state = navigation_contracts::msg::EstimatorHealth::TRACKING;
  message.navigation_valid = true;
  message.covariance_valid = true;
  message.observability_valid = false;
  message.correction_fresh = true;
  message.propagation_valid = true;
  message.last_propagated_state_stamp.sec = 12;
  message.last_propagated_state_stamp.nanosec = 34U;

  EXPECT_EQ(message.state,
            navigation_contracts::msg::EstimatorHealth::TRACKING);
  EXPECT_TRUE(message.navigation_valid);
  EXPECT_TRUE(message.covariance_valid);
  EXPECT_FALSE(message.observability_valid);
  EXPECT_TRUE(message.correction_fresh);
  EXPECT_TRUE(message.propagation_valid);
  EXPECT_EQ(message.last_propagated_state_stamp.sec, 12);
  EXPECT_EQ(message.last_propagated_state_stamp.nanosec, 34U);
}

TEST(NavigationContracts, NavigationCommandExposesAllProvenanceDimensions) {
  navigation_contracts::msg::NavigationCommand message;
  message.localization_epoch = 3U;
  message.goal_epoch = 5U;
  message.request_id = 8U;
  message.world_generation = 13U;
  message.world_revision = 21U;
  message.bundle_generation = 34U;
  message.sample_id = 55U;
  message.role = navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;
  message.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;

  EXPECT_EQ(message.role,
            navigation_contracts::msg::NavigationCommand::ROLE_BACKUP);
  EXPECT_EQ(message.status,
            navigation_contracts::msg::NavigationCommand::STATUS_READY);
  EXPECT_EQ(message.localization_epoch, 3U);
  EXPECT_EQ(message.goal_epoch, 5U);
  EXPECT_EQ(message.world_generation, 13U);
  EXPECT_EQ(message.world_revision, 21U);
  EXPECT_EQ(message.bundle_generation, 34U);
  EXPECT_EQ(message.sample_id, 55U);
}

TEST(NavigationContracts, CommandsRequireHealthyTypedEpochHandshake) {
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(false, true, 3U, 3U));
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(true, false, 3U, 3U));
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(true, true, 2U, 3U));
  EXPECT_TRUE(navigation_contracts::estimatorHealthAllowsCommand(true, true, 3U, 3U));
}

TEST(NavigationContracts, NavigationCommandContractRejectsMalformedOrRegressedIdentity) {
  navigation_contracts::msg::NavigationCommand previous;
  previous.header.frame_id = "lio_odom";
  previous.header.stamp.sec = 10;
  previous.valid_until.sec = 11;
  previous.world_observation_stamp.sec = 9;
  previous.state_source_stamp.sec = 9;
  previous.mission_id = "mission-a";
  previous.localization_epoch = 3U;
  previous.goal_epoch = 5U;
  previous.world_generation = 13U;
  previous.world_revision = 21U;
  previous.bundle_generation = 34U;
  previous.sample_id = 55U;
  previous.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  previous.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  EXPECT_TRUE(navigation_contracts::commandContractValid(previous, "lio_odom"));

  auto newer = previous;
  newer.header.stamp.sec = 12;
  newer.valid_until.sec = 13;
  newer.world_revision = 22U;
  newer.state_source_stamp.sec = 10;
  newer.sample_id = 56U;
  EXPECT_TRUE(navigation_contracts::commandWorldIdentityNonRegressing(newer, previous));

  auto regressed = newer;
  regressed.world_revision = previous.world_revision - 1U;
  EXPECT_FALSE(navigation_contracts::commandWorldIdentityNonRegressing(regressed, previous));
  regressed = newer;
  regressed.header.frame_id = "map";
  EXPECT_FALSE(navigation_contracts::commandContractValid(regressed, "lio_odom"));
  auto malformed = previous;
  malformed.mission_id.clear();
  EXPECT_FALSE(navigation_contracts::commandContractValid(malformed, "lio_odom"));
  malformed = previous;
  malformed.trajectory_time_s = -1.0;
  EXPECT_FALSE(navigation_contracts::commandContractValid(malformed, "lio_odom"));
  EXPECT_FALSE(navigation_contracts::commandContractValid(previous, ""));
  EXPECT_TRUE(navigation_contracts::commandValidAt(previous, 10'500'000'000LL));
  EXPECT_FALSE(navigation_contracts::commandValidAt(previous, 11'000'000'001LL));
  EXPECT_TRUE(navigation_contracts::commandMissionIdentityMatches(
      previous, "", 0U, 0U) == false);
  previous.mission_id = "mission-a";
  previous.waypoint_index = 2U;
  previous.request_id = 8U;
  EXPECT_TRUE(navigation_contracts::commandMissionIdentityMatches(
      previous, "mission-a", 2U, 8U));
  EXPECT_FALSE(navigation_contracts::commandMissionIdentityMatches(
      previous, "mission-b", 2U, 8U));
}

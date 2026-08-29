#include <gtest/gtest.h>

#include <navigation_common/frame_conventions.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include "px4_navigation_external_mode/reject_provenance.hpp"
#include "px4_navigation_external_mode/certified_command_handoff.hpp"
#include "px4_navigation_external_mode/command_acceptance_gate.hpp"
#include "px4_navigation_external_mode/mission_command_identity.hpp"
#include "px4_navigation_external_mode/planner_recovery.hpp"
#include "px4_navigation_external_mode/runtime_metrics_policy.hpp"

TEST(RuntimeMetricsPolicy, RejectsClockRegressionWithoutOverflow) {
  EXPECT_FALSE(px4_navigation_external_mode::runtimeMetricsLogDue(
      std::numeric_limits<std::int64_t>::max(), 1));
  EXPECT_FALSE(px4_navigation_external_mode::runtimeMetricsLogDue(
      2'000'000'000LL, 1'000'000'000LL));
  EXPECT_TRUE(px4_navigation_external_mode::runtimeMetricsLogDue(
      1'000'000'000LL, 2'000'000'000LL));
}

TEST(NavigationCommandContract, StaleOdometryPrecedesDuplicateMessageRejection) {
  navigation_contracts::ExecutionStateFreshness stale;
  stale.reason = navigation_contracts::ExecutionStateFreshnessReason::kReceiveStale;
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(stale, 10U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kOdometryStale);

  navigation_contracts::ExecutionStateFreshness fresh;
  fresh.reason = navigation_contracts::ExecutionStateFreshnessReason::kValid;
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(fresh, 10U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kNonIncreasingMessageId);
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(fresh, 11U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kAccept);
}

TEST(NavigationCommandContract, ExactLateTerminalIdentityIsAccepted) {
  navigation_contracts::msg::NavigationCommand command;
  command.mission_id = "mission";
  command.waypoint_index = 8U;
  command.request_id = 17U;

  EXPECT_TRUE(px4_navigation_external_mode::missionCommandIdentityMatches(
      command, "mission", 9U, 17U, true, 8U, 17U));
}

TEST(NavigationCommandContract, WrongLateTerminalIdentityIsRejected) {
  navigation_contracts::msg::NavigationCommand command;
  command.mission_id = "mission";
  command.waypoint_index = 8U;
  command.request_id = 18U;

  EXPECT_FALSE(px4_navigation_external_mode::missionCommandIdentityMatches(
      command, "mission", 9U, 18U, true, 8U, 17U));
  command.request_id = 17U;
  command.waypoint_index = 7U;
  EXPECT_FALSE(px4_navigation_external_mode::missionCommandIdentityMatches(
      command, "mission", 9U, 17U, true, 8U, 17U));
}

TEST(NavigationCommandContract, DuplicateLateTerminalSampleStillFailsMonotonicGate) {
  navigation_contracts::ExecutionStateFreshness fresh;
  fresh.reason = navigation_contracts::ExecutionStateFreshnessReason::kValid;
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(fresh, 42U, 42U),
            px4_navigation_external_mode::CommandAcceptanceGate::kNonIncreasingMessageId);
}

TEST(NavigationCommandContract, GoalHandoffRetainsExactCertifiedCommandIdentity) {
  navigation_contracts::msg::NavigationCommand current;
  current.mission_id = "mission";
  current.waypoint_index = 3U;
  current.request_id = 8U;
  current.sample_id = 21U;

  const auto retained = px4_navigation_external_mode::transitionCertifiedCommand(
      current, std::nullopt,
      px4_navigation_external_mode::CertifiedCommandTransition::kRetain);
  ASSERT_TRUE(retained.has_value());
  EXPECT_EQ(retained->waypoint_index, 3U);
  EXPECT_EQ(retained->request_id, 8U);
  EXPECT_EQ(retained->sample_id, 21U);
}

TEST(NavigationCommandContract, AcceptedReplacementCommitsAtomically) {
  navigation_contracts::msg::NavigationCommand current;
  current.waypoint_index = 3U;
  current.request_id = 8U;
  navigation_contracts::msg::NavigationCommand replacement;
  replacement.waypoint_index = 4U;
  replacement.request_id = 9U;

  const auto committed = px4_navigation_external_mode::transitionCertifiedCommand(
      current, replacement,
      px4_navigation_external_mode::CertifiedCommandTransition::kCommit);
  ASSERT_TRUE(committed.has_value());
  EXPECT_EQ(committed->waypoint_index, 4U);
  EXPECT_EQ(committed->request_id, 9U);
}

TEST(NavigationCommandContract, LifecycleInvalidationClearsCertifiedCommand) {
  navigation_contracts::msg::NavigationCommand current;
  current.sample_id = 21U;
  EXPECT_FALSE(px4_navigation_external_mode::transitionCertifiedCommand(
      current, std::nullopt,
      px4_navigation_external_mode::CertifiedCommandTransition::kInvalidate).has_value());
}

TEST(NavigationCommandContract, BackupRecoveryWindowIsBoundedAndHoldOnly) {
  EXPECT_FALSE(px4_navigation_external_mode::plannerRecoveryWaitExpired(
      false, 2'000'000'000LL, 1'000'000'000LL));
  EXPECT_FALSE(px4_navigation_external_mode::plannerRecoveryWaitExpired(
      true, 999'000'000LL, 1'000'000'000LL));
  EXPECT_TRUE(px4_navigation_external_mode::plannerRecoveryWaitExpired(
      true, 1'000'000'000LL, 1'000'000'000LL));
  EXPECT_TRUE(px4_navigation_external_mode::plannerRecoveryWaitExpired(
      true, 1'100'000'000LL, 1'000'000'000LL));
}

TEST(NavigationCommandContract, BackupEndpointNearAcceptanceEdgeUsesAnchorEnvelope) {
  EXPECT_TRUE(px4_navigation_external_mode::backupEndpointHoldIsAnchored(
      true, true, true, 0.24, 0.25));
  EXPECT_FALSE(px4_navigation_external_mode::backupEndpointHoldIsAnchored(
      true, true, true, 0.26, 0.25));
  EXPECT_FALSE(px4_navigation_external_mode::backupEndpointHoldIsAnchored(
      false, true, true, 0.01, 0.25));
  EXPECT_FALSE(px4_navigation_external_mode::backupEndpointHoldIsAnchored(
      true, false, true, 0.01, 0.25));
}

TEST(NavigationCommandContract, UsesDistinctMainAndBackupRoles) {
  navigation_contracts::msg::NavigationCommand command;
  command.sample_id = 17U;
  command.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  command.role = navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;

  EXPECT_EQ(command.status, navigation_contracts::msg::NavigationCommand::STATUS_READY);
  EXPECT_EQ(command.role, navigation_contracts::msg::NavigationCommand::ROLE_BACKUP);
  EXPECT_NE(command.role, navigation_contracts::msg::NavigationCommand::ROLE_MAIN);
}

TEST(NavigationCommandContract, KeepsSampleIdDistinctFromBundleAndTime) {
  navigation_contracts::msg::NavigationCommand command;
  command.sample_id = 130;
  command.bundle_generation = 5;
  command.trajectory_time_s = 1.25;
  command.role = navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;

  EXPECT_EQ(command.sample_id, 130U);
  EXPECT_EQ(command.bundle_generation, 5U);
  EXPECT_DOUBLE_EQ(command.trajectory_time_s, 1.25);
  EXPECT_EQ(command.role, navigation_contracts::msg::NavigationCommand::ROLE_BACKUP);
}

TEST(NavigationCommandContract, ConvertsYawAndYawRateFromEnuToNed) {
  EXPECT_FLOAT_EQ(1.57079632679F, px4_ros2::yawEnuToNed(0.0F));
  EXPECT_FLOAT_EQ(-0.7F, px4_ros2::yawRateEnuToNed(0.7F));
}

TEST(NavigationCommandContract, ConvertsPvaWithTheSharedEnuNedMatrix) {
  const Eigen::Vector3d enu_position{1.0, 2.0, 3.0};
  const Eigen::Vector3d enu_velocity{-4.0, 5.0, -6.0};
  EXPECT_TRUE(navigation_common::enuToNed(enu_position).isApprox(
      Eigen::Vector3d{2.0, 1.0, -3.0}));
  EXPECT_TRUE(navigation_common::enuToNed(enu_velocity).isApprox(
      Eigen::Vector3d{5.0, -4.0, 6.0}));
  EXPECT_TRUE((navigation_common::c_enu_ned() *
               navigation_common::c_enu_ned()).isApprox(Eigen::Matrix3d::Identity()));
}

TEST(NavigationCommandContract, RejectProvenanceIsExactWithoutPreviousCommand) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = 1;
  odometry.header.stamp.nanosec = 100'000'000U;
  odometry.pose.pose.position.x = 1.0;
  odometry.twist.twist.linear.y = 2.0;
  navigation_contracts::msg::NavigationCommand command;
  const auto result = px4_navigation_external_mode::buildRejectProvenance(
      1'300'000'000LL, 1'250'000'000LL, odometry, command, std::nullopt);
  EXPECT_DOUBLE_EQ(result.odometry_header_age_ms, 200.0);
  EXPECT_DOUBLE_EQ(result.odometry_receive_age_ms, 50.0);
  EXPECT_FALSE(result.previous_valid);
  EXPECT_TRUE(result.measured_position.isApprox(Eigen::Vector3d{1.0, 0.0, 0.0}));
  EXPECT_TRUE(result.measured_velocity.isApprox(Eigen::Vector3d{0.0, 2.0, 0.0}));
  EXPECT_TRUE(std::isnan(result.command_delta_position_m));
}

TEST(NavigationCommandContract, RejectProvenancePreservesPreviousPvajAndSignedDelta) {
  nav_msgs::msg::Odometry odometry;
  navigation_contracts::msg::NavigationCommand previous;
  previous.bundle_generation = 9U;
  previous.trajectory_time_s = 1.5;
  previous.position.x = 1.0;
  previous.velocity.y = 2.0;
  previous.acceleration.z = 3.0;
  previous.jerk.x = 4.0;
  navigation_contracts::msg::NavigationCommand command;
  command.bundle_generation = 7U;
  command.position.x = 2.0;
  command.velocity.y = 4.0;
  command.acceleration.z = 6.0;
  command.jerk.x = 8.0;
  auto result = px4_navigation_external_mode::buildRejectProvenance(
      0, 0, odometry, command, previous);
  EXPECT_TRUE(result.previous_valid);
  EXPECT_TRUE(result.generation_changed);
  EXPECT_EQ(result.generation_delta, -2);
  EXPECT_DOUBLE_EQ(result.command_delta_position_m, 1.0);
  EXPECT_DOUBLE_EQ(result.command_delta_velocity_mps, 2.0);
  EXPECT_DOUBLE_EQ(result.command_delta_acceleration_mps2, 3.0);
  EXPECT_DOUBLE_EQ(result.command_delta_jerk_mps3, 4.0);

  command.bundle_generation = 9U;
  result = px4_navigation_external_mode::buildRejectProvenance(
      0, 0, odometry, command, previous);
  EXPECT_FALSE(result.generation_changed);
  EXPECT_EQ(result.generation_delta, 0);
}

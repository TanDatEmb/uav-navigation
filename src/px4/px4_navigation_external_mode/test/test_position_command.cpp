#include <gtest/gtest.h>

#include <coordinate_conventions/frame_conventions.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include "px4_navigation_external_mode/reject_provenance.hpp"
#include "px4_navigation_external_mode/command_acceptance_gate.hpp"

TEST(PositionCommandContract, StaleOdometryPrecedesDuplicateMessageRejection) {
  navigation_interfaces::ExecutionStateFreshness stale;
  stale.reason = navigation_interfaces::ExecutionStateFreshnessReason::kReceiveStale;
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(stale, 10U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kOdometryStale);

  navigation_interfaces::ExecutionStateFreshness fresh;
  fresh.reason = navigation_interfaces::ExecutionStateFreshnessReason::kValid;
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(fresh, 10U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kNonIncreasingMessageId);
  EXPECT_EQ(px4_navigation_external_mode::classifyCommandAcceptance(fresh, 11U, 10U),
            px4_navigation_external_mode::CommandAcceptanceGate::kAccept);
}

TEST(PositionCommandContract, UsesDistinctMainAndBackupFlags) {
  mars_quadrotor_msgs::msg::PositionCommand command;
  command.trajectory_id = 17U;
  command.trajectory_status =
      mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  command.trajectory_flag =
      mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP;

  EXPECT_NE(command.trajectory_status, command.trajectory_flag);
  EXPECT_EQ(command.trajectory_flag,
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP);
  EXPECT_NE(command.trajectory_flag,
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_MAIN);
}

TEST(PositionCommandContract, KeepsMessageIdDistinctFromCommittedGenerationAndTime) {
  mars_quadrotor_msgs::msg::PositionCommand command;
  command.trajectory_id = 130;
  command.trajectory_generation = 5;
  command.trajectory_time_s = 1.25;
  command.trajectory_flag =
      mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP;

  EXPECT_EQ(command.trajectory_id, 130U);
  EXPECT_EQ(command.trajectory_generation, 5U);
  EXPECT_DOUBLE_EQ(command.trajectory_time_s, 1.25);
  EXPECT_EQ(command.trajectory_flag,
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP);
}

TEST(PositionCommandContract, ConvertsSuperYawAndYawRateFromEnuToNed) {
  EXPECT_FLOAT_EQ(1.57079632679F, px4_ros2::yawEnuToNed(0.0F));
  EXPECT_FLOAT_EQ(-0.7F, px4_ros2::yawRateEnuToNed(0.7F));
}

TEST(PositionCommandContract, ConvertsPvaWithTheSharedEnuNedMatrix) {
  const Eigen::Vector3d enu_position{1.0, 2.0, 3.0};
  const Eigen::Vector3d enu_velocity{-4.0, 5.0, -6.0};
  EXPECT_TRUE(coordinate_conventions::enuToNed(enu_position).isApprox(
      Eigen::Vector3d{2.0, 1.0, -3.0}));
  EXPECT_TRUE(coordinate_conventions::enuToNed(enu_velocity).isApprox(
      Eigen::Vector3d{5.0, -4.0, 6.0}));
  EXPECT_TRUE((coordinate_conventions::c_enu_ned() *
               coordinate_conventions::c_enu_ned()).isApprox(Eigen::Matrix3d::Identity()));
}

TEST(PositionCommandContract, RejectProvenanceIsExactWithoutPreviousCommand) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = 1;
  odometry.header.stamp.nanosec = 100'000'000U;
  odometry.pose.pose.position.x = 1.0;
  odometry.twist.twist.linear.y = 2.0;
  mars_quadrotor_msgs::msg::PositionCommand command;
  const auto result = px4_navigation_external_mode::buildRejectProvenance(
      1'300'000'000LL, 1'250'000'000LL, odometry, command, std::nullopt);
  EXPECT_DOUBLE_EQ(result.odometry_header_age_ms, 200.0);
  EXPECT_DOUBLE_EQ(result.odometry_receive_age_ms, 50.0);
  EXPECT_FALSE(result.previous_valid);
  EXPECT_TRUE(result.measured_position.isApprox(Eigen::Vector3d{1.0, 0.0, 0.0}));
  EXPECT_TRUE(result.measured_velocity.isApprox(Eigen::Vector3d{0.0, 2.0, 0.0}));
  EXPECT_TRUE(std::isnan(result.command_delta_position_m));
}

TEST(PositionCommandContract, RejectProvenancePreservesPreviousPvajAndSignedDelta) {
  nav_msgs::msg::Odometry odometry;
  mars_quadrotor_msgs::msg::PositionCommand previous;
  previous.trajectory_generation = 9U;
  previous.trajectory_time_s = 1.5;
  previous.position.x = 1.0;
  previous.velocity.y = 2.0;
  previous.acceleration.z = 3.0;
  previous.jerk.x = 4.0;
  mars_quadrotor_msgs::msg::PositionCommand command;
  command.trajectory_generation = 7U;
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

  command.trajectory_generation = 9U;
  result = px4_navigation_external_mode::buildRejectProvenance(
      0, 0, odometry, command, previous);
  EXPECT_FALSE(result.generation_changed);
  EXPECT_EQ(result.generation_delta, 0);
}

#include <gtest/gtest.h>

#include <coordinate_conventions/frame_conventions.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

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

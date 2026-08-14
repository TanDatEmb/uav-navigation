#include <cmath>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/trajectory_contract.hpp"

namespace {

navigation_interfaces::msg::PlannedTrajectory validTrajectory() {
  navigation_interfaces::msg::PlannedTrajectory message;
  message.header.frame_id = "lio_odom";
  message.success = true;
  message.duration_s = 2.0;
  message.time_from_start = {0.0, 2.0};
  message.position.resize(2);
  message.velocity.resize(2);
  message.acceleration.resize(2);
  message.position[1].x = 2.0;
  message.velocity[1].x = 1.0;
  return message;
}

}  // namespace

TEST(TrajectoryContract, ValidatesAndSamplesInEnu) {
  const auto message = validTrajectory();
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").valid());
  const auto sample = px4_navigation_external_mode::sampleTrajectory(message, 1.0);
  EXPECT_DOUBLE_EQ(sample.position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(sample.velocity_enu.x(), 0.5);
  EXPECT_DOUBLE_EQ(sample.acceleration_enu.x(), 0.0);
}

TEST(TrajectoryContract, ConvertsEnuToNedWithoutYawGuess) {
  const Eigen::Vector3f result = px4_navigation_external_mode::enuToNed({1.0, 2.0, 3.0});
  EXPECT_FLOAT_EQ(result.x(), 2.0F);
  EXPECT_FLOAT_EQ(result.y(), 1.0F);
  EXPECT_FLOAT_EQ(result.z(), -3.0F);
}

TEST(TrajectoryContract, RejectsInvalidProvenanceAndShape) {
  auto message = validTrajectory();
  message.header.frame_id = "px4_odom";
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::WrongFrame);

  message = validTrajectory();
  message.time_from_start[1] = 0.0;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::NonMonotonicTime);

  message = validTrajectory();
  message.acceleration.pop_back();
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::SizeMismatch);
}

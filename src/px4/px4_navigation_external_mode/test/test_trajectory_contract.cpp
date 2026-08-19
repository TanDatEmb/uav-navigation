#include <cmath>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/trajectory_contract.hpp"
#include "px4_navigation_external_mode/velocity_tracker.hpp"

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

TEST(TrajectoryContract, AcceptsSafetyRoleAndRejectsUnknownRole) {
  auto message = validTrajectory();
  message.trajectory_role = navigation_interfaces::msg::PlannedTrajectory::ROLE_SAFETY;
  message.safety_plan_kind =
      navigation_interfaces::msg::PlannedTrajectory::SAFETY_KIND_BRAKING_STOP;
  message.velocity[1].x = 0.0;
  message.velocity[1].y = 0.0;
  message.velocity[1].z = 0.0;
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").valid());

  message.trajectory_role = 99U;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidRole);
}

TEST(TrajectoryContract, RejectsSafetyTrajectoryThatDoesNotStop) {
  auto message = validTrajectory();
  message.trajectory_role = navigation_interfaces::msg::PlannedTrajectory::ROLE_SAFETY;
  message.safety_plan_kind =
      navigation_interfaces::msg::PlannedTrajectory::SAFETY_KIND_BRAKING_STOP;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidSafetyTerminalState);

  message = validTrajectory();
  message.trajectory_role = navigation_interfaces::msg::PlannedTrajectory::ROLE_SAFETY;
  message.safety_plan_kind =
      navigation_interfaces::msg::PlannedTrajectory::SAFETY_KIND_BRAKING_STOP;
  message.velocity[1].x = 0.0;
  message.velocity[1].y = 0.0;
  message.velocity[1].z = 0.0;
  message.acceleration[1].z = 0.01;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidSafetyTerminalState);
}

TEST(TrajectoryContract, SafetyRouteMayCarryContinuationVelocity) {
  auto message = validTrajectory();
  message.trajectory_role = navigation_interfaces::msg::PlannedTrajectory::ROLE_SAFETY;
  message.safety_plan_kind = navigation_interfaces::msg::PlannedTrajectory::SAFETY_KIND_ROUTE;
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").valid());
}

TEST(TrajectoryContract, AcceptsOnlyCurrentGoalCorrelation) {
  auto message = validTrajectory();
  message.mission_id = "route";
  message.waypoint_index = 2U;
  message.request_id = 7U;
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryMatchesGoal(message, "route", 2U, 7U));
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryMatchesGoal(message, "route", 1U, 7U));
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryMatchesGoal(message, "route", 2U, 6U));
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryMatchesGoal(message, "older", 2U, 7U));
}

TEST(TrajectoryContract, RejectsOlderWorldRevisionButAcceptsCurrentOrNewer) {
  auto message = validTrajectory();
  message.world_generation = 4U;
  message.world_revision = 10U;
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, false, 99U, 99U));
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, true, 3U, 99U));
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, true, 4U, 10U));
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, true, 4U, 9U));
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, true, 4U, 11U));
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryRevisionIsNotOlder(message, true, 5U, 0U));
}

TEST(VelocityTracker, UsesPositionFeedbackAndLimitsAcceleration) {
  px4_navigation_external_mode::VelocityTrackerConfig config;
  config.position_gain = 1.0;
  config.max_velocity_mps = 1.0;
  config.max_acceleration_mps2 = 2.0;
  config.max_position_error_m = 10.0;
  px4_navigation_external_mode::VelocityTracker tracker(config);
  px4_navigation_external_mode::TrajectorySample reference;
  reference.position_enu.x() = 4.0;

  const auto first = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_DOUBLE_EQ(first.x(), 0.2);

  const auto second = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_DOUBLE_EQ(second.x(), 0.4);
  EXPECT_LE(second.norm(), 1.0);
}

TEST(VelocityTracker, ResetRemovesPreviousCommandMemory) {
  px4_navigation_external_mode::VelocityTracker tracker;
  px4_navigation_external_mode::TrajectorySample reference;
  reference.velocity_enu.x() = 1.0;
  EXPECT_GT(tracker.update(reference, Eigen::Vector3d::Zero(), 0.1).x(), 0.0);
  tracker.reset();
  EXPECT_DOUBLE_EQ(tracker.update({}, Eigen::Vector3d::Zero(), 0.1).norm(), 0.0);
}

TEST(VelocityTracker, UsesSeparateDecelerationLimit) {
  px4_navigation_external_mode::VelocityTrackerConfig config;
  config.max_velocity_mps = 2.0;
  config.max_acceleration_mps2 = 10.0;
  config.max_deceleration_mps2 = 0.5;
  config.position_gain = 0.0;
  px4_navigation_external_mode::VelocityTracker tracker(config);
  px4_navigation_external_mode::TrajectorySample reference;
  reference.velocity_enu.x() = 2.0;

  const auto accelerating = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_DOUBLE_EQ(accelerating.x(), 1.0);

  reference.velocity_enu.setZero();
  const auto braking = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_DOUBLE_EQ(braking.x(), 0.95);
}

TEST(VelocityTracker, PreviewUsesVelocityOnlyAndCurrentPositionFeedback) {
  px4_navigation_external_mode::VelocityTracker tracker;
  px4_navigation_external_mode::TrajectorySample current;
  current.position_enu = Eigen::Vector3d{0.0, 0.0, 0.0};
  current.velocity_enu = Eigen::Vector3d::Zero();
  px4_navigation_external_mode::TrajectorySample preview = current;
  preview.position_enu = Eigen::Vector3d{10.0, 0.0, 0.0};
  preview.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  const auto command = tracker.update(current, preview, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_GT(command.x(), 0.0);
  // A large preview position must not be interpreted as a ten-metre position
  // error; the command remains bounded by the configured velocity envelope.
  EXPECT_LE(command.norm(), 2.0 + 1e-9);
}

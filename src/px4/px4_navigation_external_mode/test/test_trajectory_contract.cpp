#include <cmath>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/trajectory_contract.hpp"
#include "px4_navigation_external_mode/velocity_tracker.hpp"

namespace {

navigation_interfaces::msg::PlannedTrajectory validTrajectory() {
  navigation_interfaces::msg::PlannedTrajectory message;
  message.header.frame_id = "lio_odom";
  message.trajectory_id = 1U;
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

navigation_interfaces::msg::TrajectoryCandidate candidateFrom(
    const navigation_interfaces::msg::PlannedTrajectory& source, std::uint8_t role,
    std::uint8_t safety_kind) {
  navigation_interfaces::msg::TrajectoryCandidate candidate;
  candidate.header = source.header;
  candidate.trajectory_id = source.trajectory_id;
  candidate.success = source.success;
  candidate.duration_s = source.duration_s;
  candidate.time_from_start = source.time_from_start;
  candidate.position = source.position;
  candidate.velocity = source.velocity;
  candidate.acceleration = source.acceleration;
  candidate.trajectory_role = role;
  candidate.safety_plan_kind = safety_kind;
  candidate.known_free_only = role == navigation_interfaces::msg::TrajectoryCandidate::ROLE_SAFETY;
  return candidate;
}

navigation_interfaces::msg::TrajectorySegment segment(double start_x) {
  navigation_interfaces::msg::TrajectorySegment result;
  result.duration_s = 1.0;
  result.time_from_start = {0.0, 1.0};
  result.position.resize(2);
  result.velocity.resize(2);
  result.acceleration.resize(2);
  result.position[0].x = start_x;
  result.position[1].x = start_x + 1.0;
  result.velocity[0].x = 1.0;
  result.velocity[1].x = 1.0;
  return result;
}

navigation_interfaces::msg::TrajectoryBundle validBranchableBundle() {
  navigation_interfaces::msg::TrajectoryBundle bundle;
  bundle.header.frame_id = "lio_odom";
  bundle.bundle_id = 9U;
  bundle.mission_id = "route";
  bundle.common_prefix = segment(0.0);
  bundle.nominal_valid = true;
  bundle.nominal_suffix = segment(1.0);
  bundle.safety_suffix = segment(1.0);
  bundle.safety_kind = navigation_interfaces::msg::TrajectoryBundle::SAFETY_ROUTE;
  bundle.selected_branch = navigation_interfaces::msg::TrajectoryBundle::BRANCH_NOMINAL;
  bundle.requested_branch = navigation_interfaces::msg::TrajectoryBundle::BRANCH_NOMINAL;
  return bundle;
}

}  // namespace

TEST(TrajectoryContract, ValidatesAndSamplesInEnu) {
  const auto message = validTrajectory();
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").valid());
  const auto sample = px4_navigation_external_mode::sampleTrajectory(message, 1.0);
  EXPECT_DOUBLE_EQ(sample.position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(sample.velocity_enu.x(), 0.5);
  EXPECT_DOUBLE_EQ(sample.acceleration_enu.x(), 0.0);
  const auto start = px4_navigation_external_mode::sampleTrajectory(message, 0.0);
  const auto finish = px4_navigation_external_mode::sampleTrajectory(message, 2.0);
  EXPECT_NEAR(start.position_enu.x(), 0.0, 1e-12);
  EXPECT_NEAR(start.velocity_enu.x(), 0.0, 1e-12);
  EXPECT_NEAR(finish.position_enu.x(), 2.0, 1e-12);
  EXPECT_NEAR(finish.velocity_enu.x(), 1.0, 1e-12);
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

TEST(TrajectoryContract, BundleRequiresKnownFreeSafetyBackupForNominal) {
  const auto source = validTrajectory();
  navigation_interfaces::msg::PlannedTrajectoryBundle bundle;
  bundle.bundle_id = 1U;
  bundle.header.frame_id = "lio_odom";
  bundle.mission_id = "route";
  bundle.nominal_available = true;
  bundle.safety_available = true;
  bundle.selected_candidate =
      navigation_interfaces::msg::PlannedTrajectoryBundle::SELECTED_NOMINAL;
  bundle.nominal = candidateFrom(
      source, navigation_interfaces::msg::TrajectoryCandidate::ROLE_NOMINAL,
      navigation_interfaces::msg::TrajectoryCandidate::SAFETY_KIND_NONE);
  bundle.safety = candidateFrom(
      source, navigation_interfaces::msg::TrajectoryCandidate::ROLE_SAFETY,
      navigation_interfaces::msg::TrajectoryCandidate::SAFETY_KIND_ROUTE);
  bundle.nominal.mission_id = bundle.safety.mission_id = bundle.mission_id;
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").valid());

  bundle.safety_available = false;
  EXPECT_FALSE(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").valid());
}

TEST(TrajectoryContract, BundleConvertsSelectedCandidateWithoutLosingPva) {
  const auto source = validTrajectory();
  const auto candidate = candidateFrom(
      source, navigation_interfaces::msg::TrajectoryCandidate::ROLE_SAFETY,
      navigation_interfaces::msg::TrajectoryCandidate::SAFETY_KIND_ROUTE);
  const auto converted = px4_navigation_external_mode::candidateToPlannedTrajectory(candidate);
  EXPECT_EQ(converted.trajectory_id, source.trajectory_id);
  EXPECT_EQ(converted.position.size(), source.position.size());
  EXPECT_DOUBLE_EQ(converted.position.back().x, source.position.back().x);
  EXPECT_DOUBLE_EQ(converted.velocity.back().x, source.velocity.back().x);
}

TEST(TrajectoryContract, ValidatesBranchableBundleAndSplice) {
  const auto bundle = validBranchableBundle();
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").valid());
}

TEST(TrajectoryContract, BranchableBundleRequiresSafetySuffix) {
  auto bundle = validBranchableBundle();
  bundle.safety_suffix.time_from_start.clear();
  bundle.safety_suffix.position.clear();
  bundle.safety_suffix.velocity.clear();
  bundle.safety_suffix.acceleration.clear();
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::Empty);
}

TEST(TrajectoryContract, BranchableBundleRejectsSpliceJumpAndInvalidStop) {
  auto bundle = validBranchableBundle();
  bundle.nominal_suffix.position.front().x += 0.2;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidRole);

  bundle = validBranchableBundle();
  bundle.safety_kind = navigation_interfaces::msg::TrajectoryBundle::SAFETY_STOP;
  bundle.safety_suffix.velocity.back().x = 1.0;
  bundle.selected_branch = navigation_interfaces::msg::TrajectoryBundle::BRANCH_SAFETY;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectoryBundle(bundle, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidSafetyTerminalState);
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

TEST(TrajectoryContract, CarriesFutureSwitchMetadata) {
  auto message = validTrajectory();
  message.valid_from.sec = 10;
  message.commitment_horizon_s = 0.12;
  EXPECT_TRUE(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").valid());
  EXPECT_FALSE(px4_navigation_external_mode::trajectoryValidFromIsNotOlder(message, 9LL * 1000000000LL));
  EXPECT_TRUE(px4_navigation_external_mode::trajectoryValidFromIsNotOlder(message, 10LL * 1000000000LL));

  message.trajectory_id = 0U;
  EXPECT_EQ(px4_navigation_external_mode::validateTrajectory(message, "lio_odom").failure,
            px4_navigation_external_mode::TrajectoryInputFailure::InvalidTrajectoryId);
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

TEST(VelocityTracker, ForwardGuardPreventsReverseCommandOnStraightCorridor) {
  px4_navigation_external_mode::VelocityTrackerConfig config;
  config.position_gain = 1.0;
  config.max_position_error_m = 10.0;
  px4_navigation_external_mode::VelocityTracker tracker(config);
  px4_navigation_external_mode::TrajectorySample current;
  current.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  px4_navigation_external_mode::TrajectorySample preview = current;
  const auto command = tracker.update(current, preview, Eigen::Vector3d{2.0, 0.0, 0.0}, 0.1);
  EXPECT_GE(command.dot(Eigen::Vector3d::UnitX()), 0.0);
  EXPECT_GT(tracker.forwardGuardCount(), 0U);
}

TEST(VelocityTracker, ForwardGuardDoesNotBlockAChangingCornerTangent) {
  px4_navigation_external_mode::VelocityTrackerConfig config;
  config.position_gain = 1.0;
  config.max_position_error_m = 10.0;
  px4_navigation_external_mode::VelocityTracker tracker(config);
  px4_navigation_external_mode::TrajectorySample current;
  current.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  px4_navigation_external_mode::TrajectorySample preview = current;
  preview.velocity_enu = Eigen::Vector3d{0.0, 1.0, 0.0};
  const auto command = tracker.update(current, preview, Eigen::Vector3d{0.0, -1.0, 0.0}, 0.1);
  EXPECT_GT(command.y(), 0.0);
  EXPECT_EQ(tracker.forwardGuardCount(), 0U);
}

TEST(VelocityTracker, FiniteJerkLimitConstrainsAccelerationState) {
  px4_navigation_external_mode::VelocityTrackerConfig config;
  config.max_velocity_mps = 10.0;
  config.max_acceleration_mps2 = 10.0;
  config.max_deceleration_mps2 = 10.0;
  config.max_jerk_mps3 = 2.0;
  config.max_position_error_m = 10.0;
  px4_navigation_external_mode::VelocityTracker tracker(config);
  px4_navigation_external_mode::TrajectorySample reference;
  reference.velocity_enu.x() = 5.0;

  const auto first = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  const auto second = tracker.update(reference, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_NEAR(first.x(), 0.02, 1e-9);
  EXPECT_NEAR(second.x(), 0.06, 1e-9);
}

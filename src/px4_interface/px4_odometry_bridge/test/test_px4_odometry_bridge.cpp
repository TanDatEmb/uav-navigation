#include <gtest/gtest.h>

#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "px4_odometry_bridge/frame_converter.hpp"
#include "px4_odometry_bridge/odometry_ring_buffer.hpp"
#include "px4_odometry_bridge/reset_compensator.hpp"
#include "px4_odometry_bridge/time_validator.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace {
struct V0 { static constexpr std::uint32_t MESSAGE_VERSION = 0; };
struct V2 { static constexpr std::uint32_t MESSAGE_VERSION = 2; };

px4_odometry_bridge::ConvertedOdometry sample(std::int64_t time) {
  px4_odometry_bridge::ConvertedOdometry value;
  value.timestamp_ns = time;
  value.position.x() = static_cast<double>(time) * 1e-9;
  value.velocity_world.x() = 1.0;
  value.velocity_body.x() = 1.0;
  value.angular_velocity_body.z() = 0.1;
  value.position_variance = Eigen::Vector3d::Ones();
  value.velocity_variance = Eigen::Vector3d::Ones();
  value.orientation_variance = Eigen::Vector3d::Ones();
  return value;
}
}  // namespace

TEST(Px4TopicVersion, KeepsZeroUnversionedAndSuffixesPositiveVersions) {
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<V0>("/fmu/out/vehicle_odometry"),
            "/fmu/out/vehicle_odometry");
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<V2>("/fmu/out/vehicle_local_position"),
            "/fmu/out/vehicle_local_position_v2");
}

TEST(Px4TopicVersion, UsesRealV117GeneratedMessageDefinitions) {
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            "/fmu/out/vehicle_odometry");
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position"),
            "/fmu/out/vehicle_local_position_v1");
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<px4_msgs::msg::VehicleAttitude>(
                "/fmu/out/vehicle_attitude"),
            "/fmu/out/vehicle_attitude");
  EXPECT_EQ(px4_odometry_bridge::versioned_topic<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            "/fmu/out/timesync_status");
}

TEST(Px4FrameConverter, ConvertsNedPositionAndBodyVelocity) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kBodyFrd;
  input.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  input.velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  input.angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.1);
  auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output.value->position.isApprox(Eigen::Vector3d(2.0, 1.0, -3.0)));
  EXPECT_TRUE(output.value->velocity_body.isApprox(Eigen::Vector3d(4.0, -5.0, -6.0)));
}

TEST(Px4FrameConverter, ConvertsNedPoseAndWorldVelocityWithPublishedBasis) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  input.velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  input.angular_velocity = Eigen::Vector3d::Zero();
  input.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));

  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_EQ(output.value->world_convention, px4_odometry_bridge::WorldConvention::kRosEnu);
  EXPECT_TRUE(output.value->position.isApprox(
      px4_odometry_bridge::FrameConverter::c_enu_ned() * input.position));
  EXPECT_TRUE(output.value->velocity_world.isApprox(
      px4_odometry_bridge::FrameConverter::c_enu_ned() * input.velocity));
  const Eigen::Matrix3d expected_orientation =
      px4_odometry_bridge::FrameConverter::c_enu_ned() *
      input.orientation.toRotationMatrix() *
      px4_odometry_bridge::FrameConverter::c_flu_frd();
  EXPECT_TRUE(output.value->orientation.toRotationMatrix().isApprox(expected_orientation));
}

TEST(Px4FrameConverter, KeepsFrdWorldLocalAndDoesNotClaimEnu) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kFrd;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kFrd;
  input.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  input.velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  input.angular_velocity = Eigen::Vector3d::Zero();

  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_EQ(output.value->world_convention,
            px4_odometry_bridge::WorldConvention::kPx4FrdLocal);
  EXPECT_TRUE(output.value->position.isApprox(Eigen::Vector3d(1.0, -2.0, -3.0)));
}

TEST(Px4FrameConverter, RejectsWorldVelocityFromDifferentWorldFrame) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kFrd;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.angular_velocity = Eigen::Vector3d::Zero();
  EXPECT_FALSE(px4_odometry_bridge::FrameConverter{}.convert(input));
}

TEST(Px4FrameConverter, QuaternionSignProducesTheSameOrientation) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.angular_velocity = Eigen::Vector3d::Zero();
  input.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  auto positive = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(positive);
  input.orientation.coeffs() *= -1.0;
  auto negative = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(negative);
  EXPECT_NEAR(positive.value->orientation.angularDistance(negative.value->orientation), 0.0,
              1e-12);
}

TEST(Px4FrameConverter, AcceptsZeroStationaryAngularVelocity) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.angular_velocity = Eigen::Vector3d::Zero();
  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output.value->angular_velocity_body.isZero());
}

TEST(Px4FrameConverter, RejectsNonfiniteAngularVelocity) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.angular_velocity.x() = std::numeric_limits<double>::quiet_NaN();
  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  EXPECT_FALSE(output);
}

TEST(Px4FrameConverter, UsesBodyFrdBasisForBodyVelocityCovariance) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kBodyFrd;
  input.velocity_variance = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output.value->velocity_variance.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(Px4FrameConverter, UsesNedBasisForPositionCovariance) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kNed;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kNed;
  input.position_variance = Eigen::Vector3d(1.0, 2.0, 3.0);
  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output.value->position_variance.isApprox(Eigen::Vector3d(2.0, 1.0, 3.0)));
  EXPECT_TRUE(output.value->position_covariance_available);
}

TEST(Px4FrameConverter, KeepsUnavailableAngularCovarianceExplicit) {
  px4_odometry_bridge::Px4OdometrySample input;
  input.timestamp_ns = 1'000'000;
  input.pose_frame = px4_odometry_bridge::PoseFrame::kFrd;
  input.velocity_frame = px4_odometry_bridge::VelocityFrame::kFrd;
  input.position_variance = Eigen::Vector3d::Ones();
  input.velocity_variance = Eigen::Vector3d::Ones();
  input.orientation_variance = Eigen::Vector3d::Ones();
  const auto output = px4_odometry_bridge::FrameConverter{}.convert(input);
  ASSERT_TRUE(output);
  EXPECT_TRUE(output.value->angular_velocity_valid);
  EXPECT_TRUE(output.value->orientation_covariance_available);
}

TEST(Px4TimeValidator, RejectsZeroDuplicateOverflowAndFuture) {
  EXPECT_FALSE(px4_odometry_bridge::checked_microseconds_to_nanoseconds(0).has_value());
  EXPECT_FALSE(px4_odometry_bridge::checked_microseconds_to_nanoseconds(
                   std::numeric_limits<std::uint64_t>::max()).has_value());
  px4_odometry_bridge::TimestampValidator validator({.max_stale_ns = 200'000'000,
                                                      .max_future_ns = 100});
  EXPECT_TRUE(validator.observe(1'000, 1'000'000).accepted);
  EXPECT_FALSE(validator.observe(1'000, 1'000'000).accepted);
  EXPECT_FALSE(validator.observe(2'000, 1'000'000).accepted);
}

TEST(Px4TimeValidator, RejectsStaleAndAcceptsPausedClockWithoutWallTimeDecay) {
  px4_odometry_bridge::TimestampValidator validator({.max_stale_ns = 100,
                                                      .max_future_ns = 100});
  EXPECT_FALSE(validator.observe(1, 1'000'000).accepted);
  px4_odometry_bridge::TimestampValidator paused({.max_stale_ns = 100,
                                                   .max_future_ns = 100});
  EXPECT_TRUE(paused.observe(1'000, 1'000'000).accepted);
  EXPECT_EQ(paused.observe(1'001, 1'000'000).reason,
            "future PX4 sample timestamp");
}

TEST(Px4TimeValidator, ClassifiesLargeLowEpochRegressionAsSourceRestart) {
  px4_odometry_bridge::TimestampValidator validator({
      .max_stale_ns = 200'000'000,
      .max_future_ns = 200'000'000,
      .probable_restart_regression_ns = 1'000'000'000,
      .restart_low_epoch_max_ns = 10'000'000'000});
  ASSERT_TRUE(validator.observe(20'000'000, 20'000'000'000).accepted);
  const auto restart = validator.observe(1'000'000, 1'000'000'000);
  EXPECT_TRUE(restart.accepted);
  EXPECT_EQ(restart.event, px4_odometry_bridge::TimestampEvent::kProbableSourceRestart);
  EXPECT_EQ(restart.generation, 1U);
  EXPECT_TRUE(validator.observe(1'001'000, 1'001'000'000).accepted);
}

TEST(Px4TimeValidator, DoesNotTreatSmallRegressionAsRestart) {
  px4_odometry_bridge::TimestampValidator validator;
  ASSERT_TRUE(validator.observe(20'000'000, 20'000'000'000).accepted);
  const auto result = validator.observe(19'999'000, 19'999'000'000);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.event, px4_odometry_bridge::TimestampEvent::kSmallRegression);
  EXPECT_EQ(result.generation, 0U);
}

TEST(Px4TimeValidator, UsesCheckedNanosecondServiceTime) {
  EXPECT_EQ(px4_odometry_bridge::checked_ros_time_to_nanoseconds(2, 345),
            std::optional<std::int64_t>(2'000'000'345));
  EXPECT_FALSE(px4_odometry_bridge::checked_ros_time_to_nanoseconds(-1, 0).has_value());
  EXPECT_FALSE(px4_odometry_bridge::checked_ros_time_to_nanoseconds(1, 1'000'000'000U).has_value());
}

TEST(Px4RingBuffer, InterpolatesWithoutExtrapolationOrGenerationCrossing) {
  px4_odometry_bridge::OdometryRingBuffer buffer({
      .duration_ns = 2'000'000'000, .capacity = 512, .max_gap_ns = 50'000'000, .stable_samples = 1});
  ASSERT_TRUE(buffer.push(sample(1'000'000'000)));
  ASSERT_TRUE(buffer.push(sample(1'020'000'000)));
  auto result = buffer.sample(1'010'000'000);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->interpolated);
  EXPECT_DOUBLE_EQ(result->value.position.x(), 1.01);
  EXPECT_FALSE(buffer.sample(999'000'000).has_value());
}

TEST(Px4ResetCompensator, SuppressesTransitionAndPreservesPoseContinuity) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  first.reset_counter = 0;
  ASSERT_TRUE(compensator.observe(first).has_value());
  auto reset = sample(1'010'000'000);
  reset.reset_counter = 1;
  reset.position.x() = 10.0;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.position_xy_reset = true;
  metadata.position_delta_source = Eigen::Vector3d(0.0, 9.0, 0.0);
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());
  auto after = sample(1'020'000'000);
  after.reset_counter = 1;
  after.position.x() = 10.01;
  auto output = compensator.observe(after, metadata);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->reset_generation, 1U);
  EXPECT_DOUBLE_EQ(output->position.x(), 1.01);
}

TEST(Px4ResetCompensator, RejectsCounterJumpAndMissingMetadata) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  ASSERT_TRUE(compensator.observe(first).has_value());
  auto jump = sample(1'010'000'000);
  jump.reset_counter = 2;
  const auto counter_jump = compensator.observe(jump);
  EXPECT_FALSE(counter_jump.has_value());
  EXPECT_EQ(counter_jump.status,
            px4_odometry_bridge::ResetObservationStatus::kCounterDiscontinuity);
  auto missing = sample(1'020'000'000);
  missing.reset_counter = 1;
  const auto metadata_pending = compensator.observe(missing);
  EXPECT_FALSE(metadata_pending.has_value());
  EXPECT_EQ(metadata_pending.status,
            px4_odometry_bridge::ResetObservationStatus::kMetadataPending);
}

TEST(Px4ResetCompensator, InvalidMetadataCannotBeUsedForStartupRebaseline) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  ASSERT_TRUE(compensator.observe(first).has_value());

  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.association_invalid = true;
  const auto observation = compensator.observe(reset, metadata);
  EXPECT_FALSE(observation.has_value());
  EXPECT_EQ(observation.status,
            px4_odometry_bridge::ResetObservationStatus::kInvalidMetadata);
  EXPECT_EQ(compensator.reset_generation(), 0U);
}

TEST(Px4ResetCompensator, TypedResetTransitionPreservesGenerationAndSuppressesOnlyTransition) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  ASSERT_TRUE(compensator.observe(first).has_value());

  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.position_z_reset = true;
  metadata.position_delta_source = Eigen::Vector3d(0.0, 0.0, 2.0);
  const auto transition = compensator.observe(reset, metadata);
  EXPECT_FALSE(transition.has_value());
  EXPECT_EQ(transition.status,
            px4_odometry_bridge::ResetObservationStatus::kResetTransitionSuppressed);
  EXPECT_EQ(transition.reset_generation, 1U);

  reset.timestamp_ns += 10'000'000;
  const auto stable = compensator.observe(reset, metadata);
  ASSERT_TRUE(stable.has_value());
  EXPECT_EQ(stable.status, px4_odometry_bridge::ResetObservationStatus::kAccepted);
  EXPECT_EQ(stable->reset_generation, 1U);
}

TEST(Px4ResetCompensator, PreservesVelocityDuringMovingVelocityReset) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  first.velocity_world = Eigen::Vector3d(2.0, 0.0, 0.0);
  first.velocity_body = first.velocity_world;
  ASSERT_TRUE(compensator.observe(first).has_value());
  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  reset.velocity_world.x() = 5.0;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.velocity_xy_reset = true;
  metadata.velocity_delta_source = Eigen::Vector3d(0.0, 3.0, 0.0);
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());
  auto after = reset;
  after.timestamp_ns = 1'020'000'000;
  after.velocity_world.x() = 5.1;
  after.velocity_body = after.velocity_world;
  const auto output = compensator.observe(after, metadata);
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->velocity_world.x(), 2.1, 1e-12);
}

TEST(Px4ResetCompensator, StartupOriginRebaseKeepsTakeoffDeltaAtZero) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  first.position = Eigen::Vector3d(1.0, 2.0, 10.0);
  ASSERT_TRUE(compensator.observe(first).has_value());
  const auto origin = compensator.rebasePositionAtCurrentOutput();
  ASSERT_TRUE(origin.has_value());
  EXPECT_TRUE(origin->isApprox(Eigen::Vector3d(1.0, 2.0, 10.0)));

  auto at_takeoff_height = first;
  at_takeoff_height.timestamp_ns = 1'010'000'000;
  at_takeoff_height.position = Eigen::Vector3d(1.0, 2.0, 13.0);
  const auto output = compensator.observe(at_takeoff_height);
  ASSERT_TRUE(output.has_value());
  EXPECT_TRUE(output->position.isApprox(Eigen::Vector3d(0.0, 0.0, 3.0)));
}

TEST(Px4ResetCompensator, StartupOriginRebaseAlsoRemovesBootstrapResetOffset) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto before_reset = sample(1'000'000'000);
  before_reset.position = Eigen::Vector3d(0.0, 0.0, 10.0);
  ASSERT_TRUE(compensator.observe(before_reset).has_value());

  auto reset = before_reset;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  reset.position = Eigen::Vector3d::Zero();
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.position_z_reset = true;
  // The converted ENU reset delta is -10 m when the source NED delta is +10 m.
  metadata.position_delta_source = Eigen::Vector3d(0.0, 0.0, 10.0);
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());

  auto after_reset = reset;
  after_reset.timestamp_ns = 1'020'000'000;
  auto continuous = compensator.observe(after_reset, metadata);
  ASSERT_TRUE(continuous.has_value());
  const auto origin = compensator.rebasePositionAtCurrentOutput();
  ASSERT_TRUE(origin.has_value());
  continuous->position -= *origin;
  EXPECT_TRUE(continuous->position.isApprox(Eigen::Vector3d::Zero()));

  auto at_takeoff_height = after_reset;
  at_takeoff_height.timestamp_ns = 1'030'000'000;
  at_takeoff_height.position.z() = 3.0;
  const auto output = compensator.observe(at_takeoff_height);
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->position.z(), 3.0, 1e-12);
}

TEST(Px4ResetCompensator, ClearAllowsStartupBaselineWithArbitraryResetCounter) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto before_restart = sample(1'000'000'000);
  before_restart.reset_counter = 3;
  before_restart.position = Eigen::Vector3d(4.0, 5.0, 6.0);
  ASSERT_TRUE(compensator.observe(before_restart).has_value());

  compensator.clear();
  auto after_restart = before_restart;
  after_restart.timestamp_ns = 1'010'000'000;
  after_restart.reset_counter = 11;
  after_restart.position = Eigen::Vector3d::Zero();
  const auto output = compensator.observe(after_restart);
  ASSERT_TRUE(output.has_value());
  EXPECT_TRUE(output->position.isApprox(Eigen::Vector3d::Zero()));
}

TEST(Px4ResetCompensator, AppliesResetRotationToWorldPoseAndIgnoresStaleDeltas) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  first.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  first.orientation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(compensator.observe(first).has_value());

  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  reset.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.heading_reset = true;
  metadata.heading_delta_rad = 0.4;
  const Eigen::Matrix3d source_reset =
      px4_odometry_bridge::FrameConverter::c_enu_ned() *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      px4_odometry_bridge::FrameConverter::c_enu_ned().transpose();
  reset.orientation = Eigen::Quaterniond(source_reset);
  // These values belong to an earlier position reset and must be ignored.
  metadata.position_delta_source = Eigen::Vector3d(0.0, 0.0, 10.0);
  metadata.velocity_delta_source = Eigen::Vector3d(0.0, 0.0, 5.0);
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());

  auto after = reset;
  after.timestamp_ns = 1'020'000'000;
  const auto output = compensator.observe(after, metadata);
  ASSERT_TRUE(output.has_value());
  const Eigen::Matrix3d expected_continuity_rotation = source_reset.transpose();
  EXPECT_TRUE(output->position.isApprox(expected_continuity_rotation * first.position));
  EXPECT_NEAR(output->orientation.angularDistance(first.orientation), 0.0, 1e-12);
}

TEST(Px4ResetCompensator, SupportsCounterWrapAndQuaternionReset) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  first.reset_counter = 255;
  first.orientation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(compensator.observe(first).has_value());

  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 0;
  const Eigen::Quaterniond source_delta(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond raw_delta(
      px4_odometry_bridge::FrameConverter::c_enu_ned() *
      source_delta.toRotationMatrix() *
      px4_odometry_bridge::FrameConverter::c_enu_ned().transpose());
  reset.orientation = raw_delta;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.attitude_reset = true;
  metadata.attitude_delta = source_delta;
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());

  auto after = reset;
  after.timestamp_ns = 1'020'000'000;
  const auto output = compensator.observe(after, metadata);
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->reset_generation, 1U);
  EXPECT_NEAR(output->orientation.angularDistance(Eigen::Quaterniond::Identity()), 0.0,
              1e-12);
}

TEST(Px4ResetCompensator, CombinedResetRequiresDetailedMetadata) {
  px4_odometry_bridge::ResetCompensator compensator;
  auto first = sample(1'000'000'000);
  ASSERT_TRUE(compensator.observe(first).has_value());
  auto reset = first;
  reset.timestamp_ns = 1'010'000'000;
  reset.reset_counter = 1;
  px4_odometry_bridge::DetailedResetMetadata metadata;
  metadata.available = true;
  metadata.timestamp_ns = reset.timestamp_ns;
  metadata.position_xy_reset = true;
  metadata.velocity_xy_reset = true;
  metadata.heading_reset = true;
  metadata.position_delta_source = Eigen::Vector3d(0.0, 1.0, 0.0);
  metadata.velocity_delta_source = Eigen::Vector3d(0.0, 1.0, 0.0);
  metadata.heading_delta_rad = 0.1;
  EXPECT_FALSE(compensator.observe(reset, metadata).has_value());
}

TEST(Px4RingBuffer, RequiresStableSamplesAfterGenerationChange) {
  px4_odometry_bridge::OdometryRingBuffer buffer({
      .duration_ns = 2'000'000'000, .capacity = 512, .max_gap_ns = 50'000'000, .stable_samples = 3});
  auto first = sample(1'000'000'000);
  first.reset_generation = 2;
  EXPECT_FALSE(buffer.push(first));
  first.timestamp_ns += 10'000'000;
  EXPECT_FALSE(buffer.push(first));
  first.timestamp_ns += 10'000'000;
  EXPECT_TRUE(buffer.push(first));
  EXPECT_TRUE(buffer.postResetStable());
  EXPECT_EQ(buffer.stableSampleCount(), 3U);
  EXPECT_TRUE(buffer.sample(first.timestamp_ns).has_value());
}

TEST(Px4RingBuffer, InvalidTimestampResetsStableGate) {
  px4_odometry_bridge::OdometryRingBuffer buffer({
      .duration_ns = 2'000'000'000, .capacity = 512, .max_gap_ns = 50'000'000, .stable_samples = 2});
  EXPECT_FALSE(buffer.push(sample(1'000'000'000)));
  auto invalid = sample(1'000'000'000);
  invalid.timestamp_ns = 0;
  EXPECT_FALSE(buffer.push(invalid));
  EXPECT_EQ(buffer.stableSampleCount(), 0U);
  EXPECT_FALSE(buffer.postResetStable());
}

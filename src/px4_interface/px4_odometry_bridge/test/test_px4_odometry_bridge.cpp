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
  EXPECT_FALSE(compensator.observe(jump).has_value());
  auto missing = sample(1'020'000'000);
  missing.reset_counter = 1;
  EXPECT_FALSE(compensator.observe(missing).has_value());
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

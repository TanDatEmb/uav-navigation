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

TEST(Px4RingBuffer, InterpolatesWithoutExtrapolationOrGenerationCrossing) {
  px4_odometry_bridge::OdometryRingBuffer buffer;
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
  EXPECT_FALSE(compensator.observe(reset, {.available = true}).has_value());
  auto after = sample(1'020'000'000);
  after.reset_counter = 1;
  after.position.x() = 10.01;
  auto output = compensator.observe(after, {.available = true});
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->reset_generation, 1U);
  EXPECT_DOUBLE_EQ(output->position.x(), 1.01);
}

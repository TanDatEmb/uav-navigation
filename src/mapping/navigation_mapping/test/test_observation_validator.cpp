#include <gtest/gtest.h>

#include <limits>

#include "navigation_mapping/observation_validator.hpp"

namespace navigation_mapping {
namespace {

builtin_interfaces::msg::Time makeStamp(std::int32_t sec, std::uint32_t nanosec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

geometry_msgs::msg::Pose identityPose() {
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

TEST(ObservationValidatorTest, AcceptsMatchingContractFramesAndStamps) {
  ObservationValidator validator;
  const auto stamp = makeStamp(10, 500);
  const auto result = validator.validateFrames("lio_odom", "livox_frame", stamp, stamp);
  EXPECT_TRUE(result.valid);
}

TEST(ObservationValidatorTest, RejectsWrongHeaderFrameId) {
  ObservationValidator validator;
  const auto stamp = makeStamp(10, 500);
  const auto result = validator.validateFrames("world", "livox_frame", stamp, stamp);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, ObservationRejectionReason::kInvalidFrameId);
}

TEST(ObservationValidatorTest, RejectsWrongPointsFrameId) {
  ObservationValidator validator;
  const auto stamp = makeStamp(10, 500);
  const auto result = validator.validateFrames("lio_odom", "base_link", stamp, stamp);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, ObservationRejectionReason::kInvalidPointsFrameId);
}

TEST(ObservationValidatorTest, RejectsStampMismatchBetweenHeaderAndPoints) {
  ObservationValidator validator;
  const auto header_stamp = makeStamp(10, 500);
  const auto points_stamp = makeStamp(10, 600);
  const auto result =
      validator.validateFrames("lio_odom", "livox_frame", header_stamp, points_stamp);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, ObservationRejectionReason::kPointsStampMismatch);
}

TEST(ObservationValidatorTest, RespectsConfiguredFrameNames) {
  ObservationValidator validator(ObservationContract{"custom_odom", "custom_lidar"});
  const auto stamp = makeStamp(0, 0);
  EXPECT_TRUE(validator.validateFrames("custom_odom", "custom_lidar", stamp, stamp).valid);
  EXPECT_FALSE(validator.validateFrames("lio_odom", "custom_lidar", stamp, stamp).valid);
}

TEST(ObservationValidatorTest, AcceptsFinitePose) {
  ObservationValidator validator;
  EXPECT_TRUE(validator.validatePose(identityPose()).valid);
}

TEST(ObservationValidatorTest, RejectsNonFinitePosition) {
  ObservationValidator validator;
  auto pose = identityPose();
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = validator.validatePose(pose);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, ObservationRejectionReason::kInvalidPose);
}

TEST(ObservationValidatorTest, RejectsZeroQuaternion) {
  ObservationValidator validator;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 0.0;
  const auto result = validator.validatePose(pose);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, ObservationRejectionReason::kInvalidPose);
}

}  // namespace
}  // namespace navigation_mapping

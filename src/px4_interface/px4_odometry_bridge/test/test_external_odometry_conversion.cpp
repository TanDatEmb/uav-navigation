#include <gtest/gtest.h>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"

namespace px4_odometry_bridge {
namespace {

TEST(ExternalOdometryConversionTest, ConvertsRosZUpFluToPx4Frd) {
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 1;
  message.header.stamp.nanosec = 2;
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = 2.0;
  message.pose.pose.position.z = 3.0;
  message.twist.twist.linear.y = 4.0;
  message.twist.twist.angular.z = 5.0;
  for (int index = 0; index < 3; ++index) {
    message.pose.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
    message.pose.covariance[static_cast<std::size_t>((index + 3) * 6 + index + 3)] = 1.0;
    message.twist.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
  }
  const auto converted = convert_ros_lio_odometry(message);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(converted->timestamp_ns, 1'000'000'002LL);
  EXPECT_DOUBLE_EQ(converted->position_frd.x(), 1.0);
  EXPECT_DOUBLE_EQ(converted->position_frd.y(), -2.0);
  EXPECT_DOUBLE_EQ(converted->position_frd.z(), -3.0);
  EXPECT_DOUBLE_EQ(converted->velocity_body_frd.y(), -4.0);
  EXPECT_DOUBLE_EQ(converted->angular_velocity_body_frd.z(), -5.0);
}

TEST(ExternalOdometryConversionTest, RejectsZeroTimestampAndQuaternion) {
  nav_msgs::msg::Odometry message;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message.header.stamp.sec = 1;
  message.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
}

TEST(ExternalOdometryConversionTest, RejectsUnknownOrNonPositiveCovariance) {
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 1;
  message.pose.pose.orientation.w = 1.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  for (int index = 0; index < 3; ++index) {
    message.pose.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
    message.pose.covariance[static_cast<std::size_t>((index + 3) * 6 + index + 3)] = 1.0;
    message.twist.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
  }
  ASSERT_TRUE(convert_ros_lio_odometry(message).has_value());
  message.pose.covariance[0] = 0.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
}

}  // namespace
}  // namespace px4_odometry_bridge

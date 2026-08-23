#include <gtest/gtest.h>

#include <navigation_runtime/input_pairing.hpp>

namespace {

nav_msgs::msg::Odometry odometryAt(std::int64_t stamp_ns) {
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  return message;
}

}  // namespace

TEST(InputPairing, SelectsNearestTimestampRatherThanLatestSample) {
  std::deque<nav_msgs::msg::Odometry> history{
      odometryAt(1000000000LL), odometryAt(1050000000LL), odometryAt(1100000000LL)};
  const auto index = navigation_runtime::input_pairing::nearestOdometryIndex(
      history, 1040000000LL, 20000000LL);
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(*index, 1U);
}

TEST(InputPairing, RejectsCloudWhenNoOdometryIsWithinSkew) {
  std::deque<nav_msgs::msg::Odometry> history{odometryAt(1000000000LL)};
  EXPECT_FALSE(navigation_runtime::input_pairing::nearestOdometryIndex(
      history, 1200000000LL, 100000000LL));
}

TEST(InputPairing, RejectsInvalidTimestamps) {
  std::deque<nav_msgs::msg::Odometry> history{odometryAt(0), odometryAt(1000000000LL)};
  EXPECT_FALSE(navigation_runtime::input_pairing::nearestOdometryIndex(
      history, 0, 100000000LL));
}

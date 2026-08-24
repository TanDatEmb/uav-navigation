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

TEST(InputPairing, ExactPairRequiresTheCorrectionEpoch) {
  std::deque<nav_msgs::msg::Odometry> history{
      odometryAt(1000000000LL), odometryAt(1050000000LL)};
  const auto exact = navigation_runtime::input_pairing::exactOdometryIndex(
      history, 1050000000LL);
  ASSERT_TRUE(exact.has_value());
  EXPECT_EQ(*exact, 1U);
  EXPECT_FALSE(navigation_runtime::input_pairing::exactOdometryIndex(
      history, 1049999999LL));
}

TEST(InputPairing, PendingObservationIsRetainedUntilExactPairAndConsumedOnce) {
  std::optional<navigation_runtime::input_pairing::StampedObservation<int>> pending{
      navigation_runtime::input_pairing::StampedObservation<int>{42, 1050000000LL}};
  std::deque<nav_msgs::msg::Odometry> history{odometryAt(1000000000LL)};
  EXPECT_FALSE(navigation_runtime::input_pairing::tryTakeExactPair(pending, history));
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending->payload, 42);

  history.push_back(odometryAt(1050000000LL));
  const auto pair = navigation_runtime::input_pairing::tryTakeExactPair(pending, history);
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->payload, 42);
  EXPECT_FALSE(pending.has_value());
  EXPECT_TRUE(history.empty());
  EXPECT_FALSE(navigation_runtime::input_pairing::tryTakeExactPair(pending, history));
}

#include "navigation_runtime/heading_tracker.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

navigation_runtime::HeadingTargetIdentity identity(
    const std::uint32_t waypoint_index, const std::uint64_t request_id = 1U) {
  return navigation_runtime::HeadingTargetIdentity{
      "mission", 7U, waypoint_index, request_id, 3U};
}

}  // namespace

TEST(HeadingTracker, SeedsFromMeasuredYawAndAcceleratesTowardFirstTarget) {
  navigation_runtime::HeadingTracker tracker;
  const auto first = tracker.step(identity(0U), M_PI_2, 0.25, 1'000'000'000LL,
                                  2.0, 2.0);
  ASSERT_TRUE(first.valid);
  EXPECT_DOUBLE_EQ(first.yaw_rad, 0.25);
  EXPECT_DOUBLE_EQ(first.yaw_rate_rad_s, 0.0);

  const auto second = tracker.step(identity(0U), M_PI_2, 0.25,
                                   1'020'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(second.valid);
  EXPECT_NEAR(second.yaw_rate_rad_s, 0.04, 1.0e-12);
  EXPECT_NEAR(second.yaw_rad, 0.2504, 1.0e-12);
  EXPECT_NEAR(second.yaw_acceleration_rad_s2, 2.0, 1.0e-12);
}

TEST(HeadingTracker, WaypointIdentityChangePreservesPublishedTurnState) {
  navigation_runtime::HeadingTracker tracker;
  ASSERT_TRUE(tracker.step(identity(0U), 0.0, 0.0, 1'000'000'000LL,
                           2.0, 2.0).valid);
  const auto before = tracker.step(identity(0U), M_PI_2, 0.0,
                                   1'020'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(before.valid);

  const auto after = tracker.step(identity(1U, 2U), M_PI, 0.0,
                                  1'040'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(after.valid);
  EXPECT_GT(after.yaw_rate_rad_s, 0.0);
  EXPECT_LE(std::abs(after.yaw_rate_rad_s - before.yaw_rate_rad_s), 0.04 + 1.0e-12);
  EXPECT_EQ(tracker.targetIdentity().waypoint_index, 1U);
  EXPECT_EQ(tracker.targetIdentity().request_id, 2U);
}

TEST(HeadingTracker, ClockResetHoldsStateAndReanchorsNextTick) {
  navigation_runtime::HeadingTracker tracker;
  ASSERT_TRUE(tracker.step(identity(0U), M_PI_2, 0.0, 2'000'000'000LL,
                           2.0, 2.0).valid);
  const auto moving = tracker.step(identity(0U), M_PI_2, 0.0,
                                   2'020'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(moving.valid);

  const auto reset = tracker.step(identity(0U), M_PI_2, 0.0,
                                  1'000'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(reset.valid);
  EXPECT_DOUBLE_EQ(reset.yaw_rad, moving.yaw_rad);
  EXPECT_DOUBLE_EQ(reset.yaw_rate_rad_s, moving.yaw_rate_rad_s);

  const auto resumed = tracker.step(identity(0U), M_PI_2, 0.0,
                                    1'020'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(resumed.valid);
  EXPECT_NEAR(resumed.yaw_rate_rad_s, moving.yaw_rate_rad_s + 0.04, 1.0e-12);
}

TEST(HeadingTracker, LargeClockGapDoesNotCreateAnUnboundedTurn) {
  navigation_runtime::HeadingTracker tracker;
  ASSERT_TRUE(tracker.step(identity(0U), M_PI, 0.0, 1'000'000'000LL,
                           2.0, 2.0).valid);
  const auto delayed = tracker.step(identity(0U), M_PI, 0.0,
                                    2'000'000'000LL, 2.0, 2.0);
  ASSERT_TRUE(delayed.valid);
  EXPECT_DOUBLE_EQ(delayed.yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(delayed.yaw_rate_rad_s, 0.0);
}

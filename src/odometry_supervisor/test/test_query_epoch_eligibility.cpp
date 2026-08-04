#include <gtest/gtest.h>

#include <deque>

#include "odometry_supervisor/query_epoch_eligibility.hpp"

namespace {

odometry_supervisor::OdometryState state(std::int64_t epoch_ns) {
  odometry_supervisor::OdometryState value;
  value.timestamp_ns = epoch_ns;
  value.valid = true;
  value.frame_id = "lio_odom";
  value.child_frame_id = "base_link";
  return value;
}

}  // namespace

TEST(QueryEpochEligibility, DoesNotQueryWhenLioIsAheadOfPx4) {
  const std::deque<odometry_supervisor::OdometryState> history{state(100), state(200)};
  const auto result = odometry_supervisor::latest_propagated_epoch_available_to_px4(
      history, state(50));
  EXPECT_FALSE(result.state.has_value());
  EXPECT_EQ(result.reason,
            odometry_supervisor::QueryEpochEligibilityReason::kLioEpochAheadOfPx4);
}

TEST(QueryEpochEligibility, SelectsHistoricalEpochWithoutExtrapolation) {
  const std::deque<odometry_supervisor::OdometryState> history{state(100), state(200), state(300)};
  const auto result = odometry_supervisor::latest_propagated_epoch_available_to_px4(
      history, state(250));
  ASSERT_TRUE(result.state.has_value());
  EXPECT_EQ(result.state->timestamp_ns, 200);
  EXPECT_EQ(result.reason, odometry_supervisor::QueryEpochEligibilityReason::kEligible);
}

TEST(QueryEpochEligibility, QueryKeyIncludesAllGenerations) {
  const auto value = state(200);
  const auto a = odometry_supervisor::query_epoch_key(value, 1, 2, 3);
  EXPECT_EQ(a, (odometry_supervisor::QueryEpochKey{200, 1, 2, 3}));
  EXPECT_NE(a, (odometry_supervisor::QueryEpochKey{200, 1, 2, 4}));
}

#include <gtest/gtest.h>

#include <limits>

#include "navigation_planning/horizon_policy.hpp"

namespace navigation_planning {
namespace {

HorizonRequest requestAt(double speed) {
  HorizonRequest request;
  request.route.projected_arc_length_m = 0.0;
  request.route.route_length_m = 100.0;
  request.speed_mps = speed;
  request.max_deceleration_mps2 = 3.0;
  return request;
}

TEST(HorizonPolicy, UsesPreviewDistanceAtThreeMetersPerSecond) {
  const auto result = HorizonPolicy{}.compute(requestAt(3.0));
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.forward_distance_m, 15.0, 1e-9);
}

TEST(HorizonPolicy, UsesPreviewDistanceAtFiveMetersPerSecond) {
  const auto result = HorizonPolicy{}.compute(requestAt(5.0));
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.forward_distance_m, 25.0, 1e-9);
}

TEST(HorizonPolicy, AppliesMapMargin) {
  auto request = requestAt(5.0);
  request.route.usable_forward_distance_m = 18.0;
  const auto result = HorizonPolicy{}.compute(request);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.forward_distance_m, 16.0, 1e-9);
  EXPECT_TRUE(result.shortened_by_usable_space);
}

TEST(HorizonPolicy, ClampsToTerminalWaypoint) {
  auto request = requestAt(5.0);
  request.route.projected_arc_length_m = 20.0;
  request.terminal_waypoint_arc_length_m = 32.0;
  const auto result = HorizonPolicy{}.compute(request);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.endpoint_arc_length_m, 32.0, 1e-9);
  EXPECT_TRUE(result.terminal_waypoint_clamped);
}

TEST(HorizonPolicy, RejectsBackwardProjection) {
  auto request = requestAt(3.0);
  request.route.projected_arc_length_m = 19.0;
  request.route.previous_projected_arc_length_m = 20.0;
  const auto result = HorizonPolicy{}.compute(request);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, HorizonFailureCode::BackwardProjection);
}

TEST(HorizonPolicy, RejectsNegativeInfiniteUsableDistance) {
  auto request = requestAt(3.0);
  request.route.usable_forward_distance_m =
      -std::numeric_limits<double>::infinity();
  const auto result = HorizonPolicy{}.compute(request);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, HorizonFailureCode::InvalidInput);
}

}  // namespace
}  // namespace navigation_planning

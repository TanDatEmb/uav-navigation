#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "navigation_planning/bspline_trajectory.hpp"

namespace {

using navigation_mapping::Vec3;
using navigation_planning::BsplineGenerationConfig;

TEST(BsplineTrajectory, MatchesRecedingHorizonBoundaryState) {
  BsplineGenerationConfig config;
  config.knot_dt_s = 0.7;
  config.sample_dt_s = 0.01;
  config.smoothing_iterations = 4;
  config.max_velocity_mps = 20.0;
  config.max_acceleration_mps2 = 20.0;
  config.max_deceleration_mps2 = 20.0;
  config.max_jerk_mps3 = 200.0;
  const Vec3 start_velocity{0.4, 0.1, 0.0};
  const Vec3 start_acceleration{0.3, 0.0, 0.0};
  const auto result = navigation_planning::generateBsplineTrajectory(
      {Vec3{0.0, 0.0, 1.0}, Vec3{2.0, 0.4, 1.0}, Vec3{4.0, 0.0, 1.0}}, start_velocity,
      start_acceleration, Vec3::Zero(), Vec3::Zero(), config);

  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.trajectory.valid());
  const auto first = result.trajectory.evaluate(0.0);
  const auto last = result.trajectory.evaluate(result.trajectory.duration());
  EXPECT_NEAR(first.position.x(), 0.0, 1e-9);
  EXPECT_NEAR(first.velocity.x(), start_velocity.x(), 1e-9);
  EXPECT_NEAR(first.velocity.y(), start_velocity.y(), 1e-9);
  EXPECT_NEAR(first.acceleration.x(), start_acceleration.x(), 1e-9);
  EXPECT_NEAR(last.position.x(), 4.0, 1e-9);
  EXPECT_NEAR(last.velocity.norm(), 0.0, 1e-8);
  EXPECT_NEAR(last.acceleration.norm(), 0.0, 1e-8);
}

TEST(BsplineTrajectory, InternalKnotsHaveHighOrderContinuity) {
  BsplineGenerationConfig config;
  config.knot_dt_s = 0.5;
  config.sample_dt_s = 0.01;
  config.smoothing_iterations = 0;
  config.max_velocity_mps = 100.0;
  config.max_acceleration_mps2 = 100.0;
  config.max_deceleration_mps2 = 100.0;
  config.max_jerk_mps3 = 1000.0;
  const auto result = navigation_planning::generateBsplineTrajectory(
      {Vec3{0.0, 0.0, 0.0}, Vec3{2.0, 1.0, 0.0}, Vec3{4.0, -1.0, 0.0}, Vec3{6.0, 0.0, 0.0}},
      Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), config);

  ASSERT_TRUE(result.success);
  ASSERT_GT(result.trajectory.spanCount(), 2U);
  for (std::size_t span = 1; span < result.trajectory.spanCount(); ++span) {
    const double knot = static_cast<double>(span) * result.trajectory.knot_dt_s;
    const auto left = result.trajectory.evaluate(std::nextafter(knot, 0.0));
    const auto right = result.trajectory.evaluate(std::nextafter(knot, result.trajectory.duration()));
    EXPECT_LT((left.position - right.position).norm(), 1e-5);
    EXPECT_LT((left.velocity - right.velocity).norm(), 1e-4);
    EXPECT_LT((left.acceleration - right.acceleration).norm(), 1e-3);
    EXPECT_LT((left.jerk - right.jerk).norm(), 1e-2);
  }
}

TEST(BsplineTrajectory, ControlPolygonDoesNotDwellAtEndpoint) {
  BsplineGenerationConfig config;
  config.knot_dt_s = 0.5;
  config.sample_dt_s = 0.01;
  config.smoothing_iterations = 0;
  config.max_velocity_mps = 100.0;
  config.max_acceleration_mps2 = 100.0;
  config.max_deceleration_mps2 = 100.0;
  config.max_jerk_mps3 = 1000.0;
  const auto result = navigation_planning::generateBsplineTrajectory(
      {Vec3{0.0, 0.0, 0.0}, Vec3{4.0, 0.0, 0.0}, Vec3{8.0, 0.0, 0.0}},
      Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), config);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.trajectory.spanCount(), 2U);
  const auto last_control_point_before_endpoint =
      result.trajectory.control_points[result.trajectory.spanCount() - 1U];
  EXPECT_GT((last_control_point_before_endpoint - Vec3{8.0, 0.0, 0.0}).norm(), 0.5);
}

TEST(BsplineTrajectory, UsesCompactTopologyForLongStraightReference) {
  BsplineGenerationConfig config;
  config.knot_dt_s = 2.6;
  config.sample_dt_s = 0.01;
  config.smoothing_iterations = 12;
  config.max_velocity_mps = 3.0;
  config.max_acceleration_mps2 = 2.0;
  config.max_deceleration_mps2 = 2.0;
  config.max_jerk_mps3 = 8.0;
  const auto result = navigation_planning::generateBsplineTrajectory(
      {Vec3{0.0, 0.0, 0.0}, Vec3{10.0, 0.0, 0.0}}, Vec3::Zero(), Vec3::Zero(),
      Vec3{1.5, 0.0, 0.0}, Vec3::Zero(), config);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.trajectory.spanCount(), 2U);
  EXPECT_LT(result.trajectory.duration(), 6.0);
  EXPECT_LE(result.maximum_velocity_mps, config.max_velocity_mps * 0.995 + 1e-9);
  EXPECT_LE(result.maximum_acceleration_mps2,
            config.max_acceleration_mps2 * 0.995 + 1e-9);
  EXPECT_LE(result.maximum_jerk_mps3, config.max_jerk_mps3 * 0.995 + 1e-9);
}

TEST(BsplineTrajectory, PreservesShortReferenceCornerInControlPolygon) {
  BsplineGenerationConfig config;
  config.knot_dt_s = 1.0;
  config.sample_dt_s = 0.01;
  config.smoothing_iterations = 0;
  config.max_velocity_mps = 100.0;
  config.max_acceleration_mps2 = 100.0;
  config.max_deceleration_mps2 = 100.0;
  config.max_jerk_mps3 = 1000.0;
  const auto result = navigation_planning::generateBsplineTrajectory(
      {Vec3{0.0, 0.0, 0.0}, Vec3{4.0, 0.0, 0.0}, Vec3{4.0, 0.6, 0.0}}, Vec3::Zero(),
      Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), config);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.trajectory.spanCount(), 2U);
  ASSERT_EQ(result.trajectory.control_points.size(), 7U);
  const auto corner = std::find_if(
      result.trajectory.control_points.begin(), result.trajectory.control_points.end(),
      [](const Vec3& point) { return (point - Vec3{4.0, 0.0, 0.0}).norm() <= 1e-9; });
  EXPECT_NE(corner, result.trajectory.control_points.end());
}

}  // namespace

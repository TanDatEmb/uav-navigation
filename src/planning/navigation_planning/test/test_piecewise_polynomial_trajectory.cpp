#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "navigation_planning/piecewise_polynomial_trajectory.hpp"

namespace {

using navigation_planning::PiecewisePolynomialTrajectory;
using navigation_planning::PolynomialSegment;
using navigation_planning::PolynomialTrajectoryValidationCode;
using Vector3 = Eigen::Vector3d;

PolynomialSegment constantSegment(double duration_s, double value) {
  return PolynomialSegment{duration_s, {Vector3{value, value, value}}};
}

TEST(PiecewisePolynomialTrajectory, EvaluatesPositionAndDerivativesThroughSnap) {
  PolynomialSegment segment;
  segment.duration_s = 2.0;
  segment.coefficients = {
      Vector3{1.0, 0.0, 0.0}, Vector3{2.0, 1.0, 0.0}, Vector3{3.0, 0.0, 1.0},
      Vector3{4.0, 0.0, 0.0}, Vector3{5.0, 0.0, 0.0}, Vector3{6.0, 0.0, 0.0}};

  PiecewisePolynomialTrajectory trajectory({segment});
  ASSERT_TRUE(trajectory.validate().valid());

  const auto sample = trajectory.evaluate(1.0);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->segment_index, 0U);
  EXPECT_NEAR(sample->position.x(), 21.0, 1e-12);
  EXPECT_NEAR(sample->velocity.x(), 70.0, 1e-12);
  EXPECT_NEAR(sample->acceleration.x(), 210.0, 1e-12);
  EXPECT_NEAR(sample->jerk.x(), 504.0, 1e-12);
  EXPECT_NEAR(sample->snap.x(), 840.0, 1e-12);
  EXPECT_NEAR(sample->position.y(), 1.0, 1e-12);
  EXPECT_NEAR(sample->velocity.y(), 1.0, 1e-12);
  EXPECT_NEAR(sample->acceleration.y(), 0.0, 1e-12);
  EXPECT_TRUE(sample->allFinite());
}

TEST(PiecewisePolynomialTrajectory, UsesRightSegmentAtInternalBoundary) {
  PiecewisePolynomialTrajectory trajectory(
      {constantSegment(1.0, 7.0), constantSegment(2.0, 9.0)});

  const auto at_start = trajectory.evaluate(0.0);
  const auto at_boundary = trajectory.evaluate(1.0);
  const auto at_end = trajectory.evaluate(3.0);
  ASSERT_TRUE(at_start.has_value());
  ASSERT_TRUE(at_boundary.has_value());
  ASSERT_TRUE(at_end.has_value());

  EXPECT_EQ(at_start->segment_index, 0U);
  EXPECT_EQ(at_boundary->segment_index, 1U);
  EXPECT_EQ(at_end->segment_index, 1U);
  EXPECT_NEAR(at_boundary->local_time_s, 0.0, 1e-12);
  EXPECT_NEAR(at_boundary->position.x(), 9.0, 1e-12);
  EXPECT_NEAR(at_end->local_time_s, 2.0, 1e-12);
}

TEST(PiecewisePolynomialTrajectory, ClampsFiniteQueryTimeToTrajectory) {
  PolynomialSegment segment;
  segment.duration_s = 2.0;
  segment.coefficients = {Vector3::Zero(), Vector3{1.0, 2.0, 3.0}};
  PiecewisePolynomialTrajectory trajectory({segment});

  const auto before_start = trajectory.evaluate(-100.0);
  const auto after_end = trajectory.evaluate(100.0);
  ASSERT_TRUE(before_start.has_value());
  ASSERT_TRUE(after_end.has_value());
  EXPECT_NEAR(before_start->time_s, 0.0, 1e-12);
  EXPECT_NEAR(before_start->local_time_s, 0.0, 1e-12);
  EXPECT_TRUE(before_start->position.isApprox(Vector3::Zero(), 1e-12));
  EXPECT_NEAR(after_end->time_s, 2.0, 1e-12);
  EXPECT_NEAR(after_end->local_time_s, 2.0, 1e-12);
  EXPECT_TRUE(after_end->position.isApprox(Vector3{2.0, 4.0, 6.0}, 1e-12));
  EXPECT_TRUE(after_end->velocity.isApprox(Vector3{1.0, 2.0, 3.0}, 1e-12));
}

TEST(PiecewisePolynomialTrajectory, RejectsEmptyAndNonFiniteOrNonMonotonicInput) {
  PiecewisePolynomialTrajectory empty;
  EXPECT_EQ(empty.validate().code,
            PolynomialTrajectoryValidationCode::EmptyTrajectory);
  EXPECT_FALSE(empty.evaluate(0.0).has_value());

  PiecewisePolynomialTrajectory zero_duration({constantSegment(0.0, 1.0)});
  EXPECT_EQ(zero_duration.validate().code,
            PolynomialTrajectoryValidationCode::NonPositiveDuration);

  PiecewisePolynomialTrajectory negative_duration({constantSegment(-1.0, 1.0)});
  EXPECT_EQ(negative_duration.validate().code,
            PolynomialTrajectoryValidationCode::NonPositiveDuration);

  PiecewisePolynomialTrajectory nan_duration({
      PolynomialSegment{std::numeric_limits<double>::quiet_NaN(),
                        {Vector3::Zero()}}});
  EXPECT_EQ(nan_duration.validate().code,
            PolynomialTrajectoryValidationCode::NonFiniteDuration);

  PiecewisePolynomialTrajectory no_coefficients({PolynomialSegment{1.0, {}}});
  EXPECT_EQ(no_coefficients.validate().code,
            PolynomialTrajectoryValidationCode::EmptyCoefficients);

  PiecewisePolynomialTrajectory nan_coefficient({PolynomialSegment{
      1.0, {Vector3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}}});
  EXPECT_EQ(nan_coefficient.validate().code,
            PolynomialTrajectoryValidationCode::NonFiniteCoefficient);

  PolynomialSegment huge = constantSegment(std::numeric_limits<double>::max(), 0.0);
  PiecewisePolynomialTrajectory overflowed_total({huge, huge});
  EXPECT_EQ(overflowed_total.validate().code,
            PolynomialTrajectoryValidationCode::NonFiniteTotalDuration);

  PiecewisePolynomialTrajectory non_monotonic_time(
      {constantSegment(1.0e308, 0.0), constantSegment(1.0e-300, 0.0)});
  EXPECT_EQ(non_monotonic_time.validate().code,
            PolynomialTrajectoryValidationCode::NonMonotonicTime);
}

TEST(PiecewisePolynomialTrajectory, RejectsNonFiniteQueryTime) {
  PiecewisePolynomialTrajectory trajectory({constantSegment(1.0, 2.0)});
  EXPECT_FALSE(trajectory.evaluate(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(trajectory.evaluate(std::numeric_limits<double>::infinity()).has_value());
}

}  // namespace

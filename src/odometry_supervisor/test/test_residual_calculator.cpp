#include <gtest/gtest.h>

#include <cmath>

#include "odometry_supervisor/residual_calculator.hpp"

namespace {
odometry_supervisor::OdometryState state(std::int64_t timestamp, double yaw = 0.0) {
  odometry_supervisor::OdometryState result;
  result.timestamp_ns = timestamp;
  result.position_odom = Eigen::Vector3d(1.0, 2.0, 3.0);
  result.orientation_odom_base =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  result.velocity_base = Eigen::Vector3d(1.0, 0.0, 0.0);
  result.frame_id = "odom";
  result.child_frame_id = "base_link";
  result.valid = true;
  return result;
}
}  // namespace

TEST(OdometryResidual, IdenticalStatesProduceZeroResidual) {
  const auto result = odometry_supervisor::ResidualCalculator::compare(state(1'000'000'000),
                                                                        state(1'000'000'000));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->position_error_m, 0.0, 1e-12);
  EXPECT_NEAR(result->velocity_error_m_s, 0.0, 1e-12);
  EXPECT_NEAR(result->orientation_error_rad, 0.0, 1e-12);
}

TEST(OdometryResidual, ConvertsBodyVelocityToCommonWorldFrame) {
  auto lio = state(1'000'000'000, M_PI / 2.0);
  auto px4 = state(1'000'000'000, 0.0);
  px4.velocity_base = Eigen::Vector3d(0.0, 1.0, 0.0);
  const auto result = odometry_supervisor::ResidualCalculator::compare(lio, px4);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->velocity_error_m_s, 0.0, 1e-12);
}

TEST(OdometryResidual, QuaternionSignDoesNotCreateError) {
  auto lio = state(1'000'000'000);
  auto px4 = lio;
  px4.orientation_odom_base.coeffs() *= -1.0;
  const auto result = odometry_supervisor::ResidualCalculator::compare(lio, px4);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->orientation_error_rad, 0.0, 1e-12);
}

TEST(OdometryResidual, WrapsYawAcrossPi) {
  const auto result = odometry_supervisor::ResidualCalculator::compare(
      state(1'000'000'000, -M_PI + 0.01), state(1'000'000'000, M_PI - 0.01));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(std::abs(result->yaw_error_rad), 0.02, 1e-9);
}

TEST(OdometryResidual, RequiresSameEpochAndRejectsInvalidState) {
  EXPECT_FALSE(odometry_supervisor::ResidualCalculator::compare(
                   state(1'000'000'000), state(1'000'000'001))
                   .has_value());
  auto invalid = state(1'000'000'000);
  invalid.frame_id = "map";
  EXPECT_FALSE(odometry_supervisor::ResidualCalculator::compare(
                   invalid, state(1'000'000'000))
                   .has_value());
}

TEST(OdometryResidual, ComputesPositionErrorGrowthRate) {
  auto previous = state(1'000'000'000);
  auto current = state(2'000'000'000);
  current.position_odom.x() += 1.0;
  const auto previous_residual = odometry_supervisor::ResidualCalculator::compare(previous, previous);
  ASSERT_TRUE(previous_residual.has_value());
  const auto current_reference = state(2'000'000'000);
  const auto result = odometry_supervisor::ResidualCalculator::compare(
      current, current_reference, previous_residual);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->position_error_growth_m_s, 0.0);
}

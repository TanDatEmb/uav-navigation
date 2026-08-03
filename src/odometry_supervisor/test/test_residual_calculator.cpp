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
  result.frame_id = "lio_odom";
  result.child_frame_id = "base_link";
  result.valid = true;
  return result;
}

odometry_supervisor::OdometryState state(std::int64_t timestamp,
                                         const Eigen::Quaterniond& orientation) {
  auto result = state(timestamp);
  result.orientation_odom_base = orientation;
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

TEST(OdometryResidual, UsesProjectedBodyXHeadingInsteadOfEulerYaw) {
  constexpr double yaw = 0.7;
  const Eigen::Quaterniond tilted =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitX()));
  const auto result = odometry_supervisor::ResidualCalculator::compare(
      state(1'000'000'000, tilted), state(1'000'000'000, yaw));
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->heading_observable);
  EXPECT_NEAR(result->yaw_error_rad, 0.0, 1e-12);
  EXPECT_NEAR(result->robust_heading_lio_rad, yaw, 1e-12);
  EXPECT_NEAR(result->robust_heading_px4_rad, yaw, 1e-12);
}

TEST(OdometryResidual, RejectsUnobservableVerticalBodyXHeading) {
  const Eigen::Quaterniond vertical_body_x(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  const auto result = odometry_supervisor::ResidualCalculator::compare(
      state(1'000'000'000, vertical_body_x), state(1'000'000'000));
  EXPECT_FALSE(result.has_value());
}

TEST(OdometryResidual, ExposesQuaternionErrorProvenance) {
  const auto result = odometry_supervisor::ResidualCalculator::compare(
      state(1'000'000'000, 0.2), state(1'000'000'000, 0.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->orientation_error_rad, 0.2, 1e-12);
  EXPECT_NEAR(result->q_error_axis.x(), 0.0, 1e-12);
  EXPECT_NEAR(result->q_error_axis.y(), 0.0, 1e-12);
  EXPECT_NEAR(result->q_error_axis.z(), 1.0, 1e-12);
  EXPECT_NEAR(result->body_z_dot, 1.0, 1e-12);
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

TEST(OdometryResidual, KeepsPx4AndLioWorldFramesDistinct) {
  auto lio = state(1'000'000'000);
  auto px4 = state(1'000'000'000);
  lio.frame_id = "lio_odom";
  px4.frame_id = "px4_odom";

  EXPECT_TRUE(odometry_supervisor::ResidualCalculator::valid(lio));
  EXPECT_TRUE(odometry_supervisor::ResidualCalculator::valid(px4));
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

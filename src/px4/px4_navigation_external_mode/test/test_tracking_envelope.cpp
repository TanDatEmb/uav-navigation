#include <Eigen/Core>
#include <gtest/gtest.h>

#include "px4_navigation_external_mode/tracking_envelope.hpp"

using px4_navigation_external_mode::evaluateTrackingEnvelope;

TEST(TrackingEnvelope, AllowsBoundedForwardTrackingLagAtSpeed) {
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{1.5, 0.2, 0.0},
      Eigen::Vector3d{4.0, 0.0, 0.0}, 0.75, 0.25);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.longitudinal_limit_m, 1.75);
  EXPECT_NEAR(result.longitudinal_error_m, 1.5, 1e-12);
  EXPECT_NEAR(result.lateral_error_m, 0.2, 1e-12);
}

TEST(TrackingEnvelope, RejectsRunawayReferenceForBlockedVehicle) {
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{2.0, 0.0, 0.0},
      Eigen::Vector3d{4.0, 0.0, 0.0}, 0.75, 0.25);
  EXPECT_FALSE(result.valid);
}

TEST(TrackingEnvelope, NeverRelaxesLateralOrReverseError) {
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.8, 0.0},
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75, 0.25).valid);
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{-0.8, 0.0, 0.0},
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75, 0.25).valid);
}

TEST(TrackingEnvelope, StationaryCommandUsesStrictGeometricLimit) {
  EXPECT_TRUE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.7, 0.0, 0.0},
      Eigen::Vector3d::Zero(), 0.75, 0.25).valid);
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.8, 0.0, 0.0},
      Eigen::Vector3d::Zero(), 0.75, 0.25).valid);
}

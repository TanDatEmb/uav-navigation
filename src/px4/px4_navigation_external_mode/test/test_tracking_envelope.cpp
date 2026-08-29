#include <limits>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "px4_navigation_external_mode/tracking_envelope.hpp"

using px4_navigation_external_mode::evaluateTrackingEnvelope;

TEST(TrackingEnvelope, AppliesOneFiniteGeometricLimitToForwardError) {
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{1.5, 0.2, 0.0},
      Eigen::Vector3d{4.0, 0.0, 0.0}, 0.75);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.longitudinal_limit_m, 0.75);
  EXPECT_NEAR(result.longitudinal_error_m, 1.5, 1e-12);
  EXPECT_NEAR(result.lateral_error_m, 0.2, 1e-12);
}

TEST(TrackingEnvelope, RejectsRunawayReferenceForBlockedVehicle) {
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{2.0, 0.0, 0.0},
      Eigen::Vector3d{4.0, 0.0, 0.0}, 0.75);
  EXPECT_FALSE(result.valid);
}

TEST(TrackingEnvelope, NeverRelaxesLateralOrReverseError) {
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.8, 0.0},
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75).valid);
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{-0.8, 0.0, 0.0},
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75).valid);
}

TEST(TrackingEnvelope, ReportsReverseOvershootSeparatelyFromForwardError) {
  const auto reverse = evaluateTrackingEnvelope(
      Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d::Zero(),
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75);
  EXPECT_FALSE(reverse.valid);
  EXPECT_DOUBLE_EQ(reverse.longitudinal_error_m, 0.0);
  EXPECT_DOUBLE_EQ(reverse.reverse_error_m, 1.0);
  EXPECT_DOUBLE_EQ(reverse.lateral_error_m, 0.0);

  const auto forward = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{1.0, 0.0, 0.0},
      Eigen::Vector3d{5.0, 0.0, 0.0}, 0.75);
  EXPECT_FALSE(forward.valid);
  EXPECT_DOUBLE_EQ(forward.longitudinal_error_m, 1.0);
  EXPECT_DOUBLE_EQ(forward.reverse_error_m, 0.0);
}

TEST(TrackingEnvelope, PhaseCDiagnosticRejectsMeasuredVehicleAheadOfReference) {
  // Values captured from the delayed-activation Phase C artifact.  The command
  // is finite and has a modest lateral error, but the measured vehicle is
  // already 0.76 m ahead of the command along its tangent.  This must remain a
  // genuine reverse-envelope rejection; it must not be relabelled as a
  // permissible forward tracking allowance.
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d{1.867377580704609, -0.18677978882325794,
                     3.086457165904786},
      Eigen::Vector3d{1.0945213975793624, -0.15572469095729685,
                     3.1925036335561443},
      Eigen::Vector3d{2.392748109836808, -0.036991851469775475,
                     0.20054273837458872},
      0.75);

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.longitudinal_error_m, 0.0);
  EXPECT_NEAR(result.reverse_error_m, 0.7616869676147896, 1.0e-12);
  EXPECT_NEAR(result.lateral_error_m, 0.17131817056432147, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.longitudinal_limit_m, 0.75);
  EXPECT_GT(result.reverse_error_m, 0.75);
  EXPECT_LT(result.lateral_error_m, 0.75);
}

TEST(TrackingEnvelope, ReverseBoundaryIsClosedOnlyAtTheGeometricLimit) {
  const Eigen::Vector3d velocity{3.0, 4.0, 0.0};
  const Eigen::Vector3d tangent = velocity.normalized();
  const auto boundary = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), -0.75 * tangent, velocity, 0.75);
  EXPECT_TRUE(boundary.valid);
  EXPECT_DOUBLE_EQ(boundary.reverse_error_m, 0.75);

  const auto over = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), -(0.75 + 1.0e-9) * tangent, velocity, 0.75);
  EXPECT_FALSE(over.valid);
  EXPECT_GT(over.reverse_error_m, 0.75);
}

TEST(TrackingEnvelope, DiagonalReverseAndLateralErrorsRemainIndependent) {
  const Eigen::Vector3d velocity{3.0, 4.0, 0.0};
  const Eigen::Vector3d tangent = velocity.normalized();
  const Eigen::Vector3d lateral{-tangent.y(), tangent.x(), 0.0};
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), -0.7 * tangent + 0.8 * lateral,
      velocity, 0.75);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.reverse_error_m, 0.7, 1.0e-12);
  EXPECT_NEAR(result.lateral_error_m, 0.8, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.longitudinal_error_m, 0.0);
}

TEST(TrackingEnvelope, StationaryCommandUsesStrictGeometricLimit) {
  EXPECT_TRUE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.7, 0.0, 0.0},
      Eigen::Vector3d::Zero(), 0.75).valid);
  EXPECT_FALSE(evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.8, 0.0, 0.0},
      Eigen::Vector3d::Zero(), 0.75).valid);
}

TEST(TrackingEnvelope, RejectsFiniteVelocityWhoseNormOverflows) {
  const double huge = std::numeric_limits<double>::max();
  const auto result = evaluateTrackingEnvelope(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d{huge, huge, huge}, 0.75);
  EXPECT_FALSE(result.valid);
}

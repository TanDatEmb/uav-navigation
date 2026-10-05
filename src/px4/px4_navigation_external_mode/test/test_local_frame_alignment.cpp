#include <cmath>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/local_frame_alignment.hpp"

TEST(LocalFrameAlignment, CapturesTranslationAfterBasisConversion) {
  const Eigen::Vector3d lio_position{1.0, 2.0, 3.0};
  const Eigen::Vector3d px4_position{2.5, -0.25, -3.5};
  const auto translation =
      px4_navigation_external_mode::localNedTranslationFromStationaryPair(
          lio_position, px4_position);
  ASSERT_TRUE(translation.has_value());
  EXPECT_TRUE(translation->isApprox(Eigen::Vector3d{0.5, -1.25, -0.5}));
  const auto converted = px4_navigation_external_mode::lioPositionToLocalNed(
      lio_position, *translation);
  ASSERT_TRUE(converted.has_value());
  EXPECT_TRUE(converted->isApprox(px4_position));
}

TEST(LocalFrameAlignment, RejectsNonFinitePair) {
  const auto translation =
      px4_navigation_external_mode::localNedTranslationFromStationaryPair(
          Eigen::Vector3d{NAN, 0.0, 0.0}, Eigen::Vector3d::Zero());
  EXPECT_FALSE(translation.has_value());
}

TEST(LocalFrameAlignment, ComputesWrappedLioPx4YawResidual) {
  const auto lio_yaw =
      px4_navigation_external_mode::yawFromQuaternionEnu(
          Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitZ()}});
  ASSERT_TRUE(lio_yaw.has_value());
  EXPECT_NEAR(*lio_yaw, 0.3, 1.0e-12);
  EXPECT_NEAR(
      *px4_navigation_external_mode::yawResidualLioToPx4(
          *lio_yaw, 0.5 * std::acos(-1.0) - 0.3),
      0.0, 1.0e-12);
  EXPECT_NEAR(
      *px4_navigation_external_mode::yawResidualLioToPx4(
          std::acos(-1.0) - 0.1, -0.5 * std::acos(-1.0)),
      -0.1, 1.0e-12);
}

TEST(LocalFrameAlignment, RejectsInvalidYawWitness) {
  EXPECT_FALSE(px4_navigation_external_mode::yawFromQuaternionEnu(
      Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(px4_navigation_external_mode::yawResidualLioToPx4(NAN, 0.0));
}

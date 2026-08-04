#include <gtest/gtest.h>

#include <cmath>

#include "odometry_supervisor/odometry_alignment_estimator.hpp"

namespace {

odometry_supervisor::AlignmentSample sample(std::int64_t timestamp,
                                             const Eigen::Vector3d& px4_position,
                                             double yaw = 0.0) {
  odometry_supervisor::AlignmentSample result;
  result.timestamp_ns = timestamp;
  result.px4_position = px4_position;
  result.px4_orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond target_from_source(
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()));
  result.lio_position = target_from_source * px4_position + Eigen::Vector3d(4.0, -2.0, 1.5);
  result.lio_orientation = target_from_source * result.px4_orientation;
  result.lio_generation = 3;
  result.px4_reset_generation = 7;
  result.px4_time_generation = 2;
  result.lio_tracking = true;
  result.px4_continuity_valid = true;
  result.yaw_authoritative = true;
  return result;
}

}  // namespace

TEST(OdometryAlignmentEstimator, RecoversKnownTranslationAndYawWithCircularMean) {
  odometry_supervisor::OdometryAlignmentEstimator estimator;
  for (int index = 0; index < 10; ++index) {
    ASSERT_TRUE(estimator.addSample(sample(
        1'000'000'000 + index * 20'000'000,
        Eigen::Vector3d(0.1 * index, 0.05 * index, 0.2))));
  }
  const auto result = estimator.estimate();
  ASSERT_TRUE(result.valid()) << result.rejection_reason;
  EXPECT_NEAR(result.alignment.yaw_rad, 0.35, 1e-12);
  EXPECT_TRUE(result.alignment.target_from_source_translation.isApprox(
      Eigen::Vector3d(4.0, -2.0, 1.5), 1e-12));
  EXPECT_EQ(result.alignment.sample_count, 10U);
  EXPECT_TRUE(result.covariance.allFinite());
}

TEST(OdometryAlignmentEstimator, RejectsRollPitchInsteadOfAbsorbingIt) {
  odometry_supervisor::OdometryAlignmentEstimator estimator;
  for (int index = 0; index < 8; ++index) {
    auto value = sample(1'000'000'000 + index * 20'000'000,
                        Eigen::Vector3d(0.2 * index, 0.1 * index, 0.0));
    value.lio_orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())) *
        value.px4_orientation;
    ASSERT_TRUE(estimator.addSample(value));
  }
  const auto result = estimator.estimate();
  EXPECT_EQ(result.status,
            odometry_supervisor::AlignmentEstimateStatus::kRollPitchDisagreement);
  EXPECT_FALSE(result.valid());
}

TEST(OdometryAlignmentEstimator, RejectsStationaryYawWhenNoAuthoritativeSourceExists) {
  odometry_supervisor::OdometryAlignmentEstimatorConfig config;
  config.minimum_horizontal_excitation_m = 0.2;
  odometry_supervisor::OdometryAlignmentEstimator estimator(config);
  for (int index = 0; index < 8; ++index) {
    auto value = sample(1'000'000'000 + index * 20'000'000,
                        Eigen::Vector3d::Zero());
    value.yaw_authoritative = false;
    ASSERT_TRUE(estimator.addSample(value));
  }
  const auto result = estimator.estimate();
  EXPECT_EQ(result.status,
            odometry_supervisor::AlignmentEstimateStatus::kYawUnobservable);
}

TEST(OdometryAlignmentEstimator, GenerationChangeClearsWindow) {
  odometry_supervisor::OdometryAlignmentEstimator estimator;
  for (int index = 0; index < 8; ++index) {
    ASSERT_TRUE(estimator.addSample(sample(
        1'000'000'000 + index * 20'000'000,
        Eigen::Vector3d(0.2 * index, 0.0, 0.0))));
  }
  auto changed = sample(1'200'000'000, Eigen::Vector3d(1.0, 0.0, 0.0));
  changed.lio_generation = 4;
  ASSERT_TRUE(estimator.addSample(changed));
  EXPECT_EQ(estimator.size(), 1U);
  EXPECT_EQ(estimator.estimate().status,
            odometry_supervisor::AlignmentEstimateStatus::kInsufficientSamples);
}

TEST(OdometryAlignmentEstimator, QuaternionSignDoesNotChangeEstimate) {
  odometry_supervisor::OdometryAlignmentEstimator estimator;
  for (int index = 0; index < 8; ++index) {
    auto value = sample(1'000'000'000 + index * 20'000'000,
                        Eigen::Vector3d(0.2 * index, 0.03 * index, 0.0));
    if (index % 2 == 0) {
      value.lio_orientation.coeffs() *= -1.0;
      value.px4_orientation.coeffs() *= -1.0;
    }
    ASSERT_TRUE(estimator.addSample(value));
  }
  const auto result = estimator.estimate();
  ASSERT_TRUE(result.valid()) << result.rejection_reason;
  EXPECT_NEAR(result.alignment.yaw_rad, 0.35, 1e-12);
}

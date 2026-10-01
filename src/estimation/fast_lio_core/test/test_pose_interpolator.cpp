#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <numbers>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/pose_interpolator.hpp"

namespace uav::nav::lio {
namespace {

TEST(PoseInterpolatorTest, InterpolatesTranslationAndRotation) {
  const PoseStamped lower{Timestamp(0, ClockDomain::kSensorTime),
                          RigidTransform(lioOdomFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d::Zero())};
  const PoseStamped upper{Timestamp(1'000, ClockDomain::kSensorTime),
                          RigidTransform(lioOdomFrame(), imuFrame(),
                                         Eigen::Quaterniond(Eigen::AngleAxisd(
                                             std::numbers::pi, Eigen::Vector3d::UnitZ())),
                                         Eigen::Vector3d(2.0, 4.0, 6.0))};
  const auto result =
      PoseInterpolator::interpolate(lower, upper, Timestamp(500, ClockDomain::kSensorTime));
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_TRUE(result.value().translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-12));
  EXPECT_TRUE(result.value()
                  .rotation()
                  .operator*(Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(PoseInterpolatorTest, RejectsExtrapolation) {
  const PoseStamped lower{Timestamp(0, ClockDomain::kSensorTime),
                          RigidTransform::Identity(imuFrame())};
  const PoseStamped upper{Timestamp(100, ClockDomain::kSensorTime),
                          RigidTransform::Identity(imuFrame())};
  const auto result =
      PoseInterpolator::interpolate(lower, upper, Timestamp(101, ClockDomain::kSensorTime));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kOutOfRange);
}

TEST(PoseInterpolatorTest, RejectsDifferentPoseDirections) {
  const PoseStamped lower{Timestamp(0, ClockDomain::kSensorTime),
                          RigidTransform(lioOdomFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d::Zero())};
  const PoseStamped upper{Timestamp(100, ClockDomain::kSensorTime),
                          RigidTransform(lioOdomFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d::Zero())};
  const auto result =
      PoseInterpolator::interpolate(lower, upper, Timestamp(50, ClockDomain::kSensorTime));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kFrameMismatch);
}

TEST(PoseInterpolatorTest, RejectsTimestampBracketWhoseDifferenceOverflows) {
  const PoseStamped lower{Timestamp(std::numeric_limits<std::int64_t>::min(),
                                    ClockDomain::kSensorTime),
                          RigidTransform::Identity(imuFrame())};
  const PoseStamped upper{Timestamp(std::numeric_limits<std::int64_t>::max(),
                                    ClockDomain::kSensorTime),
                          RigidTransform::Identity(imuFrame())};
  const auto result =
      PoseInterpolator::interpolate(lower, upper, Timestamp(0, ClockDomain::kSensorTime));
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kOutOfRange);
}

}  // namespace
}  // namespace uav::nav::lio

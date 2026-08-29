#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <numbers>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/navigation/rigid_body_state.hpp"

namespace uav::nav::lio {
namespace {

TEST(RigidTransformTest, AppliesTargetFromSourceDirection) {
  const RigidTransform T_imu_lidar(
      imuFrame(), lidarFrame(),
      Eigen::Quaterniond(Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(1.0, 0.0, 0.0));
  const Eigen::Vector3d point_imu = T_imu_lidar.apply(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(point_imu.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0), 1e-12));
}

TEST(RigidTransformTest, CompositionAndInversePreserveFrameDirection) {
  const RigidTransform T_odom_imu(lioOdomFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                                  Eigen::Vector3d(5.0, 0.0, 0.0));
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d(0.0, 0.0, 1.0));
  const auto T_odom_lidar = T_odom_imu.compose(T_imu_lidar);
  ASSERT_TRUE(T_odom_lidar.ok());
  EXPECT_EQ(T_odom_lidar.value().targetFrame(), lioOdomFrame());
  EXPECT_EQ(T_odom_lidar.value().sourceFrame(), lidarFrame());

  const Eigen::Vector3d point_lidar(2.0, 3.0, 4.0);
  const Eigen::Vector3d point_odom = T_odom_lidar.value().apply(point_lidar);
  EXPECT_TRUE(T_odom_lidar.value().inverse().apply(point_odom).isApprox(point_lidar, 1e-12));
}

TEST(RigidTransformTest, RejectsInvalidCompositionDirection) {
  const RigidTransform T_odom_imu(lioOdomFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                                  Eigen::Vector3d::Zero());
  const RigidTransform T_base_lidar(baseFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                    Eigen::Vector3d::Zero());
  const auto result = T_odom_imu.compose(T_base_lidar);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kFrameMismatch);
}

TEST(RigidTransformTest, RejectsQuaternionWhoseSquaredNormOverflows) {
  const double huge = std::numeric_limits<double>::max();
  const Eigen::Quaterniond malformed(huge, huge, huge, huge);
  const auto result = RigidTransform::Create(lioOdomFrame(), imuFrame(), malformed,
                                             Eigen::Vector3d::Zero());
  EXPECT_FALSE(result.ok());

  RigidBodyState state;
  state.orientation_reference_body = malformed;
  EXPECT_FALSE(state.allFinite());
}

}  // namespace
}  // namespace uav::nav::lio

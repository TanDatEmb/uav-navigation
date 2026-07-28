#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"

namespace uav::nav::lio {
namespace {

TEST(FrameConventionsTest, ExtrinsicIsImuTargetLidarSource) {
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d(0.1, -0.2, 0.3));
  EXPECT_EQ(T_imu_lidar.targetFrame().name(), "imu_link");
  EXPECT_EQ(T_imu_lidar.sourceFrame().name(), "lidar_link");
  EXPECT_TRUE(
      T_imu_lidar.apply(Eigen::Vector3d::Zero()).isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12));
}

TEST(FrameConventionsTest, FluBasisIsNotSilentlySwapped) {
  const RigidTransform identity = RigidTransform::Identity(baseFrame());
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitZ()));
}

}  // namespace
}  // namespace uav::nav::lio

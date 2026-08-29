#include <gtest/gtest.h>

#include <array>

#include <Eigen/Geometry>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"

namespace uav::nav::lio {
namespace {

TEST(FrameConventionsTest, ExtrinsicIsImuTargetLidarSource) {
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d(0.1, -0.2, 0.3));
  EXPECT_EQ(T_imu_lidar.targetFrame().name(), "livox_imu_frame");
  EXPECT_EQ(T_imu_lidar.sourceFrame().name(), "livox_frame");
  EXPECT_TRUE(
      T_imu_lidar.apply(Eigen::Vector3d::Zero()).isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12));
}

TEST(FrameConventionsTest, FactoryNominalLivoxExtrinsicHasExactInverse) {
  const Eigen::Vector3d factory_translation{0.011, 0.02329, -0.04412};
  const RigidTransform T_L_I(
      lidarFrame(), imuFrame(), Eigen::Quaterniond::Identity(), factory_translation);
  const auto T_I_L_result = T_L_I.inverse();
  ASSERT_TRUE(T_I_L_result.ok());
  const RigidTransform& T_I_L = T_I_L_result.value();

  EXPECT_EQ(T_L_I.targetFrame().name(), "livox_frame");
  EXPECT_EQ(T_L_I.sourceFrame().name(), "livox_imu_frame");
  EXPECT_EQ(T_I_L.targetFrame().name(), "livox_imu_frame");
  EXPECT_EQ(T_I_L.sourceFrame().name(), "livox_frame");

  const std::array<Eigen::Vector3d, 5> points_lidar{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(0.37, -1.2, 2.05)};
  for (const Eigen::Vector3d& point_lidar : points_lidar) {
    const Eigen::Vector3d point_imu = T_I_L.apply(point_lidar);
    EXPECT_TRUE(T_L_I.apply(point_imu).isApprox(point_lidar, 1e-12));
    EXPECT_TRUE(T_I_L.apply(T_L_I.apply(point_lidar)).isApprox(point_lidar, 1e-12));
  }

  EXPECT_TRUE(T_I_L.apply(Eigen::Vector3d::Zero()).isApprox(
      Eigen::Vector3d(-0.011, -0.02329, 0.04412), 1e-12));
}

TEST(FrameConventionsTest, FluBasisIsNotSilentlySwapped) {
  const RigidTransform identity = RigidTransform::Identity(baseFrame());
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(identity.apply(Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(FrameConventionsTest, Mid360DatasetExtrinsicPreservesBasisAndInverse) {
  const Eigen::Vector3d translation{-0.019391, -0.000278, 0.080926};
  const RigidTransform T_imu_lidar(
      imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(), translation);
  for (const Eigen::Vector3d basis :
       {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()}) {
    const Eigen::Vector3d point_imu = T_imu_lidar.apply(basis);
    EXPECT_TRUE(point_imu.isApprox(basis + translation, 1e-12));
    EXPECT_TRUE(
        T_imu_lidar.inverse().value().apply(point_imu).isApprox(basis, 1e-12));
  }

  const RigidTransform T_odom_imu(
      lioOdomFrame(), imuFrame(),
      Eigen::Quaterniond(
          Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(1.0, 2.0, 3.0));
  const auto T_odom_lidar = T_odom_imu.compose(T_imu_lidar);
  ASSERT_TRUE(T_odom_lidar.ok());
  const Eigen::Vector3d point_lidar{0.4, -0.7, 1.1};
  EXPECT_TRUE(T_odom_lidar.value().apply(point_lidar).isApprox(
      T_odom_imu.apply(T_imu_lidar.apply(point_lidar)), 1e-12));
}

}  // namespace
}  // namespace uav::nav::lio

#include <gtest/gtest.h>

#include "rog_map_ros/frame_transform.hpp"

namespace uav::nav::rog {
constexpr double kPi = 3.14159265358979323846;

TEST(FrameTransformTest, IdentityPosePreservesWallCoordinates) {
  const auto pose = composeSensorPose(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
                                      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  EXPECT_TRUE(transformLidarPointToLioOdom(pose, Eigen::Vector3d{4.0, 2.0, 1.0}).isApprox(Eigen::Vector3d{4.0, 2.0, 1.0}));
}

TEST(FrameTransformTest, PositiveAndNegativeYawKeepWorldGeometry) {
  const Eigen::Vector3d point_lidar{2.0, 0.0, 0.0};
  const auto positive = composeSensorPose(
      Eigen::Quaterniond(Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  const auto negative = composeSensorPose(
      Eigen::Quaterniond(Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  EXPECT_TRUE(transformLidarPointToLioOdom(positive, point_lidar).isApprox(Eigen::Vector3d{0.0, 2.0, 0.0}));
  EXPECT_TRUE(transformLidarPointToLioOdom(negative, point_lidar).isApprox(Eigen::Vector3d{0.0, -2.0, 0.0}));
}

TEST(FrameTransformTest, ExtrinsicRotationAndTranslationAreAppliedInOrder) {
  const auto pose = composeSensorPose(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d{10.0, 20.0, 30.0},
      Eigen::Quaterniond(Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d{1.0, 2.0, 3.0});
  EXPECT_TRUE(transformLidarPointToLioOdom(pose, Eigen::Vector3d{2.0, 0.0, 0.0}).isApprox(Eigen::Vector3d{11.0, 24.0, 33.0}));
}

}  // namespace uav::nav::rog

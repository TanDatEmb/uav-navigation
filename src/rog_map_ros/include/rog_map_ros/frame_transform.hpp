#pragma once

#include <Eigen/Geometry>

namespace uav::nav::rog {

struct SensorPoseInLioOdom {
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

[[nodiscard]] inline SensorPoseInLioOdom composeSensorPose(
    const Eigen::Quaterniond& q_odom_child,
    const Eigen::Vector3d& p_odom_child,
    const Eigen::Quaterniond& q_child_lidar,
    const Eigen::Vector3d& p_child_lidar) {
  return {q_odom_child * q_child_lidar,
          p_odom_child + q_odom_child * p_child_lidar};
}

[[nodiscard]] inline Eigen::Vector3d transformLidarPointToLioOdom(
    const SensorPoseInLioOdom& sensor_pose, const Eigen::Vector3d& point_lidar) {
  return sensor_pose.translation + sensor_pose.rotation * point_lidar;
}

}  // namespace uav::nav::rog

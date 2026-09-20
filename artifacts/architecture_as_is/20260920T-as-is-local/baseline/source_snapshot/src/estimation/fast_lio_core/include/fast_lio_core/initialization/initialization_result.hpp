#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

namespace uav::nav::lio {

struct InitializationQuality {
  std::size_t samples_collected{0};
  Eigen::Vector3d gyro_mean_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_variance_rad2_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_mean_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_variance_m2_s4{Eigen::Vector3d::Zero()};
  double measured_gravity_norm_m_s2{0.0};
  bool stationary{false};
};

struct InitializationResult {
  Eigen::Quaterniond orientation_odom_imu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d gyro_bias_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_odom_m_s2{Eigen::Vector3d::Zero()};
  InitializationQuality quality;
};

}  // namespace uav::nav::lio

#include "fast_lio_core/estimation/manifold_state.hpp"

#include <cmath>

namespace uav::nav::lio {
namespace {

bool finiteQuaternion(const Eigen::Quaterniond& value) {
  return value.coeffs().allFinite() && std::isfinite(value.squaredNorm()) &&
         value.squaredNorm() > 1e-18;
}

}  // namespace

const Eigen::Vector3d& ManifoldState::position_odom_imu_m() const noexcept {
  return position_odom_imu_m_;
}

const Eigen::Quaterniond& ManifoldState::orientation_odom_imu() const noexcept {
  return orientation_odom_imu_;
}

const Eigen::Quaterniond& ManifoldState::rotation_imu_lidar() const noexcept {
  return rotation_imu_lidar_;
}

const Eigen::Vector3d& ManifoldState::position_imu_lidar_m() const noexcept {
  return position_imu_lidar_m_;
}

const Eigen::Vector3d& ManifoldState::velocity_odom_imu_m_s() const noexcept {
  return velocity_odom_imu_m_s_;
}

const Eigen::Vector3d& ManifoldState::gyro_bias_rad_s() const noexcept { return gyro_bias_rad_s_; }

const Eigen::Vector3d& ManifoldState::accel_bias_m_s2() const noexcept { return accel_bias_m_s2_; }

const Eigen::Vector3d& ManifoldState::gravity_odom_m_s2() const noexcept {
  return gravity_odom_m_s2_;
}

void ManifoldState::set_position_odom_imu_m(const Eigen::Vector3d& value) noexcept {
  position_odom_imu_m_ = value;
}

void ManifoldState::set_orientation_odom_imu(const Eigen::Quaterniond& value) {
  orientation_odom_imu_ = value;
  if (finiteQuaternion(orientation_odom_imu_)) {
    orientation_odom_imu_.normalize();
  }
}

void ManifoldState::set_rotation_imu_lidar(const Eigen::Quaterniond& value) {
  rotation_imu_lidar_ = value;
  if (finiteQuaternion(rotation_imu_lidar_)) {
    rotation_imu_lidar_.normalize();
  }
}

void ManifoldState::set_position_imu_lidar_m(const Eigen::Vector3d& value) noexcept {
  position_imu_lidar_m_ = value;
}

void ManifoldState::set_velocity_odom_imu_m_s(const Eigen::Vector3d& value) noexcept {
  velocity_odom_imu_m_s_ = value;
}

void ManifoldState::set_gyro_bias_rad_s(const Eigen::Vector3d& value) noexcept {
  gyro_bias_rad_s_ = value;
}

void ManifoldState::set_accel_bias_m_s2(const Eigen::Vector3d& value) noexcept {
  accel_bias_m_s2_ = value;
}

void ManifoldState::set_gravity_odom_m_s2(const Eigen::Vector3d& value) noexcept {
  gravity_odom_m_s2_ = value;
}

Eigen::Vector3d ManifoldState::transformLidarPointToOdom(
    const Eigen::Vector3d& point_lidar_m) const noexcept {
  const Eigen::Vector3d point_imu_m = rotation_imu_lidar_ * point_lidar_m + position_imu_lidar_m_;
  return orientation_odom_imu_ * point_imu_m + position_odom_imu_m_;
}

void ManifoldState::normalize() {
  if (finiteQuaternion(orientation_odom_imu_)) {
    orientation_odom_imu_.normalize();
  }
  if (finiteQuaternion(rotation_imu_lidar_)) {
    rotation_imu_lidar_.normalize();
  }
}

bool ManifoldState::allFinite() const noexcept {
  return position_odom_imu_m_.allFinite() && finiteQuaternion(orientation_odom_imu_) &&
         finiteQuaternion(rotation_imu_lidar_) && position_imu_lidar_m_.allFinite() &&
         velocity_odom_imu_m_s_.allFinite() && gyro_bias_rad_s_.allFinite() &&
         accel_bias_m_s2_.allFinite() && gravity_odom_m_s2_.allFinite() &&
         std::isfinite(gravity_odom_m_s2_.squaredNorm()) &&
         gravity_odom_m_s2_.squaredNorm() > 1e-18;
}

}  // namespace uav::nav::lio

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

namespace uav::nav::lio {

// FAST-LIO2/IKFoM-compatible nominal state. Gravity is represented on S2,
// therefore the nominal state has dimension 24 and its tangent/error state has
// 23 degrees of freedom. This is deliberately not a custom 15-state model.
class ManifoldState {
 public:
  static constexpr int kErrorStateDimension = 23;
  static constexpr int kPositionOffset = 0;
  static constexpr int kOrientationOffset = 3;
  static constexpr int kExtrinsicRotationOffset = 6;
  static constexpr int kExtrinsicPositionOffset = 9;
  static constexpr int kVelocityOffset = 12;
  static constexpr int kGyroBiasOffset = 15;
  static constexpr int kAccelBiasOffset = 18;
  static constexpr int kGravityOffset = 21;

  using ErrorVector = Eigen::Matrix<double, kErrorStateDimension, 1>;
  using Covariance = Eigen::Matrix<double, kErrorStateDimension, kErrorStateDimension>;

  ManifoldState() = default;

  [[nodiscard]] const Eigen::Vector3d& position_odom_imu_m() const noexcept;
  [[nodiscard]] const Eigen::Quaterniond& orientation_odom_imu() const noexcept;
  [[nodiscard]] const Eigen::Quaterniond& rotation_imu_lidar() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& position_imu_lidar_m() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& velocity_odom_imu_m_s() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& gyro_bias_rad_s() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& accel_bias_m_s2() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& gravity_odom_m_s2() const noexcept;

  void set_position_odom_imu_m(const Eigen::Vector3d& value) noexcept;
  void set_orientation_odom_imu(const Eigen::Quaterniond& value);
  void set_rotation_imu_lidar(const Eigen::Quaterniond& value);
  void set_position_imu_lidar_m(const Eigen::Vector3d& value) noexcept;
  void set_velocity_odom_imu_m_s(const Eigen::Vector3d& value) noexcept;
  void set_gyro_bias_rad_s(const Eigen::Vector3d& value) noexcept;
  void set_accel_bias_m_s2(const Eigen::Vector3d& value) noexcept;
  void set_gravity_odom_m_s2(const Eigen::Vector3d& value) noexcept;

  [[nodiscard]] Eigen::Vector3d transformLidarPointToOdom(
      const Eigen::Vector3d& point_lidar_m) const noexcept;

  void normalize();
  [[nodiscard]] bool allFinite() const noexcept;

 private:
  Eigen::Vector3d position_odom_imu_m_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_odom_imu_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond rotation_imu_lidar_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_imu_lidar_m_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_odom_imu_m_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_m_s2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_odom_m_s2_{Eigen::Vector3d(0.0, 0.0, -9.80665)};
};

}  // namespace uav::nav::lio

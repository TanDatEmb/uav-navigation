#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"

namespace uav::nav::lio {

struct ImuNoise {
  double gyro_noise_std_rad_s_sqrt_hz{1.5e-3};
  double accel_noise_std_m_s2_sqrt_hz{2.0e-2};
  double gyro_bias_random_walk_std_rad_s2_sqrt_hz{1.0e-5};
  double accel_bias_random_walk_std_m_s3_sqrt_hz{1.0e-4};
};

// Euclidean covariance transport around the IKFoM-compatible nominal state.
// SO(3) state updates remain owned by ManifoldState; this class does not
// implement boxplus, boxminus, exp/log, or manifold Jacobians.
class ProcessModel {
 public:
  [[nodiscard]] static Status propagateCovariance(
      ManifoldState::Covariance& covariance, const Eigen::Quaterniond& orientation_odom_imu,
      const Eigen::Vector3d& corrected_accel_imu_m_s2, double dt_s, const ImuNoise& noise,
      const Eigen::Matrix<double, 3, 2>& gravity_tangent_basis_odom);
};

}  // namespace uav::nav::lio

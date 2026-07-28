#include "fast_lio_core/estimation/process_model.hpp"

#include <cmath>

namespace uav::nav::lio {
namespace {

[[nodiscard]] Eigen::Matrix3d skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(),
      0.0;
  return matrix;
}

[[nodiscard]] bool validNoise(const ImuNoise& noise) {
  return std::isfinite(noise.gyro_noise_std_rad_s_sqrt_hz) &&
         std::isfinite(noise.accel_noise_std_m_s2_sqrt_hz) &&
         std::isfinite(noise.gyro_bias_random_walk_std_rad_s2_sqrt_hz) &&
         std::isfinite(noise.accel_bias_random_walk_std_m_s3_sqrt_hz) &&
         noise.gyro_noise_std_rad_s_sqrt_hz >= 0.0 && noise.accel_noise_std_m_s2_sqrt_hz >= 0.0 &&
         noise.gyro_bias_random_walk_std_rad_s2_sqrt_hz >= 0.0 &&
         noise.accel_bias_random_walk_std_m_s3_sqrt_hz >= 0.0;
}

}  // namespace

Status ProcessModel::propagateCovariance(
    ManifoldState::Covariance& covariance, const Eigen::Quaterniond& orientation_odom_imu,
    const Eigen::Vector3d& corrected_accel_imu_m_s2, double dt_s, const ImuNoise& noise,
    const Eigen::Matrix<double, 3, 2>& gravity_tangent_basis_odom) {
  if (!covariance.allFinite() || !orientation_odom_imu.coeffs().allFinite() ||
      !corrected_accel_imu_m_s2.allFinite() || !std::isfinite(dt_s) || dt_s < 0.0 ||
      !validNoise(noise) || !gravity_tangent_basis_odom.allFinite()) {
    return Status(StatusCode::kInvalidArgument, "Invalid process-model input");
  }
  if (dt_s == 0.0) {
    return Status::Ok();
  }

  const Eigen::Matrix3d rotation = orientation_odom_imu.normalized().toRotationMatrix();
  ManifoldState::Covariance transition = ManifoldState::Covariance::Identity();
  transition.block<3, 3>(ManifoldState::kPositionOffset, ManifoldState::kVelocityOffset) =
      Eigen::Matrix3d::Identity() * dt_s;
  transition.block<3, 3>(ManifoldState::kOrientationOffset, ManifoldState::kGyroBiasOffset) =
      -Eigen::Matrix3d::Identity() * dt_s;
  transition.block<3, 3>(ManifoldState::kVelocityOffset, ManifoldState::kOrientationOffset) =
      -rotation * skew(corrected_accel_imu_m_s2) * dt_s;
  transition.block<3, 3>(ManifoldState::kVelocityOffset, ManifoldState::kAccelBiasOffset) =
      -rotation * dt_s;
  transition.block<3, 3>(ManifoldState::kPositionOffset, ManifoldState::kOrientationOffset) =
      -0.5 * rotation * skew(corrected_accel_imu_m_s2) * dt_s * dt_s;
  transition.block<3, 3>(ManifoldState::kPositionOffset, ManifoldState::kAccelBiasOffset) =
      -0.5 * rotation * dt_s * dt_s;
  transition.block<3, 2>(ManifoldState::kVelocityOffset, ManifoldState::kGravityOffset) =
      gravity_tangent_basis_odom * dt_s;
  transition.block<3, 2>(ManifoldState::kPositionOffset, ManifoldState::kGravityOffset) =
      0.5 * gravity_tangent_basis_odom * dt_s * dt_s;

  ManifoldState::Covariance process_noise = ManifoldState::Covariance::Zero();
  const double gyro_variance =
      noise.gyro_noise_std_rad_s_sqrt_hz * noise.gyro_noise_std_rad_s_sqrt_hz;
  const double accel_variance =
      noise.accel_noise_std_m_s2_sqrt_hz * noise.accel_noise_std_m_s2_sqrt_hz;
  const double gyro_bias_variance = noise.gyro_bias_random_walk_std_rad_s2_sqrt_hz *
                                    noise.gyro_bias_random_walk_std_rad_s2_sqrt_hz;
  const double accel_bias_variance =
      noise.accel_bias_random_walk_std_m_s3_sqrt_hz * noise.accel_bias_random_walk_std_m_s3_sqrt_hz;
  process_noise.block<3, 3>(ManifoldState::kOrientationOffset, ManifoldState::kOrientationOffset) =
      Eigen::Matrix3d::Identity() * gyro_variance * dt_s;
  process_noise.block<3, 3>(ManifoldState::kVelocityOffset, ManifoldState::kVelocityOffset) =
      Eigen::Matrix3d::Identity() * accel_variance * dt_s;
  process_noise.block<3, 3>(ManifoldState::kPositionOffset, ManifoldState::kPositionOffset) =
      Eigen::Matrix3d::Identity() * accel_variance * dt_s * dt_s * dt_s / 3.0;
  process_noise.block<3, 3>(ManifoldState::kGyroBiasOffset, ManifoldState::kGyroBiasOffset) =
      Eigen::Matrix3d::Identity() * gyro_bias_variance * dt_s;
  process_noise.block<3, 3>(ManifoldState::kAccelBiasOffset, ManifoldState::kAccelBiasOffset) =
      Eigen::Matrix3d::Identity() * accel_bias_variance * dt_s;

  covariance = transition * covariance * transition.transpose() + process_noise;
  covariance = 0.5 * (covariance + covariance.transpose());
  if (!covariance.allFinite()) {
    return Status(StatusCode::kNumericalFailure, "Covariance became non-finite during propagation");
  }
  return Status::Ok();
}

}  // namespace uav::nav::lio

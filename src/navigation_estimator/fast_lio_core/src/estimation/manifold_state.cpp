#include "fast_lio_core/estimation/manifold_state.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace uav::nav::lio {
namespace {

Eigen::Quaterniond retractSo3(const Eigen::Quaterniond& orientation,
                              const Eigen::Vector3d& tangent) {
  const double angle = tangent.norm();
  Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
  if (angle > 1e-12) {
    delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, tangent / angle));
  } else {
    // The first-order quaternion avoids an unstable axis normalization while
    // delegating quaternion composition and normalization to Eigen.
    delta = Eigen::Quaterniond(1.0, 0.5 * tangent.x(), 0.5 * tangent.y(), 0.5 * tangent.z());
    delta.normalize();
  }
  return (orientation * delta).normalized();
}

Eigen::Vector3d so3Difference(const Eigen::Quaterniond& value,
                              const Eigen::Quaterniond& reference) {
  Eigen::Quaterniond difference = reference.conjugate() * value;
  difference.normalize();
  if (difference.w() < 0.0) {
    difference.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(difference);
  if (!std::isfinite(angle_axis.angle()) || std::abs(angle_axis.angle()) < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix<double, 3, 2> makeGravityTangentBasis(const Eigen::Vector3d& gravity) {
  Eigen::Vector3d direction = gravity.normalized();
  if (!direction.allFinite() || direction.squaredNorm() < 0.5) {
    direction = Eigen::Vector3d(0.0, 0.0, -1.0);
  }
  const Eigen::Vector3d first = direction.unitOrthogonal().normalized();
  const Eigen::Vector3d second = direction.cross(first).normalized();
  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = first;
  basis.col(1) = second;
  return basis;
}

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

void ManifoldState::applyErrorState(const ErrorVector& increment, bool estimate_extrinsic) {
  position_odom_imu_m_ += increment.segment<3>(kPositionOffset);
  orientation_odom_imu_ =
      retractSo3(orientation_odom_imu_, increment.segment<3>(kOrientationOffset));
  if (estimate_extrinsic) {
    rotation_imu_lidar_ =
        retractSo3(rotation_imu_lidar_, increment.segment<3>(kExtrinsicRotationOffset));
    position_imu_lidar_m_ += increment.segment<3>(kExtrinsicPositionOffset);
  }
  velocity_odom_imu_m_s_ += increment.segment<3>(kVelocityOffset);
  gyro_bias_rad_s_ += increment.segment<3>(kGyroBiasOffset);
  accel_bias_m_s2_ += increment.segment<3>(kAccelBiasOffset);

  const double gravity_norm = gravity_odom_m_s2_.norm();
  const double safe_norm = gravity_norm > 1e-9 ? gravity_norm : 9.80665;
  const auto basis = makeGravityTangentBasis(gravity_odom_m_s2_);
  const Eigen::Vector3d updated_gravity =
      gravity_odom_m_s2_ + basis * increment.segment<2>(kGravityOffset);
  if (updated_gravity.allFinite() && updated_gravity.norm() > 1e-9) {
    gravity_odom_m_s2_ = safe_norm * updated_gravity.normalized();
  }
  normalize();
}

ManifoldState::ErrorVector ManifoldState::boxMinus(const ManifoldState& reference,
                                                   bool estimate_extrinsic) const {
  ErrorVector difference = ErrorVector::Zero();
  difference.segment<3>(kPositionOffset) = position_odom_imu_m_ - reference.position_odom_imu_m_;
  difference.segment<3>(kOrientationOffset) =
      so3Difference(orientation_odom_imu_, reference.orientation_odom_imu_);
  if (estimate_extrinsic) {
    difference.segment<3>(kExtrinsicRotationOffset) =
        so3Difference(rotation_imu_lidar_, reference.rotation_imu_lidar_);
    difference.segment<3>(kExtrinsicPositionOffset) =
        position_imu_lidar_m_ - reference.position_imu_lidar_m_;
  }
  difference.segment<3>(kVelocityOffset) =
      velocity_odom_imu_m_s_ - reference.velocity_odom_imu_m_s_;
  difference.segment<3>(kGyroBiasOffset) = gyro_bias_rad_s_ - reference.gyro_bias_rad_s_;
  difference.segment<3>(kAccelBiasOffset) = accel_bias_m_s2_ - reference.accel_bias_m_s2_;
  difference.segment<2>(kGravityOffset) =
      makeGravityTangentBasis(reference.gravity_odom_m_s2_).transpose() *
      (gravity_odom_m_s2_ - reference.gravity_odom_m_s2_);
  return difference;
}

Eigen::Vector3d ManifoldState::transformLidarPointToOdom(
    const Eigen::Vector3d& point_lidar_m) const noexcept {
  const Eigen::Vector3d point_imu_m = rotation_imu_lidar_ * point_lidar_m + position_imu_lidar_m_;
  return orientation_odom_imu_ * point_imu_m + position_odom_imu_m_;
}

Eigen::Matrix<double, 3, 2> ManifoldState::gravityTangentBasis() const {
  return makeGravityTangentBasis(gravity_odom_m_s2_);
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
         gravity_odom_m_s2_.norm() > 1e-9;
}

}  // namespace uav::nav::lio

#include "fast_lio_core/initialization/initial_state_prior_applicator.hpp"

#include <algorithm>
#include <cmath>

#include "fast_lio_core/geometry/frame_ids.hpp"

namespace uav::nav::lio {
namespace {

constexpr double kGeometryTolerance = 1e-12;

double wrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace

InitialStatePriorApplicator::InitialStatePriorApplicator(RigidTransform base_to_imu)
    : base_to_imu_(std::move(base_to_imu)) {}

Status InitialStatePriorApplicator::setGeometry(RigidTransform base_to_imu) {
  if (base_to_imu.targetFrame() != baseFrame() || base_to_imu.sourceFrame() != imuFrame() ||
      !base_to_imu.allFinite()) {
    return Status(StatusCode::kFrameMismatch, "initial prior geometry must be ^base_link T_livox_imu_frame");
  }
  base_to_imu_ = std::move(base_to_imu);
  return Status::Ok();
}

Status InitialStatePriorApplicator::apply(
    const InitialStatePrior& prior, const ManifoldState& imu_initialized_state,
    double maximum_full_attitude_tilt_disagreement_rad, ManifoldState& output) const {
  if (prior.reference_frame != lioOdomFrame() || prior.body_frame != baseFrame()) {
    return Status(StatusCode::kFrameMismatch, "initial prior must use lio_odom -> base_link semantics");
  }
  if (!prior.allFinite() || !imu_initialized_state.allFinite() ||
      !std::isfinite(maximum_full_attitude_tilt_disagreement_rad) ||
      maximum_full_attitude_tilt_disagreement_rad < 0.0) {
    return Status(StatusCode::kInvalidArgument, "initial prior contains invalid values");
  }
  if (!base_to_imu_.allFinite() || base_to_imu_.targetFrame() != baseFrame() ||
      base_to_imu_.sourceFrame() != imuFrame()) {
    return Status(StatusCode::kFrameMismatch, "initial prior geometry frame contract is invalid");
  }

  const Eigen::Matrix3d R_BI = base_to_imu_.rotation().toRotationMatrix();
  const Eigen::Matrix3d R_IB = R_BI.transpose();
  const Eigen::Vector3d r_BI = base_to_imu_.translation();
  const Eigen::Matrix3d R_OI_imu = imu_initialized_state.orientation_odom_imu().toRotationMatrix();
  const Eigen::Matrix3d R_OB_imu = R_OI_imu * R_IB;
  Eigen::Matrix3d R_OB = R_OB_imu;

  switch (prior.mask.attitude) {
    case PriorAttitudeMode::kNone:
      break;
    case PriorAttitudeMode::kYawOnly: {
      const double current_yaw = std::atan2(R_OB_imu(1, 0), R_OB_imu(0, 0));
      const Eigen::Matrix3d R_OB_prior = prior.orientation_odom_base.toRotationMatrix();
      const double target_yaw = std::atan2(R_OB_prior(1, 0), R_OB_prior(0, 0));
      R_OB = Eigen::AngleAxisd(wrapAngle(target_yaw - current_yaw), Eigen::Vector3d::UnitZ()) *
             R_OB_imu;
      break;
    }
    case PriorAttitudeMode::kFull: {
      const Eigen::Matrix3d R_OB_prior = prior.orientation_odom_base.toRotationMatrix();
      const Eigen::Vector3d gravity = imu_initialized_state.gravity_odom_m_s2().normalized();
      const Eigen::Vector3d gravity_base_imu = R_OB_imu.transpose() * gravity;
      const Eigen::Vector3d gravity_base_prior = R_OB_prior.transpose() * gravity;
      const double disagreement = std::acos(std::clamp(gravity_base_imu.dot(gravity_base_prior), -1.0, 1.0));
      if (disagreement > maximum_full_attitude_tilt_disagreement_rad) {
        return Status(StatusCode::kInitializationRejected, "full prior tilt disagrees with IMU gravity");
      }
      R_OB = R_OB_prior;
      break;
    }
  }

  output = imu_initialized_state;
  output.set_orientation_odom_imu(Eigen::Quaterniond(R_OB * R_BI));
  if (prior.mask.position) {
    output.set_position_odom_imu_m(prior.position_odom_base_m + R_OB * r_BI);
  }
  if (prior.mask.velocity) {
    if (!prior.linear_velocity_base_m_s.has_value() ||
        (!prior.angular_velocity_base_rad_s.has_value() && r_BI.norm() > kGeometryTolerance)) {
      return Status(StatusCode::kInvalidArgument, "velocity prior requires linear and angular base velocity");
    }
    const Eigen::Vector3d omega = prior.angular_velocity_base_rad_s.value_or(Eigen::Vector3d::Zero());
    output.set_velocity_odom_imu_m_s(R_OB * (*prior.linear_velocity_base_m_s) +
                                     R_OB * omega.cross(r_BI));
  }
  output.normalize();
  if (!output.allFinite()) {
    return Status(StatusCode::kNumericalFailure, "initial prior application produced non-finite state");
  }
  return Status::Ok();
}

}  // namespace uav::nav::lio

#include "fast_lio_core/navigation/base_link_state_converter.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "fast_lio_core/geometry/frame_ids.hpp"

namespace uav::nav::lio {
namespace {

[[nodiscard]] Status invalidArgument(std::string message) {
  return Status(StatusCode::kInvalidArgument, std::move(message));
}

}  // namespace

BaseLinkStateConverter::BaseLinkStateConverter(RigidTransform base_to_imu)
    : base_to_imu_(std::move(base_to_imu)),
      imu_to_base_(base_to_imu_.inverse()),
      r_base_imu_m_(base_to_imu_.translation()),
      R_base_imu_(base_to_imu_.rotation().toRotationMatrix()),
      source_frame_(imuFrame()),
      reference_frame_(lioOdomFrame()),
      body_frame_(baseFrame()) {
  validateBaseToImu(base_to_imu_);
}

Result<BaseLinkStateConverter> BaseLinkStateConverter::Create(
    RigidTransform base_to_imu) {
  try {
    return BaseLinkStateConverter(std::move(base_to_imu));
  } catch (const std::invalid_argument& error) {
    return invalidArgument(error.what());
  }
}

void BaseLinkStateConverter::validateBaseToImu(
    const RigidTransform& base_to_imu) {
  if (base_to_imu.targetFrame() != baseFrame() ||
      base_to_imu.sourceFrame() != imuFrame()) {
    throw std::invalid_argument(
        "BaseLinkStateConverter requires ^base_link T_livox_imu_frame");
  }
  if (!base_to_imu.allFinite() ||
      !base_to_imu.rotation().coeffs().allFinite() ||
      base_to_imu.rotation().norm() < 1e-12) {
    throw std::invalid_argument(
        "BaseLinkStateConverter received a non-finite or degenerate static transform");
  }
}

Result<RigidBodyState> BaseLinkStateConverter::convert(
    const StateEstimate& imu_estimate,
    const Eigen::Vector3d& angular_velocity_imu_rad_s) const {
  if (!imu_estimate.allFinite()) {
    return invalidArgument("StateEstimate is not finite");
  }
  if (!angular_velocity_imu_rad_s.allFinite()) {
    return invalidArgument("IMU angular velocity is not finite");
  }

  const auto& state = imu_estimate.state;
  const RigidTransform T_odom_imu(
      reference_frame_, source_frame_, state.orientation_odom_imu(),
      state.position_odom_imu_m());
  const Result<RigidTransform> T_odom_base_result =
      T_odom_imu.compose(imu_to_base_);
  if (!T_odom_base_result.ok()) {
    return T_odom_base_result.status();
  }

  const RigidTransform& T_odom_base = T_odom_base_result.value();
  const Eigen::Matrix3d R_odom_base = T_odom_base.rotation().toRotationMatrix();
  const Eigen::Vector3d omega_base =
      R_base_imu_ * angular_velocity_imu_rad_s;
  const Eigen::Vector3d lever_velocity_odom =
      R_odom_base * omega_base.cross(r_base_imu_m_);
  const Eigen::Vector3d velocity_odom_base =
      state.velocity_odom_imu_m_s() - lever_velocity_odom;

  RigidBodyState output;
  output.time = imu_estimate.time;
  output.source_frame = source_frame_;
  output.reference_frame = reference_frame_;
  output.body_frame = body_frame_;
  output.position_reference_body_m = T_odom_base.translation();
  output.orientation_reference_body = T_odom_base.rotation();
  output.linear_velocity_reference_body_m_s = velocity_odom_base;
  output.linear_velocity_body_m_s = R_odom_base.transpose() * velocity_odom_base;
  output.angular_velocity_body_rad_s = omega_base;
  if (!output.allFinite() ||
      std::abs(output.orientation_reference_body.norm() - 1.0) > 1e-12) {
    return invalidArgument("Base-link state conversion produced an invalid state");
  }
  return output;
}

const RigidTransform& BaseLinkStateConverter::baseToImu() const noexcept {
  return base_to_imu_;
}

const RigidTransform& BaseLinkStateConverter::imuToBase() const noexcept {
  return imu_to_base_;
}

}  // namespace uav::nav::lio

#pragma once

#include <Eigen/Core>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/navigation/rigid_body_state.hpp"
#include "fast_lio_core/pipeline/process_result.hpp"

namespace uav::nav::lio {

// Converts a StateEstimate whose reference point is livox_imu_frame into
// kinematics at base_link. The static transform is resolved by the caller and
// cached here; this class has no ROS, TF, YAML, Xacro, or PX4 dependency.
class BaseLinkStateConverter {
 public:
  explicit BaseLinkStateConverter(RigidTransform base_to_imu);

  [[nodiscard]] static Result<BaseLinkStateConverter> Create(
      RigidTransform base_to_imu);

  [[nodiscard]] Result<RigidBodyState> convert(
      const StateEstimate& imu_estimate,
      const Eigen::Vector3d& angular_velocity_imu_rad_s) const;

  [[nodiscard]] const RigidTransform& baseToImu() const noexcept;
  [[nodiscard]] const RigidTransform& imuToBase() const noexcept;

 private:
  static void validateBaseToImu(const RigidTransform& base_to_imu);

  RigidTransform base_to_imu_;
  RigidTransform imu_to_base_;
  Eigen::Vector3d r_base_imu_m_;
  Eigen::Matrix3d R_base_imu_;
  FrameId source_frame_;
  FrameId reference_frame_;
  FrameId body_frame_;
};

}  // namespace uav::nav::lio

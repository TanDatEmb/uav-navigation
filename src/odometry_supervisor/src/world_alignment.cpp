#include "odometry_supervisor/world_alignment.hpp"

#include <cmath>

namespace odometry_supervisor {

std::optional<OdometryState> applyWorldAlignment(
    const OdometryState& source, const WorldAlignment& alignment) {
  if (!alignment.valid || source.frame_id != alignment.source_frame ||
      source.child_frame_id != "base_link" || !source.valid ||
      !source.position_odom.allFinite() || !source.velocity_base.allFinite() ||
      !source.orientation_odom_base.coeffs().allFinite() ||
      !std::isfinite(source.orientation_odom_base.norm()) ||
      source.orientation_odom_base.norm() < 1e-9 ||
      !alignment.target_from_source_translation.allFinite() ||
      !alignment.target_from_source_orientation.coeffs().allFinite() ||
      !std::isfinite(alignment.target_from_source_orientation.norm()) ||
      alignment.target_from_source_orientation.norm() < 1e-9) {
    return std::nullopt;
  }

  OdometryState result = source;
  const auto rotation = alignment.target_from_source_orientation.normalized();
  const Eigen::Matrix3d world_rotation = rotation.toRotationMatrix();
  // World alignment is XYZ plus yaw. A full-SO(3) quaternion here would hide
  // roll/pitch disagreement instead of reporting it as a residual.
  if ((world_rotation.col(2) - Eigen::Vector3d::UnitZ()).norm() > 1e-9 ||
      !std::isfinite(alignment.yaw_rad)) {
    return std::nullopt;
  }
  result.frame_id = alignment.target_frame;
  result.position_odom = world_rotation * source.position_odom +
                         alignment.target_from_source_translation;
  result.orientation_odom_base =
      (rotation * source.orientation_odom_base.normalized()).normalized();
  if (!result.position_odom.allFinite() ||
      !result.orientation_odom_base.coeffs().allFinite()) {
    return std::nullopt;
  }
  return result;
}

}  // namespace odometry_supervisor

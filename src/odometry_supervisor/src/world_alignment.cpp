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
  result.frame_id = alignment.target_frame;
  result.position_odom = rotation * source.position_odom +
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

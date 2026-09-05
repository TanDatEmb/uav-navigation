#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <navigation_mapping/mapping_types.hpp>

namespace navigation_runtime {

[[nodiscard]] inline navigation_mapping::MappingObservationRejectionReason
classifySensorOriginContract(
    const bool sensor_origin_valid,
    const geometry_msgs::msg::Pose& sensor_origin) noexcept {
  if (!sensor_origin_valid) {
    return navigation_mapping::MappingObservationRejectionReason::kMissingSensorOrigin;
  }
  const auto& position = sensor_origin.position;
  const auto& orientation = sensor_origin.orientation;
  const double orientation_norm = std::sqrt(
      orientation.w * orientation.w + orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation_norm) ||
      orientation_norm <= 1.0e-9) {
    return navigation_mapping::MappingObservationRejectionReason::kSensorOriginContractMismatch;
  }
  return navigation_mapping::MappingObservationRejectionReason::kNone;
}

}  // namespace navigation_runtime

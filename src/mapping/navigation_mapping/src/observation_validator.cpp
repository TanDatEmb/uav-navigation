#include "navigation_mapping/observation_validator.hpp"

#include <cmath>

namespace navigation_mapping {

ObservationValidator::ObservationValidator(ObservationContract contract)
    : contract_(std::move(contract)) {}

ObservationValidationResult ObservationValidator::validateFrames(
    const std::string& header_frame_id, const std::string& points_frame_id,
    const builtin_interfaces::msg::Time& header_stamp,
    const builtin_interfaces::msg::Time& points_stamp) const {
  if (header_frame_id != contract_.odom_frame_id) {
    return {false, ObservationRejectionReason::kInvalidFrameId};
  }
  if (points_frame_id != contract_.lidar_frame_id) {
    return {false, ObservationRejectionReason::kInvalidPointsFrameId};
  }
  if (points_stamp.sec != header_stamp.sec ||
      points_stamp.nanosec != header_stamp.nanosec) {
    return {false, ObservationRejectionReason::kPointsStampMismatch};
  }
  return {true, ObservationRejectionReason::kNone};
}

ObservationValidationResult ObservationValidator::validatePose(
    const geometry_msgs::msg::Pose& pose) const {
  const bool position_finite = std::isfinite(pose.position.x) &&
                                std::isfinite(pose.position.y) &&
                                std::isfinite(pose.position.z);
  const bool orientation_finite =
      std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
  constexpr double kMinimumQuaternionNormSquared = 1e-6;
  const double quaternion_norm_squared =
      (pose.orientation.x * pose.orientation.x) + (pose.orientation.y * pose.orientation.y) +
      (pose.orientation.z * pose.orientation.z) + (pose.orientation.w * pose.orientation.w);
  if (!position_finite || !orientation_finite ||
      quaternion_norm_squared < kMinimumQuaternionNormSquared) {
    return {false, ObservationRejectionReason::kInvalidPose};
  }
  return {true, ObservationRejectionReason::kNone};
}

}  // namespace navigation_mapping

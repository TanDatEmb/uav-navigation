#pragma once

#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace navigation_mapping {

struct ObservationContract {
  std::string odom_frame_id{"lio_odom"};
  std::string lidar_frame_id{"livox_frame"};
};

enum class ObservationRejectionReason {
  kNone,
  kInvalidFrameId,
  kInvalidPointsFrameId,
  kPointsStampMismatch,
  kInvalidPose,
  kNonFinitePoints,
};

struct ObservationValidationResult {
  bool valid{false};
  ObservationRejectionReason reason{ObservationRejectionReason::kNone};
};

// P1 mandatory correctness test B (frame contract) and part of A (epoch
// contract): reject observations whose header/points frame or stamp does not
// match the contract, or whose pose is non-finite, before any transform or
// ROG update is attempted. See navigation_interfaces/msg/LidarMappingObservation.msg.
class ObservationValidator {
 public:
  explicit ObservationValidator(ObservationContract contract = {});

  [[nodiscard]] ObservationValidationResult validateFrames(
      const std::string& header_frame_id, const std::string& points_frame_id,
      const builtin_interfaces::msg::Time& header_stamp,
      const builtin_interfaces::msg::Time& points_stamp) const;

  [[nodiscard]] ObservationValidationResult validatePose(
      const geometry_msgs::msg::Pose& pose) const;

 private:
  ObservationContract contract_;
};

}  // namespace navigation_mapping

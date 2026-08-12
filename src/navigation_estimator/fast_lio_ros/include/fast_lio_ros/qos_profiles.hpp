#pragma once

#include <rclcpp/qos.hpp>

namespace uav::nav::lio {

class QosProfiles {
 public:
  [[nodiscard]] static rclcpp::QoS sensorInput();
  [[nodiscard]] static rclcpp::QoS reliableSensorInput();
  [[nodiscard]] static rclcpp::QoS livoxLidarInput();
  [[nodiscard]] static rclcpp::QoS livoxImuInput();
  [[nodiscard]] static rclcpp::QoS estimatorOutput();
  [[nodiscard]] static rclcpp::QoS mapOutput();
  // P1 mapping observation (see docs/architecture/navigation_layers.md):
  // freshness over backlog, so this is deliberately depth-1 and volatile.
  // BestEffort is the initial candidate; switch to reliable() if
  // target-machine testing shows unacceptable transport loss.
  [[nodiscard]] static rclcpp::QoS mappingObservation();
};

}  // namespace uav::nav::lio

#pragma once

#include <rclcpp/qos.hpp>

namespace uav::nav::lio {

class QosProfiles {
 public:
  [[nodiscard]] static rclcpp::QoS sensorInput();
  [[nodiscard]] static rclcpp::QoS reliableSensorInput();
  [[nodiscard]] static rclcpp::QoS livoxLidarInput();
  [[nodiscard]] static rclcpp::QoS livoxImuInput();
  // The `/lio/mapping_observation` publisher uses this profile:
  // reliable delivery with keep_last depth 10.
  [[nodiscard]] static rclcpp::QoS estimatorOutput();
};

}  // namespace uav::nav::lio

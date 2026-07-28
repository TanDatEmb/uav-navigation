#pragma once

#include <rclcpp/qos.hpp>

namespace uav::nav::lio {

class QosProfiles {
 public:
  [[nodiscard]] static rclcpp::QoS sensorInput();
  [[nodiscard]] static rclcpp::QoS estimatorOutput();
  [[nodiscard]] static rclcpp::QoS mapOutput();
};

}  // namespace uav::nav::lio

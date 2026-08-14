#include "fast_lio_ros/qos_profiles.hpp"

namespace uav::nav::lio {

rclcpp::QoS QosProfiles::sensorInput() { return rclcpp::SensorDataQoS().keep_last(100); }

rclcpp::QoS QosProfiles::reliableSensorInput() {
  return rclcpp::QoS{rclcpp::KeepLast{100}}.reliable().durability_volatile();
}

rclcpp::QoS QosProfiles::livoxLidarInput() {
  return rclcpp::QoS{rclcpp::KeepLast{256}}.reliable().durability_volatile();
}

rclcpp::QoS QosProfiles::livoxImuInput() {
  return rclcpp::QoS{rclcpp::KeepLast{256}}.reliable().durability_volatile();
}

rclcpp::QoS QosProfiles::estimatorOutput() { return rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(); }

rclcpp::QoS QosProfiles::mapOutput() {
  return rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local();
}

rclcpp::QoS QosProfiles::mappingObservation() {
  return rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort().durability_volatile();
}

}  // namespace uav::nav::lio

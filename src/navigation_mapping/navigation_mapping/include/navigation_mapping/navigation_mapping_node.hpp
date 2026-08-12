#pragma once

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <rclcpp/rclcpp.hpp>

#include "navigation_mapping/mapping_pipeline.hpp"

namespace navigation_mapping {

// Product-owned mapper node (P1 section 10). This is the single subscription
// path into ROG-Map: the upstream ROS wrapper's automatic subscription path
// (rog_map_ros1/rog_map_ros2) is never vendored or used as the production
// integration boundary (see rog_map_vendor/UPSTREAM.md).
class NavigationMappingNode : public rclcpp::Node {
 public:
  explicit NavigationMappingNode(const rclcpp::NodeOptions& options = {});

 private:
  void onObservation(
      const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr& message);
  void publishDiagnostics();

  std::unique_ptr<MappingPipeline> pipeline_;
  rclcpp::Subscription<navigation_interfaces::msg::LidarMappingObservation>::SharedPtr
      observation_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;
  std::uint64_t invalid_cloud_count_{0};
};

}  // namespace navigation_mapping

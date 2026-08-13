#pragma once

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "navigation_mapping/mapping_pipeline.hpp"

namespace navigation_mapping {

// Product-owned mapper node. This is the single subscription
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
  void publishMapVisualization();
  sensor_msgs::msg::PointCloud2 makePointCloud(
      const rog_map::vec_E<rog_map::Vec3f>& points,
      const builtin_interfaces::msg::Time& stamp) const;

  std::unique_ptr<MappingPipeline> pipeline_;
  rclcpp::Subscription<navigation_interfaces::msg::LidarMappingObservation>::SharedPtr
      observation_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;
  std::uint64_t invalid_cloud_count_{0};
  bool visualization_enabled_{false};
  bool publish_unknown_{false};
  bool publish_frontier_{false};
  double visualization_range_x_m_{15.0};
  double visualization_range_y_m_{15.0};
  double visualization_range_z_m_{6.0};
  std::size_t visualization_max_points_{150000};
  std::string visualization_frame_id_;
  std::uint64_t last_visualization_update_count_{0};
};

}  // namespace navigation_mapping

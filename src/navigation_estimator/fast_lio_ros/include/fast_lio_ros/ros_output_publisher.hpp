#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class RosOutputPublisher {
 public:
  RosOutputPublisher(rclcpp::Node& node, RosParameters parameters);
  void publish(const ProcessResult& result);

 private:
  [[nodiscard]] sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Eigen::Vector3d>& points, const builtin_interfaces::msg::Time& stamp) const;
  void publishDiagnostics(const ProcessResult& result, const builtin_interfaces::msg::Time& stamp);

  RosParameters parameters_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};

}  // namespace uav::nav::lio

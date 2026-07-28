#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/ros_imu_adapter.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"
#include "fast_lio_ros/ros_output_publisher.hpp"
#include "fast_lio_ros/ros_transform_publisher.hpp"

namespace uav::nav::lio {

class FastLioNode : public rclcpp::Node {
 public:
  FastLioNode();

 private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message);
  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
  void drainPipeline();

  RosParameters parameters_;
  FastLioPipeline pipeline_;
  RosImuAdapter imu_adapter_;
  RosLidarAdapter lidar_adapter_;
  RosOutputPublisher output_publisher_;
  RosTransformPublisher transform_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
};

}  // namespace uav::nav::lio

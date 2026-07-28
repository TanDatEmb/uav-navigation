#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <variant>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/ros_imu_adapter.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"
#include "fast_lio_ros/ros_livox_custom_adapter.hpp"
#include "fast_lio_ros/ros_output_publisher.hpp"
#include "fast_lio_ros/ros_transform_publisher.hpp"

namespace uav::nav::lio {

class FastLioNode : public rclcpp::Node {
 public:
  FastLioNode();
  ~FastLioNode() override;

 private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message);
  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
  void onLivoxCustom(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& message);
  using InputMeasurement = std::variant<ImuSample, LidarScan>;
  void enqueue(InputMeasurement measurement);
  void processingLoop();
  void publishAvailableResults();

  RosParameters parameters_;
  FastLioPipeline pipeline_;
  RosImuAdapter imu_adapter_;
  RosLidarAdapter lidar_adapter_;
  RosLivoxCustomAdapter livox_custom_adapter_;
  RosOutputPublisher output_publisher_;
  RosTransformPublisher transform_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_custom_subscription_;
  static constexpr std::size_t kMaximumInputQueueSize = 16384;
  std::mutex input_mutex_;
  std::condition_variable input_ready_;
  std::deque<InputMeasurement> input_queue_;
  bool stopping_{false};
  std::thread processing_worker_;
  SensorDiagnostics ingress_diagnostics_;
  std::int64_t previous_ros_imu_ns_{-1};
};

}  // namespace uav::nav::lio

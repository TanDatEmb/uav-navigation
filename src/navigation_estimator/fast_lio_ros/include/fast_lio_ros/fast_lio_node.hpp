#pragma once

#include <chrono>
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
#include <nav_msgs/msg/odometry.hpp>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/lio_public_frame_generation.hpp"
#include "fast_lio_ros/propagated_odometry_worker.hpp"
#include "fast_lio_ros/ros_imu_adapter.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"
#include "fast_lio_ros/ros_livox_custom_adapter.hpp"
#include "fast_lio_ros/ros_output_publisher.hpp"
#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"
#include "fast_lio_ros/runtime_diagnostics.hpp"
#include "fast_lio_ros/ros_transform_publisher.hpp"

namespace uav::nav::lio {

enum class PropagatedImuFanoutAction {
  kRequestLoadSheddingOnly,
  kEnqueue,
  kRequestLoadSheddingAndEnqueue,
};

[[nodiscard]] constexpr PropagatedImuFanoutAction propagatedImuFanoutAction(
    const bool main_accepted, const bool overload_threshold_reached) noexcept {
  if (!main_accepted) {
    return PropagatedImuFanoutAction::kRequestLoadSheddingOnly;
  }
  return overload_threshold_reached
             ? PropagatedImuFanoutAction::kRequestLoadSheddingAndEnqueue
             : PropagatedImuFanoutAction::kEnqueue;
}

class FastLioNode : public rclcpp::Node {
 public:
  explicit FastLioNode(const rclcpp::NodeOptions& options = {});
  ~FastLioNode() override;

 private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message);
  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
  void onLivoxCustom(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& message);
  void onInitialStatePrior(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void closeInitialStatePriorStream();
  void startProcessingWorker();
  void startProcessingWorkerWhenPriorPublisherMatches();
  using InputMeasurement = std::variant<ImuSample, LidarScan>;
  [[nodiscard]] bool enqueue(InputMeasurement measurement);
  void processingLoop();
  void publishAvailableResults();
  void publishTransportSnapshot();

  struct PendingLidarTiming {
    std::int64_t scan_end_ns{0};
    std::chrono::steady_clock::time_point started_at;
  };

  RosParameters parameters_;
  EstimatorProfile profile_;
  FastLioPipeline pipeline_;
  RosImuAdapter imu_adapter_;
  RosLidarAdapter lidar_adapter_;
  RosLivoxCustomAdapter livox_custom_adapter_;
  std::shared_ptr<LioPublicFrameGeneration> public_frame_generation_;
  RosOutputPublisher output_publisher_;
  RosTransformPublisher transform_publisher_;
  std::unique_ptr<RosPropagatedOdometryPublisher>
      propagated_odometry_publisher_;
  std::unique_ptr<PropagatedOdometryWorker> propagated_odometry_worker_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_custom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      initial_state_prior_subscription_;
  std::mutex initial_state_prior_subscription_mutex_;
  rclcpp::TimerBase::SharedPtr initial_prior_startup_timer_;
  std::mutex processing_worker_start_mutex_;
  bool processing_worker_start_closed_{false};
  std::mutex input_mutex_;
  std::condition_variable input_ready_;
  std::deque<ImuSample> imu_queue_;
  std::deque<LidarScan> lidar_queue_;
  bool stopping_{false};
  std::thread processing_worker_;
  SensorDiagnostics ingress_diagnostics_;
  ProcessingStatistics processing_statistics_;
  RuntimeDiagnostics runtime_diagnostics_;
  RuntimeStatistics runtime_statistics_;
  std::deque<PendingLidarTiming> pending_lidar_timings_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  std::int64_t previous_ros_imu_ns_{-1};
  std::uint64_t correction_sequence_{0U};
  rclcpp::TimerBase::SharedPtr transport_diagnostics_timer_;
};

}  // namespace uav::nav::lio

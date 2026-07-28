#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <utility>

#include "fast_lio_core/deskew/deskew_mode.hpp"
#include "fast_lio_ros/qos_profiles.hpp"

namespace uav::nav::lio {
namespace {

LidarTimingMode adapterTiming(const RosParameters& parameters) {
  return parameters.lidar_timing_mode == "per_point" ? LidarTimingMode::kPerPoint
                                                     : LidarTimingMode::kSimultaneousScan;
}

EstimatorConfig estimatorConfig(const RosParameters& parameters) {
  EstimatorConfig config;
  config.synchronization.maximum_imu_gap_ns = parameters.maximum_imu_gap_ns;
  config.deskew.mode = parameters.lidar_timing_mode == "per_point" ? DeskewMode::kPerPoint
                                                                   : DeskewMode::kSimultaneousScan;
  config.preprocessing.point_filter.minimum_range_m = parameters.minimum_range_m;
  config.preprocessing.point_filter.maximum_range_m = parameters.maximum_range_m;
  config.preprocessing.voxel_filter.voxel_size_m = parameters.voxel_size_m;
  config.initialization.minimum_imu_samples =
      static_cast<std::size_t>(parameters.minimum_imu_samples);
  config.initialization.require_stationary = parameters.require_stationary;
  config.iterated_filter.convergence.maximum_iterations =
      static_cast<std::size_t>(parameters.maximum_registration_iterations);
  config.extrinsic.estimate_online = parameters.estimate_extrinsic_online;
  config.extrinsic.translation_imu_lidar_m = {parameters.translation_imu_lidar_m[0],
                                              parameters.translation_imu_lidar_m[1],
                                              parameters.translation_imu_lidar_m[2]};
  const auto& q = parameters.rotation_imu_lidar_xyzw;
  config.extrinsic.rotation_imu_lidar = Eigen::Quaterniond{q[3], q[0], q[1], q[2]}.normalized();
  return config;
}

}  // namespace

FastLioNode::FastLioNode()
    : Node("fast_lio"),
      parameters_(ParameterLoader::declareAndLoad(*this)),
      pipeline_(estimatorConfig(parameters_)),
      imu_adapter_(parameters_.imu_frame),
      lidar_adapter_(parameters_.lidar_frame, adapterTiming(parameters_)),
      output_publisher_(*this, parameters_),
      transform_publisher_(*this, parameters_) {
  if (parameters_.lidar_message_type != "pointcloud2") {
    throw std::invalid_argument(
        "this build accepts PointCloud2; livox_custom requires its explicit "
        "optional adapter dependency");
  }
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      parameters_.imu_topic, QosProfiles::sensorInput(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) { onImu(message); });
  lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      parameters_.lidar_topic, QosProfiles::sensorInput(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onLidar(message); });
  RCLCPP_WARN(get_logger(),
              "M1 baseline assumes base_link and imu_link origins coincide for public "
              "odometry; replace placeholder calibration before real operation");
}

void FastLioNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message) {
  try {
    const auto status = pipeline_.pushImu(imu_adapter_.convert(*message));
    if (!status.ok()) {
      RCLCPP_WARN(get_logger(), "rejected IMU: %s", status.message().c_str());
    }
    drainPipeline();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid IMU message: %s", error.what());
  }
}

void FastLioNode::onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  try {
    const auto status = pipeline_.pushLidar(lidar_adapter_.convert(*message));
    if (!status.ok()) {
      RCLCPP_WARN(get_logger(), "rejected LiDAR: %s", status.message().c_str());
    }
    drainPipeline();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid LiDAR message: %s", error.what());
  }
}

void FastLioNode::drainPipeline() {
  while (const auto result = pipeline_.processNext()) {
    output_publisher_.publish(*result);
    transform_publisher_.publish(*result);
  }
}

}  // namespace uav::nav::lio

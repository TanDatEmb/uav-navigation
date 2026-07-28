#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <utility>

#include "fast_lio_core/deskew/deskew_mode.hpp"
#include "fast_lio_ros/qos_profiles.hpp"

namespace uav::nav::lio {
namespace {

LidarTimingMode adapterTiming(const RosParameters& parameters) {
  if (parameters.lidar_timing_mode == "per_point") {
    return LidarTimingMode::kPerPoint;
  }
  if (parameters.lidar_timing_mode == "simultaneous_scan") {
    return LidarTimingMode::kSimultaneousScan;
  }
  throw std::invalid_argument("unsupported production LiDAR timing mode");
}

LivoxTimestampPolicy livoxTimestampPolicy(
    const RosParameters& parameters) {
  if (parameters.livox_timestamp_policy == "require_header_match") {
    return LivoxTimestampPolicy::kRequireHeaderMatchesTimebase;
  }
  if (parameters.livox_timestamp_policy == "timebase_authoritative") {
    return LivoxTimestampPolicy::kTimebaseAuthoritative;
  }
  throw std::invalid_argument("unsupported Livox timestamp policy");
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
  config.ikfom.maximum_iterations =
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
      imu_adapter_(parameters_.imu_input_frame,
                   parseClockDomain(parameters_.input_clock_domain)),
      lidar_adapter_(parameters_.lidar_input_frame, adapterTiming(parameters_),
                     parseClockDomain(parameters_.input_clock_domain)),
      livox_custom_adapter_(
          parameters_.lidar_input_frame,
          parseClockDomain(parameters_.input_clock_domain),
          livoxTimestampPolicy(parameters_)),
      output_publisher_(*this, parameters_),
      transform_publisher_(*this, parameters_) {
  const bool livox_input =
      parameters_.lidar_message_type == "livox_custom";
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      parameters_.imu_topic,
      livox_input ? QosProfiles::livoxImuInput()
                  : QosProfiles::sensorInput(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) { onImu(message); });
  if (livox_input) {
    livox_custom_subscription_ =
        create_subscription<livox_ros_driver2::msg::CustomMsg>(
            parameters_.lidar_topic, QosProfiles::livoxLidarInput(),
            [this](
                const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr
                    message) { onLivoxCustom(message); });
  } else {
    lidar_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            parameters_.lidar_topic, QosProfiles::sensorInput(),
            [this](
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                    message) { onLidar(message); });
  }
  RCLCPP_INFO(get_logger(),
              "Publishing the estimator state as odom -> %s; no base_link "
              "transform is inferred from the IMU state",
              parameters_.imu_frame.c_str());
}

void FastLioNode::onLivoxCustom(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& message) {
  try {
    const auto status =
        pipeline_.pushLidar(livox_custom_adapter_.convert(*message));
    if (!status.ok()) {
      RCLCPP_WARN(get_logger(), "rejected Livox CustomMsg: %s",
                  status.message().c_str());
    }
    drainPipeline();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid Livox CustomMsg: %s",
                error.what());
  }
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

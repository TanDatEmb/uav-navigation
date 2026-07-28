#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <algorithm>
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
  processing_worker_ = std::thread([this] { processingLoop(); });
}

FastLioNode::~FastLioNode() {
  {
    std::lock_guard lock(input_mutex_);
    stopping_ = true;
  }
  input_ready_.notify_all();
  if (processing_worker_.joinable()) {
    processing_worker_.join();
  }
}

void FastLioNode::onLivoxCustom(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& message) {
  try {
    enqueue(livox_custom_adapter_.convert(*message));
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid Livox CustomMsg: %s",
                error.what());
  }
}

void FastLioNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message) {
  try {
    enqueue(imu_adapter_.convert(*message));
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid IMU message: %s", error.what());
  }
}

void FastLioNode::onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  try {
    enqueue(lidar_adapter_.convert(*message));
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid LiDAR message: %s", error.what());
  }
}

void FastLioNode::enqueue(InputMeasurement measurement) {
  {
    std::lock_guard lock(input_mutex_);
    const bool is_imu = std::holds_alternative<ImuSample>(measurement);
    if (is_imu) {
      ++ingress_diagnostics_.ros_received_imu_count;
      const auto time_ns = std::get<ImuSample>(measurement).time.nanoseconds();
      if (previous_ros_imu_ns_ >= 0) {
        const auto gap_ns = time_ns - previous_ros_imu_ns_;
        if (gap_ns <= 0) {
          ++ingress_diagnostics_.timestamp_regression_count;
        } else {
          ingress_diagnostics_.ros_maximum_imu_gap_ns =
              std::max(ingress_diagnostics_.ros_maximum_imu_gap_ns, gap_ns);
        }
      }
      previous_ros_imu_ns_ = time_ns;
    } else {
      ++ingress_diagnostics_.ros_received_lidar_count;
    }
    if (input_queue_.size() >= kMaximumInputQueueSize) {
      is_imu ? ++ingress_diagnostics_.imu_drop_count
             : ++ingress_diagnostics_.lidar_drop_count;
      RCLCPP_ERROR(get_logger(), "bounded estimator input queue overflow");
      return;
    }
    input_queue_.push_back(std::move(measurement));
    ingress_diagnostics_.processing_queue_high_water_mark =
        std::max(ingress_diagnostics_.processing_queue_high_water_mark,
                 input_queue_.size());
  }
  input_ready_.notify_one();
}

void FastLioNode::processingLoop() {
  while (true) {
    InputMeasurement measurement;
    {
      std::unique_lock lock(input_mutex_);
      input_ready_.wait(lock,
                        [this] { return stopping_ || !input_queue_.empty(); });
      if (stopping_ && input_queue_.empty()) {
        return;
      }
      measurement = std::move(input_queue_.front());
      input_queue_.pop_front();
    }
    Status status;
    if (std::holds_alternative<ImuSample>(measurement)) {
      status = pipeline_.pushImu(std::get<ImuSample>(measurement));
      std::lock_guard lock(input_mutex_);
      status.ok() ? ++ingress_diagnostics_.core_accepted_imu_count
                  : ++ingress_diagnostics_.imu_drop_count;
    } else {
      status = pipeline_.pushLidar(std::move(std::get<LidarScan>(measurement)));
      std::lock_guard lock(input_mutex_);
      status.ok() ? ++ingress_diagnostics_.core_accepted_lidar_count
                  : ++ingress_diagnostics_.lidar_drop_count;
    }
    if (!status.ok()) {
      RCLCPP_WARN(get_logger(), "core rejected queued measurement: %s",
                  status.message().c_str());
    }
    publishAvailableResults();
  }
}

void FastLioNode::publishAvailableResults() {
  while (const auto result = pipeline_.processNext()) {
    auto augmented = *result;
    {
      std::lock_guard lock(input_mutex_);
      augmented.diagnostics.sensor = ingress_diagnostics_;
    }
    output_publisher_.publish(augmented);
    transform_publisher_.publish(augmented);
  }
}

}  // namespace uav::nav::lio

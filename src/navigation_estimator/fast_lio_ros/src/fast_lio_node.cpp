#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <algorithm>
#include <chrono>
#include <utility>

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

PointTimeConfig pointTimeConfig(const RosParameters& parameters) {
  PointTimeConfig result;
  result.field = parameters.point_time_field;
  result.encoding =
      parameters.point_time_encoding == "float64_absolute_nanoseconds"
          ? PointTimeEncoding::kFloat64AbsoluteNanoseconds
          : PointTimeEncoding::kUint32RelativeNanoseconds;
  result.scan_reference =
      parameters.point_time_scan_reference == "minimum_point_time"
          ? ScanReference::kMinimumPointTime
          : ScanReference::kHeaderStamp;
  result.maximum_scan_duration_ns = parameters.maximum_scan_duration_ns;
  result.maximum_header_offset_ns = parameters.maximum_header_offset_ns;
  result.reject_scan_timestamp_regression =
      parameters.reject_timestamp_regression;
  return result;
}

}  // namespace

FastLioNode::FastLioNode(const rclcpp::NodeOptions& options)
    : Node("fast_lio", options),
      parameters_(ParameterLoader::declareAndLoad(*this)),
      profile_(makeEstimatorProfile(parameters_)),
      pipeline_(profile_.estimator),
      imu_adapter_(parameters_.imu_input_frame,
                   parseClockDomain(parameters_.input_clock_domain)),
      lidar_adapter_(parameters_.lidar_input_frame, adapterTiming(parameters_),
                     parseClockDomain(parameters_.input_clock_domain),
                     pointTimeConfig(parameters_)),
      livox_custom_adapter_(
          parameters_.lidar_input_frame,
          parseClockDomain(parameters_.input_clock_domain),
          livoxTimestampPolicy(parameters_)),
      output_publisher_(*this, parameters_),
      transform_publisher_(*this, parameters_) {
  const bool livox_input =
      parameters_.lidar_message_type == "livox_custom";
  const auto pointcloud_qos =
      parameters_.input_qos_reliability == "reliable"
          ? QosProfiles::reliableSensorInput()
          : QosProfiles::sensorInput();
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      parameters_.imu_topic,
      livox_input ? QosProfiles::livoxImuInput()
                  : pointcloud_qos,
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
            parameters_.lidar_topic, pointcloud_qos,
            [this](
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                    message) { onLidar(message); });
  }
  RCLCPP_INFO(get_logger(),
              "Publishing the estimator state as odom -> %s; no base_link "
              "transform is inferred from the IMU state; config_sha256=%s",
              parameters_.imu_frame.c_str(), profile_.config_sha256.c_str());
  transport_diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this] { publishTransportSnapshot(); });
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
      ++runtime_diagnostics_.received_imu_count;
      const auto time_ns = std::get<ImuSample>(measurement).time.nanoseconds();
      runtime_diagnostics_.latest_received_time_ns =
          std::max(runtime_diagnostics_.latest_received_time_ns, time_ns);
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
      ++runtime_diagnostics_.received_lidar_count;
      runtime_diagnostics_.latest_received_time_ns = std::max(
          runtime_diagnostics_.latest_received_time_ns,
          std::get<LidarScan>(measurement).end_time.nanoseconds());
    }
    const bool queue_full =
        is_imu
            ? imu_queue_.size() >=
                  static_cast<std::size_t>(parameters_.imu_queue_capacity)
            : lidar_queue_.size() >=
                  static_cast<std::size_t>(parameters_.lidar_queue_capacity);
    if (queue_full) {
      is_imu ? ++ingress_diagnostics_.imu_drop_count
             : ++ingress_diagnostics_.lidar_drop_count;
      is_imu ? ++runtime_diagnostics_.imu_drop_count
             : ++runtime_diagnostics_.lidar_drop_count;
      runtime_diagnostics_.overflow_detected = true;
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "bounded estimator input queue overflow; acceptance replay must fail");
      return;
    }
    if (is_imu) {
      imu_queue_.push_back(std::move(std::get<ImuSample>(measurement)));
    } else {
      lidar_queue_.push_back(std::move(std::get<LidarScan>(measurement)));
    }
    ++runtime_diagnostics_.current_input_queue_depth;
    is_imu ? ++runtime_diagnostics_.current_imu_queue_depth
           : ++runtime_diagnostics_.current_lidar_queue_depth;
    runtime_diagnostics_.maximum_queue_depth =
        std::max(runtime_diagnostics_.maximum_queue_depth,
                 runtime_diagnostics_.current_input_queue_depth);
    ingress_diagnostics_.processing_queue_high_water_mark =
        std::max(ingress_diagnostics_.processing_queue_high_water_mark,
                 imu_queue_.size() + lidar_queue_.size());
    ingress_diagnostics_.imu_queue_high_water_mark =
        std::max(ingress_diagnostics_.imu_queue_high_water_mark,
                 imu_queue_.size());
    ingress_diagnostics_.lidar_queue_high_water_mark =
        std::max(ingress_diagnostics_.lidar_queue_high_water_mark,
                 lidar_queue_.size());
  }
  input_ready_.notify_one();
}

void FastLioNode::processingLoop() {
  while (true) {
    InputMeasurement measurement;
    {
      std::unique_lock lock(input_mutex_);
      input_ready_.wait(lock, [this] {
        return stopping_ || !imu_queue_.empty() || !lidar_queue_.empty();
      });
      if (stopping_ && imu_queue_.empty() && lidar_queue_.empty()) {
        return;
      }
      const bool take_imu =
          lidar_queue_.empty() ||
          (!imu_queue_.empty() &&
           imu_queue_.front().time.nanoseconds() <=
               lidar_queue_.front().end_time.nanoseconds());
      if (take_imu) {
        measurement = std::move(imu_queue_.front());
        imu_queue_.pop_front();
      } else {
        measurement = std::move(lidar_queue_.front());
        lidar_queue_.pop_front();
      }
      --runtime_diagnostics_.current_input_queue_depth;
      std::holds_alternative<ImuSample>(measurement)
          ? --runtime_diagnostics_.current_imu_queue_depth
          : --runtime_diagnostics_.current_lidar_queue_depth;
    }
    const auto processing_started = std::chrono::steady_clock::now();
    const bool is_imu = std::holds_alternative<ImuSample>(measurement);
    const auto measurement_time_ns =
        is_imu ? std::get<ImuSample>(measurement).time.nanoseconds()
               : std::get<LidarScan>(measurement).end_time.nanoseconds();
    Status status;
    if (is_imu) {
      status = pipeline_.pushImu(std::get<ImuSample>(measurement));
      std::lock_guard lock(input_mutex_);
      status.ok() ? ++ingress_diagnostics_.core_accepted_imu_count
                  : ++ingress_diagnostics_.imu_drop_count;
      if (!status.ok()) {
        ++runtime_diagnostics_.imu_drop_count;
      }
    } else {
      status = pipeline_.pushLidar(std::move(std::get<LidarScan>(measurement)));
      std::lock_guard lock(input_mutex_);
      status.ok() ? ++ingress_diagnostics_.core_accepted_lidar_count
                  : ++ingress_diagnostics_.lidar_drop_count;
      if (!status.ok()) {
        ++runtime_diagnostics_.lidar_drop_count;
      }
    }
    if (!status.ok()) {
      RCLCPP_WARN(get_logger(), "core rejected queued measurement: %s",
                  status.message().c_str());
    }
    publishAvailableResults();
    const auto processing_finished = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(input_mutex_);
      processing_statistics_ = pipeline_.diagnostics().processing;
      is_imu ? ++runtime_diagnostics_.processed_imu_count
             : ++runtime_diagnostics_.processed_lidar_count;
      runtime_diagnostics_.latest_processed_time_ns =
          std::max(runtime_diagnostics_.latest_processed_time_ns,
                   measurement_time_ns);
      runtime_diagnostics_.processing_lag_ns = std::max<std::int64_t>(
          0, runtime_diagnostics_.latest_received_time_ns -
                 runtime_diagnostics_.latest_processed_time_ns);
      runtime_diagnostics_.processing_lag_exceeded =
          runtime_diagnostics_.processing_lag_exceeded ||
          runtime_diagnostics_.processing_lag_ns >
              parameters_.maximum_processing_lag_ms * 1'000'000;
      const auto elapsed = processing_finished - processing_started;
      runtime_statistics_.recordBusy(elapsed);
      if (!is_imu) {
        runtime_statistics_.recordScan(
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
                .count());
      }
    }
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

void FastLioNode::publishTransportSnapshot() {
  SensorDiagnostics sensor;
  ProcessingStatistics processing;
  RuntimeDiagnostics runtime;
  {
    std::lock_guard lock(input_mutex_);
    sensor = ingress_diagnostics_;
    processing = processing_statistics_;
    runtime = runtime_diagnostics_;
    runtime_statistics_.populate(runtime);
  }
  output_publisher_.publishTransportSnapshot(sensor, processing, runtime);
}

}  // namespace uav::nav::lio

#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <algorithm>
#include <chrono>
#include <pthread.h>
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
  result.maximum_boundary_overlap_ns =
      parameters.maximum_boundary_overlap_ns;
  result.minimum_points_after_overlap_trim =
      static_cast<std::size_t>(parameters.minimum_points_after_overlap_trim);
  result.reject_scan_timestamp_regression =
      parameters.reject_timestamp_regression;
  return result;
}

void countRejection(RuntimeDiagnostics& diagnostics, const Status& status,
                    bool is_imu) {
  switch (status.code()) {
    case StatusCode::kBufferFull:
      is_imu ? ++diagnostics.reject_reason_imu_buffer_full
             : ++diagnostics.reject_reason_lidar_buffer_full;
      break;
    case StatusCode::kTimestampRegression:
      ++diagnostics.reject_reason_timestamp_regression;
      break;
    case StatusCode::kImuGap:
      ++diagnostics.reject_reason_imu_gap;
      break;
    case StatusCode::kNotReady:
    case StatusCode::kInitializationRejected:
      ++diagnostics.reject_reason_not_initialized;
      break;
    case StatusCode::kInsufficientData:
      ++diagnostics.reject_reason_too_few_points;
      break;
    default:
      if (status.message().find("non-finite") != std::string::npos) {
        ++diagnostics.reject_reason_nonfinite_xyz;
      } else if (status.message().find("relative time") != std::string::npos) {
        ++diagnostics.reject_reason_invalid_point_time;
      }
      break;
  }
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
  runtime_diagnostics_.imu_queue_capacity =
      static_cast<std::size_t>(parameters_.imu_queue_capacity);
  runtime_diagnostics_.lidar_queue_capacity =
      static_cast<std::size_t>(parameters_.lidar_queue_capacity);
  if (parameters_.propagated_odometry_enabled) {
    propagated_odometry_publisher_ =
        std::make_unique<RosPropagatedOdometryPublisher>(*this, parameters_);
    PropagatedOdometryWorkerConfig worker_config;
    worker_config.propagator.ikfom = profile_.estimator.ikfom;
    worker_config.propagator.residual_builder =
        profile_.estimator.residual_builder;
    worker_config.propagator.imu_history_duration_ns =
        parameters_.propagated_odometry_imu_history_duration_ns;
    worker_config.imu_ingress_capacity = static_cast<std::size_t>(
        parameters_.propagated_odometry_imu_ingress_capacity);
    worker_config.maximum_correction_age_ns =
        parameters_.propagated_odometry_maximum_correction_age_ns;
    worker_config.publish_rate_hz =
        parameters_.propagated_odometry_publish_rate_hz;
    propagated_odometry_worker_ = std::make_unique<PropagatedOdometryWorker>(
        worker_config, [this](const std::optional<KinematicStateEstimate>& estimate) {
          if (estimate.has_value()) {
            propagated_odometry_publisher_->publish(estimate->estimate);
          }
        });
    propagated_odometry_worker_->start();
  }
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
  if (propagated_odometry_worker_) {
    propagated_odometry_worker_->stop();
  }
  if (processing_worker_.joinable()) {
    processing_worker_.join();
  }
}

void FastLioNode::onLivoxCustom(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& message) {
  try {
    (void)enqueue(livox_custom_adapter_.convert(*message));
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid Livox CustomMsg: %s",
                error.what());
  }
}

void FastLioNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& message) {
  try {
    const ImuSample sample = imu_adapter_.convert(*message);
    const bool main_accepted = enqueue(InputMeasurement{sample});
    if (propagated_odometry_worker_) {
      bool overload_threshold_reached = false;
      if (main_accepted) {
        bool shed = false;
        {
          std::lock_guard lock(input_mutex_);
          shed = imu_queue_.size() + lidar_queue_.size() >=
                     static_cast<std::size_t>(parameters_.imu_queue_capacity / 8) ||
                 lidar_queue_.size() >=
                     static_cast<std::size_t>(std::max<std::int64_t>(1,
                         parameters_.lidar_queue_capacity / 2));
        }
        overload_threshold_reached = shed;
      }
      const auto action = propagatedImuFanoutAction(
          main_accepted, overload_threshold_reached);
      if (action == PropagatedImuFanoutAction::kRequestLoadSheddingOnly ||
          action ==
              PropagatedImuFanoutAction::kRequestLoadSheddingAndEnqueue) {
        propagated_odometry_worker_->requestLoadShedding();
      }
      if (action == PropagatedImuFanoutAction::kEnqueue ||
          action == PropagatedImuFanoutAction::kRequestLoadSheddingAndEnqueue) {
        if (!propagated_odometry_worker_->enqueueImu(sample)) {
          propagated_odometry_worker_->requestLoadShedding();
        }
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid IMU message: %s", error.what());
  }
}

void FastLioNode::onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  try {
    (void)enqueue(lidar_adapter_.convert(*message));
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "invalid LiDAR message: %s", error.what());
  }
}

bool FastLioNode::enqueue(InputMeasurement measurement) {
  {
    std::lock_guard lock(input_mutex_);
    if (stopping_) {
      return false;
    }
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
      is_imu ? ++runtime_diagnostics_.reject_reason_imu_buffer_full
             : ++runtime_diagnostics_.reject_reason_lidar_buffer_full;
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s queue full: total_rejected=%zu current_depth=%zu capacity=%zu "
          "last_processed_stamp=%ld last_received_stamp=%ld",
          is_imu ? "IMU" : "LiDAR",
          is_imu ? runtime_diagnostics_.reject_reason_imu_buffer_full
                 : runtime_diagnostics_.reject_reason_lidar_buffer_full,
          is_imu ? imu_queue_.size() : lidar_queue_.size(),
          is_imu ? runtime_diagnostics_.imu_queue_capacity
                 : runtime_diagnostics_.lidar_queue_capacity,
          runtime_diagnostics_.latest_processed_time_ns,
          runtime_diagnostics_.latest_received_time_ns);
      return false;
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
    runtime_diagnostics_.maximum_imu_queue_depth =
        std::max(runtime_diagnostics_.maximum_imu_queue_depth,
                 runtime_diagnostics_.current_imu_queue_depth);
    runtime_diagnostics_.maximum_lidar_queue_depth =
        std::max(runtime_diagnostics_.maximum_lidar_queue_depth,
                 runtime_diagnostics_.current_lidar_queue_depth);
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
  return true;
}

void FastLioNode::processingLoop() {
  (void)pthread_setname_np(pthread_self(), "fast_lio_main");
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
        countRejection(runtime_diagnostics_, status, true);
      }
    } else {
      status = pipeline_.pushLidar(std::move(std::get<LidarScan>(measurement)));
      std::lock_guard lock(input_mutex_);
      status.ok() ? ++ingress_diagnostics_.core_accepted_lidar_count
                  : ++ingress_diagnostics_.lidar_drop_count;
      if (!status.ok()) {
        ++runtime_diagnostics_.lidar_drop_count;
        countRejection(runtime_diagnostics_, status, false);
      }
    }
    if (!status.ok()) {
      if (is_imu && propagated_odometry_worker_) {
        (void)propagated_odometry_worker_->enqueueEstimatorState(
            {EstimatorStatus::kDegraded, false, std::nullopt, 0U});
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "core rejected queued measurement: %s; total_imu_rejected=%zu "
          "total_lidar_rejected=%zu current_depth=%zu",
          status.message().c_str(), runtime_diagnostics_.imu_drop_count,
          runtime_diagnostics_.lidar_drop_count,
          runtime_diagnostics_.current_input_queue_depth);
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
      ++runtime_diagnostics_.worker_heartbeat;
      runtime_diagnostics_.worker_last_progress_wall_time_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              processing_finished.time_since_epoch()).count();
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
    if (propagated_odometry_worker_) {
      const bool corrected = augmented.hasCorrectedOutput();
      if (corrected) {
        ++correction_sequence_;
      }
      EstimatorStateUpdate update;
      update.status = augmented.status_after;
      update.navigation_valid = augmented.diagnostics.navigation_valid;
      if (corrected) {
        update.corrected_estimate = augmented.corrected_estimate;
      }
      update.correction_sequence = correction_sequence_;
      (void)propagated_odometry_worker_->enqueueEstimatorState(
          std::move(update));
    }
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
  if (propagated_odometry_worker_ && propagated_odometry_publisher_) {
    const auto propagated = propagated_odometry_worker_->diagnostics();
    output_publisher_.publishPropagatedOdometryDiagnostics(
        propagated, propagated.publication_count,
        propagated.publication_skip_count, propagated.last_published_time,
        propagated.next_publish_deadline);
  } else {
    output_publisher_.publishPropagatedOdometryDiagnostics(
        PropagatedOdometryWorkerDiagnostics{}, 0U, 0U, std::nullopt,
        std::nullopt);
  }
}

}  // namespace uav::nav::lio

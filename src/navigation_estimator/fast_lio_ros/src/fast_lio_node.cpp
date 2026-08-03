#include "fast_lio_ros/fast_lio_node.hpp"

#include <exception>
#include <algorithm>
#include <chrono>
#include <pthread.h>
#include <utility>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_static_transform_resolver.hpp"

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

PriorAttitudeMode initialPriorAttitude(std::string_view value) {
  if (value == "none") return PriorAttitudeMode::kNone;
  if (value == "full") return PriorAttitudeMode::kFull;
  return PriorAttitudeMode::kYawOnly;
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
  RosStaticTransformResolver static_tf(*this);
  const auto base_to_imu = static_tf.resolve(parameters_.base_frame,
                                             parameters_.imu_frame);
  if (!base_to_imu.ok()) {
    throw std::runtime_error("P0.3 static sensor geometry unavailable: " +
                             base_to_imu.status().message());
  }
  const Status prior_geometry_status =
      pipeline_.setInitialStatePriorGeometry(base_to_imu.value());
  if (!prior_geometry_status.ok()) {
    throw std::runtime_error("invalid initial-state prior geometry: " +
                             prior_geometry_status.message());
  }
  const auto converter = BaseLinkStateConverter::Create(base_to_imu.value());
  if (!converter.ok()) {
    throw std::runtime_error("invalid base-link static geometry: " +
                             converter.status().message());
  }
  base_link_converter_ = std::make_shared<const BaseLinkStateConverter>(
      converter.value());
  runtime_diagnostics_.static_geometry_ready = true;
  runtime_diagnostics_.static_geometry_source = "robot_state_publisher:/tf_static";
  runtime_diagnostics_.dynamic_tf_owner =
      parameters_.propagated_odometry_enabled ? "propagated" : "corrected";
  output_publisher_.setBaseLinkConverter(base_link_converter_);
  transform_publisher_.setBaseLinkConverter(base_link_converter_);
  if (parameters_.propagated_odometry_enabled) {
    propagated_odometry_publisher_ =
        std::make_unique<RosPropagatedOdometryPublisher>(
            *this, parameters_, output_publisher_.covarianceProjectionRuntime());
    propagated_odometry_publisher_->setBaseLinkConverter(base_link_converter_);
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
            propagated_odometry_publisher_->publish(*estimate);
            transform_publisher_.publishPropagated(*estimate);
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
  if (profile_.estimator.initial_prior.source == InitialStatePriorSource::kTopic) {
    initial_state_prior_subscription_ =
        create_subscription<nav_msgs::msg::Odometry>(
            parameters_.initial_prior_topic, QosProfiles::reliableSensorInput(),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
              onInitialStatePrior(message);
            });
    RCLCPP_INFO(get_logger(), "Initial-state prior topic subscription: %s",
                parameters_.initial_prior_topic.c_str());
  }
  RCLCPP_INFO(get_logger(),
              "Publishing corrected and propagated odometry in %s -> %s; "
              "dynamic TF owner=%s; static geometry resolved from %s -> %s; "
              "config_sha256=%s",
              parameters_.odom_frame.c_str(),
              parameters_.base_frame.c_str(),
              parameters_.propagated_odometry_enabled ? "propagated" : "corrected",
              parameters_.base_frame.c_str(), parameters_.imu_frame.c_str(),
              profile_.config_sha256.c_str());
  transport_diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this] { publishTransportSnapshot(); });
  if (profile_.estimator.initial_prior.source == InitialStatePriorSource::kTopic) {
    initial_prior_startup_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        [this] { startProcessingWorkerWhenPriorPublisherMatches(); });
  } else {
    startProcessingWorker();
  }
}

FastLioNode::~FastLioNode() {
  if (initial_prior_startup_timer_) {
    initial_prior_startup_timer_->cancel();
  }
  {
    std::lock_guard lock(input_mutex_);
    stopping_ = true;
  }
  input_ready_.notify_all();
  if (propagated_odometry_worker_) {
    propagated_odometry_worker_->stop();
  }
  std::thread processing_worker;
  {
    std::lock_guard lock(processing_worker_start_mutex_);
    processing_worker_start_closed_ = true;
    if (processing_worker_.joinable()) {
      processing_worker = std::move(processing_worker_);
    }
  }
  if (processing_worker.joinable()) {
    processing_worker.join();
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

void FastLioNode::onInitialStatePrior(
    const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  InitialStatePrior prior;
  const auto clock_domain = parseClockDomain(parameters_.input_clock_domain);
  prior.sample_time = Timestamp(
      static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
          static_cast<std::int64_t>(message->header.stamp.nanosec),
      clock_domain);
  const std::string source_frame = message->header.frame_id;
  if (source_frame == parameters_.odom_frame &&
      parameters_.initial_prior_source_frame_transform == "same_frame") {
    prior.reference_frame = FrameId(parameters_.odom_frame);
  } else if (source_frame == parameters_.initial_prior_source_frame &&
             parameters_.initial_prior_source_frame_transform == "startup_coincident" &&
             parameters_.initial_prior_context == "ground_startup") {
    // This one-time identity defines lio_odom to coincide with the PX4 prior
    // frame at the sampled startup epoch; it is not a general relabeling.
    prior.reference_frame = FrameId(parameters_.odom_frame);
  } else {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "initial-state prior rejected: source frame %s has no configured transform to %s",
        source_frame.c_str(), parameters_.odom_frame.c_str());
    return;
  }
  prior.body_frame = FrameId(message->child_frame_id);
  prior.source = InitialStatePriorSource::kTopic;
  prior.context = InitialStatePriorContext::kGroundStartup;
  if (parameters_.initial_prior_context == "in_flight_reinitialization") {
    prior.context = InitialStatePriorContext::kInFlightReinitialization;
  }
  prior.mask.position = parameters_.initial_prior_position_enabled;
  prior.mask.velocity = parameters_.initial_prior_velocity_enabled;
  prior.mask.attitude = initialPriorAttitude(parameters_.initial_prior_attitude);
  prior.position_odom_base_m = Eigen::Vector3d(message->pose.pose.position.x,
                                               message->pose.pose.position.y,
                                               message->pose.pose.position.z);
  prior.orientation_odom_base = Eigen::Quaterniond(
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z);
  prior.linear_velocity_base_m_s = Eigen::Vector3d(
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z);
  prior.angular_velocity_base_rad_s = Eigen::Vector3d(
      message->twist.twist.angular.x, message->twist.twist.angular.y,
      message->twist.twist.angular.z);
  prior.provenance = "topic:" + parameters_.initial_prior_topic + "|source_frame=" +
                     source_frame + "|target_frame=" + parameters_.odom_frame +
                     "|transform=" + parameters_.initial_prior_source_frame_transform;
  const auto prior_target_frame = std::string(prior.reference_frame.name());
  const auto prior_body_frame = std::string(prior.body_frame.name());
  const Status status = pipeline_.submitInitialStatePrior(
      std::move(prior), InitialStatePriorLateSubmissionPolicy::kIgnore);
  if (status.code() == StatusCode::kNotReady) {
    closeInitialStatePriorStream();
    return;
  }
  if (!status.ok()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "initial-state prior rejected: %s "
                         "source=%s target=%s body=%s configured_target=%s",
                         status.message().c_str(), source_frame.c_str(),
                         prior_target_frame.c_str(), prior_body_frame.c_str(),
                         parameters_.odom_frame.c_str());
  }
}

void FastLioNode::closeInitialStatePriorStream() {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
  {
    std::lock_guard lock(initial_state_prior_subscription_mutex_);
    subscription = std::move(initial_state_prior_subscription_);
  }
  if (subscription) {
    subscription.reset();
    RCLCPP_INFO_ONCE(get_logger(),
                     "initial-state prior stream closed after startup gate");
  }
}

void FastLioNode::startProcessingWorker() {
  std::lock_guard lock(processing_worker_start_mutex_);
  if (processing_worker_start_closed_ || processing_worker_.joinable()) {
    return;
  }
  processing_worker_ = std::thread([this] { processingLoop(); });
}

void FastLioNode::startProcessingWorkerWhenPriorPublisherMatches() {
  if (!initial_state_prior_subscription_ ||
      initial_state_prior_subscription_->get_publisher_count() == 0U) {
    return;
  }
  if (initial_prior_startup_timer_) {
    initial_prior_startup_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(),
              "Initial-state prior publisher matched; starting estimator worker");
  startProcessingWorker();
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
    const auto pipeline_push_started = std::chrono::steady_clock::now();
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
      const auto pipeline_push_finished = std::chrono::steady_clock::now();
      if (status.ok()) {
        pending_lidar_timings_.push_back(
            PendingLidarTiming{measurement_time_ns, processing_started});
        constexpr std::size_t kMaximumPendingLidarTimings = 8192U;
        if (pending_lidar_timings_.size() > kMaximumPendingLidarTimings) {
          pending_lidar_timings_.pop_front();
        }
      }
      std::lock_guard lock(input_mutex_);
      runtime_statistics_.recordPipelinePushLidar(
          std::chrono::duration_cast<std::chrono::microseconds>(
              pipeline_push_finished - pipeline_push_started)
              .count());
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
  while (true) {
    const auto result_started = std::chrono::steady_clock::now();
    const auto result = pipeline_.processNext();
    if (!result.has_value()) {
      break;
    }
    auto augmented = *result;
    {
      std::lock_guard lock(input_mutex_);
      augmented.diagnostics.sensor = ingress_diagnostics_;
    }
    output_publisher_.publish(augmented);
    if (augmented.diagnostics.initial_prior.applied) {
      closeInitialStatePriorStream();
    }
    if (augmented.corrected_kinematic_estimate.has_value()) {
      transform_publisher_.publishCorrected(
          *augmented.corrected_kinematic_estimate);
    }
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
    const auto result_finished = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(input_mutex_);
      runtime_statistics_.recordResultProcessing(
          std::chrono::duration_cast<std::chrono::microseconds>(
              result_finished - result_started)
              .count());
      if (augmented.scan_time.has_value()) {
        const auto scan_end_ns = augmented.scan_time->nanoseconds();
        for (auto timing = pending_lidar_timings_.begin();
             timing != pending_lidar_timings_.end(); ++timing) {
          if (timing->scan_end_ns != scan_end_ns) {
            continue;
          }
          if (augmented.hasCorrectedOutput()) {
            runtime_statistics_.recordCorrectedScanEndToEnd(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    result_finished - timing->started_at)
                    .count());
            runtime_statistics_.recordRegistrationUpdate(
                augmented.diagnostics.timing.residual_build_us +
                augmented.diagnostics.timing.ikfom_update_us);
          }
          pending_lidar_timings_.erase(timing);
          break;
        }
      }
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
    runtime.covariance_projection =
        output_publisher_.covarianceProjectionRuntime()->snapshot();
    runtime_statistics_.populate(runtime);
    const auto tf = transform_publisher_.diagnostics();
    runtime.dynamic_tf_publication_count = tf.publication_count;
    runtime.dynamic_tf_timestamp_suppressed_count =
        tf.timestamp_suppressed_count;
    runtime.dynamic_tf_conversion_failure_count =
        tf.conversion_failure_count;
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

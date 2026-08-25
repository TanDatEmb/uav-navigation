#include "fast_lio_ros/ros_output_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utility>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_odometry_serializer.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {
namespace {

diagnostic_msgs::msg::KeyValue keyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

}  // namespace

RosOutputPublisher::RosOutputPublisher(
    rclcpp::Node& node, RosParameters parameters,
    std::shared_ptr<LioPublicFrameGeneration> public_frame_generation)
    : parameters_(std::move(parameters)),
      clock_(node.get_clock()),
      covariance_runtime_(std::make_shared<CovarianceProjectionRuntime>()),
      public_frame_generation_(std::move(public_frame_generation)) {
  odometry_ = node.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry_corrected", QosProfiles::estimatorOutput());
  if (parameters_.publish_registered_points) {
    registered_points_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/registered_points", QosProfiles::estimatorOutput());
  }
  registered_scan_ = node.create_publisher<navigation_contracts::msg::RegisteredScan>(
      "/lio/mapping_observation", QosProfiles::estimatorOutput());
  typed_health_ = node.create_publisher<navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", QosProfiles::estimatorOutput());
  diagnostics_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/lio/diagnostics", QosProfiles::estimatorOutput());
}

void RosOutputPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  if (converter) {
    covariance_projector_.emplace(converter->baseToImu());
  }
  base_link_converter_ = std::move(converter);
}

sensor_msgs::msg::PointCloud2 RosOutputPublisher::makeCloud(
    const std::vector<Eigen::Vector3d>& points, const builtin_interfaces::msg::Time& stamp) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = parameters_.odom_frame;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto& point : points) {
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = static_cast<float>(point.z());
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

void RosOutputPublisher::publish(const ProcessResult& result,
                                 const std::uint64_t scan_sequence) {
  EstimatorHealthSnapshot health;
  health.status = result.status_after;
  health.failure_class = result.diagnostics.last_update_failure_class;
  health.corrected_output = result.hasCorrectedOutput();
  health.navigation_valid = result.diagnostics.navigation_valid;
  health.corrected_estimate_valid =
      result.diagnostics.output.corrected_estimate_valid;
  health.failure_reason = result.rejection_reason.empty()
                              ? result.diagnostics.reason
                              : result.rejection_reason;
  health.output_time_ns = result.diagnostics.output.output_time_ns;
  health.last_lidar_correction_time_ns =
      result.diagnostics.output.last_lidar_correction_time_ns;
  health.lio_generation = result.diagnostics.lio_generation;
  health.imu_received_count = result.diagnostics.sensor.ros_received_imu_count;
  health.lidar_received_count = result.diagnostics.sensor.ros_received_lidar_count;
  health.lidar_processed_count = result.diagnostics.sensor.core_accepted_lidar_count;
  health.imu_drop_count = result.diagnostics.sensor.imu_drop_count;
  health.lidar_drop_count = result.diagnostics.sensor.lidar_drop_count;
  health.timestamp_regression_count =
      result.diagnostics.sensor.timestamp_regression_count;
  health.queue_maximum =
      result.diagnostics.sensor.processing_queue_high_water_mark;
  health.correction_accepted_count =
      result.diagnostics.processing.correction_success_count;
  health.correction_rejected_count =
      result.diagnostics.processing.correction_failure_count;
  health.recovery_covariance_clamp_count =
      result.diagnostics.recovery_covariance_clamp_count;
  health.recovery_covariance_maximum_eigenvalue_before_clamp =
      result.diagnostics
          .recovery_covariance_maximum_eigenvalue_before_clamp;
  health.recovery_covariance_maximum_eigenvalue_after_clamp =
      result.diagnostics.recovery_covariance_maximum_eigenvalue_after_clamp;
  health.map_point_count = result.diagnostics.map.map_point_count;
  health.valid_point_count_busy_count =
      result.diagnostics.map.valid_point_count_busy_count;
  health.measurement_callback_count =
      result.diagnostics.registration.measurement_callback_count;
  health.observability_rejection_count =
      result.diagnostics.registration.observability_rejection_count;
  health.translation_observability_min_eigenvalue =
      result.diagnostics.registration.translation_observability_min_eigenvalue;
  health.translation_observability_max_eigenvalue =
      result.diagnostics.registration.translation_observability_max_eigenvalue;
  health.translation_observability_ratio =
      result.diagnostics.registration.translation_observability_ratio;
  health.translation_observability_valid =
      result.diagnostics.registration.translation_observability_valid;
  health.measurement_model_us = result.diagnostics.timing.measurement_model_us;
  health.ikfom_solver_only_us = result.diagnostics.timing.ikfom_solver_only_us;
  health.map_size_after_insert = result.diagnostics.map.map_size_after_insert;
  health.map_size_after_maintenance =
      result.diagnostics.map.map_size_after_maintenance;
  health.crop_performed = result.diagnostics.map.crop_performed;
  health.absolute_guard_triggered =
      result.diagnostics.map.absolute_guard_triggered;
  health.absolute_guard_recovery_failed =
      result.diagnostics.map.absolute_guard_recovery_failed;
  health.map_insertion_frozen = result.diagnostics.map.map_insertion_frozen;
  health.map_maintenance_us = result.diagnostics.map.map_maintenance_us;
  const auto is_bridge_usable = [](const EstimatorHealthSnapshot& snapshot) {
    return snapshot.status == EstimatorStatus::kTracking &&
           snapshot.corrected_output && snapshot.navigation_valid &&
           snapshot.corrected_estimate_valid;
  };
  std::optional<EstimatorHealthSnapshot> transition_health;
  {
    std::scoped_lock lock(diagnostics_mutex_);
    if (!latest_diagnostic_health_.has_value() ||
        is_bridge_usable(*latest_diagnostic_health_) !=
            is_bridge_usable(health)) {
      // The periodic 2 Hz snapshot keeps serialization off the correction hot
      // path. Validity edges are different: the PX4 bridge consumes this
      // status as a safety gate, so delaying a recovery edge until the next
      // timer tick would turn one rejected 10 Hz scan into a 500 ms EV outage.
      transition_health = health;
    }
    latest_diagnostic_health_ = std::move(health);
  }

  const auto diagnostics_stamp =
      result.scan_time.has_value() ? RosTimeConverter::toRos(*result.scan_time)
                                   : static_cast<builtin_interfaces::msg::Time>(clock_->now());
  const bool transitioned_to_usable =
      transition_health.has_value() && is_bridge_usable(*transition_health);
  if (transition_health.has_value() && !transitioned_to_usable) {
    publishDiagnostics(*transition_health, diagnostics_stamp);
  }
  if (!result.hasCorrectedOutput() ||
      !result.corrected_kinematic_estimate.has_value() ||
      !base_link_converter_) {
    publishTypedHealth(health, result, diagnostics_stamp);
    return;
  }
  const auto converted = base_link_converter_->convert(
      result.corrected_kinematic_estimate->estimate,
      result.corrected_kinematic_estimate->angular_velocity_imu_rad_s);
  if (!converted.ok()) {
    publishTypedHealth(health, result, diagnostics_stamp);
    return;
  }
  std::optional<builtin_interfaces::msg::Time> odometry_stamp;
  std::optional<nav_msgs::msg::Odometry> corrected_odometry;
  if (covariance_projector_.has_value() && covariance_runtime_) {
    BaseLinkCovarianceProjectionDiagnostics projection_diagnostics;
    const auto projection_started = std::chrono::steady_clock::now();
    const auto covariance = covariance_projector_->project(
        *result.corrected_kinematic_estimate, converted.value(),
        &projection_diagnostics);
    const auto projection_elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - projection_started)
            .count();
    covariance_runtime_->record(projection_diagnostics, projection_elapsed);
    if (covariance.ok()) {
      const auto odometry = RosOdometrySerializer::serialize(
          converted.value(), covariance.value(), parameters_);
      if (odometry.ok()) {
        odometry_stamp = odometry.value().header.stamp;
        corrected_odometry = odometry.value();
        odometry_->publish(*corrected_odometry);
      }
    }
  }
  const auto stamp = odometry_stamp.value_or(diagnostics_stamp);
  if (parameters_.publish_registered_points && result.hasRegisteredScanOutput()) {
    registered_points_->publish(makeCloud(result.registered_points_odom_m, stamp));
  }
  if (result.hasRegisteredScanOutput() && corrected_odometry.has_value()) {
    navigation_contracts::msg::RegisteredScan observation;
    observation.header = corrected_odometry->header;
    observation.localization_epoch = public_frame_generation_
                                         ? public_frame_generation_->snapshot().generation
                                         : 0U;
    observation.scan_sequence = scan_sequence;
    observation.body_frame_id = corrected_odometry->child_frame_id;
    observation.corrected_pose = corrected_odometry->pose;
    observation.points = makeCloud(result.registered_points_odom_m, stamp);
    registered_scan_->publish(std::move(observation));
  }
  publishTypedHealth(health, result, diagnostics_stamp);
  if (transitioned_to_usable) {
    // Publish the recovery edge only after covariance projection has updated
    // its availability snapshot and corrected odometry has been emitted.
    publishDiagnostics(*transition_health, diagnostics_stamp);
  }
}

void RosOutputPublisher::publishTypedHealth(
    const EstimatorHealthSnapshot& health, const ProcessResult& result,
    const builtin_interfaces::msg::Time& stamp) {
  navigation_contracts::msg::EstimatorHealth message;
  message.header.stamp = stamp;
  const auto public_frame = public_frame_generation_
                                ? public_frame_generation_->snapshot()
                                : LioPublicFrameGenerationSnapshot{0U, false, 0U,
                                                                   "OWNER_UNAVAILABLE"};
  message.localization_epoch = public_frame.generation;
  switch (health.status) {
    case EstimatorStatus::kWaitingForSensors:
      message.state = navigation_contracts::msg::EstimatorHealth::WAITING_FOR_SENSORS;
      break;
    case EstimatorStatus::kCollectingImu:
    case EstimatorStatus::kInitializingImu:
    case EstimatorStatus::kInitializingMap:
      message.state = navigation_contracts::msg::EstimatorHealth::INITIALIZING;
      break;
    case EstimatorStatus::kTracking:
      message.state = navigation_contracts::msg::EstimatorHealth::TRACKING;
      break;
    case EstimatorStatus::kDegraded:
      message.state = navigation_contracts::msg::EstimatorHealth::DEGRADED;
      break;
    case EstimatorStatus::kLost:
      message.state = navigation_contracts::msg::EstimatorHealth::LOST;
      break;
    case EstimatorStatus::kResetting:
      message.state = navigation_contracts::msg::EstimatorHealth::RESETTING;
      break;
  }
  const auto covariance = covariance_runtime_->snapshot();
  message.navigation_valid = health.navigation_valid;
  message.covariance_valid = covariance.pose_covariance_available &&
                             covariance.twist_covariance_available;
  message.observability_valid = health.translation_observability_valid;
  message.correction_fresh = result.hasCorrectedOutput();
  message.propagation_valid = !parameters_.propagated_odometry_enabled ||
                              propagation_valid_.load(std::memory_order_acquire);
  if (health.last_lidar_correction_time_ns > 0) {
    message.last_correction_stamp.sec =
        static_cast<std::int32_t>(health.last_lidar_correction_time_ns / 1'000'000'000LL);
    message.last_correction_stamp.nanosec = static_cast<std::uint32_t>(
        health.last_lidar_correction_time_ns % 1'000'000'000LL);
  }
  message.reason_code = static_cast<std::uint16_t>(health.failure_class);
  typed_health_->publish(std::move(message));
}

void RosOutputPublisher::publishDiagnosticsSnapshot() {
  std::optional<EstimatorHealthSnapshot> health;
  {
    std::scoped_lock lock(diagnostics_mutex_);
    health = latest_diagnostic_health_;
  }
  if (health.has_value()) {
    publishDiagnostics(*health,
                       static_cast<builtin_interfaces::msg::Time>(clock_->now()));
  }
}

void RosOutputPublisher::publishDiagnostics(const EstimatorHealthSnapshot& health,
                                            const builtin_interfaces::msg::Time& stamp) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/estimator";
  status.hardware_id = "lidar_imu";
  status.level =
      health.status == EstimatorStatus::kLost ||
              health.failure_class == LidarUpdateFailureClass::kStateCorruption
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : (health.status == EstimatorStatus::kTracking && health.corrected_output
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = health.failure_reason;
  const auto public_frame = public_frame_generation_
                                ? public_frame_generation_->snapshot()
                                : LioPublicFrameGenerationSnapshot{0U, false, 0U,
                                                                   "OWNER_UNAVAILABLE"};
  // Keep the public diagnostics contract small.  Detailed counters remain
  // available in the report artifacts, while this topic is the health gate
  // consumed by the runtime monitor and PX4 bridge.
  const auto covariance = covariance_runtime_->snapshot();
  status.values = {
      keyValue("state", toString(health.status)),
      keyValue("status", toString(health.status)),
      keyValue("navigation_valid", health.navigation_valid ? "true" : "false"),
      keyValue("corrected_estimate_valid",
               health.corrected_estimate_valid ? "true" : "false"),
      keyValue("translation_observability_valid",
               health.translation_observability_valid ? "true" : "false"),
      keyValue("translation_observability_min_eigenvalue",
               std::to_string(health.translation_observability_min_eigenvalue)),
      keyValue("translation_observability_max_eigenvalue",
               std::to_string(health.translation_observability_max_eigenvalue)),
      keyValue("translation_observability_ratio",
               std::to_string(health.translation_observability_ratio)),
      keyValue("observability_rejection_count",
               std::to_string(health.observability_rejection_count)),
      keyValue("last_failure_code", toString(health.failure_class)),
      keyValue("last_failure_reason", status.message),
      keyValue("output_time_ns", std::to_string(health.output_time_ns)),
      keyValue("last_lidar_correction_time_ns",
               std::to_string(health.last_lidar_correction_time_ns)),
      keyValue("lio_generation", std::to_string(health.lio_generation)),
      keyValue("lio_public_frame_generation", std::to_string(public_frame.generation)),
      keyValue("lio_public_frame_generation_valid", public_frame.valid ? "true" : "false"),
      keyValue("imu_received_count", std::to_string(health.imu_received_count)),
      keyValue("lidar_received_count", std::to_string(health.lidar_received_count)),
      keyValue("lidar_processed_count", std::to_string(health.lidar_processed_count)),
      keyValue("imu_drop_count", std::to_string(health.imu_drop_count)),
      keyValue("lidar_drop_count", std::to_string(health.lidar_drop_count)),
      keyValue("timestamp_regression_count",
               std::to_string(health.timestamp_regression_count)),
      keyValue("queue_maximum", std::to_string(health.queue_maximum)),
      keyValue("correction_accepted_count",
               std::to_string(health.correction_accepted_count)),
      keyValue("correction_rejected_count",
               std::to_string(health.correction_rejected_count)),
      keyValue("recovery_covariance_clamp_count",
               std::to_string(health.recovery_covariance_clamp_count)),
      keyValue("recovery_covariance_maximum_eigenvalue_before_clamp",
               std::to_string(
                   health.recovery_covariance_maximum_eigenvalue_before_clamp)),
      keyValue("recovery_covariance_maximum_eigenvalue_after_clamp",
               std::to_string(
                   health.recovery_covariance_maximum_eigenvalue_after_clamp)),
      keyValue("map_point_count", std::to_string(health.map_point_count)),
      keyValue("pose_covariance_available",
               covariance.pose_covariance_available ? "true" : "false"),
      keyValue("twist_covariance_available",
               covariance.twist_covariance_available ? "true" : "false"),
      keyValue("measurement_callback_count",
               std::to_string(health.measurement_callback_count)),
      keyValue("measurement_model_us",
               std::to_string(health.measurement_model_us)),
      keyValue("ikfom_solver_only_us",
               std::to_string(health.ikfom_solver_only_us)),
      // Small, per-correction map-maintenance contract. These values make a
      // guard/crop event auditable without publishing the full internal
      // diagnostic payload at LiDAR rate.
      keyValue("map_size_after_insert",
               std::to_string(health.map_size_after_insert)),
      keyValue("map_size_after_maintenance",
               std::to_string(health.map_size_after_maintenance)),
      keyValue("valid_point_count_busy_count",
               std::to_string(health.valid_point_count_busy_count)),
      keyValue("crop_performed", health.crop_performed ? "true" : "false"),
      keyValue("absolute_guard_triggered",
               health.absolute_guard_triggered ? "true" : "false"),
      keyValue("absolute_guard_recovery_failed",
               health.absolute_guard_recovery_failed ? "true" : "false"),
      keyValue("map_insertion_frozen",
               health.map_insertion_frozen ? "true" : "false"),
      keyValue("map_maintenance_us",
               std::to_string(health.map_maintenance_us)),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishTransportSnapshot(
    const SensorDiagnostics& sensor,
    const ProcessingStatistics& processing,
    const RuntimeDiagnostics& runtime) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/transport";
  status.hardware_id = "lidar_imu";
  status.level = runtime.overflow_detected || runtime.processing_lag_exceeded
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = runtime.overflow_detected
                       ? "INPUT_QUEUE_OVERFLOW"
                       : (runtime.processing_lag_exceeded
                              ? "PROCESSING_LAG_LIMIT_EXCEEDED"
                              : "TRANSPORT_COUNTER_SNAPSHOT");
  status.values = {
      keyValue("transport_ok", status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ? "true" : "false"),
      keyValue("imu_received_count", std::to_string(sensor.ros_received_imu_count)),
      keyValue("lidar_received_count", std::to_string(sensor.ros_received_lidar_count)),
      keyValue("imu_drop_count", std::to_string(runtime.imu_drop_count)),
      keyValue("lidar_drop_count", std::to_string(runtime.lidar_drop_count)),
      keyValue("timestamp_regression_count",
               std::to_string(runtime.reject_reason_timestamp_regression)),
      keyValue("queue_depth", std::to_string(runtime.current_input_queue_depth)),
      keyValue("queue_maximum", std::to_string(runtime.maximum_queue_depth)),
      keyValue("processing_lag_ns", std::to_string(runtime.processing_lag_ns)),
      keyValue("scan_processing_p95_us", std::to_string(runtime.p95_scan_processing_us)),
      keyValue("correction_accepted_count",
               std::to_string(processing.correction_success_count)),
      keyValue("correction_rejected_count",
               std::to_string(processing.correction_failure_count)),
      keyValue("pose_covariance_available",
               runtime.covariance_projection.pose_covariance_available ? "true" : "false"),
      keyValue("twist_covariance_available",
               runtime.covariance_projection.twist_covariance_available ? "true" : "false"),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishPropagatedOdometryDiagnostics(
    const PropagatedOdometryWorkerDiagnostics& propagated,
    std::uint64_t publication_count,
    std::uint64_t publication_skip_count,
    std::optional<Timestamp> last_published_time,
    std::optional<Timestamp> next_publish_deadline) {
  propagation_valid_.store(
      propagated.propagator.status == PropagatedOdometryStatus::kReady &&
          propagated.navigation_valid,
      std::memory_order_release);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = propagated.propagator.latest_imu_time.has_value()
                           ? RosTimeConverter::toRos(
                                 *propagated.propagator.latest_imu_time)
                           : static_cast<builtin_interfaces::msg::Time>(
                                 clock_->now());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/propagated_odometry";
  status.hardware_id = "lidar_imu";
  status.level = !parameters_.propagated_odometry_enabled
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : propagated.propagator.status ==
                           PropagatedOdometryStatus::kReady
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = toString(propagated.propagator.status);
  const auto timeNs = [](const std::optional<Timestamp>& time) {
    return std::to_string(time.has_value() ? time->nanoseconds() : 0);
  };
  const auto& core = propagated.propagator;
  status.values = {
      keyValue("status", toString(core.status)),
      keyValue("enabled", parameters_.propagated_odometry_enabled ? "true" : "false"),
      keyValue("navigation_valid", propagated.navigation_valid ? "true" : "false"),
      keyValue("latest_imu_time_ns", timeNs(core.latest_imu_time)),
      keyValue("propagated_time_ns", timeNs(core.propagated_time)),
      keyValue("last_correction_time_ns", timeNs(propagated.last_applied_correction_time)),
      keyValue("timestamp_regression_count", std::to_string(core.timestamp_regression_count)),
      keyValue("queue_overflow_count", std::to_string(propagated.queue_overflow_count)),
      keyValue("publication_count", std::to_string(publication_count)),
      keyValue("publication_skip_count", std::to_string(publication_skip_count)),
      keyValue("last_published_time_ns", timeNs(last_published_time)),
      keyValue("next_publish_deadline_ns", timeNs(next_publish_deadline)),
      // These 1 Hz counters explain a propagated-odometry pause without
      // inflating the high-rate estimator health status.
      keyValue("last_received_correction_time_ns", timeNs(propagated.last_correction_time)),
      keyValue("last_applied_correction_time_ns", timeNs(propagated.last_applied_correction_time)),
      keyValue("reanchor_count", std::to_string(core.reanchor_count)),
      keyValue("replay_count", std::to_string(core.replay_count)),
      keyValue("last_replay_sample_count", std::to_string(core.last_replay_sample_count)),
      keyValue("last_replay_runtime_us", std::to_string(propagated.last_replay_runtime_us)),
      keyValue("maximum_replay_runtime_us", std::to_string(propagated.maximum_replay_runtime_us)),
      keyValue("replay_in_progress", propagated.replay_in_progress ? "true" : "false"),
      keyValue("requires_reanchor", core.requires_reanchor ? "true" : "false"),
      keyValue("load_shedding_count", std::to_string(propagated.load_shedding_count)),
      keyValue("maximum_imu_batch_size", std::to_string(propagated.maximum_imu_batch_size)),
      keyValue("stale_stop_count", std::to_string(propagated.stale_stop_count)),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

}  // namespace uav::nav::lio

#include "fast_lio_ros/ros_output_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

void appendRuntimeValues(
    std::vector<diagnostic_msgs::msg::KeyValue>& values,
    const RuntimeDiagnostics& runtime) {
  values.push_back(keyValue("current_input_queue_depth",
                            std::to_string(runtime.current_input_queue_depth)));
  values.push_back(keyValue("current_imu_queue_depth",
                            std::to_string(runtime.current_imu_queue_depth)));
  values.push_back(keyValue("current_lidar_queue_depth",
                            std::to_string(runtime.current_lidar_queue_depth)));
  values.push_back(keyValue("maximum_queue_depth",
                            std::to_string(runtime.maximum_queue_depth)));
  values.push_back(keyValue("maximum_imu_queue_depth",
                            std::to_string(runtime.maximum_imu_queue_depth)));
  values.push_back(keyValue("maximum_lidar_queue_depth",
                            std::to_string(runtime.maximum_lidar_queue_depth)));
  values.push_back(keyValue("imu_queue_capacity",
                            std::to_string(runtime.imu_queue_capacity)));
  values.push_back(keyValue("lidar_queue_capacity",
                            std::to_string(runtime.lidar_queue_capacity)));
  values.push_back(keyValue("received_imu_count",
                            std::to_string(runtime.received_imu_count)));
  values.push_back(keyValue("received_lidar_count",
                            std::to_string(runtime.received_lidar_count)));
  values.push_back(keyValue("processed_imu_count",
                            std::to_string(runtime.processed_imu_count)));
  values.push_back(keyValue("processed_lidar_count",
                            std::to_string(runtime.processed_lidar_count)));
  values.push_back(
      keyValue("imu_drop_count", std::to_string(runtime.imu_drop_count)));
  values.push_back(
      keyValue("lidar_drop_count", std::to_string(runtime.lidar_drop_count)));
  values.push_back(keyValue("reject_reason_imu_buffer_full",
                            std::to_string(runtime.reject_reason_imu_buffer_full)));
  values.push_back(keyValue("reject_reason_lidar_buffer_full",
                            std::to_string(runtime.reject_reason_lidar_buffer_full)));
  values.push_back(keyValue("reject_reason_nonfinite_xyz",
                            std::to_string(runtime.reject_reason_nonfinite_xyz)));
  values.push_back(keyValue("reject_reason_invalid_point_time",
                            std::to_string(runtime.reject_reason_invalid_point_time)));
  values.push_back(keyValue("reject_reason_too_few_points",
                            std::to_string(runtime.reject_reason_too_few_points)));
  values.push_back(keyValue("reject_reason_imu_gap",
                            std::to_string(runtime.reject_reason_imu_gap)));
  values.push_back(keyValue("reject_reason_timestamp_regression",
                            std::to_string(runtime.reject_reason_timestamp_regression)));
  values.push_back(keyValue("reject_reason_not_initialized",
                            std::to_string(runtime.reject_reason_not_initialized)));
  values.push_back(keyValue("reject_reason_correction_failed",
                            std::to_string(runtime.reject_reason_correction_failed)));
  values.push_back(keyValue("latest_received_time_ns",
                            std::to_string(runtime.latest_received_time_ns)));
  values.push_back(keyValue("latest_processed_time_ns",
                            std::to_string(runtime.latest_processed_time_ns)));
  values.push_back(keyValue("processing_lag_ns",
                            std::to_string(runtime.processing_lag_ns)));
  values.push_back(keyValue("worker_heartbeat",
                            std::to_string(runtime.worker_heartbeat)));
  values.push_back(keyValue("worker_last_progress_wall_time_ns",
                            std::to_string(runtime.worker_last_progress_wall_time_ns)));
  values.push_back(keyValue("last_scan_processing_us",
                            std::to_string(runtime.last_scan_processing_us)));
  values.push_back(keyValue("scan_processing_count",
                            std::to_string(runtime.scan_processing_count)));
  values.push_back(keyValue("mean_scan_processing_us",
                            std::to_string(runtime.mean_scan_processing_us)));
  values.push_back(keyValue("p50_scan_processing_us",
                            std::to_string(runtime.p50_scan_processing_us)));
  values.push_back(keyValue("p95_scan_processing_us",
                            std::to_string(runtime.p95_scan_processing_us)));
  values.push_back(keyValue("p99_scan_processing_us",
                            std::to_string(runtime.p99_scan_processing_us)));
  values.push_back(keyValue("maximum_scan_processing_us",
                            std::to_string(runtime.maximum_scan_processing_us)));
  values.push_back(keyValue("pipeline_push_lidar_count",
                            std::to_string(runtime.pipeline_push_lidar_count)));
  values.push_back(keyValue("mean_pipeline_push_lidar_us",
                            std::to_string(runtime.mean_pipeline_push_lidar_us)));
  values.push_back(keyValue("p50_pipeline_push_lidar_us",
                            std::to_string(runtime.p50_pipeline_push_lidar_us)));
  values.push_back(keyValue("p95_pipeline_push_lidar_us",
                            std::to_string(runtime.p95_pipeline_push_lidar_us)));
  values.push_back(keyValue("p99_pipeline_push_lidar_us",
                            std::to_string(runtime.p99_pipeline_push_lidar_us)));
  values.push_back(keyValue("maximum_pipeline_push_lidar_us",
                            std::to_string(runtime.maximum_pipeline_push_lidar_us)));
  values.push_back(keyValue("result_processing_count",
                            std::to_string(runtime.result_processing_count)));
  values.push_back(keyValue("mean_result_processing_us",
                            std::to_string(runtime.mean_result_processing_us)));
  values.push_back(keyValue("p50_result_processing_us",
                            std::to_string(runtime.p50_result_processing_us)));
  values.push_back(keyValue("p95_result_processing_us",
                            std::to_string(runtime.p95_result_processing_us)));
  values.push_back(keyValue("p99_result_processing_us",
                            std::to_string(runtime.p99_result_processing_us)));
  values.push_back(keyValue("maximum_result_processing_us",
                            std::to_string(runtime.maximum_result_processing_us)));
  values.push_back(keyValue("corrected_scan_end_to_end_count",
                            std::to_string(runtime.corrected_scan_end_to_end_count)));
  values.push_back(keyValue("mean_corrected_scan_end_to_end_us",
                            std::to_string(runtime.mean_corrected_scan_end_to_end_us)));
  values.push_back(keyValue("p50_corrected_scan_end_to_end_us",
                            std::to_string(runtime.p50_corrected_scan_end_to_end_us)));
  values.push_back(keyValue("p95_corrected_scan_end_to_end_us",
                            std::to_string(runtime.p95_corrected_scan_end_to_end_us)));
  values.push_back(keyValue("p99_corrected_scan_end_to_end_us",
                            std::to_string(runtime.p99_corrected_scan_end_to_end_us)));
  values.push_back(keyValue("maximum_corrected_scan_end_to_end_us",
                            std::to_string(runtime.maximum_corrected_scan_end_to_end_us)));
  values.push_back(keyValue("registration_update_count",
                            std::to_string(runtime.registration_update_count)));
  values.push_back(keyValue("mean_registration_update_us",
                            std::to_string(runtime.mean_registration_update_us)));
  values.push_back(keyValue("p50_registration_update_us",
                            std::to_string(runtime.p50_registration_update_us)));
  values.push_back(keyValue("p95_registration_update_us",
                            std::to_string(runtime.p95_registration_update_us)));
  values.push_back(keyValue("p99_registration_update_us",
                            std::to_string(runtime.p99_registration_update_us)));
  values.push_back(keyValue("maximum_registration_update_us",
                            std::to_string(runtime.maximum_registration_update_us)));
  values.push_back(keyValue("worker_busy_ratio",
                            std::to_string(runtime.worker_busy_ratio)));
  values.push_back(keyValue("overflow_detected",
                            runtime.overflow_detected ? "true" : "false"));
  values.push_back(keyValue("processing_lag_exceeded",
                            runtime.processing_lag_exceeded ? "true" : "false"));
  values.push_back(keyValue("static_geometry_ready",
                            runtime.static_geometry_ready ? "true" : "false"));
  values.push_back(keyValue("static_geometry_source",
                            runtime.static_geometry_source));
  values.push_back(keyValue("dynamic_tf_owner", runtime.dynamic_tf_owner));
  values.push_back(keyValue("dynamic_tf_publication_count",
                            std::to_string(runtime.dynamic_tf_publication_count)));
  values.push_back(keyValue("dynamic_tf_timestamp_suppressed_count",
                            std::to_string(runtime.dynamic_tf_timestamp_suppressed_count)));
  values.push_back(keyValue("dynamic_tf_conversion_failure_count",
                            std::to_string(runtime.dynamic_tf_conversion_failure_count)));
  const auto& covariance = runtime.covariance_projection;
  values.push_back(keyValue("covariance_semantic",
                            "state_conditional_on_resolved_gyro"));
  values.push_back(keyValue("pose_covariance_available",
                            covariance.pose_covariance_available ? "true" : "false"));
  values.push_back(keyValue("twist_covariance_available",
                            covariance.twist_covariance_available ? "true" : "false"));
  values.push_back(keyValue("pose_covariance_expression_frame", "lio_odom"));
  values.push_back(keyValue("twist_covariance_expression_frame", "base_link"));
  values.push_back(keyValue("covariance_projection_success_count",
                            std::to_string(covariance.projection_success_count)));
  values.push_back(keyValue("covariance_projection_failure_count",
                            std::to_string(covariance.projection_failure_count)));
  values.push_back(keyValue("source_covariance_nonfinite_count",
                            std::to_string(covariance.source_nonfinite_count)));
  values.push_back(keyValue("source_covariance_asymmetry_count",
                            std::to_string(covariance.source_asymmetry_count)));
  values.push_back(keyValue("source_covariance_non_psd_count",
                            std::to_string(covariance.source_non_psd_count)));
  values.push_back(keyValue("source_covariance_zero_count",
                            std::to_string(covariance.source_zero_count)));
  values.push_back(keyValue("output_pose_covariance_nonfinite_count",
                            std::to_string(covariance.output_pose_nonfinite_count)));
  values.push_back(keyValue("output_twist_covariance_nonfinite_count",
                            std::to_string(covariance.output_twist_nonfinite_count)));
  values.push_back(keyValue("output_pose_covariance_non_psd_count",
                            std::to_string(covariance.output_pose_non_psd_count)));
  values.push_back(keyValue("output_twist_covariance_non_psd_count",
                            std::to_string(covariance.output_twist_non_psd_count)));
  values.push_back(keyValue("covariance_roundoff_repair_count",
                            std::to_string(covariance.roundoff_repair_count)));
  values.push_back(keyValue("pose_covariance_trace",
                            std::to_string(covariance.pose_covariance_trace)));
  values.push_back(keyValue("twist_covariance_trace",
                            std::to_string(covariance.twist_covariance_trace)));
  values.push_back(keyValue("pose_covariance_minimum_eigenvalue",
                            std::to_string(covariance.pose_covariance_minimum_eigenvalue)));
  values.push_back(keyValue("twist_covariance_minimum_eigenvalue",
                            std::to_string(covariance.twist_covariance_minimum_eigenvalue)));
  values.push_back(keyValue("covariance_projection_us",
                            std::to_string(covariance.covariance_projection_us)));
  values.push_back(keyValue("maximum_covariance_projection_us",
                            std::to_string(covariance.maximum_covariance_projection_us)));
  values.push_back(keyValue("mean_covariance_projection_us",
                            std::to_string(covariance.mean_covariance_projection_us)));
}

void appendCovarianceProjectionValues(
    std::vector<diagnostic_msgs::msg::KeyValue>& values,
    const CovarianceProjectionRuntimeSnapshot& covariance) {
  values.push_back(keyValue("covariance_semantic",
                            "state_conditional_on_resolved_gyro"));
  values.push_back(keyValue("pose_covariance_available",
                            covariance.pose_covariance_available ? "true" : "false"));
  values.push_back(keyValue("twist_covariance_available",
                            covariance.twist_covariance_available ? "true" : "false"));
  values.push_back(keyValue("pose_covariance_expression_frame", "lio_odom"));
  values.push_back(keyValue("twist_covariance_expression_frame", "base_link"));
  values.push_back(keyValue("covariance_projection_success_count",
                            std::to_string(covariance.projection_success_count)));
  values.push_back(keyValue("covariance_projection_failure_count",
                            std::to_string(covariance.projection_failure_count)));
  values.push_back(keyValue("source_covariance_nonfinite_count",
                            std::to_string(covariance.source_nonfinite_count)));
  values.push_back(keyValue("source_covariance_asymmetry_count",
                            std::to_string(covariance.source_asymmetry_count)));
  values.push_back(keyValue("source_covariance_non_psd_count",
                            std::to_string(covariance.source_non_psd_count)));
  values.push_back(keyValue("source_covariance_zero_count",
                            std::to_string(covariance.source_zero_count)));
  values.push_back(keyValue("output_pose_covariance_nonfinite_count",
                            std::to_string(covariance.output_pose_nonfinite_count)));
  values.push_back(keyValue("output_twist_covariance_nonfinite_count",
                            std::to_string(covariance.output_twist_nonfinite_count)));
  values.push_back(keyValue("output_pose_covariance_non_psd_count",
                            std::to_string(covariance.output_pose_non_psd_count)));
  values.push_back(keyValue("output_twist_covariance_non_psd_count",
                            std::to_string(covariance.output_twist_non_psd_count)));
  values.push_back(keyValue("covariance_roundoff_repair_count",
                            std::to_string(covariance.roundoff_repair_count)));
  values.push_back(keyValue("pose_covariance_trace",
                            std::to_string(covariance.pose_covariance_trace)));
  values.push_back(keyValue("twist_covariance_trace",
                            std::to_string(covariance.twist_covariance_trace)));
  values.push_back(keyValue("pose_covariance_minimum_eigenvalue",
                            std::to_string(covariance.pose_covariance_minimum_eigenvalue)));
  values.push_back(keyValue("twist_covariance_minimum_eigenvalue",
                            std::to_string(covariance.twist_covariance_minimum_eigenvalue)));
  values.push_back(keyValue("covariance_projection_us",
                            std::to_string(covariance.covariance_projection_us)));
  values.push_back(keyValue("maximum_covariance_projection_us",
                            std::to_string(covariance.maximum_covariance_projection_us)));
  values.push_back(keyValue("mean_covariance_projection_us",
                            std::to_string(covariance.mean_covariance_projection_us)));
}

}  // namespace

RosOutputPublisher::RosOutputPublisher(rclcpp::Node& node, RosParameters parameters)
    : parameters_(std::move(parameters)),
      clock_(node.get_clock()),
      covariance_runtime_(std::make_shared<CovarianceProjectionRuntime>()) {
  odometry_ = node.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry_corrected", QosProfiles::estimatorOutput());
  registered_points_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/registered_points", QosProfiles::estimatorOutput());
  local_map_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("/lio/local_map",
                                                                    QosProfiles::mapOutput());
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

void RosOutputPublisher::publish(const ProcessResult& result) {
  const auto diagnostics_stamp =
      result.scan_time.has_value() ? RosTimeConverter::toRos(*result.scan_time)
                                   : static_cast<builtin_interfaces::msg::Time>(clock_->now());
  if (!result.hasCorrectedOutput() ||
      !result.corrected_kinematic_estimate.has_value() ||
      !base_link_converter_) {
    publishDiagnostics(result, diagnostics_stamp);
    return;
  }
  const auto converted = base_link_converter_->convert(
      result.corrected_kinematic_estimate->estimate,
      result.corrected_kinematic_estimate->angular_velocity_imu_rad_s);
  if (!converted.ok()) {
    publishDiagnostics(result, diagnostics_stamp);
    return;
  }
  std::optional<builtin_interfaces::msg::Time> odometry_stamp;
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
        odometry_->publish(odometry.value());
      }
    }
  }
  const auto stamp = odometry_stamp.value_or(diagnostics_stamp);
  if (parameters_.publish_registered_points && result.hasRegisteredScanOutput()) {
    registered_points_->publish(makeCloud(result.registered_points_odom_m, stamp));
  }
  if (parameters_.publish_local_map && !result.local_map_points_odom_m.empty()) {
    local_map_->publish(makeCloud(result.local_map_points_odom_m, stamp));
  }
  publishDiagnostics(result, diagnostics_stamp);
}

void RosOutputPublisher::publishDiagnostics(const ProcessResult& result,
                                            const builtin_interfaces::msg::Time& stamp) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/estimator";
  status.hardware_id = "lidar_imu";
  status.level =
      result.status_after == EstimatorStatus::kLost ||
              result.diagnostics.last_update_failure_class ==
                  LidarUpdateFailureClass::kStateCorruption
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : (result.status_after == EstimatorStatus::kTracking && result.hasCorrectedOutput()
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message =
      result.rejection_reason.empty() ? result.diagnostics.reason : result.rejection_reason;
  status.values = {
      keyValue("diagnostic_schema_version", "1"),
      keyValue("status", toString(result.status_after)),
      keyValue("config_path", parameters_.config_path),
      keyValue("config_sha256", parameters_.config_sha256),
      keyValue("estimate_validity", toString(result.estimate_validity)),
      keyValue("lidar_update_status", toString(result.lidar_update_status)),
      keyValue("predicted_estimate_valid",
               result.diagnostics.output.predicted_estimate_valid ? "true" : "false"),
      keyValue("corrected_estimate_valid",
               result.diagnostics.output.corrected_estimate_valid ? "true" : "false"),
      keyValue("registered_scan_valid",
               result.diagnostics.output.registered_scan_valid ? "true" : "false"),
      keyValue("output_time_ns", std::to_string(result.diagnostics.output.output_time_ns)),
      keyValue("last_lidar_correction_time_ns",
               std::to_string(result.diagnostics.output.last_lidar_correction_time_ns)),
      keyValue("clock_domain", result.diagnostics.output.clock_domain),
      keyValue("deskew_applied", result.diagnostics.deskew.deskew_applied ? "true" : "false"),
      keyValue("imu_samples_per_scan",
               std::to_string(result.diagnostics.synchronization.imu_samples_per_scan)),
      keyValue("accepted_residual_count",
               std::to_string(result.diagnostics.registration.accepted_residual_count)),
      keyValue("residual_rms_m", std::to_string(result.diagnostics.registration.residual_rms_m)),
      keyValue("map_point_count", std::to_string(result.diagnostics.map.map_point_count)),
      keyValue("covariance_trace", std::to_string(result.diagnostics.state.covariance_trace)),
      keyValue("consecutive_uncorrected_lidar_updates",
               std::to_string(result.diagnostics.consecutive_uncorrected_lidar_updates)),
      keyValue("consecutive_recovery_successes",
               std::to_string(result.diagnostics.consecutive_recovery_successes)),
      keyValue("recovery_confirmation_updates_required",
               std::to_string(result.diagnostics.recovery_confirmation_updates_required)),
      keyValue("map_insertion_frozen",
               result.diagnostics.map_insertion_frozen ? "true" : "false"),
      keyValue("navigation_valid",
               result.diagnostics.navigation_valid ? "true" : "false"),
      keyValue("last_update_failure_class",
               toString(result.diagnostics.last_update_failure_class)),
      keyValue("propagation_discontinuity_count",
               std::to_string(result.diagnostics.propagation_discontinuity_count)),
      keyValue("last_propagation_gap_ns",
               std::to_string(result.diagnostics.last_propagation_gap_ns)),
      keyValue("initial_prior_status",
               toString(result.diagnostics.initial_prior.status)),
      keyValue("initial_prior_source",
               toString(result.diagnostics.initial_prior.source)),
      keyValue("initial_prior_context",
               toString(result.diagnostics.initial_prior.context)),
      keyValue("initial_prior_attitude",
               toString(result.diagnostics.initial_prior.attitude_mode)),
      keyValue("initial_prior_applied",
               result.diagnostics.initial_prior.applied ? "true" : "false"),
      keyValue("initial_prior_fallback_applied",
               result.diagnostics.initial_prior.fallback_applied ? "true" : "false"),
      keyValue("initial_prior_candidate_timestamp_ns",
               std::to_string(result.diagnostics.initial_prior.candidate_timestamp_ns)),
      keyValue("initial_prior_application_timestamp_ns",
               std::to_string(result.diagnostics.initial_prior.application_timestamp_ns)),
      keyValue("initial_prior_candidate_age_ns",
               std::to_string(result.diagnostics.initial_prior.candidate_age_ns)),
      keyValue("initial_prior_time_delta_ns",
               std::to_string(result.diagnostics.initial_prior.time_delta_ns)),
      keyValue("initial_prior_clock_domain",
               result.diagnostics.initial_prior.clock_domain),
      keyValue("initial_prior_waiting_for_sensor_time",
               result.diagnostics.initial_prior.waiting_for_sensor_time ? "true" : "false"),
      keyValue("initial_prior_candidate_count",
               std::to_string(result.diagnostics.initial_prior.candidate_count)),
      keyValue("initial_prior_rejected_count",
               std::to_string(result.diagnostics.initial_prior.rejected_count)),
      keyValue("initial_prior_stale_rejected_count",
               std::to_string(result.diagnostics.initial_prior.stale_rejected_count)),
      keyValue("initial_prior_future_rejected_count",
               std::to_string(result.diagnostics.initial_prior.future_rejected_count)),
      keyValue("initial_prior_timestamp_rejected_count",
               std::to_string(result.diagnostics.initial_prior.timestamp_rejected_count)),
      keyValue("initial_prior_wait_timeout_count",
               std::to_string(result.diagnostics.initial_prior.wait_timeout_count)),
      keyValue("initial_prior_zero_fallback_count",
               std::to_string(result.diagnostics.initial_prior.zero_fallback_count)),
      keyValue("initial_prior_reason", result.diagnostics.initial_prior.reason),
      keyValue("corrected_angular_velocity_available",
               result.diagnostics.corrected_angular_velocity.angular_velocity_available
                   ? "true"
                   : "false"),
      keyValue("corrected_angular_velocity_exact_sample_count",
               std::to_string(
                   result.diagnostics.corrected_angular_velocity.exact_sample_count)),
      keyValue("corrected_angular_velocity_interpolated_count",
               std::to_string(
                   result.diagnostics.corrected_angular_velocity.interpolated_count)),
      keyValue("corrected_angular_velocity_missing_bracket_count",
               std::to_string(result.diagnostics.corrected_angular_velocity
                                  .missing_bracket_count)),
      keyValue("corrected_angular_velocity_timestamp_mismatch_count",
               std::to_string(result.diagnostics.corrected_angular_velocity
                                  .timestamp_mismatch_count)),
      keyValue("corrected_angular_velocity_nonfinite_reject_count",
               std::to_string(result.diagnostics.corrected_angular_velocity
                                  .nonfinite_reject_count))};
  status.values.push_back(keyValue(
      "ros_received_imu_count",
      std::to_string(result.diagnostics.sensor.ros_received_imu_count)));
  status.values.push_back(keyValue(
      "ros_received_lidar_count",
      std::to_string(result.diagnostics.sensor.ros_received_lidar_count)));
  status.values.push_back(keyValue(
      "core_accepted_imu_count",
      std::to_string(result.diagnostics.sensor.core_accepted_imu_count)));
  status.values.push_back(keyValue(
      "core_accepted_lidar_count",
      std::to_string(result.diagnostics.sensor.core_accepted_lidar_count)));
  status.values.push_back(keyValue(
      "ros_maximum_imu_gap_ns",
      std::to_string(result.diagnostics.sensor.ros_maximum_imu_gap_ns)));
  status.values.push_back(keyValue(
      "processing_queue_high_water_mark",
      std::to_string(
          result.diagnostics.sensor.processing_queue_high_water_mark)));
  status.values.push_back(keyValue(
      "iteration_count",
      std::to_string(result.diagnostics.registration.iteration_count)));
  status.values.push_back(keyValue(
      "final_increment_norm",
      std::to_string(result.diagnostics.registration.final_increment_norm)));
  status.values.push_back(keyValue(
      "covariance_minimum_eigenvalue",
      std::to_string(
          result.diagnostics.state.covariance_minimum_eigenvalue)));
  status.values.push_back(keyValue(
      "covariance_maximum_asymmetry",
      std::to_string(result.diagnostics.state.covariance_maximum_asymmetry)));
  status.values.push_back(keyValue(
      "prediction_us",
      std::to_string(result.diagnostics.timing.imu_prediction_us)));
  status.values.push_back(keyValue(
      "deskew_us", std::to_string(result.diagnostics.timing.deskew_us)));
  status.values.push_back(keyValue(
      "preprocessing_us",
      std::to_string(result.diagnostics.timing.preprocessing_us)));
  status.values.push_back(keyValue(
      "residual_build_us",
      std::to_string(result.diagnostics.timing.residual_build_us)));
  status.values.push_back(keyValue(
      "ikfom_update_us",
      std::to_string(result.diagnostics.timing.ikfom_update_us)));
  status.values.push_back(keyValue(
      "map_insert_crop_us",
      std::to_string(result.diagnostics.timing.map_insert_crop_us)));
  status.values.push_back(keyValue(
      "map_maintenance_us",
      std::to_string(result.diagnostics.timing.map_maintenance_us)));
  status.values.push_back(keyValue(
      "snapshot_us",
      std::to_string(result.diagnostics.timing.snapshot_us)));
  status.values.push_back(keyValue(
      "total_processing_us",
      std::to_string(result.diagnostics.timing.total_processing_us)));
  const auto& map = result.diagnostics.map;
  status.values.push_back(keyValue("map_size_before_insert", std::to_string(map.map_size_before_insert)));
  status.values.push_back(keyValue("map_candidate_count", std::to_string(map.map_candidate_count)));
  status.values.push_back(keyValue("map_inserted_count", std::to_string(map.map_inserted_count)));
  status.values.push_back(keyValue("map_size_after_insert", std::to_string(map.map_size_after_insert)));
  status.values.push_back(keyValue("crop_performed", map.crop_performed ? "true" : "false"));
  status.values.push_back(keyValue("crop_removed_count", std::to_string(map.crop_removed_count)));
  status.values.push_back(keyValue("crop_triggered_by_motion", map.crop_triggered_by_motion ? "true" : "false"));
  status.values.push_back(keyValue("crop_triggered_by_point_threshold", map.crop_triggered_by_point_threshold ? "true" : "false"));
  status.values.push_back(keyValue("map_size_before_maintenance", std::to_string(map.map_size_before_maintenance)));
  status.values.push_back(keyValue("confidence_pruned_count", std::to_string(map.confidence_pruned_count)));
  status.values.push_back(keyValue("distance_pruned_count", std::to_string(map.distance_pruned_count)));
  status.values.push_back(keyValue("redundancy_pruned_count", std::to_string(map.redundancy_pruned_count)));
  status.values.push_back(keyValue("map_size_after_maintenance", std::to_string(map.map_size_after_maintenance)));
  status.values.push_back(keyValue("local_map_center_x", std::to_string(map.local_map_center_odom_m.x())));
  status.values.push_back(keyValue("local_map_center_y", std::to_string(map.local_map_center_odom_m.y())));
  status.values.push_back(keyValue("local_map_center_z", std::to_string(map.local_map_center_odom_m.z())));
  status.values.push_back(keyValue("local_map_half_extent_x", std::to_string(map.local_map_half_extent_m.x())));
  status.values.push_back(keyValue("local_map_half_extent_y", std::to_string(map.local_map_half_extent_m.y())));
  status.values.push_back(keyValue("local_map_half_extent_z", std::to_string(map.local_map_half_extent_m.z())));
  status.values.push_back(keyValue("snapshot_point_count", std::to_string(map.snapshot_point_count)));
  status.values.push_back(keyValue("dynamic_filter_enabled", map.dynamic_filter_enabled ? "true" : "false"));
  status.values.push_back(keyValue("dynamic_evidence_voxel_count", std::to_string(map.dynamic_evidence_voxel_count)));
  status.values.push_back(keyValue("dynamic_candidate_count", std::to_string(map.dynamic_candidate_count)));
  const auto& processing = result.diagnostics.processing;
  status.values.push_back(keyValue(
      "raw_lidar_count", std::to_string(processing.raw_lidar_count)));
  status.values.push_back(keyValue(
      "buffer_accepted_lidar_count",
      std::to_string(processing.buffer_accepted_lidar_count)));
  status.values.push_back(keyValue(
      "overlap_rejected_count",
      std::to_string(processing.overlap_rejected_count)));
  status.values.push_back(keyValue(
      "missing_bracket_rejected_count",
      std::to_string(processing.missing_bracket_rejected_count)));
  status.values.push_back(keyValue(
      "invalid_timestamp_rejected_count",
      std::to_string(processing.invalid_timestamp_rejected_count)));
  status.values.push_back(keyValue(
      "synchronized_group_count",
      std::to_string(processing.synchronized_group_count)));
  status.values.push_back(keyValue(
      "correction_attempt_count",
      std::to_string(processing.correction_attempt_count)));
  status.values.push_back(keyValue(
      "correction_success_count",
      std::to_string(processing.correction_success_count)));
  status.values.push_back(keyValue(
      "correction_failure_count",
      std::to_string(processing.correction_failure_count)));
  status.values.push_back(keyValue(
      "buffer_acceptance_ratio",
      std::to_string(processing.bufferAcceptanceRatio())));
  status.values.push_back(keyValue(
      "synchronization_ratio",
      std::to_string(processing.synchronizationRatio())));
  status.values.push_back(keyValue(
      "correction_success_ratio",
      std::to_string(processing.correctionSuccessRatio())));
  appendCovarianceProjectionValues(
      status.values, covariance_runtime_->snapshot());
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
      keyValue("config_path", parameters_.config_path),
      keyValue("config_sha256", parameters_.config_sha256),
      keyValue("ros_received_imu_count",
               std::to_string(sensor.ros_received_imu_count)),
      keyValue("ros_received_lidar_count",
               std::to_string(sensor.ros_received_lidar_count)),
      keyValue("core_accepted_imu_count",
               std::to_string(sensor.core_accepted_imu_count)),
      keyValue("core_accepted_lidar_count",
               std::to_string(sensor.core_accepted_lidar_count)),
      keyValue("ros_maximum_imu_gap_ns",
               std::to_string(sensor.ros_maximum_imu_gap_ns)),
      keyValue("processing_queue_high_water_mark",
               std::to_string(sensor.processing_queue_high_water_mark)),
      keyValue("raw_lidar_count",
               std::to_string(processing.raw_lidar_count)),
      keyValue("buffer_accepted_lidar_count",
               std::to_string(processing.buffer_accepted_lidar_count)),
      keyValue("overlap_rejected_count",
               std::to_string(processing.overlap_rejected_count)),
      keyValue("missing_bracket_rejected_count",
               std::to_string(processing.missing_bracket_rejected_count)),
      keyValue("invalid_timestamp_rejected_count",
               std::to_string(processing.invalid_timestamp_rejected_count)),
      keyValue("synchronized_group_count",
               std::to_string(processing.synchronized_group_count)),
      keyValue("correction_attempt_count",
               std::to_string(processing.correction_attempt_count)),
      keyValue("correction_success_count",
               std::to_string(processing.correction_success_count)),
      keyValue("correction_failure_count",
               std::to_string(processing.correction_failure_count)),
      keyValue("buffer_acceptance_ratio",
               std::to_string(processing.bufferAcceptanceRatio())),
      keyValue("synchronization_ratio",
               std::to_string(processing.synchronizationRatio())),
      keyValue("correction_success_ratio",
               std::to_string(processing.correctionSuccessRatio())),
  };
  appendRuntimeValues(status.values, runtime);
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishPropagatedOdometryDiagnostics(
    const PropagatedOdometryWorkerDiagnostics& propagated,
    std::uint64_t publication_count,
    std::uint64_t publication_skip_count,
    std::optional<Timestamp> last_published_time,
    std::optional<Timestamp> next_publish_deadline) {
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
  std::int64_t correction_age_ns = 0;
  if (propagated.propagator.propagated_time.has_value() &&
      propagated.last_applied_correction_time.has_value()) {
    correction_age_ns =
        propagated.propagator.propagated_time->nanoseconds() -
        propagated.last_applied_correction_time->nanoseconds();
  }
  const auto& core = propagated.propagator;
  status.values = {
      keyValue("status", toString(core.status)),
      keyValue("enabled", parameters_.propagated_odometry_enabled ? "true" : "false"),
      keyValue("navigation_valid", propagated.navigation_valid ? "true" : "false"),
      keyValue("latest_imu_time_ns", timeNs(core.latest_imu_time)),
      keyValue("propagated_time_ns", timeNs(core.propagated_time)),
      keyValue("last_received_correction_time_ns", timeNs(propagated.last_correction_time)),
      keyValue("last_applied_correction_time_ns", timeNs(propagated.last_applied_correction_time)),
      keyValue("correction_age_ns", std::to_string(correction_age_ns)),
      keyValue("last_published_time_ns", timeNs(last_published_time)),
      keyValue("next_publish_deadline_ns", timeNs(next_publish_deadline)),
      keyValue("last_received_correction_sequence", std::to_string(propagated.last_received_correction_sequence)),
      keyValue("last_applied_correction_sequence", std::to_string(propagated.last_applied_correction_sequence)),
      keyValue("reanchor_count", std::to_string(core.reanchor_count)),
      keyValue("replay_count", std::to_string(core.replay_count)),
      keyValue("last_replay_sample_count", std::to_string(core.last_replay_sample_count)),
      keyValue("last_replay_runtime_us", std::to_string(propagated.last_replay_runtime_us)),
      keyValue("maximum_replay_runtime_us", std::to_string(propagated.maximum_replay_runtime_us)),
      keyValue("replay_in_progress", propagated.replay_in_progress ? "true" : "false"),
      keyValue("requires_reanchor", core.requires_reanchor ? "true" : "false"),
      keyValue("continuity_epoch", std::to_string(core.continuity_epoch)),
      keyValue("continuity_reset_count", std::to_string(core.continuity_reset_count)),
      keyValue("last_continuity_reset_time_ns", timeNs(core.last_continuity_reset_time)),
      keyValue("timestamp_regression_count", std::to_string(core.timestamp_regression_count)),
      keyValue("imu_gap_count", std::to_string(core.imu_gap_count)),
      keyValue("missing_bracket_count", std::to_string(core.missing_bracket_count)),
      keyValue("queue_overflow_count", std::to_string(propagated.queue_overflow_count)),
      keyValue("load_shedding_count", std::to_string(propagated.load_shedding_count)),
      keyValue("load_shedding_transition_count", std::to_string(propagated.load_shedding_transition_count)),
      keyValue("control_generation", std::to_string(propagated.control_generation)),
      keyValue("worker_wakeup_count", std::to_string(propagated.worker_wakeup_count)),
      keyValue("imu_batch_count", std::to_string(propagated.imu_batch_count)),
      keyValue("total_imu_samples_drained", std::to_string(propagated.total_imu_samples_drained)),
      keyValue("maximum_imu_batch_size", std::to_string(propagated.maximum_imu_batch_size)),
      keyValue("correction_waiting_for_bracket_count", std::to_string(propagated.correction_waiting_for_bracket_count)),
      keyValue("correction_missing_end_wait_count", std::to_string(propagated.correction_missing_end_wait_count)),
      keyValue("correction_missing_start_drop_count", std::to_string(propagated.correction_missing_start_drop_count)),
      keyValue("old_sequence_correction_drop_count", std::to_string(propagated.old_sequence_correction_drop_count)),
      keyValue("old_timestamp_correction_drop_count", std::to_string(propagated.old_timestamp_correction_drop_count)),
      keyValue("duplicate_correction_drop_count", std::to_string(propagated.duplicate_correction_drop_count)),
      keyValue("correction_superseded_during_replay_count", std::to_string(propagated.correction_superseded_during_replay_count)),
      keyValue("correction_coalesced_count", std::to_string(propagated.correction_coalesced_count)),
      keyValue("stale_generation_correction_drop_count", std::to_string(propagated.stale_generation_correction_drop_count)),
      keyValue("suspended_imu_drop_count", std::to_string(propagated.suspended_imu_drop_count)),
      keyValue("stale_stop_count", std::to_string(propagated.stale_stop_count)),
      keyValue("invalid_state_count", std::to_string(core.invalid_state_count)),
      keyValue("publication_count", std::to_string(publication_count)),
      keyValue("publication_skip_count", std::to_string(publication_skip_count)),
      keyValue("current_imu_ingress_depth", std::to_string(propagated.current_imu_ingress_depth)),
      keyValue("maximum_imu_ingress_depth", std::to_string(propagated.maximum_imu_ingress_depth)),
      keyValue("current_imu_history_size", std::to_string(core.current_imu_history_size)),
      keyValue("maximum_imu_history_size", std::to_string(core.maximum_imu_history_size))};
  status.values.push_back(keyValue(
      "angular_velocity_available",
      core.angular_velocity.angular_velocity_available ? "true" : "false"));
  status.values.push_back(keyValue(
      "angular_velocity_exact_sample_count",
      std::to_string(core.angular_velocity.exact_sample_count)));
  status.values.push_back(keyValue(
      "angular_velocity_interpolated_count",
      std::to_string(core.angular_velocity.interpolated_count)));
  status.values.push_back(keyValue(
      "angular_velocity_missing_bracket_count",
      std::to_string(core.angular_velocity.missing_bracket_count)));
  status.values.push_back(keyValue(
      "angular_velocity_timestamp_mismatch_count",
      std::to_string(core.angular_velocity.timestamp_mismatch_count)));
  status.values.push_back(keyValue(
      "angular_velocity_nonfinite_reject_count",
      std::to_string(core.angular_velocity.nonfinite_reject_count)));
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

}  // namespace uav::nav::lio

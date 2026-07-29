#include "fast_lio_ros/ros_output_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "fast_lio_ros/qos_profiles.hpp"
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

RosOutputPublisher::RosOutputPublisher(rclcpp::Node& node, RosParameters parameters)
    : parameters_(std::move(parameters)), clock_(node.get_clock()) {
  odometry_ = node.create_publisher<nav_msgs::msg::Odometry>("/lio/odometry",
                                                             QosProfiles::estimatorOutput());
  registered_points_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/registered_points", QosProfiles::estimatorOutput());
  local_map_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("/lio/local_map",
                                                                    QosProfiles::mapOutput());
  diagnostics_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/lio/diagnostics", QosProfiles::estimatorOutput());
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
  publishDiagnostics(result, diagnostics_stamp);
  if (!result.hasCorrectedOutput()) {
    return;
  }
  const StateEstimate& corrected = *result.corrected_estimate;
  const auto stamp = RosTimeConverter::toRos(corrected.time);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = parameters_.odom_frame;
  odometry.child_frame_id = parameters_.imu_frame;
  const auto& state = corrected.state;
  const auto& position = state.position_odom_imu_m();
  const auto& orientation = state.orientation_odom_imu();
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  const auto& velocity = state.velocity_odom_imu_m_s();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
  odometry_->publish(odometry);
  if (parameters_.publish_registered_points && result.hasRegisteredScanOutput()) {
    registered_points_->publish(makeCloud(result.registered_points_odom_m, stamp));
  }
  if (parameters_.publish_local_map && !result.local_map_points_odom_m.empty()) {
    local_map_->publish(makeCloud(result.local_map_points_odom_m, stamp));
  }
}

void RosOutputPublisher::publishDiagnostics(const ProcessResult& result,
                                            const builtin_interfaces::msg::Time& stamp) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/estimator";
  status.hardware_id = "lidar_imu";
  status.level =
      result.status_after == EstimatorStatus::kLost
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : (result.status_after == EstimatorStatus::kTracking && result.hasCorrectedOutput()
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message =
      result.rejection_reason.empty() ? result.diagnostics.reason : result.rejection_reason;
  status.values = {
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
      keyValue("covariance_trace", std::to_string(result.diagnostics.state.covariance_trace))};
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
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishTransportSnapshot(
    const SensorDiagnostics& sensor,
    const ProcessingStatistics& processing) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/transport";
  status.hardware_id = "lidar_imu";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "TRANSPORT_COUNTER_SNAPSHOT";
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
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

}  // namespace uav::nav::lio

#include "navigation_mapping/mapping_pipeline.hpp"

#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace navigation_mapping {

MappingPipeline::MappingPipeline(MappingPipelineConfig config,
                                 std::function<double()> wall_clock_seconds,
                                 std::string generated_config_directory)
    : config_(config),
      validator_(config_.contract),
      point_filter_(config_.point_filter),
      adapter_(std::move(wall_clock_seconds), std::move(generated_config_directory)) {}

void MappingPipeline::process(const ObservationInput& input) {
  ++diagnostics_.received_observation_count;
  diagnostics_.last_input_stamp_ns =
      static_cast<std::int64_t>(input.header_stamp.sec) * 1'000'000'000 +
      static_cast<std::int64_t>(input.header_stamp.nanosec);

  const auto frame_result = validator_.validateFrames(
      input.header_frame_id, input.points_frame_id, input.header_stamp, input.points_stamp);
  if (!frame_result.valid) {
    if (frame_result.reason == ObservationRejectionReason::kPointsStampMismatch) {
      ++diagnostics_.invalid_stamp_count;
    } else {
      ++diagnostics_.invalid_frame_count;
    }
    return;
  }

  const auto pose_result = validator_.validatePose(input.sensor_pose);
  if (!pose_result.valid) {
    ++diagnostics_.invalid_pose_count;
    return;
  }

  // Generation handling (P1 section 7): decide before touching the map.
  const GenerationDecision decision = generation_tracker_.decide(input.public_frame_generation);
  if (decision == GenerationDecision::kRejectStaleGeneration) {
    ++diagnostics_.old_generation_drop_count;
    return;
  }
  if (decision == GenerationDecision::kResetAndAdoptNewGeneration) {
    adapter_.reset(config_.rog);
    generation_tracker_.adopt(input.public_frame_generation);
    diagnostics_.generation_reset_count = generation_tracker_.resetCount();
  }
  diagnostics_.generation = generation_tracker_.currentGeneration();

  diagnostics_.input_point_count += input.points_lidar_m.size();

  MappingPointFilterStats filter_stats;
  const std::vector<Eigen::Vector3d> filtered_points_lidar_m =
      point_filter_.filter(input.points_lidar_m, &filter_stats);
  diagnostics_.nonfinite_point_count += filter_stats.nonfinite_point_count;
  diagnostics_.filtered_point_count += filtered_points_lidar_m.size();

  const T_odom_lidar sensor_pose{
      Eigen::Vector3d(input.sensor_pose.position.x, input.sensor_pose.position.y,
                      input.sensor_pose.position.z),
      Eigen::Quaterniond(input.sensor_pose.orientation.w, input.sensor_pose.orientation.x,
                         input.sensor_pose.orientation.y, input.sensor_pose.orientation.z)
          .normalized()};

  // The single required transform (P1 section 11): p_odom = T_odom_lidar * p_lidar.
  rog_map::PointCloud cloud_odom_m;
  cloud_odom_m.reserve(filtered_points_lidar_m.size());
  for (const Eigen::Vector3d& point_lidar_m : filtered_points_lidar_m) {
    const Eigen::Vector3d point_odom_m = sensor_pose.apply(point_lidar_m);
    pcl::PointXYZI point;
    point.x = static_cast<float>(point_odom_m.x());
    point.y = static_cast<float>(point_odom_m.y());
    point.z = static_cast<float>(point_odom_m.z());
    point.intensity = 0.0F;
    cloud_odom_m.push_back(point);
  }
  diagnostics_.mapping_point_count += cloud_odom_m.size();

  const auto update_started = std::chrono::steady_clock::now();
  adapter_.updateMap(cloud_odom_m, sensor_pose);
  diagnostics_.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - update_started)
                                   .count();

  ++diagnostics_.accepted_observation_count;
  diagnostics_.last_successful_update_stamp_ns = diagnostics_.last_input_stamp_ns;
}

}  // namespace navigation_mapping

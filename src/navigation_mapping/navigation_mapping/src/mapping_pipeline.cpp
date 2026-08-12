#include "navigation_mapping/mapping_pipeline.hpp"

#include <algorithm>
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
  const auto callback_started = std::chrono::steady_clock::now();
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
  const auto filter_started = std::chrono::steady_clock::now();
  const std::vector<Eigen::Vector3d> filtered_points_lidar_m =
      point_filter_.filter(input.points_lidar_m, &filter_stats);
  diagnostics_.mapping_filter_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - filter_started).count();
  diagnostics_.nonfinite_point_count += filter_stats.nonfinite_point_count;
  diagnostics_.mapping_filter_input_point_count += filter_stats.input_point_count;
  diagnostics_.mapping_filter_output_point_count += filter_stats.output_point_count;
  diagnostics_.filtered_point_count += filtered_points_lidar_m.size();

  const T_odom_lidar sensor_pose{
      Eigen::Vector3d(input.sensor_pose.position.x, input.sensor_pose.position.y,
                      input.sensor_pose.position.z),
      Eigen::Quaterniond(input.sensor_pose.orientation.w, input.sensor_pose.orientation.x,
                         input.sensor_pose.orientation.y, input.sensor_pose.orientation.z)
          .normalized()};

  // The single required transform (P1 section 11): p_odom = T_odom_lidar * p_lidar.
  const auto transform_started = std::chrono::steady_clock::now();
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
  diagnostics_.transform_to_odom_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - transform_started).count();
  diagnostics_.mapping_point_count += cloud_odom_m.size();

  const auto update_started = std::chrono::steady_clock::now();
  adapter_.updateMap(cloud_odom_m, sensor_pose);
  diagnostics_.rog_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - update_started)
                                   .count();
  diagnostics_.map_update_us = diagnostics_.rog_update_us;

  const auto& rog = adapter_.lastDiagnostics();
  diagnostics_.rog_endpoint_count += rog.endpoint_count;
  diagnostics_.rog_ray_attempt_count += rog.attempt_count;
  diagnostics_.rog_ray_processed_count += rog.processed_count;
  diagnostics_.rog_ray_clipped_count += rog.clipped_count;
  diagnostics_.rog_ray_skipped_count += rog.skipped_count;
  diagnostics_.rog_skip_nonfinite += rog.skip_nonfinite;
  diagnostics_.rog_skip_intensity += rog.skip_intensity;
  diagnostics_.rog_skip_point_filter += rog.skip_point_filter;
  diagnostics_.rog_skip_below_raycast_min_range += rog.skip_below_raycast_min_range;
  diagnostics_.rog_skip_endpoint_outside_local_map += rog.skip_endpoint_outside_local_map;
  diagnostics_.rog_clipped_virtual_ground_or_ceiling += rog.clipped_virtual_ground_or_ceiling;
  diagnostics_.rog_clipped_raycast_max_range += rog.clipped_raycast_max_range;
  diagnostics_.rog_clipped_local_update_box += rog.clipped_local_update_box;
  diagnostics_.rog_ray_outside_local_map_step += rog.ray_outside_local_map_step;
  diagnostics_.rog_voxel_traversal_count += rog.voxel_traversal_count_total;
  diagnostics_.rog_voxel_traversal_count_max = std::max(
      diagnostics_.rog_voxel_traversal_count_max, rog.voxel_traversal_count_max);
  diagnostics_.rog_hit_candidate_count += rog.hit_candidate_count;
  diagnostics_.rog_miss_candidate_count += rog.miss_candidate_count;
  diagnostics_.rog_unique_hit_voxel_count += rog.unique_hit_voxel_count;
  diagnostics_.rog_unique_miss_voxel_count += rog.unique_miss_voxel_count;
  diagnostics_.rog_update_cache_entry_count += rog.update_cache_entry_count;
  diagnostics_.rog_unique_update_cache_voxel_count += rog.unique_update_cache_voxel_count;
  diagnostics_.map_slide_check_count += rog.map_slide_check_count;
  diagnostics_.map_slide_count += rog.map_slide_count;
  diagnostics_.map_slide_voxel_shift_x += rog.map_slide_voxel_shift_x;
  diagnostics_.map_slide_voxel_shift_y += rog.map_slide_voxel_shift_y;
  diagnostics_.map_slide_voxel_shift_z += rog.map_slide_voxel_shift_z;
  diagnostics_.map_slide_cells_cleared += rog.map_slide_cells_cleared;
  diagnostics_.rog_inflation_update_count += rog.inflation_update_count;
  diagnostics_.rog_allocated_voxel_count = rog.allocated_voxel_count;
  diagnostics_.rog_total_update_us = rog.rog_total_update_us;
  diagnostics_.rog_raycast_us = rog.rog_raycast_us;
  diagnostics_.rog_probability_update_us = rog.rog_probability_update_us;
  diagnostics_.rog_inflation_us = rog.rog_inflation_us;
  diagnostics_.rog_slide_us = rog.rog_slide_us;
  const auto grid_type = adapter_.map().getGridType(rog_map::Vec3f(
      static_cast<float>(sensor_pose.translation_odom_m.x()),
      static_cast<float>(sensor_pose.translation_odom_m.y()),
      static_cast<float>(sensor_pose.translation_odom_m.z())));
  const auto grid_index = static_cast<std::size_t>(grid_type);
  diagnostics_.sensor_origin_grid_type = grid_index < super_utils::GridTypeStr.size()
                                             ? super_utils::GridTypeStr[grid_index]
                                             : "UNDEFINED";
  diagnostics_.mapping_callback_total_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - callback_started).count();

  ++diagnostics_.accepted_observation_count;
  diagnostics_.last_successful_update_stamp_ns = diagnostics_.last_input_stamp_ns;
}

}  // namespace navigation_mapping

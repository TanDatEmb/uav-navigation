#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace navigation_mapping {

// Section 18 diagnostics surface. Deliberately a plain counters struct: no
// rolling percentile logic here (see P1 section 18 — offline benchmarking
// owns percentile reporting).
struct MappingDiagnostics {
  std::uint64_t received_observation_count{0};
  std::uint64_t accepted_observation_count{0};

  std::uint64_t generation{0};
  std::uint64_t generation_reset_count{0};
  std::uint64_t old_generation_drop_count{0};

  std::uint64_t invalid_stamp_count{0};
  std::uint64_t invalid_frame_count{0};
  std::uint64_t invalid_pose_count{0};
  std::uint64_t nonfinite_point_count{0};
  std::uint64_t post_filter_nonfinite_point_count{0};
  std::uint64_t transform_nonfinite_point_count{0};
  std::uint64_t invalid_cloud_count{0};

  std::uint64_t input_point_count{0};
  std::uint64_t filtered_point_count{0};
  std::uint64_t mapping_point_count{0};

  // Per-observation point/ray accounting. These counters are aggregate
  // totals; percentile calculations remain an offline concern.
  std::uint64_t mapping_filter_input_point_count{0};
  std::uint64_t mapping_filter_output_point_count{0};
  std::uint64_t rog_endpoint_count{0};
  std::uint64_t rog_ray_attempt_count{0};
  std::uint64_t rog_ray_processed_count{0};
  std::uint64_t rog_ray_clipped_count{0};
  std::uint64_t rog_ray_skipped_count{0};
  std::uint64_t rog_skip_nonfinite{0};
  std::uint64_t rog_skip_intensity{0};
  std::uint64_t rog_skip_point_filter{0};
  std::uint64_t rog_skip_below_raycast_min_range{0};
  std::uint64_t rog_skip_endpoint_outside_local_map{0};
  std::uint64_t rog_clipped_virtual_ground_or_ceiling{0};
  std::uint64_t rog_clipped_raycast_max_range{0};
  std::uint64_t rog_clipped_local_update_box{0};
  std::uint64_t rog_ray_outside_local_map_step{0};
  std::uint64_t rog_voxel_traversal_count{0};
  std::uint64_t rog_voxel_traversal_count_max{0};
  std::uint64_t rog_hit_candidate_count{0};
  std::uint64_t rog_miss_candidate_count{0};
  std::uint64_t rog_unique_hit_voxel_count{0};
  std::uint64_t rog_unique_miss_voxel_count{0};
  std::uint64_t rog_update_cache_entry_count{0};
  std::uint64_t rog_unique_update_cache_voxel_count{0};
  std::uint64_t map_slide_check_count{0};
  std::uint64_t map_slide_count{0};
  std::int64_t map_slide_voxel_shift_x{0};
  std::int64_t map_slide_voxel_shift_y{0};
  std::int64_t map_slide_voxel_shift_z{0};
  std::uint64_t map_slide_cells_cleared{0};
  std::uint64_t rog_inflation_update_count{0};
  std::uint64_t rog_allocated_voxel_count{0};

  std::int64_t ros_pointcloud_decode_us{0};
  std::int64_t mapping_filter_us{0};
  std::int64_t transform_to_odom_us{0};
  std::int64_t rog_update_us{0};
  std::int64_t rog_total_update_us{0};
  std::int64_t rog_raycast_us{0};
  std::int64_t rog_probability_update_us{0};
  std::int64_t rog_inflation_us{0};
  std::int64_t rog_slide_us{0};
  std::int64_t mapping_callback_total_us{0};

  std::uint64_t mapping_observation_publish_count{0};
  std::uint64_t mapping_observation_publish_skip_count{0};
  std::uint64_t mapping_observation_receive_count{0};
  std::uint64_t mapping_observation_rejection_count{0};
  std::uint64_t last_received_observation_sequence{0};
  std::uint64_t last_received_observation_stream_id{0};
  std::uint64_t observation_sequence_stream_switch_count{0};
  std::uint64_t observation_sequence_missing_count{0};
  std::uint64_t observation_sequence_duplicate_count{0};
  std::uint64_t observation_sequence_regression_count{0};
  std::uint64_t observation_sequence_max_consecutive_missing{0};

  std::string sensor_origin_grid_type{"NOT_QUERIED"};

  std::int64_t last_input_stamp_ns{0};
  std::int64_t first_input_stamp_ns{0};
  std::int64_t last_successful_update_stamp_ns{0};
  std::int64_t first_callback_wall_ns{0};
  std::int64_t last_callback_wall_ns{0};

  std::int64_t map_update_us{0};

  std::uint64_t processing_exception_count{0};
  std::uint64_t visualization_publish_count{0};
  std::uint64_t visualization_subscriber_count{0};
  std::uint64_t visualization_exception_count{0};
  std::uint64_t visualization_occupied_point_count{0};
  std::uint64_t visualization_inflated_occupied_point_count{0};
  std::uint64_t visualization_unknown_point_count{0};
  std::uint64_t visualization_frontier_point_count{0};
  std::int64_t visualization_occ_query_us{0};
  std::int64_t visualization_inf_occ_query_us{0};
  std::int64_t visualization_unknown_query_us{0};
  std::int64_t visualization_frontier_query_us{0};
  std::int64_t visualization_pointcloud_build_us{0};
  std::int64_t visualization_publish_us{0};
  std::int64_t visualization_total_us{0};
  std::int64_t visualization_source_age_ms{0};
  std::uint64_t map_updates_since_last_visualization{0};

  std::uint64_t latest_slot_replacement_count{0};
};

}  // namespace navigation_mapping

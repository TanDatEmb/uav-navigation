#pragma once

#include <cstddef>
#include <cstdint>

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
  std::uint64_t invalid_cloud_count{0};

  std::uint64_t input_point_count{0};
  std::uint64_t filtered_point_count{0};
  std::uint64_t mapping_point_count{0};

  std::int64_t last_input_stamp_ns{0};
  std::int64_t last_successful_update_stamp_ns{0};

  std::int64_t map_update_us{0};

  std::uint64_t processing_exception_count{0};
  std::uint64_t visualization_publish_count{0};
  std::uint64_t visualization_exception_count{0};
  std::uint64_t visualization_occupied_point_count{0};
  std::uint64_t visualization_inflated_occupied_point_count{0};
  std::uint64_t visualization_unknown_point_count{0};
  std::uint64_t visualization_frontier_point_count{0};

  std::uint64_t latest_slot_replacement_count{0};
};

}  // namespace navigation_mapping

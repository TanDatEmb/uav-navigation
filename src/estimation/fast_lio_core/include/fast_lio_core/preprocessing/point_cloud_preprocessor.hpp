#pragma once

#include <optional>
#include <vector>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/preprocessing/point_filter.hpp"
#include "fast_lio_core/preprocessing/voxel_filter.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

struct PointCloudPreprocessorConfig {
  PointFilterConfig point_filter;
  VoxelFilterConfig voxel_filter;
  bool enable_voxel_filter{true};
  // Retains the common-filtered (post range/finite filter, pre estimator
  // voxelization) point set on PreprocessingResult::mapping_candidate_points.
  // This is the P1 navigation-mapping observation candidate. It must never
  // change estimator voxelization behavior; it only copies points that were
  // already computed for the estimator path.
  bool retain_mapping_candidate{false};
};

struct PreprocessingStats {
  std::size_t input_point_count{0};
  std::size_t range_filtered_point_count{0};
  std::size_t output_point_count{0};
};

struct PreprocessingResult {
  LidarScan scan;
  PreprocessingStats stats;
  // Populated only when PointCloudPreprocessorConfig::retain_mapping_candidate
  // is set. Common-filtered points in the sensor (livox) frame, at the same
  // reference epoch as `scan`, before estimator-only voxelization.
  std::optional<std::vector<LidarPoint>> mapping_candidate_points;
};

class PointCloudPreprocessor {
 public:
  explicit PointCloudPreprocessor(PointCloudPreprocessorConfig config = {});

  [[nodiscard]] Result<PreprocessingResult> process(const LidarScan& scan) const;

 private:
  PointCloudPreprocessorConfig config_;
  PointFilter point_filter_;
  VoxelFilter voxel_filter_;
};

}  // namespace uav::nav::lio

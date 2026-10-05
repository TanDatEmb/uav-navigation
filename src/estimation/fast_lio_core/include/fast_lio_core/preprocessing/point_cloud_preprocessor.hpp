#pragma once

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
};

struct PreprocessingStats {
  std::size_t input_point_count{0};
  std::size_t range_filtered_point_count{0};
  std::size_t output_point_count{0};
};

struct PreprocessingResult {
  LidarScan scan;
  PreprocessingStats stats;
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

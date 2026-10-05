#pragma once

#include <vector>

#include "fast_lio_core/sensor/lidar_point.hpp"

namespace uav::nav::lio {

struct VoxelFilterConfig {
  double voxel_size_m{0.2};
};

class VoxelFilter {
 public:
  explicit VoxelFilter(VoxelFilterConfig config = {});

  [[nodiscard]] std::vector<LidarPoint> filter(const std::vector<LidarPoint>& points) const;

 private:
  VoxelFilterConfig config_;
};

}  // namespace uav::nav::lio

#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace navigation_mapping {

struct MappingPointFilterConfig {
  // 0 or negative disables the guard.
  double minimum_range_m{0.0};
  double maximum_range_m{0.0};
  double voxel_size_m{0.20};
};

struct MappingPointFilterStats {
  std::size_t input_point_count{0};
  std::size_t nonfinite_point_count{0};
  std::size_t range_filtered_point_count{0};
  std::size_t output_point_count{0};
};

// Deliberately minimal: this is not a generic filter graph (P1 section 12).
// Operates in the sensor (lidar) frame, before the single transform into
// lio_odom, matching the estimator's own common-filter/voxelize ordering.
class MappingPointFilter {
 public:
  explicit MappingPointFilter(MappingPointFilterConfig config = {});

  [[nodiscard]] std::vector<Eigen::Vector3d> filter(
      const std::vector<Eigen::Vector3d>& points_lidar_m,
      MappingPointFilterStats* stats = nullptr) const;

 private:
  MappingPointFilterConfig config_;
};

}  // namespace navigation_mapping

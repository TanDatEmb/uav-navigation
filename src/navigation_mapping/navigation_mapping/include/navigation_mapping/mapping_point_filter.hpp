#pragma once

#include <cstddef>
#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace navigation_mapping {

// Compact ROS-boundary representation for decoded PointCloud2 XYZ values.
// The mapping filter deliberately widens components to double for range,
// voxel-key, and centroid arithmetic so the existing map semantics remain
// unchanged.
struct Point3f {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};

  Point3f() = default;
  Point3f(double x_value, double y_value, double z_value)
      : x(static_cast<float>(x_value)),
        y(static_cast<float>(y_value)),
        z(static_cast<float>(z_value)) {}

  [[nodiscard]] bool allFinite() const noexcept {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

struct MappingPointFilterConfig {
  // Navigation endpoint acceptance guard, measured from the LiDAR origin.
  // Points with range < minimum_range_m are rejected before mapping
  // voxelization and therefore cannot reach ROG as endpoints or rays. A
  // value of 0 or negative disables the guard.
  double minimum_range_m{0.0};
  double maximum_range_m{0.0};
  double voxel_size_m{0.20};
};

struct MappingPointFilterStats {
  std::size_t input_point_count{0};
  std::size_t nonfinite_point_count{0};
  std::size_t range_filtered_point_count{0};
  std::size_t post_filter_nonfinite_point_count{0};
  std::size_t output_point_count{0};
};

// Deliberately minimal: this is not a generic filter graph.
// Operates in the sensor (lidar) frame, before the single transform into
// lio_odom, matching the estimator's own common-filter/voxelize ordering.
class MappingPointFilter {
 public:
  explicit MappingPointFilter(MappingPointFilterConfig config = {});

  [[nodiscard]] std::vector<Eigen::Vector3d> filter(
      const std::vector<Point3f>& points_lidar_m,
      MappingPointFilterStats* stats = nullptr) const;

 private:
  MappingPointFilterConfig config_;
};

}  // namespace navigation_mapping

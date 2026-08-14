#include "navigation_mapping/mapping_point_filter.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace navigation_mapping {
namespace {

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct VoxelAccumulator {
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  std::size_t count{0};
};

VoxelKey voxelKeyOf(const Point3f& point, double voxel_size_m) {
  const double x = static_cast<double>(point.x);
  const double y = static_cast<double>(point.y);
  const double z = static_cast<double>(point.z);
  return VoxelKey{static_cast<std::int64_t>(std::floor(x / voxel_size_m)),
                  static_cast<std::int64_t>(std::floor(y / voxel_size_m)),
                  static_cast<std::int64_t>(std::floor(z / voxel_size_m))};
}

}  // namespace

MappingPointFilter::MappingPointFilter(MappingPointFilterConfig config)
    : config_(config) {}

std::vector<Eigen::Vector3d> MappingPointFilter::filter(
    const std::vector<Point3f>& points_lidar_m, MappingPointFilterStats* stats) const {
  MappingPointFilterStats local_stats;
  local_stats.input_point_count = points_lidar_m.size();

  std::vector<Eigen::Vector3d> output;
  output.reserve(points_lidar_m.size());
  const bool range_guard_enabled =
      config_.minimum_range_m > 0.0 || config_.maximum_range_m > 0.0;
  const auto acceptedPoint = [&](const Point3f& point) {
    if (!point.allFinite()) {
      ++local_stats.nonfinite_point_count;
      return false;
    }
    const Eigen::Vector3d point_double(static_cast<double>(point.x),
                                       static_cast<double>(point.y),
                                       static_cast<double>(point.z));
    if (range_guard_enabled) {
      const double range_m = point_double.norm();
      // Keep this strict: the product boundary is range < min_range_m.
      // mapping.raycast.min_range_m is a separate ROG traversal parameter;
      // rejected points stop here and are never clipped into a ray.
      if ((config_.minimum_range_m > 0.0 && range_m < config_.minimum_range_m) ||
          (config_.maximum_range_m > 0.0 && range_m > config_.maximum_range_m)) {
        ++local_stats.range_filtered_point_count;
        return false;
      }
    }
    return true;
  };

  if (config_.voxel_size_m <= 0.0) {
    for (const Point3f& point : points_lidar_m) {
      if (!acceptedPoint(point)) continue;
      const Eigen::Vector3d point_double(static_cast<double>(point.x),
                                         static_cast<double>(point.y),
                                         static_cast<double>(point.z));
      output.push_back(point_double);
    }
  } else {
    // Centroid-per-voxel downsample, matching the estimator's own voxel
    // filter semantics (see fast_lio_core VoxelFilter) at a coarser,
    // mapping-specific resolution. Deliberately not shared code: this is the
    // mapping-side voxelization, not the estimator's.
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> accumulator;
    accumulator.reserve(points_lidar_m.size());
    for (const Point3f& point : points_lidar_m) {
      if (!acceptedPoint(point)) continue;
      const Eigen::Vector3d point_double(static_cast<double>(point.x),
                                         static_cast<double>(point.y),
                                         static_cast<double>(point.z));
      auto& entry = accumulator[voxelKeyOf(point, config_.voxel_size_m)];
      entry.sum += point_double;
      ++entry.count;
    }
    output.reserve(accumulator.size());
    for (const auto& [key, accumulator_entry] : accumulator) {
      static_cast<void>(key);
      const Eigen::Vector3d centroid =
          accumulator_entry.sum / static_cast<double>(accumulator_entry.count);
      if (!centroid.allFinite()) {
        ++local_stats.post_filter_nonfinite_point_count;
        continue;
      }
      output.push_back(centroid);
    }
  }
  local_stats.output_point_count = output.size();
  if (stats != nullptr) {
    *stats = local_stats;
  }
  return output;
}

}  // namespace navigation_mapping

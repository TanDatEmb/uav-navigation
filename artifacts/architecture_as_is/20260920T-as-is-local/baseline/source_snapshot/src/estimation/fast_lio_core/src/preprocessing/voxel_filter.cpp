#include "fast_lio_core/preprocessing/voxel_filter.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_set>

namespace uav::nav::lio {
namespace {

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  friend bool operator==(const VoxelKey&, const VoxelKey&) = default;
};

struct VoxelKeyHash {
  [[nodiscard]] std::size_t operator()(const VoxelKey& key) const noexcept {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

}  // namespace

VoxelFilter::VoxelFilter(VoxelFilterConfig config) : config_(config) {}

std::vector<LidarPoint> VoxelFilter::filter(const std::vector<LidarPoint>& points) const {
  if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0) {
    return {};
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(points.size());
  std::vector<LidarPoint> output;
  output.reserve(points.size());
  for (const auto& point : points) {
    if (!point.allFinite()) {
      continue;
    }
    const Eigen::Vector3d scaled = point.position_lidar_m.cast<double>() / config_.voxel_size_m;
    constexpr double kMinimumIndex = static_cast<double>(std::numeric_limits<std::int64_t>::min());
    constexpr double kMaximumIndex = static_cast<double>(std::numeric_limits<std::int64_t>::max());
    if (!scaled.allFinite() || (scaled.array() < kMinimumIndex).any() ||
        (scaled.array() >= kMaximumIndex).any()) {
      continue;
    }
    const VoxelKey key{static_cast<std::int64_t>(std::floor(scaled.x())),
                       static_cast<std::int64_t>(std::floor(scaled.y())),
                       static_cast<std::int64_t>(std::floor(scaled.z()))};
    if (occupied.insert(key).second) {
      output.push_back(point);
    }
  }
  return output;
}

}  // namespace uav::nav::lio

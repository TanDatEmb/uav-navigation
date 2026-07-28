#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace uav::nav::lio {
namespace {

std::size_t mixHash(std::size_t seed, std::uint64_t value) noexcept {
  constexpr std::size_t kMagic = static_cast<std::size_t>(0x9e3779b97f4a7c15ULL);
  return seed ^ (static_cast<std::size_t>(value) + kMagic + (seed << 6U) + (seed >> 2U));
}

bool lexicographicPointLess(const Eigen::Vector3d& left, const Eigen::Vector3d& right) {
  return std::tie(left.x(), left.y(), left.z()) < std::tie(right.x(), right.y(), right.z());
}

}  // namespace

IkdTreeRegistrationMap::IkdTreeRegistrationMap(IkdTreeRegistrationMapConfig config)
    : config_(config) {
  if (!(config_.voxel_size_m > 0.0) || !std::isfinite(config_.voxel_size_m)) {
    throw std::invalid_argument("registration map voxel size must be finite and positive");
  }
}

NearestNeighborResult IkdTreeRegistrationMap::nearestNeighbors(const Eigen::Vector3d& query_odom_m,
                                                               std::size_t neighbor_count,
                                                               double maximum_distance_m) const {
  NearestNeighborResult result;
  if (!query_odom_m.allFinite() || neighbor_count == 0U || !(maximum_distance_m > 0.0) ||
      !std::isfinite(maximum_distance_m)) {
    return result;
  }

  struct Candidate {
    double squared_distance_m2;
    Eigen::Vector3d point_odom_m;
  };
  std::vector<Candidate> candidates;
  const double maximum_squared_distance_m2 = maximum_distance_m * maximum_distance_m;
  {
    std::scoped_lock lock(mutex_);
    candidates.reserve(voxels_.size());
    for (const auto& [index, accumulator] : voxels_) {
      static_cast<void>(index);
      const Eigen::Vector3d point_odom_m = accumulator.centroid();
      const double squared_distance_m2 = (point_odom_m - query_odom_m).squaredNorm();
      if (std::isfinite(squared_distance_m2) &&
          squared_distance_m2 <= maximum_squared_distance_m2) {
        candidates.push_back({squared_distance_m2, point_odom_m});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& left, const Candidate& right) {
              if (left.squared_distance_m2 != right.squared_distance_m2) {
                return left.squared_distance_m2 < right.squared_distance_m2;
              }
              return lexicographicPointLess(left.point_odom_m, right.point_odom_m);
            });
  const std::size_t count = std::min(neighbor_count, candidates.size());
  result.points_odom_m.reserve(count);
  result.squared_distances_m2.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    result.points_odom_m.push_back(candidates[index].point_odom_m);
    result.squared_distances_m2.push_back(candidates[index].squared_distance_m2);
  }
  return result;
}

std::size_t IkdTreeRegistrationMap::insert(std::span<const Eigen::Vector3d> points_odom_m) {
  std::size_t newly_represented_points = 0;
  std::scoped_lock lock(mutex_);
  for (const Eigen::Vector3d& point_odom_m : points_odom_m) {
    if (!point_odom_m.allFinite()) {
      continue;
    }
    const VoxelIndex index = voxelIndex(point_odom_m);
    auto [iterator, inserted] = voxels_.try_emplace(index, VoxelAccumulator{});
    VoxelAccumulator& accumulator = iterator->second;
    accumulator.sum_odom_m += point_odom_m;
    ++accumulator.sample_count;
    newly_represented_points += inserted ? 1U : 0U;
  }
  return newly_represented_points;
}

std::size_t IkdTreeRegistrationMap::cropLocal(const Eigen::Vector3d& center_odom_m,
                                              const Eigen::Vector3d& half_extent_m) {
  if (!center_odom_m.allFinite() || !half_extent_m.allFinite() ||
      (half_extent_m.array() <= 0.0).any()) {
    return 0U;
  }
  std::size_t removed = 0;
  std::scoped_lock lock(mutex_);
  for (auto iterator = voxels_.begin(); iterator != voxels_.end();) {
    const Eigen::Vector3d delta = (iterator->second.centroid() - center_odom_m).cwiseAbs();
    if ((delta.array() > half_extent_m.array()).any()) {
      iterator = voxels_.erase(iterator);
      ++removed;
    } else {
      ++iterator;
    }
  }
  return removed;
}

std::vector<Eigen::Vector3d> IkdTreeRegistrationMap::snapshot() const {
  std::vector<Eigen::Vector3d> points;
  {
    std::scoped_lock lock(mutex_);
    points.reserve(voxels_.size());
    for (const auto& [index, accumulator] : voxels_) {
      static_cast<void>(index);
      points.push_back(accumulator.centroid());
    }
  }
  std::sort(points.begin(), points.end(), lexicographicPointLess);
  return points;
}

std::size_t IkdTreeRegistrationMap::size() const noexcept {
  std::scoped_lock lock(mutex_);
  return voxels_.size();
}

void IkdTreeRegistrationMap::clear() {
  std::scoped_lock lock(mutex_);
  voxels_.clear();
}

std::size_t IkdTreeRegistrationMap::VoxelIndexHash::operator()(
    const VoxelIndex& index) const noexcept {
  std::size_t seed = 0U;
  seed = mixHash(seed, static_cast<std::uint64_t>(index.x));
  seed = mixHash(seed, static_cast<std::uint64_t>(index.y));
  return mixHash(seed, static_cast<std::uint64_t>(index.z));
}

Eigen::Vector3d IkdTreeRegistrationMap::VoxelAccumulator::centroid() const noexcept {
  if (sample_count == 0U) {
    return Eigen::Vector3d::Zero();
  }
  return sum_odom_m / static_cast<double>(sample_count);
}

IkdTreeRegistrationMap::VoxelIndex IkdTreeRegistrationMap::voxelIndex(
    const Eigen::Vector3d& point_odom_m) const noexcept {
  return {
      static_cast<std::int64_t>(std::floor(point_odom_m.x() / config_.voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point_odom_m.y() / config_.voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point_odom_m.z() / config_.voxel_size_m)),
  };
}

}  // namespace uav::nav::lio

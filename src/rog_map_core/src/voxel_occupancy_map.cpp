#include "rog_map_core/voxel_occupancy_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav::nav::rog {
namespace { constexpr double kEpsilon = 1e-9; }

std::size_t VoxelOccupancyMap::KeyHash::operator()(const Key& key) const noexcept {
  std::size_t hash = 1469598103934665603ULL;
  for (const auto value : {key.x, key.y, key.z}) {
    hash ^= static_cast<std::size_t>(value);
    hash *= 1099511628211ULL;
  }
  return hash;
}

VoxelOccupancyMap::VoxelOccupancyMap(VoxelMapConfig config) : config_(config) {
  if (!std::isfinite(config_.resolution_m) || config_.resolution_m <= 0.0 ||
      !config_.size_m.allFinite() || (config_.size_m.array() <= 0.0).any() ||
      !std::isfinite(config_.inflation_radius_m) || config_.inflation_radius_m < 0.0 ||
      !std::isfinite(config_.raycast_min_range_m) || config_.raycast_min_range_m < 0.0 ||
      !std::isfinite(config_.raycast_max_range_m) ||
      config_.raycast_max_range_m <= config_.raycast_min_range_m ||
      !std::isfinite(config_.hit_log_odds) || !std::isfinite(config_.miss_log_odds) ||
      !std::isfinite(config_.occupied_threshold) || !std::isfinite(config_.clamp_min) ||
      !std::isfinite(config_.clamp_max) || config_.clamp_min >= config_.clamp_max ||
      !std::isfinite(config_.shift_threshold_m) || config_.shift_threshold_m < 0.0) {
    throw std::invalid_argument("invalid bounded voxel map configuration");
  }
  rebuildBounds();
}

void VoxelOccupancyMap::reset() {
  cells_.clear(); inflated_voxel_count_ = 0; occupied_voxel_count_ = 0;
  center_.setZero(); shift_count_ = 0;
  validity_ = MapValidity::kWaitingForLio; rebuildBounds();
}

VoxelOccupancyMap::Key VoxelOccupancyMap::keyFor(const Eigen::Vector3d& point) const noexcept {
  return {static_cast<std::int64_t>(std::floor(point.x() / config_.resolution_m)),
          static_cast<std::int64_t>(std::floor(point.y() / config_.resolution_m)),
          static_cast<std::int64_t>(std::floor(point.z() / config_.resolution_m))};
}

Eigen::Vector3d VoxelOccupancyMap::keyCenter(const Key& key) const noexcept {
  return Eigen::Vector3d(static_cast<double>(key.x) + 0.5,
                         static_cast<double>(key.y) + 0.5,
                         static_cast<double>(key.z) + 0.5) * config_.resolution_m;
}

bool VoxelOccupancyMap::keyInBounds(const Key& key) const noexcept {
  return bounds_.contains(keyCenter(key));
}

void VoxelOccupancyMap::rebuildBounds() {
  const Eigen::Vector3d half = config_.size_m * 0.5;
  bounds_.min = center_ - half; bounds_.max = center_ + half;
}

void VoxelOccupancyMap::evictOutsideBounds() {
  for (auto iterator = cells_.begin(); iterator != cells_.end();) {
    if (!keyInBounds(iterator->first)) {
      if (iterator->second.log_odds > config_.occupied_threshold) --occupied_voxel_count_;
      iterator = cells_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void VoxelOccupancyMap::shiftIfNeeded(const Eigen::Vector3d& origin) {
  if (!origin.allFinite()) return;
  const Eigen::Array3d distance = (origin - center_).cwiseAbs().array();
  const Eigen::Array3d limit =
      (config_.size_m.array() * 0.5 - config_.shift_threshold_m).max(0.0);
  if ((distance <= limit).all()) return;
  center_ = (origin / config_.resolution_m).array().floor().matrix() * config_.resolution_m;
  rebuildBounds(); evictOutsideBounds(); ++shift_count_;
}

void VoxelOccupancyMap::updateCell(const Key& key, const double delta) {
  if (!keyInBounds(key)) return;
  auto iterator = cells_.find(key);
  if (iterator == cells_.end()) iterator = cells_.emplace(key, Cell{}).first;
  const bool was_occupied = iterator->second.log_odds > config_.occupied_threshold;
  iterator->second.log_odds = std::clamp(
      iterator->second.log_odds + delta, config_.clamp_min, config_.clamp_max);
  const bool is_occupied = iterator->second.log_odds > config_.occupied_threshold;
  if (!was_occupied && is_occupied) {
    ++occupied_voxel_count_;
  } else if (was_occupied && !is_occupied) {
    --occupied_voxel_count_;
  }
}

void VoxelOccupancyMap::raycast(const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& endpoint,
                                MapUpdateStats& stats,
                                std::unordered_set<Key, KeyHash>& free_updated) {
  const Eigen::Vector3d delta = endpoint - origin;
  const double length = delta.norm();
  if (!std::isfinite(length) || length < kEpsilon) return;
  const Key end_key = keyFor(endpoint);
  Key key = keyFor(origin);
  const Eigen::Vector3d direction = delta / length;
  const Eigen::Vector3d boundary(
      direction.x() >= 0.0 ? static_cast<double>(key.x + 1) : static_cast<double>(key.x),
      direction.y() >= 0.0 ? static_cast<double>(key.y + 1) : static_cast<double>(key.y),
      direction.z() >= 0.0 ? static_cast<double>(key.z + 1) : static_cast<double>(key.z));
  const Eigen::Vector3d voxel_boundary = boundary * config_.resolution_m;
  Eigen::Vector3d t_max, t_delta;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < kEpsilon) {
      t_max[axis] = std::numeric_limits<double>::infinity();
      t_delta[axis] = std::numeric_limits<double>::infinity();
    } else {
      t_max[axis] = (voxel_boundary[axis] - origin[axis]) / direction[axis];
      t_delta[axis] = config_.resolution_m / std::abs(direction[axis]);
    }
  }
  std::size_t guard = 0;
  const std::size_t max_steps = static_cast<std::size_t>(
      std::ceil(length / config_.resolution_m) * 4.0 + 16.0);
  while (!(key == end_key) && guard++ < max_steps) {
    if (keyInBounds(key) && free_updated.insert(key).second) {
      updateCell(key, config_.miss_log_odds);
      ++stats.free_voxels_updated;
    }
    const int axis = t_max.x() < t_max.y()
                         ? (t_max.x() < t_max.z() ? 0 : 2)
                         : (t_max.y() < t_max.z() ? 1 : 2);
    if (axis == 0) key.x += direction.x() >= 0.0 ? 1 : -1;
    if (axis == 1) key.y += direction.y() >= 0.0 ? 1 : -1;
    if (axis == 2) key.z += direction.z() >= 0.0 ? 1 : -1;
    t_max[axis] += t_delta[axis];
  }
  if (key == end_key && keyInBounds(end_key)) {
    updateCell(end_key, config_.hit_log_odds); ++stats.occupied_voxels_updated;
  }
}

std::size_t VoxelOccupancyMap::inflatedVoxelEstimate() const noexcept {
  const auto radius = static_cast<std::int64_t>(
      std::ceil(config_.inflation_radius_m / config_.resolution_m));
  const auto side = static_cast<std::size_t>(2 * radius + 1);
  const auto stencil = side * side * side;
  const auto x = static_cast<std::size_t>(std::ceil(config_.size_m.x() / config_.resolution_m));
  const auto y = static_cast<std::size_t>(std::ceil(config_.size_m.y() / config_.resolution_m));
  const auto z = static_cast<std::size_t>(std::ceil(config_.size_m.z() / config_.resolution_m));
  const auto bounded_volume = x * y * z;
  const auto occupied = occupied_voxel_count_;
  if (occupied == 0 || stencil == 0 || occupied > bounded_volume / stencil) return bounded_volume;
  return std::min(bounded_volume, occupied * stencil);
}

MapUpdateStats VoxelOccupancyMap::update(
    const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points) {
  MapUpdateStats stats; stats.points_received = points.size(); shiftIfNeeded(origin);
  if (!origin.allFinite()) return stats;
  std::unordered_set<Key, KeyHash> free_updated;
  free_updated.reserve(std::min<std::size_t>(points.size() * 8U, 1'000'000U));
  for (const auto& point : points) {
    if (!point.allFinite()) continue;
    const double range = (point - origin).norm();
    if (!std::isfinite(range) || range < config_.raycast_min_range_m) continue;
    if (range > config_.raycast_max_range_m) {
      raycast(origin, origin + (point - origin) * (config_.raycast_max_range_m / range),
              stats, free_updated);
    } else {
      raycast(origin, point, stats, free_updated); ++stats.points_integrated;
    }
  }
  stats.shift_count = shift_count_; stats.occupied_voxel_count = occupied_voxel_count_;
  inflated_voxel_count_ = inflatedVoxelEstimate();
  stats.inflated_voxel_count = inflated_voxel_count_; stats.allocated_voxel_count = cells_.size();
  return stats;
}

VoxelState VoxelOccupancyMap::query(const Eigen::Vector3d& position) const {
  if (!position.allFinite() || !bounds_.contains(position)) return VoxelState::kOutside;
  const auto iterator = cells_.find(keyFor(position));
  if (iterator == cells_.end()) return VoxelState::kUnknown;
  return iterator->second.log_odds > config_.occupied_threshold ? VoxelState::kOccupied
                                                                  : VoxelState::kFree;
}

bool VoxelOccupancyMap::isInflatedOccupied(const Eigen::Vector3d& position) const {
  if (!position.allFinite() || !bounds_.contains(position)) return false;
  const Key query = keyFor(position);
  const auto radius = static_cast<std::int64_t>(
      std::ceil(config_.inflation_radius_m / config_.resolution_m));
  for (std::int64_t dx = -radius; dx <= radius; ++dx) {
    for (std::int64_t dy = -radius; dy <= radius; ++dy) {
      for (std::int64_t dz = -radius; dz <= radius; ++dz) {
        const Eigen::Vector3d offset(static_cast<double>(dx), static_cast<double>(dy),
                                     static_cast<double>(dz));
        if (offset.norm() * config_.resolution_m > config_.inflation_radius_m + kEpsilon) continue;
        const Key candidate{query.x + dx, query.y + dy, query.z + dz};
        const auto iterator = cells_.find(candidate);
        if (iterator != cells_.end() && iterator->second.log_odds > config_.occupied_threshold) return true;
      }
    }
  }
  return false;
}

std::size_t VoxelOccupancyMap::occupiedVoxelCount() const noexcept {
  return occupied_voxel_count_;
}

std::vector<Eigen::Vector3d> VoxelOccupancyMap::occupiedVoxelCenters() const {
  std::vector<Eigen::Vector3d> result;
  result.reserve(occupied_voxel_count_);
  for (const auto& [key, cell] : cells_) {
    if (cell.log_odds > config_.occupied_threshold && keyInBounds(key)) {
      result.push_back(keyCenter(key));
    }
  }
  return result;
}

std::vector<Eigen::Vector3d> VoxelOccupancyMap::inflatedVoxelCenters() const {
  std::vector<Eigen::Vector3d> result;
  const Key min_key = keyFor(bounds_.min);
  const Key max_key = keyFor(bounds_.max);
  result.reserve(inflated_voxel_count_);
  for (std::int64_t x = min_key.x; x <= max_key.x; ++x) {
    for (std::int64_t y = min_key.y; y <= max_key.y; ++y) {
      for (std::int64_t z = min_key.z; z <= max_key.z; ++z) {
        const Key key{x, y, z};
        const auto center = keyCenter(key);
        if (keyInBounds(key) && isInflatedOccupied(center)) result.push_back(center);
      }
    }
  }
  return result;
}

}  // namespace uav::nav::rog

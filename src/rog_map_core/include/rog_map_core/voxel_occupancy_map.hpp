#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rog_map_core/navigation_map.hpp"

namespace uav::nav::rog {

struct VoxelMapConfig {
  double resolution_m{0.10};
  Eigen::Vector3d size_m{30.0, 30.0, 12.0};
  double inflation_radius_m{0.50};
  double raycast_min_range_m{0.30};
  double raycast_max_range_m{15.0};
  double hit_log_odds{0.85};
  double miss_log_odds{-0.40};
  double occupied_threshold{0.0};
  double clamp_min{-2.0};
  double clamp_max{3.5};
  double shift_threshold_m{1.0};
};

struct MapUpdateStats {
  std::size_t points_received{0};
  std::size_t points_integrated{0};
  std::size_t free_voxels_updated{0};
  std::size_t occupied_voxels_updated{0};
  std::size_t shift_count{0};
  std::size_t occupied_voxel_count{0};
  std::size_t inflated_voxel_upper_bound{0};
  std::size_t allocated_voxel_count{0};
};

class VoxelOccupancyMap final : public NavigationMap {
 public:
  explicit VoxelOccupancyMap(VoxelMapConfig config = {});
  void reset();
  void setValidity(MapValidity validity) noexcept { validity_ = validity; }
  [[nodiscard]] MapUpdateStats update(const Eigen::Vector3d& origin_lio_odom,
                                      const std::vector<Eigen::Vector3d>& points_lio_odom);
  [[nodiscard]] VoxelState query(const Eigen::Vector3d& position_lio_odom) const override;
  [[nodiscard]] bool isInflatedOccupied(
      const Eigen::Vector3d& position_lio_odom) const override;
  [[nodiscard]] double resolution() const override { return config_.resolution_m; }
  [[nodiscard]] double raycastMinRange() const noexcept { return config_.raycast_min_range_m; }
  [[nodiscard]] MapBounds localBounds() const override { return bounds_; }
  [[nodiscard]] MapValidity validity() const override { return validity_; }
  [[nodiscard]] std::size_t occupiedVoxelCount() const noexcept;
  [[nodiscard]] std::size_t inflatedVoxelUpperBound() const noexcept {
    return inflated_voxel_upper_bound_;
  }
  [[nodiscard]] std::size_t allocatedVoxelCount() const noexcept { return cells_.size(); }
  [[nodiscard]] std::size_t shiftCount() const noexcept { return shift_count_; }
  [[nodiscard]] std::vector<Eigen::Vector3d> occupiedVoxelCenters() const;
  [[nodiscard]] double inflationRadius() const noexcept { return config_.inflation_radius_m; }

 private:
  struct Key {
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};
    friend bool operator==(const Key&, const Key&) = default;
  };
  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept;
  };
  struct Cell { double log_odds{0.0}; };
  [[nodiscard]] Key keyFor(const Eigen::Vector3d& point) const noexcept;
  [[nodiscard]] Eigen::Vector3d keyCenter(const Key& key) const noexcept;
  [[nodiscard]] bool keyInBounds(const Key& key) const noexcept;
  void updateCell(const Key& key, double delta);
  void shiftIfNeeded(const Eigen::Vector3d& origin);
  void rebuildBounds();
  void evictOutsideBounds();
  [[nodiscard]] std::size_t inflatedVoxelUpperBoundEstimate() const noexcept;
  void raycast(const Eigen::Vector3d& origin, const Eigen::Vector3d& endpoint,
               bool endpoint_is_hit, MapUpdateStats& stats,
               std::unordered_set<Key, KeyHash>& free_updated);
  VoxelMapConfig config_;
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
  MapBounds bounds_;
  MapValidity validity_{MapValidity::kWaitingForLio};
  std::unordered_map<Key, Cell, KeyHash> cells_;
  std::size_t inflated_voxel_upper_bound_{0};
  std::size_t occupied_voxel_count_{0};
  std::size_t shift_count_{0};
};

}  // namespace uav::nav::rog

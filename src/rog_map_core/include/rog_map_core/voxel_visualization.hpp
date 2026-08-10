#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "rog_map_core/navigation_map.hpp"

namespace uav::nav::rog {

struct VisualizationVoxelKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
  friend bool operator==(const VisualizationVoxelKey&, const VisualizationVoxelKey&) = default;
};

struct VisualizationVoxelKeyHash {
  std::size_t operator()(const VisualizationVoxelKey& key) const noexcept {
    std::size_t hash = 1469598103934665603ULL;
    for (const auto value : {key.x, key.y, key.z}) {
      hash ^= static_cast<std::size_t>(value);
      hash *= 1099511628211ULL;
    }
    return hash;
  }
};

using VisualizationVoxelSet =
    std::unordered_set<VisualizationVoxelKey, VisualizationVoxelKeyHash>;

inline VisualizationVoxelKey visualizationKeyFor(const Eigen::Vector3d& point,
                                                  const double resolution) {
  return {static_cast<std::int64_t>(std::floor(point.x() / resolution)),
          static_cast<std::int64_t>(std::floor(point.y() / resolution)),
          static_cast<std::int64_t>(std::floor(point.z() / resolution))};
}

inline Eigen::Vector3d visualizationKeyCenter(const VisualizationVoxelKey& key,
                                              const double resolution) {
  return Eigen::Vector3d(static_cast<double>(key.x) + 0.5,
                         static_cast<double>(key.y) + 0.5,
                         static_cast<double>(key.z) + 0.5) * resolution;
}

inline std::vector<VisualizationVoxelKey> sphericalInflationStencil(
    const double resolution, const double inflation_radius) {
  std::vector<VisualizationVoxelKey> stencil;
  const auto radius = static_cast<std::int64_t>(
      std::ceil(inflation_radius / resolution));
  for (std::int64_t dx = -radius; dx <= radius; ++dx) {
    for (std::int64_t dy = -radius; dy <= radius; ++dy) {
      for (std::int64_t dz = -radius; dz <= radius; ++dz) {
        const Eigen::Vector3d offset(static_cast<double>(dx), static_cast<double>(dy),
                                     static_cast<double>(dz));
        if (offset.norm() * resolution <= inflation_radius + 1e-9) {
          stencil.push_back({dx, dy, dz});
        }
      }
    }
  }
  return stencil;
}

inline VisualizationVoxelSet visualizationSetFromCenters(
    const std::vector<Eigen::Vector3d>& centers, const double resolution) {
  VisualizationVoxelSet result;
  result.reserve(centers.size());
  for (const auto& center : centers) {
    if (center.allFinite()) result.insert(visualizationKeyFor(center, resolution));
  }
  return result;
}

inline VisualizationVoxelSet deriveInflatedVoxelSet(
    const std::vector<Eigen::Vector3d>& occupied_centers, const MapBounds& bounds,
    const double resolution, const double inflation_radius) {
  const auto stencil = sphericalInflationStencil(resolution, inflation_radius);
  VisualizationVoxelSet result;
  result.reserve(occupied_centers.size() * std::max<std::size_t>(stencil.size(), 1U));
  for (const auto& occupied_center : occupied_centers) {
    if (!occupied_center.allFinite()) continue;
    const auto seed = visualizationKeyFor(occupied_center, resolution);
    for (const auto& offset : stencil) {
      const VisualizationVoxelKey candidate{seed.x + offset.x, seed.y + offset.y,
                                            seed.z + offset.z};
      if (bounds.contains(visualizationKeyCenter(candidate, resolution))) {
        result.insert(candidate);
      }
    }
  }
  return result;
}

inline VisualizationVoxelSet extractInflationSurface(
    const VisualizationVoxelSet& inflated, const MapBounds& bounds,
    const double resolution) {
  VisualizationVoxelSet surface;
  surface.reserve(inflated.size());
  constexpr VisualizationVoxelKey neighbors[] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto& voxel : inflated) {
    bool boundary = false;
    for (const auto& offset : neighbors) {
      const VisualizationVoxelKey neighbor{voxel.x + offset.x, voxel.y + offset.y,
                                           voxel.z + offset.z};
      if (!bounds.contains(visualizationKeyCenter(neighbor, resolution)) ||
          !inflated.contains(neighbor)) {
        boundary = true;
        break;
      }
    }
    if (boundary) surface.insert(voxel);
  }
  return surface;
}

inline std::vector<Eigen::Vector3d> centersFromVisualizationSet(
    const VisualizationVoxelSet& voxels, const double resolution) {
  std::vector<Eigen::Vector3d> result;
  result.reserve(voxels.size());
  for (const auto& voxel : voxels) result.push_back(visualizationKeyCenter(voxel, resolution));
  return result;
}

}  // namespace uav::nav::rog

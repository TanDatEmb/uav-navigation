#pragma once

#include <Eigen/Core>

namespace uav::nav::rog {

enum class VoxelState { kOutside, kUnknown, kFree, kOccupied };
enum class MapValidity { kWaitingForLio, kActive, kStale, kInvalid };

struct MapBounds {
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
  [[nodiscard]] bool contains(const Eigen::Vector3d& point) const noexcept {
    return (point.array() >= min.array()).all() &&
           (point.array() < max.array()).all();
  }
};

class NavigationMap {
 public:
  virtual ~NavigationMap() = default;
  [[nodiscard]] virtual VoxelState query(const Eigen::Vector3d& position_lio_odom) const = 0;
  [[nodiscard]] virtual bool isInflatedOccupied(
      const Eigen::Vector3d& position_lio_odom) const = 0;
  [[nodiscard]] virtual double resolution() const = 0;
  [[nodiscard]] virtual MapBounds localBounds() const = 0;
  [[nodiscard]] virtual MapValidity validity() const = 0;
};

}  // namespace uav::nav::rog

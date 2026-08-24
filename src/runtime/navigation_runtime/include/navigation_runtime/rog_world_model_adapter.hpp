#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <navigation_world_model/world_model_view.hpp>
#include <rog_map/rog_map.h>

namespace navigation_runtime {

class RuntimeRogMap final : public rog_map::ROGMap {
 public:
  explicit RuntimeRogMap(std::function<double()> wall_clock_seconds = {})
      : wall_clock_seconds_(std::move(wall_clock_seconds)) {}

  const double getSystemWalltimeNow() override {
    return wall_clock_seconds_ ? wall_clock_seconds_() : 0.0;
  }

  void loadConfigAndInit(const std::string& yaml_path) {
    cfg_ = rog_map::Config(yaml_path);
    init();
  }

 private:
  std::function<double()> wall_clock_seconds_;
};

// Product-owned read adapter. Mutation remains owned by navigation_runtime;
// SUPER receives only the const WorldModelView surface.
class RogWorldModelView final : public navigation_world_model::WorldModelView {
 public:
  explicit RogWorldModelView(std::shared_ptr<RuntimeRogMap> map)
      : map_(std::move(map)) {}

  void recordSuccessfulUpdate(std::int64_t observation_stamp_ns) noexcept {
    observation_stamp_ns_.store(observation_stamp_ns, std::memory_order_release);
    revision_.fetch_add(1, std::memory_order_acq_rel);
  }

  [[nodiscard]] navigation_world_model::WorldGeometry geometry() const noexcept override {
    const auto& config = map_->getMapConfig();
    return {
        map_->getResolution(),
        map_->getInfResolution(),
        config.inflation_resolution * static_cast<double>(config.inflation_step),
        config.virtual_ground_height,
        config.virtual_ceil_height,
        map_->getLocalMapOrigin(),
        map_->getLocalMapSize(),
    };
  }

  [[nodiscard]] navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {generation_.load(std::memory_order_acquire),
            revision_.load(std::memory_order_acquire),
            observation_stamp_ns_.load(std::memory_order_acquire)};
  }

  [[nodiscard]] navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    if (!point.allFinite()) return navigation_world_model::CellState::kOutOfMap;
    return toProductCell(layer == navigation_world_model::GridLayer::kInflated
                             ? map_->getInfGridType(point)
                             : map_->getGridType(point));
  }

  [[nodiscard]] bool contains(
      const navigation_world_model::Point3& point) const noexcept override {
    return point.allFinite() && map_->insideLocalMap(point);
  }

  [[nodiscard]] navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    navigation_world_model::GridIndex3 index{0, 0, 0};
    if (layer == navigation_world_model::GridLayer::kInflated) {
      map_->infMapPosToGlobalIndex(point, index);
    } else {
      map_->probMapPosToGlobalIndex(point, index);
    }
    return index;
  }

  [[nodiscard]] navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer layer) const noexcept override {
    navigation_world_model::Point3 point{navigation_world_model::Point3::Zero()};
    if (layer == navigation_world_model::GridLayer::kInflated) {
      map_->infMapGlobalIndexToPos(index, point);
    } else {
      map_->probMapGlobalIndexToPos(index, point);
    }
    return point;
  }

  [[nodiscard]] std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& start,
      navigation_world_model::GridLayer layer,
      double maximum_distance_m) const override {
    navigation_world_model::Point3 nearest = start;
    const bool found = layer == navigation_world_model::GridLayer::kInflated
        ? map_->getNearestInfCellNot(rog_map::GridType::OCCUPIED, start, nearest,
                                    maximum_distance_m)
        : map_->getNearestCellNot(rog_map::GridType::OCCUPIED, start, nearest,
                                 maximum_distance_m);
    return found ? std::optional<navigation_world_model::Point3>{nearest} : std::nullopt;
  }

  [[nodiscard]] bool isSegmentTraversable(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy unknown_policy) const noexcept override {
    if (!start.allFinite() || !end.allFinite()) return false;
    return map_->isLineFree(
        start, end, layer == navigation_world_model::GridLayer::kInflated,
        unknown_policy == navigation_world_model::UnknownPolicy::kRequireKnownFree);
  }

  [[nodiscard]] navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& requested) const noexcept override {
    auto result = requested;
    map_->boundBoxByLocalMap(result.minimum, result.maximum);
    return result;
  }

  [[nodiscard]] navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& box) const override {
    navigation_world_model::PointVector points;
    map_->boxSearchObservedOccupied(box.minimum, box.maximum, points);
    return points;
  }

 private:
  static navigation_world_model::CellState toProductCell(rog_map::GridType cell) noexcept {
    using navigation_world_model::CellState;
    switch (cell) {
      case rog_map::GridType::UNKNOWN: return CellState::kUnknown;
      case rog_map::GridType::OUT_OF_MAP: return CellState::kOutOfMap;
      case rog_map::GridType::OCCUPIED: return CellState::kOccupied;
      case rog_map::GridType::KNOWN_FREE: return CellState::kKnownFree;
      case rog_map::GridType::FRONTIER: return CellState::kFrontier;
      case rog_map::GridType::UNDEFINED: return CellState::kUndefined;
    }
    return CellState::kUndefined;
  }

  std::shared_ptr<RuntimeRogMap> map_;
  std::atomic<std::uint64_t> generation_{1};
  std::atomic<std::uint64_t> revision_{0};
  std::atomic<std::int64_t> observation_stamp_ns_{0};
};

}  // namespace navigation_runtime

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <navigation_world_model/world_model_view.hpp>
#include <navigation_world_model/continuous_clearance.hpp>
#include <rog_map/rog_map.h>

namespace navigation_mapping::internal {

class RuntimeMappingMap final : public rog_map::ROGMap {
 public:
  explicit RuntimeMappingMap(std::function<double()> wall_clock_seconds = {})
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

// Transitional live-map parity adapter. It is deliberately private to the
// mapping implementation and must not be used concurrently with map updates.
class MappingWorldModelView final : public navigation_world_model::WorldModelView {
 public:
  explicit MappingWorldModelView(std::shared_ptr<RuntimeMappingMap> map,
                                 std::uint64_t localization_epoch = 1U)
      : map_(std::move(map)), localization_epoch_(localization_epoch) {}

  void recordSuccessfulUpdate(std::int64_t observation_stamp_ns) noexcept {
    observation_stamp_ns_.store(observation_stamp_ns, std::memory_order_release);
    revision_.fetch_add(1, std::memory_order_acq_rel);
  }

  [[nodiscard]] navigation_world_model::WorldGeometry geometry() const noexcept override {
    if (!map_) return {};
    const auto& config = map_->getMapConfig();
    const auto center = map_->getLocalMapOrigin();
    const auto size = map_->getLocalMapSize();
    const double effective_ground = config.virtual_ground_ceiling_en
        ? config.virtual_ground_height : center.z() - 0.5 * size.z();
    const double effective_ceiling = config.virtual_ground_ceiling_en
        ? config.virtual_ceil_height : center.z() + 0.5 * size.z();
    const auto bounds_for = [this, &center, &size](
                                const double resolution,
                                const navigation_world_model::GridLayer layer) {
      navigation_world_model::GridBounds bounds;
      if (!std::isfinite(resolution) || resolution <= 0.0 || !size.allFinite()) {
        return bounds;
      }
      const auto count_real = size.array() / resolution;
      Eigen::Vector3i count{Eigen::Vector3i::Zero()};
      for (int axis = 0; axis < 3; ++axis) {
        const long double raw = static_cast<long double>(size(axis)) /
            static_cast<long double>(resolution);
        if (!std::isfinite(raw)) return navigation_world_model::GridBounds{};
        const long double rounded = std::round(raw);
        if (!std::isfinite(rounded) || rounded < 1.0L ||
            rounded > static_cast<long double>(std::numeric_limits<int>::max())) {
          return navigation_world_model::GridBounds{};
        }
        count(axis) = static_cast<int>(rounded);
      }
      const auto count_as_real = count.cast<double>().array();
      const auto center_index = positionToIndex(center, layer);
      const auto tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
          count_real.cwiseAbs().cwiseMax(1.0);
      if ((count.array() <= 0).any() ||
          ((count_real - count_as_real).cwiseAbs() > tolerance).any()) {
        return bounds;
      }
      for (int axis = 0; axis < 3; ++axis) {
        const std::int64_t minimum = static_cast<std::int64_t>(center_index(axis)) -
            static_cast<std::int64_t>(count(axis) / 2);
        if (minimum < static_cast<std::int64_t>(std::numeric_limits<int>::min()) ||
            minimum > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
          return navigation_world_model::GridBounds{};
        }
      }
      bounds.dimensions = count;
      bounds.global_min_index = center_index - count / 2;
      return bounds;
    };
    return {
        map_->getResolution(),
        map_->getInfResolution(),
        config.inflation_resolution * static_cast<double>(config.inflation_step),
        effective_ground,
        effective_ceiling,
        center,
        size,
        config.virtual_ground_ceiling_en,
        bounds_for(map_->getResolution(), navigation_world_model::GridLayer::kEvidence),
        bounds_for(map_->getInfResolution(), navigation_world_model::GridLayer::kInflated),
    };
  }

  [[nodiscard]] navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {localization_epoch_, generation_.load(std::memory_order_acquire),
            revision_.load(std::memory_order_acquire),
            observation_stamp_ns_.load(std::memory_order_acquire)};
  }

  [[nodiscard]] navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    if (!map_ || !point.allFinite()) return navigation_world_model::CellState::kOutOfMap;
    switch (layer) {
      case navigation_world_model::GridLayer::kEvidence:
        return toProductCell(map_->getGridType(point));
      case navigation_world_model::GridLayer::kInflated:
        return toProductCell(map_->getInfGridType(point));
    }
    return navigation_world_model::CellState::kOutOfMap;
  }

  [[nodiscard]] bool contains(
      const navigation_world_model::Point3& point) const noexcept override {
    return map_ && point.allFinite() && map_->insideLocalMap(point);
  }

  [[nodiscard]] navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    navigation_world_model::GridIndex3 index{0, 0, 0};
    if (!map_ || !point.allFinite()) return index;
    switch (layer) {
      case navigation_world_model::GridLayer::kInflated:
        map_->infMapPosToGlobalIndex(point, index);
        break;
      case navigation_world_model::GridLayer::kEvidence:
        map_->probMapPosToGlobalIndex(point, index);
        break;
      default:
        return index;
    }
    return index;
  }

  [[nodiscard]] navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer layer) const noexcept override {
    navigation_world_model::Point3 point{navigation_world_model::Point3::Zero()};
    if (!map_) return point;
    switch (layer) {
      case navigation_world_model::GridLayer::kInflated:
        map_->infMapGlobalIndexToPos(index, point);
        break;
      case navigation_world_model::GridLayer::kEvidence:
        map_->probMapGlobalIndexToPos(index, point);
        break;
      default:
        return point;
    }
    return point;
  }

  [[nodiscard]] std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& start,
      navigation_world_model::GridLayer layer,
      double maximum_distance_m) const override {
    if (!map_ || !start.allFinite() || !std::isfinite(maximum_distance_m) ||
        maximum_distance_m < 0.0) {
      return std::nullopt;
    }
    navigation_world_model::Point3 nearest = start;
    bool found = false;
    switch (layer) {
      case navigation_world_model::GridLayer::kInflated:
        found = map_->getNearestInfCellNot(rog_map::GridType::OCCUPIED, start, nearest,
                                           maximum_distance_m);
        break;
      case navigation_world_model::GridLayer::kEvidence:
        found = map_->getNearestCellNot(rog_map::GridType::OCCUPIED, start, nearest,
                                        maximum_distance_m);
        break;
      default:
        return std::nullopt;
    }
    return found && nearest.allFinite()
        ? std::optional<navigation_world_model::Point3>{nearest} : std::nullopt;
  }

  [[nodiscard]] bool isSegmentTraversable(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy unknown_policy) const noexcept override {
    if (!map_ || !start.allFinite() || !end.allFinite()) return false;
    if (unknown_policy != navigation_world_model::UnknownPolicy::kAllowUnknown &&
        unknown_policy != navigation_world_model::UnknownPolicy::kRequireKnownFree) {
      return false;
    }
    if (layer != navigation_world_model::GridLayer::kEvidence &&
        layer != navigation_world_model::GridLayer::kInflated) {
      return false;
    }
    if (!map_->isLineFree(
        start, end, layer == navigation_world_model::GridLayer::kInflated,
        unknown_policy == navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
      return false;
    }
    if (layer != navigation_world_model::GridLayer::kInflated) return true;
    return navigation_world_model::observedOccupiedTubeIsClear(
        *this, start, end, geometry().occupied_inflation_radius_m);
  }

  [[nodiscard]] navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& requested) const noexcept override {
    auto result = requested;
    if (!map_ || !result.minimum.allFinite() || !result.maximum.allFinite()) return {};
    map_->boundBoxByLocalMap(result.minimum, result.maximum);
    return result;
  }

  [[nodiscard]] navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& box) const override {
    navigation_world_model::PointVector points;
    if (!map_ || !box.minimum.allFinite() || !box.maximum.allFinite()) return points;
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

  std::shared_ptr<RuntimeMappingMap> map_;
  std::uint64_t localization_epoch_{1U};
  std::atomic<std::uint64_t> generation_{1};
  std::atomic<std::uint64_t> revision_{0};
  std::atomic<std::int64_t> observation_stamp_ns_{0};
};

}  // namespace navigation_mapping::internal

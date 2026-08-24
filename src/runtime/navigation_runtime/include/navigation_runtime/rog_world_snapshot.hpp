#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <navigation_world_model/world_model_view.hpp>
#include <rog_map/planning_grid_export.hpp>
#include <rog_map/rog_map_core/raycaster.h>
#include <super_utils/type_utils.hpp>

namespace navigation_runtime {

class RogWorldSnapshot final : public navigation_world_model::WorldModelView {
 public:
  RogWorldSnapshot(rog_map::PlanningGridExport grid,
                   navigation_world_model::WorldSnapshotIdentity identity)
      : grid_(validated(std::move(grid), identity)),
        identity_(identity),
        owned_bytes_(grid_.ownedByteSize()) {
    const auto live = live_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
    const auto bytes = live_owned_bytes_.fetch_add(owned_bytes_, std::memory_order_relaxed) +
                       owned_bytes_;
    updatePeak(peak_live_count_, live);
    updatePeak(peak_live_owned_bytes_, bytes);
  }

  ~RogWorldSnapshot() override {
    live_count_.fetch_sub(1U, std::memory_order_relaxed);
    live_owned_bytes_.fetch_sub(owned_bytes_, std::memory_order_relaxed);
  }

  RogWorldSnapshot(const RogWorldSnapshot&) = delete;
  RogWorldSnapshot& operator=(const RogWorldSnapshot&) = delete;
  RogWorldSnapshot(RogWorldSnapshot&&) = delete;
  RogWorldSnapshot& operator=(RogWorldSnapshot&&) = delete;

  [[nodiscard]] navigation_world_model::WorldGeometry geometry() const noexcept override {
    return {
        grid_.base_layout.resolution_m,
        grid_.inflated.layout.resolution_m,
        grid_.occupied_inflation_radius_m,
        grid_.virtual_ground_m,
        grid_.virtual_ceiling_m,
        grid_.base_layout.local_center_m,
        grid_.base_layout.local_size_m,
    };
  }

  [[nodiscard]] navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return identity_;
  }

  [[nodiscard]] navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    using navigation_world_model::CellState;
    using navigation_world_model::GridLayer;
    if (!point.allFinite()) return CellState::kOutOfMap;
    if (layer == GridLayer::kEvidence) {
      if (point.z() <= grid_.virtual_ground_m || point.z() >= grid_.virtual_ceiling_m) {
        return CellState::kOccupied;
      }
      if (!contains(point)) return CellState::kOutOfMap;
      return productCell(baseState(positionToIndex(point, GridLayer::kEvidence)));
    }
    if (!contains(point)) return CellState::kOutOfMap;
    if (point.z() <= grid_.inflated_virtual_ground_m ||
        point.z() >= grid_.inflated_virtual_ceiling_m) {
      return CellState::kOccupied;
    }
    const auto inflated_index = positionToIndex(point, GridLayer::kInflated);
    const auto inflated_offset = offset(grid_.inflated.layout, inflated_index);
    if (!inflated_offset) return CellState::kOutOfMap;
    if (grid_.inflated.occupied[*inflated_offset] != 0U) return CellState::kOccupied;
    if (grid_.unknown_inflation_enabled) {
      return grid_.inflated.unknown[*inflated_offset] != 0U
                 ? CellState::kUnknown
                 : CellState::kKnownFree;
    }
    return productCell(baseState(positionToIndex(point, GridLayer::kEvidence)));
  }

  [[nodiscard]] bool contains(
      const navigation_world_model::Point3& point) const noexcept override {
    if (!point.allFinite()) return false;
    return offset(grid_.base_layout,
                  positionToIndex(point, navigation_world_model::GridLayer::kEvidence))
        .has_value();
  }

  [[nodiscard]] navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? grid_.inflated.layout.resolution_m
                                  : grid_.base_layout.resolution_m;
    return (point.array() / resolution).floor().cast<int>();
  }

  [[nodiscard]] navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer layer) const noexcept override {
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? grid_.inflated.layout.resolution_m
                                  : grid_.base_layout.resolution_m;
    return (index.cast<double>() + navigation_world_model::Point3::Constant(0.5)) * resolution;
  }

  [[nodiscard]] std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& start,
      navigation_world_model::GridLayer layer,
      double maximum_distance_m) const override {
    if (!start.allFinite() || !std::isfinite(maximum_distance_m) || maximum_distance_m < 0.0) {
      return std::nullopt;
    }
    // Preserve ROG's legacy contract: both layers use the base-grid start
    // conversion and base-grid metric centers while applying layer-specific classification.
    const auto start_index = positionToIndex(start, navigation_world_model::GridLayer::kEvidence);
    for (const auto& delta : *grid_.nearest_offsets) {
      const auto candidate = indexToPosition(
          start_index + delta, navigation_world_model::GridLayer::kEvidence);
      if ((candidate - start).norm() > maximum_distance_m) return std::nullopt;
      if (classify(candidate, layer) != navigation_world_model::CellState::kOccupied) {
        return candidate;
      }
    }
    return std::nullopt;
  }

  [[nodiscard]] bool isSegmentTraversable(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy unknown_policy) const noexcept override {
    if (!start.allFinite() || !end.allFinite() || !contains(start) || !contains(end)) return false;
    rog_map::raycaster::RayCaster raycaster;
    raycaster.setResolution(layer == navigation_world_model::GridLayer::kInflated
                                ? grid_.inflated.layout.resolution_m
                                : grid_.base_layout.resolution_m);
    raycaster.setInput(start, end);
    navigation_world_model::Point3 point;
    while (raycaster.step(point)) {
      if (!contains(point)) return false;
      if (layer == navigation_world_model::GridLayer::kEvidence) {
        if (isBaseOccupiedForRay(point)) return false;
        if (unknown_policy == navigation_world_model::UnknownPolicy::kRequireKnownFree &&
            !isBaseKnownFreeForRay(point)) {
          return false;
        }
      } else {
        if (isInflatedOccupiedForRay(point)) return false;
        if (unknown_policy == navigation_world_model::UnknownPolicy::kRequireKnownFree) {
          if (!grid_.unknown_inflation_enabled) {
            if (!isBaseKnownFreeForRay(point)) return false;
          } else if (isInflatedUnknownForRay(point)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  [[nodiscard]] navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& requested) const noexcept override {
    auto result = requested;
    if (!result.minimum.allFinite() || !result.maximum.allFinite() ||
        (result.maximum - result.minimum).minCoeff() <= 0.0) {
      result.minimum = result.maximum;
      return result;
    }
    const auto minimum_center = indexToPosition(
        grid_.base_layout.global_min_index, navigation_world_model::GridLayer::kEvidence);
    const auto maximum_center = indexToPosition(
        grid_.base_layout.global_min_index + grid_.base_layout.dimensions -
            navigation_world_model::GridIndex3::Ones(),
        navigation_world_model::GridLayer::kEvidence);
    result.minimum = result.minimum.cwiseMax(minimum_center);
    result.maximum = result.maximum.cwiseMin(maximum_center);
    result.minimum.z() = std::max(result.minimum.z(), grid_.virtual_ground_m);
    result.maximum.z() = std::min(result.maximum.z(), grid_.virtual_ceiling_m);
    return result;
  }

  [[nodiscard]] navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& requested) const override {
    navigation_world_model::PointVector points;
    const auto box = clampToLocalBounds(requested);
    if ((box.maximum - box.minimum).minCoeff() <= 0.0) return points;
    const auto minimum = positionToIndex(box.minimum, navigation_world_model::GridLayer::kEvidence);
    const auto maximum = positionToIndex(box.maximum, navigation_world_model::GridLayer::kEvidence);
    const auto size = maximum - minimum;
    points.reserve(static_cast<std::size_t>(std::max(0, size.prod() / 12)));
    for (int x = minimum.x() + 1; x < maximum.x(); ++x) {
      for (int y = minimum.y() + 1; y < maximum.y(); ++y) {
        for (int z = minimum.z() + 1; z < maximum.z(); ++z) {
          const navigation_world_model::GridIndex3 index{x, y, z};
          const auto cell_offset = offset(grid_.base_layout, index);
          if (cell_offset && grid_.base_state[*cell_offset] ==
                                 static_cast<std::uint8_t>(super_utils::GridType::OCCUPIED)) {
            points.emplace_back(indexToPosition(index,
                                                navigation_world_model::GridLayer::kEvidence));
          }
        }
      }
    }
    return points;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept { return grid_.byteSize(); }
  [[nodiscard]] std::size_t ownedByteSize() const noexcept { return owned_bytes_; }
  [[nodiscard]] std::size_t sharedMetadataByteSize() const noexcept {
    return grid_.sharedMetadataByteSize();
  }
  [[nodiscard]] static std::size_t liveCount() noexcept {
    return live_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] static std::size_t peakLiveCount() noexcept {
    return peak_live_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] static std::size_t liveOwnedBytes() noexcept {
    return live_owned_bytes_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] static std::size_t peakLiveOwnedBytes() noexcept {
    return peak_live_owned_bytes_.load(std::memory_order_relaxed);
  }

 private:
  static void updatePeak(std::atomic_size_t& peak, std::size_t value) noexcept {
    auto previous = peak.load(std::memory_order_relaxed);
    while (previous < value &&
           !peak.compare_exchange_weak(previous, value, std::memory_order_relaxed)) {}
  }

  static rog_map::PlanningGridExport validated(
      rog_map::PlanningGridExport grid,
      const navigation_world_model::WorldSnapshotIdentity& identity) {
    const auto validate_layout = [](const rog_map::PlanningGridLayoutExport& layout,
                                    const char* name) -> std::size_t {
      if (!std::isfinite(layout.resolution_m) || layout.resolution_m <= 0.0 ||
          (layout.dimensions.array() <= 0).any() || !layout.local_center_m.allFinite() ||
          !layout.local_size_m.allFinite() || (layout.local_size_m.array() <= 0.0).any()) {
        throw std::invalid_argument(std::string{name} + " has invalid geometry");
      }
      std::size_t count = 1U;
      for (int axis = 0; axis < 3; ++axis) {
        const auto dimension = static_cast<std::size_t>(layout.dimensions[axis]);
        if (count > std::numeric_limits<std::size_t>::max() / dimension) {
          throw std::overflow_error(std::string{name} + " voxel count overflows size_t");
        }
        count *= dimension;
      }
      return count;
    };
    const auto base_count = validate_layout(grid.base_layout, "base layout");
    const auto inflated_count = validate_layout(grid.inflated.layout, "inflated layout");
    if (grid.base_state.size() != base_count || grid.inflated.occupied.size() != inflated_count ||
        (grid.unknown_inflation_enabled && grid.inflated.unknown.size() != inflated_count) ||
        (!grid.unknown_inflation_enabled && !grid.inflated.unknown.empty())) {
      throw std::invalid_argument("planning grid array sizes do not match layout");
    }
    if (!grid.nearest_offsets) {
      throw std::invalid_argument("planning grid nearest-offset metadata is missing");
    }
    if (!std::isfinite(grid.virtual_ground_m) || !std::isfinite(grid.virtual_ceiling_m) ||
        !std::isfinite(grid.inflated_virtual_ground_m) ||
        !std::isfinite(grid.inflated_virtual_ceiling_m) ||
        grid.virtual_ground_m >= grid.virtual_ceiling_m ||
        grid.inflated_virtual_ground_m >= grid.inflated_virtual_ceiling_m ||
        identity.generation == 0U || identity.observation_stamp_ns < 0) {
      throw std::invalid_argument("planning grid metadata or identity is invalid");
    }
    return grid;
  }

  static std::optional<std::size_t> offset(
      const rog_map::PlanningGridLayoutExport& layout,
      const navigation_world_model::GridIndex3& index) noexcept {
    const auto local = index - layout.global_min_index;
    if ((local.array() < 0).any() || (local.array() >= layout.dimensions.array()).any()) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(
        (local.x() * layout.dimensions.y() + local.y()) * layout.dimensions.z() + local.z());
  }

  [[nodiscard]] std::uint8_t baseState(
      const navigation_world_model::GridIndex3& index) const noexcept {
    const auto cell_offset = offset(grid_.base_layout, index);
    return cell_offset ? grid_.base_state[*cell_offset]
                       : static_cast<std::uint8_t>(super_utils::GridType::OUT_OF_MAP);
  }

  [[nodiscard]] bool isBaseOccupiedForRay(
      const navigation_world_model::Point3& point) const noexcept {
    if (point.z() > grid_.virtual_ceiling_m || point.z() < grid_.virtual_ground_m) return true;
    return baseState(positionToIndex(point, navigation_world_model::GridLayer::kEvidence)) ==
           static_cast<std::uint8_t>(super_utils::GridType::OCCUPIED);
  }

  [[nodiscard]] bool isBaseKnownFreeForRay(
      const navigation_world_model::Point3& point) const noexcept {
    if (point.z() > grid_.virtual_ceiling_m || point.z() < grid_.virtual_ground_m) return false;
    return baseState(positionToIndex(point, navigation_world_model::GridLayer::kEvidence)) ==
           static_cast<std::uint8_t>(super_utils::GridType::KNOWN_FREE);
  }

  [[nodiscard]] bool isInflatedOccupiedForRay(
      const navigation_world_model::Point3& point) const noexcept {
    if (point.z() > grid_.virtual_ceiling_m || point.z() < grid_.virtual_ground_m) return true;
    const auto cell_offset = offset(
        grid_.inflated.layout,
        positionToIndex(point, navigation_world_model::GridLayer::kInflated));
    return cell_offset && grid_.inflated.occupied[*cell_offset] != 0U;
  }

  [[nodiscard]] bool isInflatedUnknownForRay(
      const navigation_world_model::Point3& point) const noexcept {
    const double resolution = grid_.inflated.layout.resolution_m;
    if (point.z() >= grid_.virtual_ceiling_m - resolution ||
        point.z() <= grid_.virtual_ground_m + resolution) {
      return false;
    }
    const auto cell_offset = offset(
        grid_.inflated.layout,
        positionToIndex(point, navigation_world_model::GridLayer::kInflated));
    return cell_offset && grid_.inflated.unknown[*cell_offset] != 0U;
  }

  static navigation_world_model::CellState productCell(std::uint8_t state) noexcept {
    using navigation_world_model::CellState;
    switch (static_cast<super_utils::GridType>(state)) {
      case super_utils::GridType::UNKNOWN: return CellState::kUnknown;
      case super_utils::GridType::OUT_OF_MAP: return CellState::kOutOfMap;
      case super_utils::GridType::OCCUPIED: return CellState::kOccupied;
      case super_utils::GridType::KNOWN_FREE: return CellState::kKnownFree;
      case super_utils::GridType::FRONTIER: return CellState::kFrontier;
      case super_utils::GridType::UNDEFINED: return CellState::kUndefined;
    }
    return CellState::kUndefined;
  }

  const rog_map::PlanningGridExport grid_;
  const navigation_world_model::WorldSnapshotIdentity identity_;
  const std::size_t owned_bytes_;
  inline static std::atomic_size_t live_count_{0U};
  inline static std::atomic_size_t peak_live_count_{0U};
  inline static std::atomic_size_t live_owned_bytes_{0U};
  inline static std::atomic_size_t peak_live_owned_bytes_{0U};
};

}  // namespace navigation_runtime

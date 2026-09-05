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

#include <navigation_mapping/mapping_types.hpp>
#include <navigation_mapping/visibility_control.hpp>
#include <navigation_world_model/continuous_clearance.hpp>

namespace navigation_mapping {

// Private immutable storage representation. This header is intentionally not
// installed as part of the mapping package API; callers consume WorldModelView.
struct PlanningGridLayout {
  double resolution_m{0.0};
  navigation_world_model::GridIndex3 global_min_index{
      navigation_world_model::GridIndex3::Zero()};
  navigation_world_model::GridIndex3 dimensions{
      navigation_world_model::GridIndex3::Zero()};
  navigation_world_model::Point3 local_center_m{
      navigation_world_model::Point3::Zero()};
  navigation_world_model::Point3 local_size_m{
      navigation_world_model::Point3::Zero()};
};

struct InflatedPlanningGrid {
  PlanningGridLayout layout;
  std::vector<std::uint8_t> occupied;
  std::vector<std::uint8_t> unknown;
};

struct PlanningGrid {
  PlanningGridLayout base_layout;
  InflatedPlanningGrid inflated;
  // The backend export is already a validated uint8 cell code. Keep the
  // immutable storage byte-compatible so snapshot construction can move it
  // without a byte-to-enum conversion pass; decode only at the query edge.
  std::vector<std::uint8_t> base_state;
  std::shared_ptr<const std::vector<navigation_world_model::GridIndex3>> nearest_offsets;
  bool unknown_inflation_enabled{false};
  bool virtual_ground_ceiling_enabled{true};
  double virtual_ground_m{0.0};
  double virtual_ceiling_m{0.0};
  double inflated_virtual_ground_m{0.0};
  double inflated_virtual_ceiling_m{0.0};
  double occupied_inflation_radius_m{0.0};

  [[nodiscard]] std::size_t ownedByteSize() const noexcept {
    return base_state.size() * sizeof(std::uint8_t) +
           inflated.occupied.size() + inflated.unknown.size();
  }

  [[nodiscard]] std::size_t sharedMetadataByteSize() const noexcept {
    return nearest_offsets
               ? nearest_offsets->size() * sizeof(navigation_world_model::GridIndex3)
               : 0U;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept {
    return ownedByteSize() + sharedMetadataByteSize();
  }
};

struct PlanningGridPatch {
  PlanningGridLayout base_layout;
  InflatedPlanningGrid inflated;
  std::vector<std::uint8_t> base_state;

  [[nodiscard]] std::size_t ownedByteSize() const noexcept {
    return base_state.size() + inflated.occupied.size() + inflated.unknown.size();
  }
};

class MappingWorldSnapshot final
    : public navigation_world_model::WorldModelView {
 public:
  MappingWorldSnapshot(PlanningGrid grid,
                       navigation_world_model::WorldSnapshotIdentity identity,
                       navigation_world_model::WorldChangeHistoryPtr change_history = {})
      : grid_(validated(std::move(grid), identity)),
        identity_(identity),
        change_history_(std::move(change_history)),
        owned_bytes_(grid_->ownedByteSize()) {
    const auto live = live_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
    const auto bytes = live_owned_bytes_.fetch_add(owned_bytes_, std::memory_order_relaxed) +
                       owned_bytes_;
    updatePeak(peak_live_count_, live);
    updatePeak(peak_live_owned_bytes_, bytes);
  }

  MappingWorldSnapshot(std::shared_ptr<const MappingWorldSnapshot> parent,
                       PlanningGridPatch patch,
                       navigation_world_model::WorldSnapshotIdentity identity,
                       navigation_world_model::WorldChangeHistoryPtr change_history = {})
      : parent_(std::move(parent)),
        patch_(validatedPatch(std::move(patch), parent_, identity)),
        identity_(identity),
        change_history_(std::move(change_history)),
        owned_bytes_(patch_->ownedByteSize()) {
    if (!parent_) throw std::invalid_argument("patch snapshot parent is missing");
    const auto live = live_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
    const auto bytes = live_owned_bytes_.fetch_add(owned_bytes_, std::memory_order_relaxed) +
                       owned_bytes_;
    updatePeak(peak_live_count_, live);
    updatePeak(peak_live_owned_bytes_, bytes);
  }

  ~MappingWorldSnapshot() override {
    live_count_.fetch_sub(1U, std::memory_order_relaxed);
    live_owned_bytes_.fetch_sub(owned_bytes_, std::memory_order_relaxed);
  }

  MappingWorldSnapshot(const MappingWorldSnapshot&) = delete;
  MappingWorldSnapshot& operator=(const MappingWorldSnapshot&) = delete;
  MappingWorldSnapshot(MappingWorldSnapshot&&) = delete;
  MappingWorldSnapshot& operator=(MappingWorldSnapshot&&) = delete;

  [[nodiscard]] navigation_world_model::WorldGeometry geometry() const noexcept override {
    const auto& grid = rootGrid();
    return {
        grid.base_layout.resolution_m,
        grid.inflated.layout.resolution_m,
        grid.occupied_inflation_radius_m,
        grid.virtual_ground_m,
        grid.virtual_ceiling_m,
        grid.base_layout.local_center_m,
        grid.base_layout.local_size_m,
        grid.virtual_ground_ceiling_enabled,
        navigation_world_model::GridBounds{
            grid.base_layout.global_min_index, grid.base_layout.dimensions},
        navigation_world_model::GridBounds{
            grid.inflated.layout.global_min_index,
            grid.inflated.layout.dimensions},
    };
  }

  [[nodiscard]] navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return identity_;
  }

  [[nodiscard]] std::size_t patchDepth() const noexcept {
    return patch_ ? (parent_->patchDepth() + 1U) : 0U;
  }

  [[nodiscard]] bool changedRegionIntersectsSince(
      const navigation_world_model::WorldSnapshotIdentity& older,
      const navigation_world_model::AxisAlignedBox& protected_region) const noexcept override {
    if (!protected_region.valid() ||
        older.localization_epoch != identity_.localization_epoch ||
        older.generation != identity_.generation ||
        older.revision > identity_.revision) {
      return true;
    }
    if (older.revision == identity_.revision) {
      return older.observation_stamp_ns != identity_.observation_stamp_ns;
    }
    if (older.observation_stamp_ns > identity_.observation_stamp_ns) return true;

    auto expected_revision = identity_.revision;
    if (!change_history_ ||
        change_history_->records.size() <
            static_cast<std::size_t>(identity_.revision - older.revision)) {
      return true;
    }
    for (const auto& record : change_history_->records) {
      if (expected_revision <= older.revision) break;
      if (record.identity.localization_epoch != identity_.localization_epoch ||
          record.identity.generation != identity_.generation ||
          record.identity.revision != expected_revision) {
        return true;
      }
      if (record.affects_whole_world ||
          boxesOverlap(record.affected_region, protected_region)) {
        return true;
      }
      --expected_revision;
    }
    return expected_revision > older.revision;
  }

  [[nodiscard]] navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    using navigation_world_model::CellState;
    using navigation_world_model::GridLayer;
    const auto& grid = rootGrid();
    if (!point.allFinite()) return CellState::kOutOfMap;
    if (layer == GridLayer::kEvidence) {
      if (grid.virtual_ground_ceiling_enabled &&
          (point.z() <= grid.virtual_ground_m || point.z() >= grid.virtual_ceiling_m)) {
        return CellState::kOccupied;
      }
      if (!containsLayer(point, GridLayer::kEvidence)) return CellState::kOutOfMap;
      return baseState(positionToIndex(point, GridLayer::kEvidence));
    }
    if (!containsLayer(point, GridLayer::kInflated)) return CellState::kOutOfMap;
    if (grid.virtual_ground_ceiling_enabled &&
        (point.z() <= grid.inflated_virtual_ground_m ||
         point.z() >= grid.inflated_virtual_ceiling_m)) {
      return CellState::kOccupied;
    }
    const auto inflated_index = positionToIndex(point, GridLayer::kInflated);
    const auto inflated_offset = offset(grid.inflated.layout, inflated_index);
    if (!inflated_offset) return CellState::kOutOfMap;
    if (inflatedOccupied(inflated_index) != 0U) return CellState::kOccupied;
    if (grid.unknown_inflation_enabled) {
      return inflatedUnknown(inflated_index) != 0U
                 ? CellState::kUnknown
                 : CellState::kKnownFree;
    }
    return baseState(positionToIndex(point, GridLayer::kEvidence));
  }

  [[nodiscard]] navigation_world_model::FreeSpaceEvidence classifyFreeSpace(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer,
      const std::int64_t now_stamp_ns = 0) const noexcept override {
    const auto state = classify(point, layer);
    using navigation_world_model::CellState;
    using navigation_world_model::FreeSpaceEvidence;
    if (state == CellState::kOccupied) return FreeSpaceEvidence::kOccupied;
    if (state == CellState::kOutOfMap) return FreeSpaceEvidence::kOutOfMap;
    if (state == CellState::kKnownFree) return FreeSpaceEvidence::kSensorFree;
    static_cast<void>(now_stamp_ns);
    return FreeSpaceEvidence::kUnknown;
  }

  [[nodiscard]] navigation_world_model::HandoverClearanceReason handoverClearanceReason(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer,
      const std::int64_t now_stamp_ns = 0) const noexcept override {
    const auto state = classify(point, layer);
    using navigation_world_model::CellState;
    using navigation_world_model::HandoverClearanceReason;
    if (state == CellState::kOccupied) {
      return HandoverClearanceReason::kOccupiedContradiction;
    }
    if (state == CellState::kKnownFree) return HandoverClearanceReason::kNone;
    static_cast<void>(now_stamp_ns);
    return HandoverClearanceReason::kNoSensorEvidence;
  }

  [[nodiscard]] bool isSegmentTraversableWithCurrentBodySupport(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy unknown_policy,
      const navigation_world_model::CurrentBodySupportPtr& support = {}) const noexcept override {
    if (!start.allFinite() || !end.allFinite() || !containsLayer(start, layer) ||
        !containsLayer(end, layer) || !support || !support->valid) return false;
    const auto delta = end - start;
    const double length = delta.norm();
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? rootGrid().inflated.layout.resolution_m
                                  : rootGrid().base_layout.resolution_m;
    if (!std::isfinite(length) || !std::isfinite(resolution) || resolution <= 0.0 ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            support->snapshot_identity, identity_)) return false;
    const auto original_start_index = positionToIndex(start, layer);
    auto start_index = original_start_index;
    const auto end_index = positionToIndex(end, layer);
    auto direction = navigation_world_model::GridIndex3{
        (end_index.x() > start_index.x()) - (end_index.x() < start_index.x()),
        (end_index.y() > start_index.y()) - (end_index.y() < start_index.y()),
        (end_index.z() > start_index.z()) - (end_index.z() < start_index.z())};
    // `positionToIndex` follows the half-open floor convention.  At an exact
    // voxel face, a ray travelling in the negative direction starts in the
    // voxel on the lower side of that face.  Without this adjustment the
    // first boundary distance is zero and the traversal rejects the segment
    // (or skips the occupied lower-side voxel).
    int exact_start_face_axes = 0;
    int traversed_negative_face_axes = 0;
    for (int axis = 0; axis < 3; ++axis) {
      const double coordinate = start[axis] / resolution;
      const double nearest_boundary = std::round(coordinate);
      const double boundary_tolerance =
          32.0 * std::numeric_limits<double>::epsilon() *
          std::max(1.0, std::abs(coordinate));
      if (std::abs(coordinate - nearest_boundary) <= boundary_tolerance) {
        exact_start_face_axes |= 1 << axis;
      }
      if (direction[axis] >= 0) continue;
      traversed_negative_face_axes |= 1 << axis;
      if (std::abs(coordinate - nearest_boundary) <= boundary_tolerance &&
          start_index[axis] > std::numeric_limits<int>::min()) {
        --start_index[axis];
      }
    }
    const auto point_is_allowed = [&](const navigation_world_model::Point3& point,
                                      const double interval_begin,
                                      const double interval_end) {
      const auto state = classify(point, layer);
      using navigation_world_model::CellState;
      if (state == CellState::kOccupied || state == CellState::kOutOfMap ||
          state == CellState::kUndefined) return false;
      if (state == CellState::kKnownFree) return true;
      // Only an explicitly UNKNOWN voxel can be discharged by the measured
      // body witness. Frontier/undefined are separate map states and remain
      // rejected under the known-free handover policy. Sensor-free cells do
      // not consume the witness: UNKNOWN -> sensor-free -> UNKNOWN remains
      // valid while each unknown interval stays inside the physical OBB.
      if (state != CellState::kUnknown) return false;
      const auto a = start + interval_begin * delta;
      const auto b = start + interval_end * delta;
      const bool contained = support->containsSegment(a, b, identity_, support->source_stamp_ns);
      return contained;
    };
    // A segment beginning exactly on a face/edge/corner touches both sides
    // of every such face at t=0. Check all adjacent cells before applying
    // the negative-direction half-open-index adjustment. This keeps an
    // occupied upper-side cell authoritative and handles edge/corner ties
    // independent of traversal direction.
    for (int subset = exact_start_face_axes; subset >= 0;
         subset = (subset - 1) & exact_start_face_axes) {
      // A face on the positive travel side is only touched tangentially at
      // t=0; checking its outside neighbor would reject a valid path that
      // starts on the map boundary. Negative-side faces are entered and must
      // participate in the supercover check.
      if ((subset & ~traversed_negative_face_axes) != 0) {
        if (subset == 0) break;
        continue;
      }
      auto candidate = original_start_index;
      if ((subset & 1) != 0) --candidate.x();
      if ((subset & 2) != 0) --candidate.y();
      if ((subset & 4) != 0) --candidate.z();
      if (!point_is_allowed(indexToPosition(candidate, layer), 0.0, 0.0)) {
        return false;
      }
      if (subset == 0) break;
    }
    if (start_index == end_index) {
      if (!point_is_allowed(indexToPosition(start_index, layer), 0.0, 1.0)) return false;
    } else {
      direction = navigation_world_model::GridIndex3{
          (end_index.x() > start_index.x()) - (end_index.x() < start_index.x()),
          (end_index.y() > start_index.y()) - (end_index.y() < start_index.y()),
          (end_index.z() > start_index.z()) - (end_index.z() < start_index.z())};
      const auto absolute_delta = delta.cwiseAbs();
      const auto step_time = [&](const int axis) {
        const double component = absolute_delta[axis] / length;
        return direction[axis] == 0 ? std::numeric_limits<double>::max()
                                    : resolution / component;
      };
      const auto bound_time = [&](const int axis) {
        const double center = (static_cast<double>(start_index[axis]) + 0.5) * resolution;
        const double boundary = center + static_cast<double>(direction[axis]) * resolution * 0.5;
        const double component = absolute_delta[axis] / length;
        return direction[axis] == 0 ? std::numeric_limits<double>::max()
                                    : std::abs(boundary - start[axis]) / component;
      };
      double next_x = bound_time(0), next_y = bound_time(1), next_z = bound_time(2);
      const double step_x = step_time(0), step_y = step_time(1), step_z = step_time(2);
      auto current = start_index;
      double current_distance = 0.0;
      const double never = std::numeric_limits<double>::max();
      while (current != end_index) {
        if (current.x() == end_index.x()) next_x = never;
        if (current.y() == end_index.y()) next_y = never;
        if (current.z() == end_index.z()) next_z = never;
        const double boundary = std::min({next_x, next_y, next_z});
        const double distance_tolerance =
            32.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(boundary));
        if (!std::isfinite(boundary) ||
            boundary <= current_distance + distance_tolerance) return false;
        if (!point_is_allowed(indexToPosition(current, layer),
                              current_distance / length,
                              std::min(1.0, boundary / length))) return false;
        const double boundary_t = std::clamp(boundary / length, 0.0, 1.0);
        const double scale = std::max(1.0, std::abs(boundary));
        const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() * scale;
        int tied = 0;
        if (next_x != never && std::abs(next_x - boundary) <= tolerance) tied |= 1;
        if (next_y != never && std::abs(next_y - boundary) <= tolerance) tied |= 2;
        if (next_z != never && std::abs(next_z - boundary) <= tolerance) tied |= 4;
        for (int subset = tied; subset > 0; subset = (subset - 1) & tied) {
          auto candidate = current;
          if ((subset & 1) != 0) candidate.x() += direction.x();
          if ((subset & 2) != 0) candidate.y() += direction.y();
          if ((subset & 4) != 0) candidate.z() += direction.z();
          if (!point_is_allowed(indexToPosition(candidate, layer), boundary_t, boundary_t))
            return false;
        }
        if ((tied & 1) != 0) { current.x() += direction.x(); next_x += step_x; }
        if ((tied & 2) != 0) { current.y() += direction.y(); next_y += step_y; }
        if ((tied & 4) != 0) { current.z() += direction.z(); next_z += step_z; }
        current_distance = boundary;
      }
      if (!point_is_allowed(indexToPosition(end_index, layer), current_distance / length, 1.0)) return false;
    }
    if (layer != navigation_world_model::GridLayer::kInflated) return true;
    return navigation_world_model::observedOccupiedTubeIsClear(
        *this, start, end, rootGrid().occupied_inflation_radius_m);
  }

  [[nodiscard]] bool contains(
      const navigation_world_model::Point3& point) const noexcept override {
    if (!point.allFinite()) return false;
    return checkedOffset(rootGrid().base_layout, point,
                         navigation_world_model::GridLayer::kEvidence).has_value();
  }

  [[nodiscard]] navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept override {
    const auto& grid = rootGrid();
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? grid.inflated.layout.resolution_m
                                  : grid.base_layout.resolution_m;
    return checkedPositionToIndex(point, resolution)
        .value_or(navigation_world_model::GridIndex3::Zero());
  }

  [[nodiscard]] navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer layer) const noexcept override {
    const auto& grid = rootGrid();
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? grid.inflated.layout.resolution_m
                                  : grid.base_layout.resolution_m;
    if (!std::isfinite(resolution) || resolution <= 0.0) {
      return navigation_world_model::Point3::Zero();
    }
    navigation_world_model::Point3 result;
    for (int axis = 0; axis < 3; ++axis) {
      const long double cell_center = static_cast<long double>(index(axis)) + 0.5L;
      const long double position = cell_center * static_cast<long double>(resolution);
      if (!std::isfinite(position) ||
          position < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
          position > static_cast<long double>(std::numeric_limits<double>::max())) {
        return navigation_world_model::Point3::Zero();
      }
      result(axis) = static_cast<double>(position);
    }
    return result;
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
    const auto& grid = rootGrid();
    const auto start_index = checkedPositionToIndex(
        start, grid.base_layout.resolution_m);
    if (!start_index) return std::nullopt;
    for (const auto& delta : *grid.nearest_offsets) {
      const auto candidate_index = checkedIndexAdd(*start_index, delta);
      if (!candidate_index) continue;
      const auto candidate = indexToPosition(*candidate_index,
                                             navigation_world_model::GridLayer::kEvidence);
      if ((candidate - start).norm() > maximum_distance_m) return std::nullopt;
      if (navigation_world_model::isCellTraversable(
              classify(candidate, layer),
              navigation_world_model::UnknownPolicy::kAllowUnknown)) {
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
    if (!start.allFinite() || !end.allFinite() || !containsLayer(start, layer) ||
        !containsLayer(end, layer)) {
      return false;
    }
    const auto& grid = rootGrid();
    const double resolution = layer == navigation_world_model::GridLayer::kInflated
                                  ? grid.inflated.layout.resolution_m
                                  : grid.base_layout.resolution_m;
    const auto start_index = positionToIndex(start, layer);
    const auto end_index = positionToIndex(end, layer);
    const auto point_is_traversable = [this, layer, unknown_policy](
        const navigation_world_model::Point3& point) {
      if (!containsLayer(point, layer)) return false;
      return navigation_world_model::isCellTraversable(
          classify(point, layer), unknown_policy);
    };
    if (!point_is_traversable(start)) return false;
    if (start_index != end_index) {
      const auto delta = end - start;
      const double length = delta.norm();
      if (!std::isfinite(resolution) || resolution <= 0.0 ||
          !std::isfinite(length) || length <= 0.0) {
        return false;
      }
      const navigation_world_model::GridIndex3 direction{
          (end_index.x() > start_index.x()) - (end_index.x() < start_index.x()),
          (end_index.y() > start_index.y()) - (end_index.y() < start_index.y()),
          (end_index.z() > start_index.z()) - (end_index.z() < start_index.z())};
      const auto absolute_delta = delta.cwiseAbs();
      const auto step_time = [&](const int axis) {
        const double component = absolute_delta[axis] / length;
        return direction[axis] == 0 ? std::numeric_limits<double>::max()
                                    : resolution / component;
      };
      const auto bound_time = [&](const int axis) {
        const double center =
            (static_cast<double>(start_index[axis]) + 0.5) * resolution;
        const double boundary = center +
            static_cast<double>(direction[axis]) * resolution * 0.5;
        const double component = absolute_delta[axis] / length;
        return direction[axis] == 0 ? std::numeric_limits<double>::max()
                                    : std::abs(boundary - start[axis]) / component;
      };
      double next_x = bound_time(0);
      double next_y = bound_time(1);
      double next_z = bound_time(2);
      const double step_x = step_time(0);
      const double step_y = step_time(1);
      const double step_z = step_time(2);
      auto current = start_index;
      const double never = std::numeric_limits<double>::max();
      while (current != end_index) {
        if (!point_is_traversable(indexToPosition(current, layer))) return false;
        if (current.x() == end_index.x()) next_x = never;
        if (current.y() == end_index.y()) next_y = never;
        if (current.z() == end_index.z()) next_z = never;
        const double first_boundary = std::min({next_x, next_y, next_z});
        if (!std::isfinite(first_boundary)) return false;
        // A segment that crosses a voxel edge or corner touches every cell
        // adjacent to that boundary.  Enumerating all tied-axis subsets makes
        // the certificate independent of traversal direction; the old
        // single-axis tie break could accept A->B while rejecting B->A.
        double boundary_scale = std::max(1.0, std::abs(first_boundary));
        const auto includeBoundaryScale = [&](const double boundary) {
          if (std::isfinite(boundary) && boundary != never) {
            boundary_scale = std::max(boundary_scale, std::abs(boundary));
          }
        };
        includeBoundaryScale(next_x);
        includeBoundaryScale(next_y);
        includeBoundaryScale(next_z);
        // Opposite traversal directions can accumulate a few ulps of
        // difference while computing the same geometric edge/corner. Keep the
        // comparison conservative and direction-independent without allowing
        // an inactive-axis sentinel to participate in the scale.
        constexpr double kBoundaryTieUlps = 32.0;
        const double tie_tolerance = kBoundaryTieUlps *
                                     std::numeric_limits<double>::epsilon() *
                                     boundary_scale;
        int tied_axes = 0;
        if (next_x != never && std::abs(next_x - first_boundary) <= tie_tolerance) tied_axes |= 1;
        if (next_y != never && std::abs(next_y - first_boundary) <= tie_tolerance) tied_axes |= 2;
        if (next_z != never && std::abs(next_z - first_boundary) <= tie_tolerance) tied_axes |= 4;
        for (int subset = tied_axes; subset > 0; subset = (subset - 1) & tied_axes) {
          auto candidate = current;
          if ((subset & 1) != 0) candidate.x() += direction.x();
          if ((subset & 2) != 0) candidate.y() += direction.y();
          if ((subset & 4) != 0) candidate.z() += direction.z();
          if (!point_is_traversable(indexToPosition(candidate, layer))) return false;
        }
        if ((tied_axes & 1) != 0) {
          current.x() += direction.x();
          next_x += step_x;
        }
        if ((tied_axes & 2) != 0) {
          current.y() += direction.y();
          next_y += step_y;
        }
        if ((tied_axes & 4) != 0) {
          current.z() += direction.z();
          next_z += step_z;
        }
      }
    }
    // The DDA stops before classifying the endpoint voxel. Check it explicitly,
    // including the same-voxel case where no traversal step is needed.
    if (!point_is_traversable(end)) return false;
    if (layer != navigation_world_model::GridLayer::kInflated) return true;
    return navigation_world_model::observedOccupiedTubeIsClear(
        *this, start, end, grid.occupied_inflation_radius_m);
  }

  [[nodiscard]] navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& requested) const noexcept override {
    auto result = requested;
    if (!result.minimum.allFinite() || !result.maximum.allFinite() ||
        (result.maximum - result.minimum).minCoeff() <= 0.0) {
      result.minimum = result.maximum;
      return result;
    }
    const auto& grid = rootGrid();
    const auto minimum_center = indexToPosition(
        grid.base_layout.global_min_index, navigation_world_model::GridLayer::kEvidence);
    const auto maximum_index = checkedIndexAdd(
        grid.base_layout.global_min_index,
        grid.base_layout.dimensions - navigation_world_model::GridIndex3::Ones());
    if (!maximum_index) {
      result.minimum = result.maximum;
      return result;
    }
    const auto maximum_center = indexToPosition(
        *maximum_index, navigation_world_model::GridLayer::kEvidence);
    result.minimum = result.minimum.cwiseMax(minimum_center);
    result.maximum = result.maximum.cwiseMin(maximum_center);
    if (grid.virtual_ground_ceiling_enabled) {
      result.minimum.z() = std::max(result.minimum.z(), grid.virtual_ground_m);
      result.maximum.z() = std::min(result.maximum.z(), grid.virtual_ceiling_m);
    }
    return result;
  }

  [[nodiscard]] navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& requested) const override {
    navigation_world_model::PointVector points;
    const auto& grid = rootGrid();
    const auto box = clampToLocalBounds(requested);
    if ((box.maximum - box.minimum).minCoeff() <= 0.0) return points;
    const auto minimum = positionToIndex(box.minimum, navigation_world_model::GridLayer::kEvidence);
    const auto maximum = positionToIndex(box.maximum, navigation_world_model::GridLayer::kEvidence);
    // Include both cells containing the requested bounds. Excluding either
    // boundary can omit an occupied cell whose center is still inside the
    // clearance query by less than one voxel, making a coarse inflated layer
    // optimistically accept the segment.
    const auto x_begin = static_cast<std::int64_t>(minimum.x());
    const auto y_begin = static_cast<std::int64_t>(minimum.y());
    const auto z_begin = static_cast<std::int64_t>(minimum.z());
    const auto x_end = static_cast<std::int64_t>(maximum.x()) + 1;
    const auto y_end = static_cast<std::int64_t>(maximum.y()) + 1;
    const auto z_end = static_cast<std::int64_t>(maximum.z()) + 1;
    const auto x_count = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, x_end - x_begin));
    const auto y_count = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, y_end - y_begin));
    const auto z_count = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, z_end - z_begin));
    const auto size_max = static_cast<std::uint64_t>(
        std::numeric_limits<std::size_t>::max());
    std::size_t reserve_count = 1U;
    bool reserve_overflow = false;
    for (const auto count : {x_count, y_count, z_count}) {
      if (count == 0U) {
        reserve_count = 0U;
        break;
      }
      if (count > size_max || reserve_count >
          static_cast<std::size_t>(size_max / count)) {
        reserve_overflow = true;
        break;
      }
      reserve_count *= static_cast<std::size_t>(count);
    }
    if (!reserve_overflow && reserve_count > 0U) {
      points.reserve(reserve_count / 12U);
    }
    for (std::int64_t x = x_begin; x < x_end; ++x) {
      for (std::int64_t y = y_begin; y < y_end; ++y) {
        for (std::int64_t z = z_begin; z < z_end; ++z) {
          const navigation_world_model::GridIndex3 index{
              static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
          const auto cell_offset = offset(grid.base_layout, index);
          if (cell_offset && baseState(index) ==
                                 navigation_world_model::CellState::kOccupied) {
            points.emplace_back(indexToPosition(index,
                                                navigation_world_model::GridLayer::kEvidence));
          }
        }
      }
    }
    return points;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept {
    return owned_bytes_ + sharedMetadataByteSize();
  }
  [[nodiscard]] std::size_t ownedByteSize() const noexcept { return owned_bytes_; }
  [[nodiscard]] std::size_t sharedMetadataByteSize() const noexcept {
    return rootGrid().sharedMetadataByteSize();
  }
  [[nodiscard]] MappingSnapshotMetrics metrics() const noexcept {
    MappingSnapshotMetrics result{
        static_cast<std::uint64_t>(byteSize()),
        static_cast<std::uint64_t>(ownedByteSize()),
        static_cast<std::uint64_t>(sharedMetadataByteSize()),
        static_cast<std::uint64_t>(liveCount()),
        static_cast<std::uint64_t>(peakLiveCount()),
        static_cast<std::uint64_t>(liveOwnedBytes()),
        static_cast<std::uint64_t>(peakLiveOwnedBytes())};
    return result;
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
  [[nodiscard]] const PlanningGrid& rootGrid() const noexcept {
    return grid_ ? *grid_ : parent_->rootGrid();
  }

  [[nodiscard]] std::uint8_t inflatedOccupied(
      const navigation_world_model::GridIndex3& index) const noexcept {
    if (patch_) {
      const auto patch_offset = offset(patch_->inflated.layout, index);
      if (patch_offset) return patch_->inflated.occupied[*patch_offset];
    }
    if (parent_) return parent_->inflatedOccupied(index);
    const auto root_offset = offset(rootGrid().inflated.layout, index);
    return root_offset ? rootGrid().inflated.occupied[*root_offset] : 1U;
  }

  [[nodiscard]] std::uint8_t inflatedUnknown(
      const navigation_world_model::GridIndex3& index) const noexcept {
    if (patch_) {
      const auto patch_offset = offset(patch_->inflated.layout, index);
      if (patch_offset && !patch_->inflated.unknown.empty()) {
        return patch_->inflated.unknown[*patch_offset];
      }
    }
    if (parent_) return parent_->inflatedUnknown(index);
    const auto root_offset = offset(rootGrid().inflated.layout, index);
    return root_offset && !rootGrid().inflated.unknown.empty()
               ? rootGrid().inflated.unknown[*root_offset]
               : 1U;
  }

  static void updatePeak(std::atomic_size_t& peak, std::size_t value) noexcept {
    auto previous = peak.load(std::memory_order_relaxed);
    while (previous < value &&
           !peak.compare_exchange_weak(previous, value, std::memory_order_relaxed)) {}
  }

  static PlanningGrid validated(
      PlanningGrid grid,
      const navigation_world_model::WorldSnapshotIdentity& identity) {
    const auto validate_layout = [](const PlanningGridLayout& layout,
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
        const auto last_index = static_cast<std::int64_t>(layout.global_min_index[axis]) +
                                static_cast<std::int64_t>(layout.dimensions[axis]) - 1;
        if (last_index < static_cast<std::int64_t>(std::numeric_limits<int>::min()) ||
            last_index > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
          throw std::overflow_error(std::string{name} + " index range overflows int");
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
    for (const auto state : grid.base_state) {
      if (!navigation_world_model::isStoredCellState(
              static_cast<navigation_world_model::CellState>(state))) {
        throw std::invalid_argument(
            "planning grid contains a non-evidence cell state");
      }
    }
    if (!grid.nearest_offsets) {
      throw std::invalid_argument("planning grid nearest-offset metadata is missing");
    }
    if (!std::isfinite(grid.virtual_ground_m) || !std::isfinite(grid.virtual_ceiling_m) ||
        !std::isfinite(grid.inflated_virtual_ground_m) ||
        !std::isfinite(grid.inflated_virtual_ceiling_m) ||
        !std::isfinite(grid.occupied_inflation_radius_m) ||
        grid.occupied_inflation_radius_m < 0.0 ||
        grid.virtual_ground_m >= grid.virtual_ceiling_m ||
        grid.inflated_virtual_ground_m >= grid.inflated_virtual_ceiling_m ||
        identity.localization_epoch == 0U || identity.generation == 0U ||
        identity.observation_stamp_ns < 0) {
      throw std::invalid_argument("planning grid metadata or identity is invalid");
    }
    return grid;
  }

  static PlanningGridPatch validatedPatch(
      PlanningGridPatch patch,
      const std::shared_ptr<const MappingWorldSnapshot>& parent,
      const navigation_world_model::WorldSnapshotIdentity& identity) {
    if (!parent) throw std::invalid_argument("patch snapshot parent is missing");
    const auto parent_identity = parent->identity();
    if (identity.localization_epoch != parent_identity.localization_epoch ||
        identity.generation != parent_identity.generation ||
        parent_identity.revision == std::numeric_limits<std::uint64_t>::max() ||
        identity.revision <= parent_identity.revision ||
        identity.observation_stamp_ns <= parent_identity.observation_stamp_ns) {
      throw std::invalid_argument("patch snapshot identity is not a newer successor");
    }
    const auto validate_layout = [](const PlanningGridLayout& layout,
                                    const char* name) -> std::size_t {
      if (!std::isfinite(layout.resolution_m) || layout.resolution_m <= 0.0 ||
          (layout.dimensions.array() <= 0).any()) {
        throw std::invalid_argument(std::string{name} + " has invalid patch geometry");
      }
      std::size_t count = 1U;
      for (int axis = 0; axis < 3; ++axis) {
        const auto dimension = static_cast<std::size_t>(layout.dimensions[axis]);
        if (count > std::numeric_limits<std::size_t>::max() / dimension) {
          throw std::overflow_error(std::string{name} + " patch voxel count overflows size_t");
        }
        const auto last_index = static_cast<std::int64_t>(layout.global_min_index[axis]) +
                                static_cast<std::int64_t>(layout.dimensions[axis]) - 1;
        if (last_index < static_cast<std::int64_t>(std::numeric_limits<int>::min()) ||
            last_index > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
          throw std::overflow_error(std::string{name} + " patch index range overflows int");
        }
        count *= dimension;
      }
      return count;
    };
    const auto base_count = validate_layout(patch.base_layout, "base patch");
    const auto inflated_count = validate_layout(patch.inflated.layout, "inflated patch");
    const auto& root = parent->rootGrid();
    if (patch.base_layout.resolution_m != root.base_layout.resolution_m ||
        patch.inflated.layout.resolution_m != root.inflated.layout.resolution_m ||
        patch.base_state.size() != base_count ||
        patch.inflated.occupied.size() != inflated_count ||
        (root.unknown_inflation_enabled && patch.inflated.unknown.size() != inflated_count) ||
        (!root.unknown_inflation_enabled && !patch.inflated.unknown.empty())) {
      throw std::invalid_argument("planning grid patch does not match parent geometry");
    }
    for (const auto state : patch.base_state) {
      if (!navigation_world_model::isStoredCellState(
              static_cast<navigation_world_model::CellState>(state))) {
        throw std::invalid_argument("planning grid patch contains a non-evidence state");
      }
    }
    return patch;
  }

  static std::optional<std::size_t> offset(
      const PlanningGridLayout& layout,
      const navigation_world_model::GridIndex3& index) noexcept {
    const auto local_x = static_cast<std::int64_t>(index.x()) -
                         static_cast<std::int64_t>(layout.global_min_index.x());
    const auto local_y = static_cast<std::int64_t>(index.y()) -
                         static_cast<std::int64_t>(layout.global_min_index.y());
    const auto local_z = static_cast<std::int64_t>(index.z()) -
                         static_cast<std::int64_t>(layout.global_min_index.z());
    if (local_x < 0 || local_y < 0 || local_z < 0 ||
        local_x >= static_cast<std::int64_t>(layout.dimensions.x()) ||
        local_y >= static_cast<std::int64_t>(layout.dimensions.y()) ||
        local_z >= static_cast<std::int64_t>(layout.dimensions.z())) {
      return std::nullopt;
    }
    const auto x = static_cast<std::uint64_t>(local_x);
    const auto y = static_cast<std::uint64_t>(local_y);
    const auto z = static_cast<std::uint64_t>(local_z);
    const auto dimensions_y = static_cast<std::uint64_t>(layout.dimensions.y());
    const auto dimensions_z = static_cast<std::uint64_t>(layout.dimensions.z());
    const auto maximum = static_cast<std::uint64_t>(
        std::numeric_limits<std::size_t>::max());
    if (y > (maximum - z) / dimensions_z) {
      return std::nullopt;
    }
    const auto yz = y * dimensions_z + z;
    if (x > (maximum - yz) / dimensions_y) {
      return std::nullopt;
    }
    const auto xy = x * dimensions_y + y;
    if (xy > (maximum - z) / dimensions_z) return std::nullopt;
    return static_cast<std::size_t>(xy * dimensions_z + z);
  }

  static std::optional<navigation_world_model::GridIndex3> checkedPositionToIndex(
      const navigation_world_model::Point3& point, const double resolution) noexcept {
    if (!point.allFinite() || !std::isfinite(resolution) || resolution <= 0.0) {
      return std::nullopt;
    }
    navigation_world_model::GridIndex3 result;
    for (int axis = 0; axis < 3; ++axis) {
      const double scaled = point[axis] / resolution;
      if (!std::isfinite(scaled)) return std::nullopt;
      const double floored = std::floor(scaled);
      if (floored < static_cast<double>(std::numeric_limits<int>::min()) ||
          floored > static_cast<double>(std::numeric_limits<int>::max())) {
        return std::nullopt;
      }
      result[axis] = static_cast<int>(floored);
    }
    return result;
  }

  static std::optional<navigation_world_model::GridIndex3> checkedIndexAdd(
      const navigation_world_model::GridIndex3& lhs,
      const navigation_world_model::GridIndex3& rhs) noexcept {
    navigation_world_model::GridIndex3 result;
    for (int axis = 0; axis < 3; ++axis) {
      const auto value = static_cast<std::int64_t>(lhs[axis]) +
                         static_cast<std::int64_t>(rhs[axis]);
      if (value < static_cast<std::int64_t>(std::numeric_limits<int>::min()) ||
          value > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
        return std::nullopt;
      }
      result[axis] = static_cast<int>(value);
    }
    return result;
  }

  static bool boxesOverlap(const navigation_world_model::AxisAlignedBox& lhs,
                           const navigation_world_model::AxisAlignedBox& rhs) noexcept {
    if (!lhs.valid() || !rhs.valid()) return true;
    return (lhs.minimum.array() <= rhs.maximum.array()).all() &&
           (rhs.minimum.array() <= lhs.maximum.array()).all();
  }

  [[nodiscard]] bool containsLayer(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer layer) const noexcept {
    if (!point.allFinite()) return false;
    // The evidence grid defines the product world window.  The inflated grid
    // may have a larger storage halo for radius inflation, but that halo must
    // never extend the traversable world outside observed-map bounds.
    if (!checkedOffset(rootGrid().base_layout, point,
                       navigation_world_model::GridLayer::kEvidence)) {
      return false;
    }
    if (layer == navigation_world_model::GridLayer::kEvidence) return true;
    return checkedOffset(rootGrid().inflated.layout, point,
                         navigation_world_model::GridLayer::kInflated).has_value();
  }

  static std::optional<std::size_t> checkedOffset(
      const PlanningGridLayout& layout,
      const navigation_world_model::Point3& point,
      const navigation_world_model::GridLayer layer) noexcept {
    static_cast<void>(layer);
    const auto index = checkedPositionToIndex(point, layout.resolution_m);
    return index ? offset(layout, *index) : std::nullopt;
  }

  [[nodiscard]] navigation_world_model::CellState baseState(
      const navigation_world_model::GridIndex3& index) const noexcept {
    if (patch_) {
      const auto patch_offset = offset(patch_->base_layout, index);
      if (patch_offset) return productCell(patch_->base_state[*patch_offset]);
    }
    if (parent_) return parent_->baseState(index);
    const auto cell_offset = offset(rootGrid().base_layout, index);
    return cell_offset ? productCell(rootGrid().base_state[*cell_offset])
                       : navigation_world_model::CellState::kOutOfMap;
  }

  static navigation_world_model::CellState productCell(
      const std::uint8_t raw_state) noexcept {
    const auto state = static_cast<navigation_world_model::CellState>(raw_state);
    using navigation_world_model::CellState;
    switch (state) {
      case CellState::kUndefined:
      case CellState::kUnknown:
      case CellState::kOutOfMap:
      case CellState::kOccupied:
      case CellState::kKnownFree:
      case CellState::kFrontier:
        return state;
    }
    return CellState::kUndefined;
  }

  const std::optional<PlanningGrid> grid_;
  const std::shared_ptr<const MappingWorldSnapshot> parent_;
  const std::optional<PlanningGridPatch> patch_;
  const navigation_world_model::WorldSnapshotIdentity identity_;
  const navigation_world_model::WorldChangeHistoryPtr change_history_;
  const std::size_t owned_bytes_;
  inline static std::atomic_size_t live_count_{0U};
  inline static std::atomic_size_t peak_live_count_{0U};
  inline static std::atomic_size_t live_owned_bytes_{0U};
  inline static std::atomic_size_t peak_live_owned_bytes_{0U};
};

}  // namespace navigation_mapping

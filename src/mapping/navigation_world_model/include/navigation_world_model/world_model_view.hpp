#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace navigation_world_model {

using Point3 = Eigen::Vector3d;
using GridIndex3 = Eigen::Vector3i;
using PointVector = std::vector<Point3, Eigen::aligned_allocator<Point3>>;

enum class CellState : std::uint8_t {
  kUndefined,
  kUnknown,
  kOutOfMap,
  kOccupied,
  kKnownFree,
  kFrontier,
};

enum class GridLayer : std::uint8_t { kEvidence, kInflated };
enum class UnknownPolicy : std::uint8_t { kAllowUnknown, kRequireKnownFree };

[[nodiscard]] constexpr bool isValidCellState(CellState state) noexcept {
  switch (state) {
    case CellState::kUndefined:
    case CellState::kUnknown:
    case CellState::kOutOfMap:
    case CellState::kOccupied:
    case CellState::kKnownFree:
    case CellState::kFrontier:
      return true;
  }
  return false;
}

// The immutable planning-grid export stores only evidence states.  The other
// enum values remain part of the query result domain, but must never enter the
// backing array as if they were evidence.
[[nodiscard]] constexpr bool isStoredCellState(CellState state) noexcept {
  return state == CellState::kUnknown || state == CellState::kOccupied ||
         state == CellState::kKnownFree;
}

// One total traversability predicate for every world-model consumer. Unknown
// and frontier cells are equivalent for this decision; OUT_OF_MAP, OCCUPIED,
// UNDEFINED and any future invalid value are never traversable.
[[nodiscard]] constexpr bool isCellTraversable(
    CellState state, UnknownPolicy unknown_policy) noexcept {
  switch (state) {
    case CellState::kKnownFree:
      return true;
    case CellState::kUnknown:
    case CellState::kFrontier:
      return unknown_policy == UnknownPolicy::kAllowUnknown;
    case CellState::kUndefined:
    case CellState::kOutOfMap:
    case CellState::kOccupied:
      return false;
  }
  return false;
}

struct AxisAlignedBox {
  Point3 minimum{Point3::Zero()};
  Point3 maximum{Point3::Zero()};

  [[nodiscard]] bool valid() const noexcept {
    return minimum.allFinite() && maximum.allFinite() &&
           (maximum.array() >= minimum.array()).all();
  }
};

struct GridBounds {
  GridIndex3 global_min_index{GridIndex3::Zero()};
  GridIndex3 dimensions{GridIndex3::Zero()};

  [[nodiscard]] bool valid() const noexcept {
    return (dimensions.array() > 0).all();
  }
};

struct WorldGeometry {
  double evidence_resolution_m{0.0};
  double inflated_resolution_m{0.0};
  double occupied_inflation_radius_m{0.0};
  double effective_virtual_ground_m{0.0};
  double effective_virtual_ceiling_m{0.0};
  Point3 local_center_m{Point3::Zero()};
  Point3 local_size_m{Point3::Zero()};
  // When false, the effective Z values describe the current sliding-map
  // availability window, not physical occupied floor/ceiling planes.
  bool virtual_ground_ceiling_enabled{true};
  // Discrete bounds are authoritative for index-addressed search.  They are
  // kept per layer because the inflated storage may have a different
  // resolution and halo from the evidence grid.
  GridBounds evidence_bounds{};
  GridBounds inflated_bounds{};
};

struct WorldSnapshotIdentity {
  std::uint64_t localization_epoch{0};
  std::uint64_t generation{0};
  std::uint64_t revision{0};
  std::int64_t observation_stamp_ns{0};
};

struct WorldChangeRecord {
  WorldSnapshotIdentity identity{};
  AxisAlignedBox affected_region{};
  bool affects_whole_world{false};
};

// Each immutable snapshot carries a bounded newest-first provenance window.
// The producer owns the retention limit; consumers require contiguous revision
// coverage and fail closed when the requested interval is older than the
// retained window. A flat immutable window is deliberate: truncating a singly
// linked list at its tail would either lose a recent record or require an
// unbounded walk.
struct WorldChangeHistory {
  std::vector<WorldChangeRecord> records;
};

using WorldChangeHistoryPtr = std::shared_ptr<const WorldChangeHistory>;

[[nodiscard]] inline bool sameWorldSnapshotIdentity(
    const WorldSnapshotIdentity& lhs,
    const WorldSnapshotIdentity& rhs) noexcept {
  return lhs.localization_epoch == rhs.localization_epoch &&
         lhs.generation == rhs.generation && lhs.revision == rhs.revision &&
         lhs.observation_stamp_ns == rhs.observation_stamp_ns;
}

// Read-only planning contract. Implementations must preserve their documented
// cell centers, ray traversal, nearest-cell tie breaking, and occupied-point
// order. A view used by one solve must keep one identity for the whole solve.
class WorldModelView {
 public:
  virtual ~WorldModelView() = default;

  [[nodiscard]] virtual WorldGeometry geometry() const noexcept = 0;
  [[nodiscard]] virtual WorldSnapshotIdentity identity() const noexcept = 0;

  // Return true unless the changes after `older` are proven disjoint from the
  // protected trajectory region. Implementations that cannot provide complete
  // provenance must retain the fail-closed default.
  [[nodiscard]] virtual bool changedRegionIntersectsSince(
      const WorldSnapshotIdentity& older,
      const AxisAlignedBox& protected_region) const noexcept {
    static_cast<void>(older);
    static_cast<void>(protected_region);
    return true;
  }
  [[nodiscard]] virtual CellState classify(const Point3& point,
                                           GridLayer layer) const noexcept = 0;
  [[nodiscard]] virtual bool contains(const Point3& point) const noexcept = 0;
  [[nodiscard]] virtual GridIndex3 positionToIndex(const Point3& point,
                                                   GridLayer layer) const noexcept = 0;
  [[nodiscard]] virtual Point3 indexToPosition(const GridIndex3& index,
                                               GridLayer layer) const noexcept = 0;
  [[nodiscard]] virtual std::optional<Point3> nearestNotOccupied(
      const Point3& start, GridLayer layer, double maximum_distance_m) const = 0;
  [[nodiscard]] virtual bool isSegmentTraversable(
      const Point3& start, const Point3& end, GridLayer layer,
      UnknownPolicy unknown_policy) const noexcept = 0;
  [[nodiscard]] virtual AxisAlignedBox clampToLocalBounds(
      const AxisAlignedBox& requested) const noexcept = 0;
  [[nodiscard]] virtual PointVector observedOccupiedPoints(
      const AxisAlignedBox& box) const = 0;
};

using WorldModelViewPtr = std::shared_ptr<const WorldModelView>;

}  // namespace navigation_world_model

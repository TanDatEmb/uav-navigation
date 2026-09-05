#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
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

// Free-space provenance is deliberately separate from the legacy CellState
// domain.  SENSOR_FREE is backed by the probabilistic/raycast map; current
// body support is a short-lived geometric overlay and is never written into
// that map.  OCCUPIED has precedence over both free-space sources.
enum class FreeSpaceEvidence : std::uint8_t {
  kUnknown,
  kSensorFree,
  kCurrentBodySupport,
  kOccupied,
  kOutOfMap,
};

enum class HandoverClearanceReason : std::uint8_t {
  kNone = 0,
  kNoSensorEvidence,
  kNoCurrentBodySupport,
  kOutsideCurrentBodySupport,
  kOccupiedContradiction,
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

// Return the finite distance from `origin` to the first boundary of the
// axis-aligned local map along a unit direction.  The world model remains an
// ENU evidence grid; this helper is only a geometry contract for consumers
// that need to decide whether an oriented route has enough support.  It does
// not rotate or resample the voxel storage.
[[nodiscard]] inline std::optional<double> directionalSupportToLocalBoundary(
    const Point3& origin, const Point3& direction,
    const WorldGeometry& geometry) noexcept {
  if (!origin.allFinite() || !direction.allFinite() ||
      !geometry.local_center_m.allFinite() ||
      !geometry.local_size_m.allFinite() ||
      (geometry.local_size_m.array() <= 0.0).any()) {
    return std::nullopt;
  }

  const double direction_scale = direction.cwiseAbs().maxCoeff();
  if (!std::isfinite(direction_scale) || direction_scale <= 0.0) {
    return std::nullopt;
  }
  const double direction_norm = direction_scale *
      (direction / direction_scale).norm();
  if (!std::isfinite(direction_norm) || direction_norm <= 1.0e-12) {
    return std::nullopt;
  }
  const Point3 unit_direction = direction / direction_norm;
  Point3 local_min;
  Point3 local_max;
  for (int axis = 0; axis < 3; ++axis) {
    const long double half = 0.5L * static_cast<long double>(geometry.local_size_m(axis));
    const long double center = static_cast<long double>(geometry.local_center_m(axis));
    const long double minimum = center - half;
    const long double maximum = center + half;
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        minimum < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
        maximum > static_cast<long double>(std::numeric_limits<double>::max()) ||
        minimum > maximum) {
      return std::nullopt;
    }
    local_min(axis) = static_cast<double>(minimum);
    local_max(axis) = static_cast<double>(maximum);
  }
  constexpr double kBoundaryEpsilonM = 1.0e-9;
  if ((origin.array() < local_min.array() - kBoundaryEpsilonM).any() ||
      (origin.array() > local_max.array() + kBoundaryEpsilonM).any()) {
    return 0.0;
  }

  double support_m = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    const double component = unit_direction(axis);
    if (std::abs(component) <= 1.0e-12) {
      continue;
    }
    const double boundary = component > 0.0 ? local_max(axis) : local_min(axis);
    const long double distance =
        (static_cast<long double>(boundary) - static_cast<long double>(origin(axis))) /
        static_cast<long double>(component);
    if (!std::isfinite(distance) ||
        distance < -static_cast<long double>(kBoundaryEpsilonM) ||
        distance > static_cast<long double>(std::numeric_limits<double>::max())) {
      return std::nullopt;
    }
    support_m = std::min(support_m, std::max(0.0, static_cast<double>(distance)));
  }
  if (!std::isfinite(support_m)) {
    return std::nullopt;
  }
  return support_m;
}

struct WorldSnapshotIdentity {
  std::uint64_t localization_epoch{0};
  std::uint64_t generation{0};
  std::uint64_t revision{0};
  std::int64_t observation_stamp_ns{0};
};

// Ephemeral support for the latest measured rigid body only.  This is a
// geometry witness, never a history or occupancy update.  The primitive
// values are supplied by the mapping adapter's project-owned model contract.
struct CurrentBodySupport {
  struct Box {
    Point3 center{Point3::Zero()};
    Point3 half_extent{Point3::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };
  struct Cylinder {
    Point3 center{Point3::Zero()};
    double radius{0.0};
    double half_height{0.0};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  WorldSnapshotIdentity snapshot_identity{};
  Point3 body_position{Point3::Zero()};
  Eigen::Quaterniond body_orientation{Eigen::Quaterniond::Identity()};
  std::uint64_t localization_epoch{0};
  std::uint64_t scan_sequence{0};
  std::int64_t source_stamp_ns{0};
  std::uint64_t geometry_provenance_id{0};
  std::array<Box, 5> body_boxes{};
  Cylinder sensor_housing{};
  bool valid{false};

  [[nodiscard]] static bool finiteUnitQuaternion(
      const Eigen::Quaterniond& quaternion) noexcept {
    if (!quaternion.coeffs().allFinite()) return false;
    const double norm = quaternion.norm();
    return std::isfinite(norm) && norm > 0.0 &&
           std::abs(norm - 1.0) <= 1.0e-6;
  }

  [[nodiscard]] bool finiteGeometry() const noexcept {
    if (geometry_provenance_id == 0U || !body_position.allFinite() ||
        !finiteUnitQuaternion(body_orientation) ||
        localization_epoch == 0U || scan_sequence == 0U ||
        source_stamp_ns <= 0 || snapshot_identity.generation == 0U ||
        snapshot_identity.revision == 0U ||
        snapshot_identity.localization_epoch != localization_epoch ||
        snapshot_identity.observation_stamp_ns != source_stamp_ns) {
      return false;
    }
    for (const auto& box : body_boxes) {
      if (!box.center.allFinite() || !box.half_extent.allFinite() ||
          (box.half_extent.array() <= 0.0).any() ||
          !finiteUnitQuaternion(box.orientation)) {
        return false;
      }
    }
    return sensor_housing.center.allFinite() &&
        std::isfinite(sensor_housing.radius) && sensor_housing.radius > 0.0 &&
        std::isfinite(sensor_housing.half_height) &&
        sensor_housing.half_height > 0.0 &&
        finiteUnitQuaternion(sensor_housing.orientation);
  }

  [[nodiscard]] bool matchesMeasuredState(
      const Point3& measured_position,
      const Eigen::Quaterniond& measured_orientation,
      const std::uint64_t measured_epoch,
      const std::int64_t measured_stamp_ns) const noexcept {
    if (!finiteGeometry() || !measured_position.allFinite() ||
        !finiteUnitQuaternion(measured_orientation) ||
        measured_epoch != localization_epoch || measured_stamp_ns != source_stamp_ns) {
      return false;
    }
    // Quaternion signs represent the same rotation; compare the absolute
    // inner product after requiring both inputs to be unit quaternions.
    return (measured_position - body_position).norm() <= 1.0e-9 &&
        std::abs(measured_orientation.dot(body_orientation)) >= 1.0 - 1.0e-9;
  }

  [[nodiscard]] bool contains(const Point3& point,
                              const WorldSnapshotIdentity& identity,
                              const std::int64_t now_stamp_ns) const noexcept {
    if (!valid || !finiteGeometry() || !point.allFinite() ||
        snapshot_identity.localization_epoch != identity.localization_epoch ||
        snapshot_identity.generation != identity.generation ||
        snapshot_identity.revision != identity.revision ||
        snapshot_identity.observation_stamp_ns != identity.observation_stamp_ns ||
        now_stamp_ns != source_stamp_ns) {
      return false;
    }
    const Point3 body_point = body_orientation.conjugate() *
                              (point - body_position);
    if (!body_point.allFinite()) return false;
    for (const auto& box : body_boxes) {
      const Point3 local = box.orientation.conjugate() * (body_point - box.center);
      if (local.allFinite() &&
          (local.cwiseAbs().array() <= box.half_extent.array()).all()) {
        return true;
      }
    }
    const Point3 local = sensor_housing.orientation.conjugate() *
                         (body_point - sensor_housing.center);
    return local.allFinite() &&
        std::isfinite(sensor_housing.radius) && sensor_housing.radius >= 0.0 &&
        std::isfinite(sensor_housing.half_height) && sensor_housing.half_height >= 0.0 &&
        local.head<2>().squaredNorm() <=
            sensor_housing.radius * sensor_housing.radius + 1.0e-12 &&
        std::abs(local.z()) <= sensor_housing.half_height + 1.0e-12;
  }
};

using CurrentBodySupportPtr = std::shared_ptr<const CurrentBodySupport>;

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
  [[nodiscard]] virtual FreeSpaceEvidence classifyFreeSpace(
      const Point3& point, GridLayer layer,
      std::int64_t now_stamp_ns = 0) const noexcept {
    static_cast<void>(now_stamp_ns);
    const auto state = classify(point, layer);
    switch (state) {
      case CellState::kKnownFree: return FreeSpaceEvidence::kSensorFree;
      case CellState::kOccupied: return FreeSpaceEvidence::kOccupied;
      case CellState::kOutOfMap: return FreeSpaceEvidence::kOutOfMap;
      case CellState::kUnknown:
      case CellState::kFrontier:
      case CellState::kUndefined:
        return FreeSpaceEvidence::kUnknown;
    }
    return FreeSpaceEvidence::kUnknown;
  }
  [[nodiscard]] virtual HandoverClearanceReason handoverClearanceReason(
      const Point3& point, GridLayer layer,
      std::int64_t now_stamp_ns = 0) const noexcept {
    static_cast<void>(now_stamp_ns);
    const auto state = classify(point, layer);
    if (state == CellState::kOccupied) {
      return HandoverClearanceReason::kOccupiedContradiction;
    }
    return state == CellState::kKnownFree
               ? HandoverClearanceReason::kNone
               : HandoverClearanceReason::kNoSensorEvidence;
  }
  [[nodiscard]] bool isSensorKnownFree(
      const Point3& point, GridLayer layer) const noexcept {
    return classifyFreeSpace(point, layer) == FreeSpaceEvidence::kSensorFree;
  }
  [[nodiscard]] bool isCurrentBodySupported(
      const Point3& point, GridLayer layer,
      std::int64_t now_stamp_ns = 0) const noexcept {
    return classifyFreeSpace(point, layer, now_stamp_ns) ==
           FreeSpaceEvidence::kCurrentBodySupport;
  }
  [[nodiscard]] bool isOccupiedOrInflated(
      const Point3& point, GridLayer layer) const noexcept {
    return classifyFreeSpace(point, layer) == FreeSpaceEvidence::kOccupied;
  }
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
  // Explicit handover oracle. Mapping snapshots keep this sensor-only; a
  // planner may apply a separately validated current-body witness solely to
  // the measured start prefix. BACKUP/future validation remains sensor-only.
  [[nodiscard]] virtual bool isSegmentTraversableWithCurrentBodySupport(
      const Point3& start, const Point3& end, GridLayer layer,
      UnknownPolicy unknown_policy) const noexcept {
    return isSegmentTraversable(start, end, layer, unknown_policy);
  }
  [[nodiscard]] virtual AxisAlignedBox clampToLocalBounds(
      const AxisAlignedBox& requested) const noexcept = 0;
  [[nodiscard]] virtual PointVector observedOccupiedPoints(
      const AxisAlignedBox& box) const = 0;
};

using WorldModelViewPtr = std::shared_ptr<const WorldModelView>;

}  // namespace navigation_world_model

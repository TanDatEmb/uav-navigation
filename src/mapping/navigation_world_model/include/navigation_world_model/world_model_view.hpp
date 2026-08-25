#pragma once

#include <cstdint>
#include <memory>
#include <optional>
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

struct AxisAlignedBox {
  Point3 minimum{Point3::Zero()};
  Point3 maximum{Point3::Zero()};
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
};

struct WorldSnapshotIdentity {
  std::uint64_t generation{0};
  std::uint64_t revision{0};
  std::int64_t observation_stamp_ns{0};
};

// Read-only planning contract. Implementations must preserve their native cell
// centers, ray traversal, nearest-cell tie breaking, and occupied-point order.
// A view used by one solve must keep one identity for the whole solve.
class WorldModelView {
 public:
  virtual ~WorldModelView() = default;

  [[nodiscard]] virtual WorldGeometry geometry() const noexcept = 0;
  [[nodiscard]] virtual WorldSnapshotIdentity identity() const noexcept = 0;
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

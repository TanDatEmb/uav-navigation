#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace navigation_mapping {

class RogMapAdapter;

using Vec3 = Eigen::Vector3d;

enum class WorldLayer {
  Probability,
  Inflated,
};

enum class CellState {
  Unknown,
  KnownFree,
  Occupied,
};

// Probability reports the fine ROG occupancy state. Inflated first applies
// inflated occupied/unknown counters, then reports the underlying CounterMap
// aggregate for the represented coarse cell; it never samples one probability
// voxel at the coarse-cell center. Outside either valid map domain is Unknown.

struct GridIndex3 {
  int x{0};
  int y{0};
  int z{0};

  friend bool operator==(const GridIndex3&, const GridIndex3&) = default;
};

struct GridBounds {
  GridIndex3 min;
  GridIndex3 max;

  [[nodiscard]] bool contains(const GridIndex3& index) const noexcept {
    return index.x >= min.x && index.x <= max.x && index.y >= min.y && index.y <= max.y &&
           index.z >= min.z && index.z <= max.z;
  }
};

// Product-owned query facade. Consumers depend on this contract rather than
// on ROGMap/ProbMap/InfMap types or their upstream coordinate APIs.
class WorldModel final {
 public:
  explicit WorldModel(const RogMapAdapter& adapter);

  [[nodiscard]] bool isReady() const noexcept;
  [[nodiscard]] CellState cellState(WorldLayer layer, const GridIndex3& index) const;
  [[nodiscard]] GridIndex3 worldToGrid(WorldLayer layer, const Vec3& position) const;
  [[nodiscard]] Vec3 gridToWorld(WorldLayer layer, const GridIndex3& index) const;
  [[nodiscard]] double resolution(WorldLayer layer) const;
  [[nodiscard]] GridBounds bounds(WorldLayer layer) const;
  [[nodiscard]] std::uint64_t generation() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] double clearanceRadius() const noexcept;

 private:
  const RogMapAdapter& adapter_;
};

}  // namespace navigation_mapping

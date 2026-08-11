#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace uav::nav::lio {

struct NeighborSet {
  static constexpr std::size_t kCapacity = 5U;

  std::array<Eigen::Vector3d, kCapacity> points{};
  std::array<float, kCapacity> squared_distances{};
  std::size_t count{0};

  [[nodiscard]] bool complete(
      std::size_t requested_count = kCapacity) const noexcept {
    return requested_count <= kCapacity && count >= requested_count;
  }
};

class RegistrationMap {
 public:
  virtual ~RegistrationMap() = default;

  // Fixed-size, non-owning hot-path query. Implementations must not allocate.
  [[nodiscard]] virtual bool nearestSearch(
      const Eigen::Vector3d& query_odom_m, double maximum_distance_m,
      NeighborSet& output) const = 0;

  // Input points are already corrected and expressed in odom. The return value
  // is the number of newly represented map points after downsampling.
  virtual std::size_t insert(std::span<const Eigen::Vector3d> points_odom_m) = 0;

  // Keeps the axis-aligned local cube centered in odom and returns the number
  // of represented points removed.
  virtual std::size_t cropLocal(const Eigen::Vector3d& center_odom_m,
                                const Eigen::Vector3d& half_extent_m) = 0;
  [[nodiscard]] virtual std::vector<Eigen::Vector3d> snapshot() const = 0;
  // May throw when a backend cannot obtain a stable count before its bounded
  // rebuild-state timeout. A busy backend must never be reported as size zero.
  [[nodiscard]] virtual std::size_t size() const = 0;
  virtual void clear() = 0;

  [[nodiscard]] constexpr std::string_view frameId() const noexcept { return "lio_odom"; }
};

}  // namespace uav::nav::lio

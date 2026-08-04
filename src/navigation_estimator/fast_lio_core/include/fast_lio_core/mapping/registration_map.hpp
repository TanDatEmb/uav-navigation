#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace uav::nav::lio {

struct NearestNeighborResult {
  std::vector<Eigen::Vector3d> points_odom_m;
  std::vector<double> squared_distances_m2;

  [[nodiscard]] bool complete(std::size_t requested_count) const noexcept {
    return points_odom_m.size() == requested_count &&
           squared_distances_m2.size() == requested_count;
  }
};

class RegistrationMap {
 public:
  virtual ~RegistrationMap() = default;

  [[nodiscard]] virtual NearestNeighborResult nearestNeighbors(const Eigen::Vector3d& query_odom_m,
                                                               std::size_t neighbor_count,
                                                               double maximum_distance_m) const = 0;

  // Input points are already corrected and expressed in odom. The return value
  // is the number of newly represented map points after downsampling.
  virtual std::size_t insert(std::span<const Eigen::Vector3d> points_odom_m) = 0;

  // Keeps the axis-aligned local cube centered in odom and returns the number
  // of represented points removed.
  virtual std::size_t cropLocal(const Eigen::Vector3d& center_odom_m,
                                const Eigen::Vector3d& half_extent_m) = 0;
  // Threshold-triggered exact fallback. Implementations should use a partial
  // selection and an atomic replacement/rebuild rather than a fragile batch
  // of per-point deletions.
  virtual std::size_t pruneFarthest(
      const Eigen::Vector3d& center_odom_m,
      std::size_t target_point_count,
      double distance_shell_size_m) = 0;

  [[nodiscard]] virtual std::vector<Eigen::Vector3d> snapshot() const = 0;
  // May throw when a backend cannot obtain a stable count before its bounded
  // rebuild-state timeout. A busy backend must never be reported as size zero.
  [[nodiscard]] virtual std::size_t size() const = 0;
  virtual void clear() = 0;

  [[nodiscard]] constexpr std::string_view frameId() const noexcept { return "lio_odom"; }
};

}  // namespace uav::nav::lio

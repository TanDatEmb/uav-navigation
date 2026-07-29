#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include "fast_lio_core/mapping/registration_map.hpp"

namespace uav::nav::lio {

enum class RegistrationMapBackend {
  kUpstreamIkdTree,
};

struct IkdTreeRegistrationMapConfig {
  double voxel_size_m{0.2};
  double deletion_rebuild_ratio{0.5};
  double balance_rebuild_ratio{0.6};
};

// Production wrapper around the pinned hku-mars ikd-Tree KD_TREE.
//
// The upstream tree owns a very large inline operation queue, so the concrete
// KD_TREE is intentionally hidden behind a heap-allocated PIMPL. The wrapper
// mutex is the sole project-side concurrency boundary around every upstream
// call, including queries, incremental insertion, deletion, snapshots and
// destruction/reset.
class IkdTreeRegistrationMap final : public RegistrationMap {
 public:
  explicit IkdTreeRegistrationMap(
      IkdTreeRegistrationMapConfig config = {});
  ~IkdTreeRegistrationMap() override;

  IkdTreeRegistrationMap(const IkdTreeRegistrationMap&) = delete;
  IkdTreeRegistrationMap& operator=(
      const IkdTreeRegistrationMap&) = delete;
  IkdTreeRegistrationMap(IkdTreeRegistrationMap&&) = delete;
  IkdTreeRegistrationMap& operator=(
      IkdTreeRegistrationMap&&) = delete;

  [[nodiscard]] NearestNeighborResult nearestNeighbors(
      const Eigen::Vector3d& query_odom_m,
      std::size_t neighbor_count,
      double maximum_distance_m) const override;

  std::size_t insert(
      std::span<const Eigen::Vector3d> points_odom_m) override;
  std::size_t cropLocal(
      const Eigen::Vector3d& center_odom_m,
      const Eigen::Vector3d& half_extent_m) override;
  std::size_t pruneFarthest(
      const Eigen::Vector3d& center_odom_m,
      std::size_t target_point_count,
      double distance_shell_size_m) override;
  [[nodiscard]] std::vector<Eigen::Vector3d> snapshot() const override;
  [[nodiscard]] std::size_t size() const noexcept override;
  void clear() override;

  [[nodiscard]] constexpr RegistrationMapBackend backend() const noexcept {
    return RegistrationMapBackend::kUpstreamIkdTree;
  }

 private:
  class Impl;

  IkdTreeRegistrationMapConfig config_;
  mutable std::mutex mutex_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uav::nav::lio

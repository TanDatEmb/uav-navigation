#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include "fast_lio_core/mapping/registration_map.hpp"

namespace uav::nav::lio {

enum class RegistrationMapBackend {
  kUpstreamIkdTree,
};

struct IkdTreeRegistrationMapConfig {
  double voxel_size_m{0.2};
  double deletion_rebuild_ratio{0.5};
  double balance_rebuild_ratio{0.6};
  // Product uses the vendor's background rebuild. Deterministic tests can
  // explicitly set this false.
  bool enable_asynchronous_rebuild{true};
};

struct IkdTreeStageTimingTotals {
  std::int64_t prepare_us{0};
  std::int64_t sort_us{0};
  std::int64_t add_points_us{0};
  std::int64_t crop_us{0};
  std::int64_t bookkeeping_us{0};
};

// Production wrapper around the pinned registration-mapping KD-tree.
//
// The upstream tree owns a very large inline operation queue, so the concrete
// KD_TREE is intentionally hidden behind a heap-allocated PIMPL. This wrapper
// is single-owner: fast_lio_main is the only project-side caller. Vendor
// rebuild synchronization remains an implementation detail.
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

  [[nodiscard]] bool nearestSearch(
      const Eigen::Vector3d& query_odom_m, double maximum_distance_m,
      NeighborSet& output) const override;

  std::size_t insert(
      std::span<const Eigen::Vector3d> points_odom_m) override;
  std::size_t cropLocal(
      const Eigen::Vector3d& center_odom_m,
      const Eigen::Vector3d& half_extent_m) override;
  [[nodiscard]] std::size_t size() const override;
  [[nodiscard]] std::size_t validPointCountBusyCount() const noexcept;
  [[nodiscard]] IkdTreeStageTimingTotals stageTimingTotals() const noexcept;
  void clear() override;

  [[nodiscard]] constexpr RegistrationMapBackend backend() const noexcept {
    return RegistrationMapBackend::kUpstreamIkdTree;
  }

 private:
  class Impl;

  IkdTreeRegistrationMapConfig config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uav::nav::lio

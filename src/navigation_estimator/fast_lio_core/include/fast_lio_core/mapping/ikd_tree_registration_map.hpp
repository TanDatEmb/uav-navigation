#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <unordered_map>
#include <vector>

#include "fast_lio_core/mapping/registration_map.hpp"

namespace uav::nav::lio {

enum class RegistrationMapBackend {
  // Upstream ikd-tree is not present in this repository. This backend is a
  // deterministic voxel map with exhaustive nearest-neighbor queries. The
  // class keeps the required API name so an upstream ikd-tree backend can
  // replace the implementation without changing consumers; it does not claim
  // ikd-tree performance.
  kDeterministicVoxelBruteForce,
};

struct IkdTreeRegistrationMapConfig {
  double voxel_size_m{0.2};
};

class IkdTreeRegistrationMap final : public RegistrationMap {
 public:
  explicit IkdTreeRegistrationMap(IkdTreeRegistrationMapConfig config = {});

  [[nodiscard]] NearestNeighborResult nearestNeighbors(const Eigen::Vector3d& query_odom_m,
                                                       std::size_t neighbor_count,
                                                       double maximum_distance_m) const override;

  std::size_t insert(std::span<const Eigen::Vector3d> points_odom_m) override;
  std::size_t cropLocal(const Eigen::Vector3d& center_odom_m,
                        const Eigen::Vector3d& half_extent_m) override;
  [[nodiscard]] std::vector<Eigen::Vector3d> snapshot() const override;
  [[nodiscard]] std::size_t size() const noexcept override;
  void clear() override;

  [[nodiscard]] constexpr RegistrationMapBackend backend() const noexcept {
    return RegistrationMapBackend::kDeterministicVoxelBruteForce;
  }

 private:
  struct VoxelIndex {
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};

    bool operator==(const VoxelIndex&) const = default;
  };

  struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& index) const noexcept;
  };

  struct VoxelAccumulator {
    Eigen::Vector3d sum_odom_m{Eigen::Vector3d::Zero()};
    std::uint64_t sample_count{0};

    [[nodiscard]] Eigen::Vector3d centroid() const noexcept;
  };

  [[nodiscard]] VoxelIndex voxelIndex(const Eigen::Vector3d& point_odom_m) const noexcept;

  IkdTreeRegistrationMapConfig config_;
  mutable std::mutex mutex_;
  std::unordered_map<VoxelIndex, VoxelAccumulator, VoxelIndexHash> voxels_;
};

}  // namespace uav::nav::lio

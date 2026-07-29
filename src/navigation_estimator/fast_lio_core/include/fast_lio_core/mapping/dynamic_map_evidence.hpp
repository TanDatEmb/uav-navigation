#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>

namespace uav::nav::lio {

struct DynamicFilterConfig {
  bool enabled{false};
  double voxel_size_m{1.0};
  std::uint16_t minimum_hit_count{2};
  std::uint16_t minimum_contradiction_count{3};
  std::uint64_t minimum_age_scans{2};
  float insert_threshold{2.0F};
  float keep_threshold{0.0F};
  float delete_threshold{-0.5F};
};

struct MapVoxelEvidence {
  std::uint16_t hit_count{0};
  std::uint16_t contradiction_count{0};
  std::uint64_t last_seen_scan{0};
  float confidence_log_odds{0.0F};
  bool registration_support{false};
};

struct FreeSpaceObservation {
  bool in_fov{false};
  bool in_valid_range{false};
  bool pose_and_deskew_valid{false};
  bool ray_passes_voxel{false};
  bool occluded_by_nearer_return{false};
  double map_range_m{0.0};
  double current_range_m{0.0};
  double farther_margin_m{0.0};
};

class DynamicMapEvidence {
 public:
  explicit DynamicMapEvidence(DynamicFilterConfig config = {});

  void observeHits(std::span<const Eigen::Vector3d> points_odom_m,
                   std::uint64_t scan_index);
  bool observeContradiction(const Eigen::Vector3d& point_odom_m,
                            const FreeSpaceObservation& observation,
                            std::uint64_t scan_index);
  [[nodiscard]] std::size_t candidateCount(
      std::uint64_t scan_index) const;
  [[nodiscard]] std::size_t voxelCount() const noexcept;
  void clear();

 private:
  struct VoxelKey {
    std::int64_t x{};
    std::int64_t y{};
    std::int64_t z{};
    bool operator==(const VoxelKey&) const noexcept = default;
  };
  struct VoxelHash {
    std::size_t operator()(const VoxelKey& key) const noexcept;
  };
  [[nodiscard]] VoxelKey key(const Eigen::Vector3d& point) const;

  DynamicFilterConfig config_;
  std::unordered_map<VoxelKey, MapVoxelEvidence, VoxelHash> evidence_;
};

}  // namespace uav::nav::lio

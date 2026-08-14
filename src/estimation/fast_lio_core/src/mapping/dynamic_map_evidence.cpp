#include "fast_lio_core/mapping/dynamic_map_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav::nav::lio {

DynamicMapEvidence::DynamicMapEvidence(DynamicFilterConfig config)
    : config_(config) {
  if (!(config_.voxel_size_m > 0.0) ||
      !std::isfinite(config_.voxel_size_m) ||
      config_.minimum_hit_count == 0U ||
      config_.minimum_contradiction_count == 0U ||
      !(config_.insert_threshold > config_.keep_threshold &&
        config_.keep_threshold > config_.delete_threshold)) {
    throw std::invalid_argument("invalid dynamic-filter evidence configuration");
  }
}

std::size_t DynamicMapEvidence::VoxelHash::operator()(
    const VoxelKey& key) const noexcept {
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
          (seed >> 2U);
  seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
          (seed >> 2U);
  return seed;
}

DynamicMapEvidence::VoxelKey DynamicMapEvidence::key(
    const Eigen::Vector3d& point) const {
  return {static_cast<std::int64_t>(std::floor(point.x() / config_.voxel_size_m)),
          static_cast<std::int64_t>(std::floor(point.y() / config_.voxel_size_m)),
          static_cast<std::int64_t>(std::floor(point.z() / config_.voxel_size_m))};
}

void DynamicMapEvidence::observeHits(
    std::span<const Eigen::Vector3d> points_odom_m,
    std::uint64_t scan_index) {
  if (!config_.enabled) {
    return;
  }
  for (const auto& point : points_odom_m) {
    if (!point.allFinite()) {
      continue;
    }
    auto& value = evidence_[key(point)];
    value.hit_count = static_cast<std::uint16_t>(
        std::min<unsigned int>(std::numeric_limits<std::uint16_t>::max(),
                               value.hit_count + 1U));
    value.last_seen_scan = scan_index;
    value.confidence_log_odds =
        std::min(config_.insert_threshold, value.confidence_log_odds + 1.0F);
  }
}

bool DynamicMapEvidence::observeContradiction(
    const Eigen::Vector3d& point_odom_m,
    const FreeSpaceObservation& observation,
    std::uint64_t scan_index) {
  if (!config_.enabled || !point_odom_m.allFinite() ||
      !observation.in_fov || !observation.in_valid_range ||
      !observation.pose_and_deskew_valid || !observation.ray_passes_voxel ||
      observation.occluded_by_nearer_return ||
      !(observation.current_range_m >
        observation.map_range_m + observation.farther_margin_m)) {
    return false;
  }
  auto found = evidence_.find(key(point_odom_m));
  if (found == evidence_.end()) {
    return false;
  }
  auto& value = found->second;
  value.contradiction_count = static_cast<std::uint16_t>(
      std::min<unsigned int>(std::numeric_limits<std::uint16_t>::max(),
                             value.contradiction_count + 1U));
  value.confidence_log_odds -= 1.0F;
  return value.hit_count >= config_.minimum_hit_count &&
         value.contradiction_count >= config_.minimum_contradiction_count &&
         scan_index >= value.last_seen_scan + config_.minimum_age_scans &&
         value.confidence_log_odds < config_.delete_threshold &&
         !value.registration_support;
}

std::size_t DynamicMapEvidence::candidateCount(
    std::uint64_t scan_index) const {
  return static_cast<std::size_t>(std::count_if(
      evidence_.begin(), evidence_.end(), [&](const auto& item) {
        const auto& value = item.second;
        return value.hit_count >= config_.minimum_hit_count &&
               value.contradiction_count >=
                   config_.minimum_contradiction_count &&
               scan_index >=
                   value.last_seen_scan + config_.minimum_age_scans &&
               value.confidence_log_odds < config_.delete_threshold &&
               !value.registration_support;
      }));
}

std::size_t DynamicMapEvidence::voxelCount() const noexcept {
  return evidence_.size();
}

void DynamicMapEvidence::clear() { evidence_.clear(); }

}  // namespace uav::nav::lio

#include "fast_lio_core/mapping/local_map_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {

LocalMapManager::LocalMapManager(LocalMapManagerConfig config) : config_(config) {
  if (!config_.half_extent_m.allFinite() || (config_.half_extent_m.array() <= 0.0).any() ||
      !(config_.crop_trigger_distance_m > 0.0) || !std::isfinite(config_.crop_trigger_distance_m) ||
      config_.target_point_count_after_prune == 0U ||
      config_.target_point_count_after_prune >= config_.soft_point_limit ||
      config_.soft_point_limit >= config_.hard_point_limit ||
      !(config_.distance_shell_size_m > 0.0) ||
      !std::isfinite(config_.distance_shell_size_m)) {
    throw std::invalid_argument("invalid local map manager configuration");
  }
}

LocalMapUpdate LocalMapManager::update(RegistrationMap& map,
                                       const Eigen::Vector3d& current_position_odom_m) {
  LocalMapUpdate update;
  update.center_odom_m = current_position_odom_m;
  update.half_extent_m = config_.half_extent_m;
  if (!current_position_odom_m.allFinite()) {
    return update;
  }

  const bool moved_beyond_trigger =
      !has_crop_center_ || (current_position_odom_m - last_crop_center_odom_m_).norm() >=
                               config_.crop_trigger_distance_m;
  const bool map_over_limit = map.size() > config_.soft_point_limit;
  if (!moved_beyond_trigger && !map_over_limit) {
    return update;
  }

  update.crop_performed = true;
  update.crop_triggered_by_motion = moved_beyond_trigger;
  update.crop_triggered_by_point_threshold = map_over_limit;
  update.removed_point_count = map.cropLocal(current_position_odom_m, config_.half_extent_m);
  if (map.size() > config_.soft_point_limit) {
    update.distance_pruned_count = map.pruneFarthest(
        current_position_odom_m, config_.target_point_count_after_prune,
        config_.distance_shell_size_m);
    update.removed_point_count += update.distance_pruned_count;
  }
  last_crop_center_odom_m_ = current_position_odom_m;
  has_crop_center_ = true;
  return update;
}

void LocalMapManager::reset() noexcept {
  last_crop_center_odom_m_.setZero();
  has_crop_center_ = false;
}

}  // namespace uav::nav::lio

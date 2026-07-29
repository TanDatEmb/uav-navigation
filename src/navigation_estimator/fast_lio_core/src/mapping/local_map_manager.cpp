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

  update.map_count_before = map.size();
  update.soft_limit_triggered =
      update.map_count_before > config_.soft_point_limit;
  update.hard_limit_triggered =
      update.map_count_before > config_.hard_point_limit;
  if (update.map_count_before <= config_.target_point_count_after_prune) {
    soft_maintenance_armed_ = true;
  }
  const bool moved_beyond_trigger =
      !has_crop_center_ || (current_position_odom_m - last_crop_center_odom_m_).norm() >=
                               config_.crop_trigger_distance_m;
  const bool soft_maintenance_due =
      update.soft_limit_triggered && soft_maintenance_armed_;
  if (!moved_beyond_trigger && !soft_maintenance_due &&
      !update.hard_limit_triggered) {
    update.map_count_after_crop = update.map_count_before;
    update.map_count_after_prune = update.map_count_before;
    return update;
  }

  update.crop_performed = true;
  update.crop_triggered_by_motion = moved_beyond_trigger;
  update.crop_triggered_by_point_threshold =
      soft_maintenance_due || update.hard_limit_triggered;
  update.removed_point_count = map.cropLocal(current_position_odom_m, config_.half_extent_m);
  update.map_count_after_crop = map.size();
  if (update.map_count_after_crop > config_.hard_point_limit) {
    update.hard_limit_triggered = true;
    update.distance_pruned_count = map.pruneFarthest(
        current_position_odom_m, config_.target_point_count_after_prune,
        config_.distance_shell_size_m);
    update.removed_point_count += update.distance_pruned_count;
  }
  update.map_count_after_prune = map.size();
  update.hard_limit_recovery_failed =
      update.hard_limit_triggered &&
      update.map_count_after_prune > config_.target_point_count_after_prune;
  if (update.hard_limit_recovery_failed) {
    throw std::runtime_error(
        "local map hard-limit recovery did not reach target point count");
  }
  if (soft_maintenance_due) {
    soft_maintenance_armed_ = false;
  }
  last_crop_center_odom_m_ = current_position_odom_m;
  has_crop_center_ = true;
  return update;
}

void LocalMapManager::reset() noexcept {
  last_crop_center_odom_m_.setZero();
  has_crop_center_ = false;
  soft_maintenance_armed_ = true;
}

}  // namespace uav::nav::lio

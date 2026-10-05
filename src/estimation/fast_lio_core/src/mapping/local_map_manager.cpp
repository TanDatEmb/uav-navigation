#include "fast_lio_core/mapping/local_map_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {

LocalMapManager::LocalMapManager(LocalMapManagerConfig config) : config_(config) {
  if (!config_.half_extent_m.allFinite() || (config_.half_extent_m.array() <= 0.0).any() ||
      !(config_.crop_trigger_distance_m > 0.0) || !std::isfinite(config_.crop_trigger_distance_m) ||
      config_.absolute_map_point_guard == 0U) {
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
  const bool moved_beyond_trigger =
      !has_crop_center_ || (current_position_odom_m - last_crop_center_odom_m_).norm() >=
                               config_.crop_trigger_distance_m;
  update.absolute_guard_triggered =
      update.map_count_before > config_.absolute_map_point_guard;
  if (!moved_beyond_trigger && !update.absolute_guard_triggered) {
    update.map_count_after_crop = update.map_count_before;
    return update;
  }

  update.crop_performed = true;
  update.crop_triggered_by_motion = moved_beyond_trigger;
  update.removed_point_count = map.cropLocal(current_position_odom_m, config_.half_extent_m);
  update.map_count_after_crop = map.size();
  if (update.map_count_after_crop > config_.absolute_map_point_guard) {
    update.absolute_guard_recovery_failed = true;
    insertion_frozen_ = true;
  }
  update.insertion_frozen = insertion_frozen_;
  last_crop_center_odom_m_ = current_position_odom_m;
  has_crop_center_ = true;
  return update;
}

void LocalMapManager::reset() noexcept {
  last_crop_center_odom_m_.setZero();
  has_crop_center_ = false;
  insertion_frozen_ = false;
}

}  // namespace uav::nav::lio

#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "fast_lio_core/mapping/registration_map.hpp"

namespace uav::nav::lio {

struct LocalMapManagerConfig {
  Eigen::Vector3d half_extent_m{Eigen::Vector3d(50.0, 50.0, 25.0)};
  double crop_trigger_distance_m{5.0};
  std::size_t soft_point_limit{80000};
  std::size_t hard_point_limit{100000};
  std::size_t target_point_count_after_prune{70000};
  double distance_shell_size_m{5.0};
};

struct LocalMapUpdate {
  bool crop_performed{false};
  bool soft_limit_triggered{false};
  bool hard_limit_triggered{false};
  bool hard_limit_recovery_failed{false};
  std::size_t map_count_before{0};
  std::size_t map_count_after_crop{0};
  std::size_t map_count_after_prune{0};
  std::size_t removed_point_count{0};
  std::size_t distance_pruned_count{0};
  bool crop_triggered_by_motion{false};
  bool crop_triggered_by_point_threshold{false};
  Eigen::Vector3d center_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d half_extent_m{Eigen::Vector3d::Zero()};
};

class LocalMapManager {
 public:
  explicit LocalMapManager(LocalMapManagerConfig config = {});

  [[nodiscard]] LocalMapUpdate update(RegistrationMap& map,
                                      const Eigen::Vector3d& current_position_odom_m);
  void reset() noexcept;

 private:
  LocalMapManagerConfig config_;
  Eigen::Vector3d last_crop_center_odom_m_{Eigen::Vector3d::Zero()};
  bool has_crop_center_{false};
  bool soft_maintenance_armed_{true};
};

}  // namespace uav::nav::lio

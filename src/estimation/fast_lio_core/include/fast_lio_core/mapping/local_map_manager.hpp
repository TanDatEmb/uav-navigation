#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "fast_lio_core/mapping/registration_map.hpp"

namespace uav::nav::lio {

struct LocalMapManagerConfig {
  Eigen::Vector3d half_extent_m{Eigen::Vector3d(50.0, 50.0, 25.0)};
  double crop_trigger_distance_m{5.0};
  // Emergency-only guard for a broken geometry/crop invariant. It is not a
  // steady-state map budget and never triggers a tree rebuild.
  std::size_t absolute_map_point_guard{250000};
};

struct LocalMapUpdate {
  bool crop_performed{false};
  bool absolute_guard_triggered{false};
  bool absolute_guard_recovery_failed{false};
  bool insertion_frozen{false};
  std::size_t map_count_before{0};
  std::size_t map_count_after_crop{0};
  std::size_t removed_point_count{0};
  bool crop_triggered_by_motion{false};
  Eigen::Vector3d center_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d half_extent_m{Eigen::Vector3d::Zero()};
};

class LocalMapManager {
 public:
  explicit LocalMapManager(LocalMapManagerConfig config = {});

  [[nodiscard]] LocalMapUpdate update(RegistrationMap& map,
                                      const Eigen::Vector3d& current_position_odom_m);
  [[nodiscard]] bool insertionAllowed() const noexcept {
    return !insertion_frozen_;
  }
  void reset() noexcept;

 private:
  LocalMapManagerConfig config_;
  Eigen::Vector3d last_crop_center_odom_m_{Eigen::Vector3d::Zero()};
  bool has_crop_center_{false};
  bool insertion_frozen_{false};
};

}  // namespace uav::nav::lio

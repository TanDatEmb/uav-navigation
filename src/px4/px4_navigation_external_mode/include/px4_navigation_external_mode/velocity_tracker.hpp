#pragma once

#include <Eigen/Core>

#include "px4_navigation_external_mode/trajectory_contract.hpp"

namespace px4_navigation_external_mode {

struct VelocityTrackerConfig {
  double position_gain{1.0};
  double max_velocity_mps{2.0};
  double max_acceleration_mps2{3.0};
  double max_deceleration_mps2{3.0};
  double max_position_error_m{2.0};
};

class VelocityTracker final {
 public:
  explicit VelocityTracker(VelocityTrackerConfig config = {});

  void reset() noexcept;

  [[nodiscard]] Eigen::Vector3d update(const TrajectorySample& reference,
                                       const Eigen::Vector3d& position_enu,
                                       double dt_s);

  // Position feedback is evaluated at the current-time reference.  A short
  // preview contributes only its velocity direction, avoiding a phase lead
  // when replanning replaces the trajectory at the same request id.
  [[nodiscard]] Eigen::Vector3d update(const TrajectorySample& current_reference,
                                       const TrajectorySample& preview_reference,
                                       const Eigen::Vector3d& position_enu,
                                       double dt_s);

 private:
  VelocityTrackerConfig config_;
  Eigen::Vector3d previous_command_enu_{Eigen::Vector3d::Zero()};
};

}  // namespace px4_navigation_external_mode

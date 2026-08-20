#pragma once

#include <cstdint>
#include <limits>

#include <Eigen/Core>

#include "px4_navigation_external_mode/trajectory_contract.hpp"

namespace px4_navigation_external_mode {

struct VelocityTrackerConfig {
  double position_gain{1.0};
  double max_velocity_mps{2.0};
  double max_acceleration_mps2{3.0};
  double max_deceleration_mps2{3.0};
  // Infinity keeps source compatibility for direct unit-test users; the
  // SITL profile sets a finite jerk limit and therefore enables the actual
  // acceleration-state limiter.
  double max_jerk_mps3{std::numeric_limits<double>::infinity()};
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

  [[nodiscard]] const Eigen::Vector3d& lastCommand() const noexcept {
    return previous_command_enu_;
  }
  [[nodiscard]] std::uint64_t forwardGuardCount() const noexcept {
    return forward_guard_count_;
  }

 private:
  VelocityTrackerConfig config_;
  Eigen::Vector3d previous_command_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_acceleration_enu_{Eigen::Vector3d::Zero()};
  std::uint64_t forward_guard_count_{0U};
};

}  // namespace px4_navigation_external_mode

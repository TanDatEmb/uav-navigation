#include "px4_navigation_external_mode/velocity_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_navigation_external_mode {
namespace {

double clampNorm(Eigen::Vector3d& value, double maximum) {
  const double norm = value.norm();
  if (norm > maximum && norm > 0.0) {
    value *= maximum / norm;
  }
  return norm;
}

}  // namespace

VelocityTracker::VelocityTracker(VelocityTrackerConfig config) : config_(config) {
  if (!std::isfinite(config_.position_gain) || config_.position_gain < 0.0 ||
      !std::isfinite(config_.max_velocity_mps) || config_.max_velocity_mps <= 0.0 ||
      !std::isfinite(config_.max_acceleration_mps2) || config_.max_acceleration_mps2 <= 0.0 ||
      !std::isfinite(config_.max_deceleration_mps2) || config_.max_deceleration_mps2 <= 0.0 ||
      std::isnan(config_.max_jerk_mps3) || config_.max_jerk_mps3 <= 0.0 ||
      !std::isfinite(config_.max_position_error_m) || config_.max_position_error_m <= 0.0) {
    throw std::invalid_argument("invalid velocity tracker configuration");
  }
}

void VelocityTracker::reset() noexcept {
  previous_command_enu_.setZero();
  previous_acceleration_enu_.setZero();
}

Eigen::Vector3d VelocityTracker::update(const TrajectorySample& reference,
                                        const Eigen::Vector3d& position_enu,
                                        double dt_s) {
  return update(reference, reference, position_enu, dt_s);
}

Eigen::Vector3d VelocityTracker::update(const TrajectorySample& current_reference,
                                        const TrajectorySample& preview_reference,
                                        const Eigen::Vector3d& position_enu,
                                        double dt_s) {
  if (!position_enu.allFinite() || !current_reference.position_enu.allFinite() ||
      !preview_reference.velocity_enu.allFinite() ||
      !preview_reference.acceleration_enu.allFinite()) {
    throw std::invalid_argument("velocity tracker received non-finite state");
  }

  const double dt = std::clamp(dt_s, 1e-3, 0.25);
  Eigen::Vector3d position_error = current_reference.position_enu - position_enu;
  clampNorm(position_error, config_.max_position_error_m);
  Eigen::Vector3d command = preview_reference.velocity_enu + config_.position_gain * position_error;
  clampNorm(command, config_.max_velocity_mps);

  // On a straight corridor, position feedback must not turn a forward
  // trajectory into a reverse command when a rolling replan changes its
  // sampled position slightly.  A genuine corner has a changing preview
  // tangent and is deliberately left unrestricted so the vehicle can rotate
  // smoothly.  Braking stops also have zero preview velocity and therefore
  // retain their normal deceleration behaviour.
  if (current_reference.velocity_enu.allFinite() &&
      preview_reference.velocity_enu.allFinite() &&
      current_reference.velocity_enu.norm() > 0.1 &&
      preview_reference.velocity_enu.norm() > 0.1) {
    const auto current_direction = current_reference.velocity_enu.normalized();
    const auto preview_direction = preview_reference.velocity_enu.normalized();
    if (current_direction.dot(preview_direction) > 0.5) {
      const double longitudinal_command = command.dot(current_direction);
      if (longitudinal_command < 0.0) {
        command -= longitudinal_command * current_direction;
        ++forward_guard_count_;
      }
    }
  }

  Eigen::Vector3d desired_acceleration = (command - previous_command_enu_) / dt;
  const double previous_speed = previous_command_enu_.norm();
  const double longitudinal_delta = previous_speed > 1e-9
                                        ? command.dot(previous_command_enu_.normalized()) -
                                              previous_speed
                                        : command.norm();
  const double limit = longitudinal_delta < 0.0 ? config_.max_deceleration_mps2
                                                : config_.max_acceleration_mps2;
  clampNorm(desired_acceleration, limit);

  // The previous implementation limited only delta-v. Retain the
  // acceleration state explicitly so a velocity-only setpoint cannot make a
  // one-cycle acceleration jump when a replanned reference changes direction.
  if (std::isfinite(config_.max_jerk_mps3)) {
    Eigen::Vector3d acceleration_delta = desired_acceleration - previous_acceleration_enu_;
    clampNorm(acceleration_delta, config_.max_jerk_mps3 * dt);
    previous_acceleration_enu_ += acceleration_delta;
  } else {
    previous_acceleration_enu_ = desired_acceleration;
  }
  command = previous_command_enu_ + previous_acceleration_enu_ * dt;

  clampNorm(command, config_.max_velocity_mps);
  // Keep the stored acceleration consistent when the velocity cap clips the
  // command, otherwise the next cycle would reintroduce a stale acceleration.
  previous_acceleration_enu_ = (command - previous_command_enu_) / dt;
  previous_command_enu_ = command;
  return command;
}

}  // namespace px4_navigation_external_mode

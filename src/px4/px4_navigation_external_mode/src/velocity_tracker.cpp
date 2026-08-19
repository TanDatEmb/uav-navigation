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
      !std::isfinite(config_.max_position_error_m) || config_.max_position_error_m <= 0.0) {
    throw std::invalid_argument("invalid velocity tracker configuration");
  }
}

void VelocityTracker::reset() noexcept {
  previous_command_enu_.setZero();
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
      !preview_reference.velocity_enu.allFinite()) {
    throw std::invalid_argument("velocity tracker received non-finite state");
  }

  const double dt = std::clamp(dt_s, 1e-3, 0.25);
  Eigen::Vector3d position_error = current_reference.position_enu - position_enu;
  clampNorm(position_error, config_.max_position_error_m);
  Eigen::Vector3d command = preview_reference.velocity_enu + config_.position_gain * position_error;
  clampNorm(command, config_.max_velocity_mps);

  Eigen::Vector3d delta = command - previous_command_enu_;
  const double previous_speed = previous_command_enu_.norm();
  const double longitudinal_delta = previous_speed > 1e-9
                                        ? command.dot(previous_command_enu_.normalized()) -
                                              previous_speed
                                        : command.norm();
  const double limit = longitudinal_delta < 0.0 ? config_.max_deceleration_mps2
                                                : config_.max_acceleration_mps2;
  clampNorm(delta, limit * dt);
  command = previous_command_enu_ + delta;

  clampNorm(command, config_.max_velocity_mps);
  previous_command_enu_ = command;
  return command;
}

}  // namespace px4_navigation_external_mode

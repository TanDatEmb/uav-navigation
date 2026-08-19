#include "px4_navigation_external_mode/mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace px4_navigation_external_mode {

MissionController::MissionController(Mission mission) : mission_(std::move(mission)) {
  if (mission_.waypoints.empty()) {
    throw std::invalid_argument("mission controller requires at least one waypoint");
  }
}

void MissionController::activate(double now_s) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("mission activation time must be finite");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!checkpoint_valid_ || active_waypoint_index_ >= mission_.waypoints.size()) {
    active_waypoint_index_ = 0U;
    request_id_ = 0U;
    checkpoint_valid_ = false;
  }
  state_ = MissionControllerState::WaitingForAirborne;
  next_goal_time_s_ = now_s;
  hold_start_time_s_ = 0.0;
  braking_start_time_s_.reset();
  braking_end_time_s_.reset();
  stopped_start_time_s_.reset();
  arrival_start_time_s_.reset();
  pending_position_control_ = false;
  trajectory_ready_ = false;
}

void MissionController::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != MissionControllerState::Complete &&
      state_ != MissionControllerState::Failed) {
    checkpoint_valid_ = true;
  }
  state_ = MissionControllerState::Idle;
  braking_start_time_s_.reset();
  braking_end_time_s_.reset();
  stopped_start_time_s_.reset();
  arrival_start_time_s_.reset();
  pending_position_control_ = false;
  trajectory_ready_ = false;
}

void MissionController::onTrajectory(bool success, double now_s) {
  onTrajectory(success, 0U, 0U, now_s, 0.0);
}

void MissionController::onTrajectory(bool success, std::uint8_t trajectory_role,
                                     double now_s) {
  onTrajectory(success, trajectory_role,
               trajectory_role == kSafetyTrajectoryRole ? kSafetyStopKind : 0U, now_s, 0.0);
}

void MissionController::onTrajectory(bool success, std::uint8_t trajectory_role,
                                     std::uint8_t safety_plan_kind, double now_s,
                                     double duration_s) {
  if (!std::isfinite(now_s)) return;
  std::lock_guard<std::mutex> lock(mutex_);

  // A map revision may replace the currently executing braking stop. Keep the
  // mission in Braking and restart its confirmation window from the new
  // trajectory's own time origin; do not let the old duration cause an early
  // POSCTL handover. Conversely, if that replacement fails, fail closed into
  // Paused and request POSCTL rather than silently continuing with a cleared
  // setpoint.
  if (state_ == MissionControllerState::Braking) {
    if (success && trajectory_role == kSafetyTrajectoryRole &&
        safety_plan_kind == kSafetyStopKind) {
      braking_start_time_s_ = now_s;
      braking_end_time_s_ = now_s +
                            (std::isfinite(duration_s) ? std::max(0.0, duration_s) : 0.0);
      stopped_start_time_s_.reset();
      arrival_start_time_s_.reset();
      trajectory_ready_ = false;
      checkpoint_valid_ = true;
    } else if (!success) {
      state_ = MissionControllerState::Paused;
      checkpoint_valid_ = true;
      trajectory_ready_ = false;
      pending_position_control_ = true;
      braking_start_time_s_.reset();
      braking_end_time_s_.reset();
      stopped_start_time_s_.reset();
    }
    return;
  }

  if (state_ != MissionControllerState::ExecutingWaypoint) return;

  if (success && trajectory_role == kSafetyTrajectoryRole &&
      safety_plan_kind == kSafetyStopKind) {
    state_ = MissionControllerState::Braking;
    braking_start_time_s_ = now_s;
    braking_end_time_s_ = now_s + (std::isfinite(duration_s) ? std::max(0.0, duration_s) : 0.0);
    stopped_start_time_s_.reset();
    arrival_start_time_s_.reset();
    trajectory_ready_ = false;
    checkpoint_valid_ = true;
    return;
  }

  if (success) {
    braking_start_time_s_.reset();
    braking_end_time_s_.reset();
    stopped_start_time_s_.reset();
    trajectory_ready_ = true;
    return;
  }

  // A planner/verifier failure is an operator handover, not an instruction to
  // land. Preserve the active waypoint so a later mode activation can replan
  // from the current vehicle state.
  state_ = MissionControllerState::Paused;
  checkpoint_valid_ = true;
  trajectory_ready_ = false;
  pending_position_control_ = true;
}

MissionControllerEvent MissionController::update(
    double now_s, const std::optional<Eigen::Vector3d>& position, bool airborne,
    const std::optional<Eigen::Vector3d>& velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(now_s) || state_ == MissionControllerState::Idle ||
      state_ == MissionControllerState::Complete || state_ == MissionControllerState::Failed) {
    return {};
  }

  if (pending_position_control_) {
    pending_position_control_ = false;
    state_ = MissionControllerState::Paused;
    return {MissionControllerEvent::Type::RequestPositionControl, active_waypoint_index_,
            request_id_};
  }

  if (state_ == MissionControllerState::WaitingForAirborne) {
    if (!airborne) return {};
    state_ = MissionControllerState::ExecutingWaypoint;
  }

  if (state_ == MissionControllerState::Braking) {
    const bool stopped = velocity.has_value() && velocity->allFinite() &&
                         velocity->norm() <= kSafetyStopSpeedMps;
    const bool braking_complete = !braking_end_time_s_.has_value() ||
                                  now_s >= *braking_end_time_s_;
    if (stopped && braking_complete) {
      if (!stopped_start_time_s_.has_value()) stopped_start_time_s_ = now_s;
      if (now_s - *stopped_start_time_s_ >= kSafetyStopConfirmationS) {
        state_ = MissionControllerState::Paused;
        return {MissionControllerEvent::Type::RequestPositionControl,
                active_waypoint_index_, request_id_};
      }
    } else {
      stopped_start_time_s_.reset();
    }
    if (braking_start_time_s_.has_value() &&
        now_s - *braking_start_time_s_ >= kSafetyStopTimeoutS) {
      state_ = MissionControllerState::Paused;
      return {MissionControllerEvent::Type::RequestPositionControl,
              active_waypoint_index_, request_id_};
    }
    return {};
  }

  if (state_ == MissionControllerState::Paused) return {};

  const auto& waypoint = mission_.waypoints[active_waypoint_index_];
  const auto insideAcceptance = [&]() {
    return position.has_value() && position->allFinite() &&
           (*position - waypoint.position_enu).norm() <= waypoint.acceptance_radius_m;
  };
  const auto slowEnough = [&]() {
    // Waypoint completion is only valid with a measured, finite velocity
    // sample. Missing velocity must not turn a fly-through into an arrival.
    return velocity.has_value() && velocity->allFinite() &&
           velocity->norm() <= mission_.control.acceptance_speed_mps;
  };

  if (state_ == MissionControllerState::ExecutingWaypoint) {
    if (trajectory_ready_ && insideAcceptance() && slowEnough()) {
      if (!arrival_start_time_s_.has_value()) arrival_start_time_s_ = now_s;
      if (now_s - *arrival_start_time_s_ >= mission_.control.acceptance_confirmation_s) {
        state_ = MissionControllerState::Holding;
        hold_start_time_s_ = now_s;
        return {};
      }
    } else {
      arrival_start_time_s_.reset();
    }
    if (now_s >= next_goal_time_s_) {
      next_goal_time_s_ = std::numeric_limits<double>::infinity();
      trajectory_ready_ = false;
      ++request_id_;
      return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
    }
    return {};
  }

  if (state_ == MissionControllerState::Holding) {
    if (!insideAcceptance() || !slowEnough()) {
      state_ = MissionControllerState::ExecutingWaypoint;
      trajectory_ready_ = false;
      arrival_start_time_s_.reset();
      next_goal_time_s_ = std::numeric_limits<double>::infinity();
      ++request_id_;
      return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
    }
    if (now_s - hold_start_time_s_ >= waypoint.hold_s) {
      ++active_waypoint_index_;
      if (active_waypoint_index_ >= mission_.waypoints.size()) {
        state_ = MissionControllerState::Complete;
        checkpoint_valid_ = false;
        return {MissionControllerEvent::Type::Complete, active_waypoint_index_ - 1U,
                request_id_};
      }
      state_ = MissionControllerState::ExecutingWaypoint;
      trajectory_ready_ = false;
      arrival_start_time_s_.reset();
      next_goal_time_s_ = std::numeric_limits<double>::infinity();
      ++request_id_;
      return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
    }
  }
  return {};
}

MissionControllerState MissionController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool MissionController::holding() const { return state() == MissionControllerState::Holding; }

bool MissionController::waitingForAirborne() const {
  return state() == MissionControllerState::WaitingForAirborne;
}

std::size_t MissionController::activeWaypointIndex() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_waypoint_index_;
}

std::uint64_t MissionController::activeRequestId() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return request_id_;
}

MissionWaypoint MissionController::activeWaypoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mission_.waypoints.at(active_waypoint_index_);
}

}  // namespace px4_navigation_external_mode

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
    const bool is_braking_stop = trajectory_role == kSafetyTrajectoryRole &&
                                 safety_plan_kind == kSafetyStopKind;
    if (success && !is_braking_stop) {
      // A newly observed free corridor may replace the stop while the mode is
      // still braking. Resume the same pass-through waypoint immediately;
      // forcing a POSCTL handover here creates the stop-and-restart behavior
      // that makes a rolling local planner look discontinuous.
      state_ = MissionControllerState::ExecutingWaypoint;
      braking_start_time_s_.reset();
      braking_end_time_s_.reset();
      stopped_start_time_s_.reset();
      arrival_start_time_s_.reset();
      trajectory_ready_ = true;
      checkpoint_valid_ = true;
    } else if (success && is_braking_stop) {
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
    const auto& braking_waypoint = mission_.waypoints[active_waypoint_index_];
    const bool pass_through =
        braking_waypoint.behavior == MissionWaypoint::Behavior::PassThrough;
    const bool inside_acceptance = position.has_value() && position->allFinite() &&
                                   (*position - braking_waypoint.position_enu).norm() <=
                                       braking_waypoint.acceptance_radius_m;
    if (stopped && braking_complete) {
      // A safety trajectory can be selected at the end of a short local
      // horizon even though a pass-through waypoint has already been reached.
      // Treat that as waypoint acceptance once the vehicle is actually slow,
      // rather than handing the whole mission to POSCTL.  Stop waypoints keep
      // the fail-closed handover behavior below.
      if (pass_through && inside_acceptance) {
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
        braking_start_time_s_.reset();
        braking_end_time_s_.reset();
        stopped_start_time_s_.reset();
        return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
      }
      // For an intermediate pass-through waypoint this is a temporary
      // observation/safety stop, not a terminal fault. Keep External Mode in
      // charge and let the planner consume the next map revision. A terminal
      // stop waypoint still follows the fail-closed POSCTL path below.
      if (pass_through) return {};
      if (!stopped_start_time_s_.has_value()) stopped_start_time_s_ = now_s;
      if (now_s - *stopped_start_time_s_ >= kSafetyStopConfirmationS) {
        state_ = MissionControllerState::Paused;
        return {MissionControllerEvent::Type::RequestPositionControl,
                active_waypoint_index_, request_id_};
      }
    } else {
      stopped_start_time_s_.reset();
    }
    if (!pass_through && braking_start_time_s_.has_value() &&
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
  const auto passThroughAcceptance = [&]() {
    if (insideAcceptance()) return true;
    // Activate the next leg before the nominal acceptance sphere only when
    // there is a next waypoint and the measured velocity is actually carrying
    // the vehicle toward the active waypoint.  The lookahead is bounded by a
    // mission-level parameter, so it cannot silently change stop-waypoint
    // semantics or accept a receding waypoint while flying away from it.
    if (waypoint.behavior != MissionWaypoint::Behavior::PassThrough ||
        mission_.control.pass_through_lookahead_m <= waypoint.acceptance_radius_m ||
        active_waypoint_index_ + 1U >= mission_.waypoints.size() || !position.has_value() ||
        !velocity.has_value() || !position->allFinite() || !velocity->allFinite()) {
      return false;
    }
    // A long incoming leg is intentionally allowed to converge to its far
    // waypoint.  Early activation is reserved for short orthogonal legs,
    // where the vehicle has insufficient room to reverse a committed tangent
    // after the knot.  This keeps the long mission leg deterministic while
    // still shaping the local corner that motivated the lookahead.
    if (active_waypoint_index_ == 0U ||
        (waypoint.position_enu -
         mission_.waypoints[active_waypoint_index_ - 1U].position_enu).norm() > 12.0) {
      return false;
    }
    const auto to_waypoint = waypoint.position_enu - *position;
    const double distance = to_waypoint.norm();
    const double speed = velocity->norm();
    if (!std::isfinite(distance) || !std::isfinite(speed) || speed <= kSafetyStopSpeedMps ||
        distance > mission_.control.pass_through_lookahead_m) {
      return false;
    }
    // Do not require the instantaneous velocity to point directly at the knot:
    // an obstacle detour can legitimately carry the vehicle sideways or even
    // slightly away from the waypoint while still making forward progress on
    // the committed corridor.  The short-leg bound above and the planner's
    // collision verification remain the safety gates for this early handoff.
    return true;
  };
  const auto slowEnough = [&]() {
    // Waypoint completion is only valid with a measured, finite velocity
    // sample. Missing velocity must not turn a fly-through into an arrival.
    return velocity.has_value() && velocity->allFinite() &&
           velocity->norm() <= mission_.control.acceptance_speed_mps;
  };

  if (state_ == MissionControllerState::ExecutingWaypoint) {
    const bool pass_through = waypoint.behavior == MissionWaypoint::Behavior::PassThrough;
    const bool inside = passThroughAcceptance();
    // A receding-horizon mission may legitimately publish a waypoint that is
    // already inside the vehicle's acceptance ball (for example the takeoff
    // pose is also the first pass-through waypoint).  Waiting for a planner
    // trajectory in that case can manufacture a zero-length braking stop and
    // pause the mission forever.  Pass-through semantics only require a
    // finite measured state; a stop waypoint still requires the normal
    // trajectory/low-speed confirmation below.
    const bool immediate_pass_through = pass_through && inside && slowEnough();
    if ((trajectory_ready_ || immediate_pass_through) && inside &&
        (pass_through || slowEnough())) {
      if (pass_through) {
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

std::optional<MissionWaypoint> MissionController::nextWaypoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto next = active_waypoint_index_ + 1U;
  if (next >= mission_.waypoints.size()) return std::nullopt;
  return mission_.waypoints[next];
}

}  // namespace px4_navigation_external_mode

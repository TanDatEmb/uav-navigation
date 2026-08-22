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
      const bool has_positive_duration = std::isfinite(duration_s) && duration_s > 0.0;
      if (has_positive_duration) {
        // A positive-duration replacement is a new braking interval. Restart its
        // own confirmation window so an old stopped interval cannot trigger an
        // early handover while the replacement trajectory is still active.
        braking_start_time_s_ = now_s;
        braking_end_time_s_ = now_s + duration_s;
        stopped_start_time_s_.reset();
      } else {
        // The rolling planner may refresh a zero-duration braking stop on every
        // map update. These are status refreshes, not new braking phases: keep
        // both timers monotonic so a stationary vehicle can reach the safety
        // confirmation and request POSCTL.
        if (!braking_start_time_s_) braking_start_time_s_ = now_s;
        if (!braking_end_time_s_) braking_end_time_s_ = now_s;
      }
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

void MissionController::onNativeTrajectoryReady() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == MissionControllerState::Idle ||
      state_ == MissionControllerState::Complete ||
      state_ == MissionControllerState::Failed) {
    return;
  }
  checkpoint_valid_ = true;
  trajectory_ready_ = true;
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
        const double acceptance_error =
            (*position - braking_waypoint.position_enu).norm();
        const double acceptance_speed = velocity->norm();
        ++active_waypoint_index_;
        if (active_waypoint_index_ >= mission_.waypoints.size()) {
          state_ = MissionControllerState::Complete;
          checkpoint_valid_ = false;
          MissionControllerEvent event{MissionControllerEvent::Type::Complete,
                                       active_waypoint_index_ - 1U, request_id_};
          event.waypoint_accepted = true;
          event.accepted_waypoint_index = active_waypoint_index_ - 1U;
          event.acceptance_position_error_m = acceptance_error;
          event.acceptance_speed_mps = acceptance_speed;
          return event;
        }
        state_ = MissionControllerState::ExecutingWaypoint;
        trajectory_ready_ = false;
        arrival_start_time_s_.reset();
        next_goal_time_s_ = std::numeric_limits<double>::infinity();
        ++request_id_;
        braking_start_time_s_.reset();
        braking_end_time_s_.reset();
        stopped_start_time_s_.reset();
        MissionControllerEvent event{MissionControllerEvent::Type::PublishGoal,
                                     active_waypoint_index_, request_id_};
        event.waypoint_accepted = true;
        event.accepted_waypoint_index = active_waypoint_index_ - 1U;
        event.acceptance_position_error_m = acceptance_error;
        event.acceptance_speed_mps = acceptance_speed;
        return event;
      }
      // For an intermediate pass-through waypoint this is a temporary
      // observation/safety stop, not a terminal fault. Keep External Mode in
      // charge and let the planner consume the next map revision. A terminal
      // stop waypoint still follows the fail-closed POSCTL path below.
      if (pass_through) return {};
      if (!stopped_start_time_s_.has_value()) stopped_start_time_s_ = now_s;
      const double replan_grace_s = mission_.control.safety_stop_replan_grace_s;
      if (std::isfinite(replan_grace_s) && replan_grace_s > 0.0 &&
          now_s - *stopped_start_time_s_ < replan_grace_s) {
        // Do not hand over on the first settled local stop when the mission
        // explicitly permits a bounded map/replan grace period. A newer
        // safety route is handled by onTrajectory() and resumes Braking;
        // absence of one still falls through to the fail-closed handover.
        return {};
      }
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
  // A pass-through waypoint is still a mission waypoint, not a planner
  // horizon marker.  Never advance it from lookahead or velocity alone: the
  // measured position must enter its configured acceptance radius.
  const auto passThroughAcceptance = [&]() { return insideAcceptance(); };
  const auto slowEnough = [&]() {
    // Waypoint completion is only valid with a measured, finite velocity
    // sample. Missing velocity must not turn a fly-through into an arrival.
    return velocity.has_value() && velocity->allFinite() &&
           velocity->norm() <= mission_.control.acceptance_speed_mps;
  };
  const auto passThroughCornerReady = [&]() {
    // A pass-through waypoint may be accepted at flight speed on a straight
    // leg, but an orthogonal handover must not publish the next request while
    // the vehicle is still travelling along the incoming leg. Otherwise the
    // runtime has to fall back to a braking stop using the old tangent and
    // the vehicle can travel several metres beyond the next waypoint.
    if (!velocity.has_value() || !velocity->allFinite()) return false;
    if (active_waypoint_index_ == 0U ||
        active_waypoint_index_ + 1U >= mission_.waypoints.size()) {
      return true;
    }
    const auto& previous = mission_.waypoints[active_waypoint_index_ - 1U];
    const auto& next = mission_.waypoints[active_waypoint_index_ + 1U];
    const auto incoming = waypoint.position_enu - previous.position_enu;
    const auto outgoing = next.position_enu - waypoint.position_enu;
    if (!incoming.allFinite() || !outgoing.allFinite() || incoming.norm() <= 1e-6 ||
        outgoing.norm() <= 1e-6) {
      return true;
    }
    // Only gate a genuine corner. A value above 0.7 is approximately a
    // turn below 45 degrees and should retain the normal fly-through rule.
    if (incoming.normalized().dot(outgoing.normalized()) > 0.7) return true;
    const double speed = velocity->norm();
    if (!std::isfinite(speed) || speed <= mission_.control.acceptance_speed_mps) return true;
    return velocity->normalized().dot(outgoing.normalized()) >= 0.25;
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
    const bool acceptance_ready = pass_through ? passThroughCornerReady() : slowEnough();
    if ((trajectory_ready_ || immediate_pass_through) && inside && acceptance_ready) {
      if (pass_through) {
        const double acceptance_error = (*position - waypoint.position_enu).norm();
        const double acceptance_speed = velocity->norm();
        ++active_waypoint_index_;
        if (active_waypoint_index_ >= mission_.waypoints.size()) {
          state_ = MissionControllerState::Complete;
          checkpoint_valid_ = false;
          MissionControllerEvent event{MissionControllerEvent::Type::Complete,
                                       active_waypoint_index_ - 1U, request_id_};
          event.waypoint_accepted = true;
          event.accepted_waypoint_index = active_waypoint_index_ - 1U;
          event.acceptance_position_error_m = acceptance_error;
          event.acceptance_speed_mps = acceptance_speed;
          return event;
        }
        state_ = MissionControllerState::ExecutingWaypoint;
        trajectory_ready_ = false;
        arrival_start_time_s_.reset();
        next_goal_time_s_ = std::numeric_limits<double>::infinity();
        ++request_id_;
        MissionControllerEvent event{MissionControllerEvent::Type::PublishGoal,
                                     active_waypoint_index_, request_id_};
        event.waypoint_accepted = true;
        event.accepted_waypoint_index = active_waypoint_index_ - 1U;
        event.acceptance_position_error_m = acceptance_error;
        event.acceptance_speed_mps = acceptance_speed;
        return event;
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
      const double acceptance_error = (*position - waypoint.position_enu).norm();
      const double acceptance_speed = velocity->norm();
      ++active_waypoint_index_;
      if (active_waypoint_index_ >= mission_.waypoints.size()) {
        state_ = MissionControllerState::Complete;
        checkpoint_valid_ = false;
        MissionControllerEvent event{MissionControllerEvent::Type::Complete,
                                     active_waypoint_index_ - 1U, request_id_};
        event.waypoint_accepted = true;
        event.accepted_waypoint_index = active_waypoint_index_ - 1U;
        event.acceptance_position_error_m = acceptance_error;
        event.acceptance_speed_mps = acceptance_speed;
        return event;
      }
      state_ = MissionControllerState::ExecutingWaypoint;
      trajectory_ready_ = false;
      arrival_start_time_s_.reset();
      next_goal_time_s_ = std::numeric_limits<double>::infinity();
      ++request_id_;
      MissionControllerEvent event{MissionControllerEvent::Type::PublishGoal,
                                   active_waypoint_index_, request_id_};
      event.waypoint_accepted = true;
      event.accepted_waypoint_index = active_waypoint_index_ - 1U;
      event.acceptance_position_error_m = acceptance_error;
      event.acceptance_speed_mps = acceptance_speed;
      return event;
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

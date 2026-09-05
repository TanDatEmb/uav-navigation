#include "px4_navigation_external_mode/mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace px4_navigation_external_mode {
namespace {

std::optional<double> checkedAddSeconds(const double base_s,
                                        const double offset_s) noexcept {
  if (!std::isfinite(base_s) || !std::isfinite(offset_s) || offset_s < 0.0) {
    return std::nullopt;
  }
  const long double sum = static_cast<long double>(base_s) +
                          static_cast<long double>(offset_s);
  if (!std::isfinite(sum) ||
      sum > static_cast<long double>(std::numeric_limits<double>::max())) {
    return std::nullopt;
  }
  return static_cast<double>(sum);
}

}  // namespace

MissionController::MissionController(Mission mission)
    : mission_(std::move(mission)), route_progress_(mission_) {
  if (mission_.waypoints.empty()) {
    throw std::invalid_argument("mission controller requires at least one waypoint");
  }
}

navigation_mission::ImmutableRouteSnapshot MissionController::routeSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return route_progress_.snapshot(
      mission_.id, mission_.frame, 1U, request_id_, active_waypoint_index_);
}

bool MissionController::activePassThroughHasCoincidentStop() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return navigation_mission::passThroughNextWaypointIsCoincidentStop(
      route_progress_.snapshot(
          mission_.id, mission_.frame, 1U, request_id_, active_waypoint_index_));
}

void MissionController::activate(double now_s) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("mission activation time must be finite");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  route_progress_.reset();
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
  terminal_hold_pending_ = false;
  previous_position_.reset();
  previous_position_time_s_ = 0.0;
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
  terminal_hold_pending_ = false;
  previous_position_.reset();
  previous_position_time_s_ = 0.0;
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

  // The callback is a public transport boundary. Do not let an unknown
  // role/kind pair look like a successful nominal trajectory: in Braking it
  // could clear the safety phase, and in ExecutingWaypoint it could mark an
  // uncertified callback as ready.
  const bool nominal_trajectory =
      trajectory_role == 0U &&
      (safety_plan_kind == 0U || safety_plan_kind == kSafetyRouteKind);
  const bool safety_trajectory =
      trajectory_role == kSafetyTrajectoryRole &&
      (safety_plan_kind == kSafetyRouteKind ||
       safety_plan_kind == kSafetyStopKind);
  if (success && !nominal_trajectory && !safety_trajectory) {
    if (state_ == MissionControllerState::ExecutingWaypoint) {
      state_ = MissionControllerState::Paused;
      checkpoint_valid_ = true;
      trajectory_ready_ = false;
      pending_position_control_ = true;
    }
    return;
  }

  if (success && trajectory_role == kSafetyTrajectoryRole &&
      safety_plan_kind == kSafetyStopKind &&
      (!std::isfinite(duration_s) || duration_s < 0.0)) {
    // A safety-stop duration is transport metadata, not an optional tuning
    // hint.  Clamping NaN/Inf/negative values to zero would make the
    // confirmation window immediately eligible and hide a malformed command.
    state_ = MissionControllerState::Paused;
    checkpoint_valid_ = true;
    trajectory_ready_ = false;
    pending_position_control_ = true;
    braking_start_time_s_.reset();
    braking_end_time_s_.reset();
    stopped_start_time_s_.reset();
    return;
  }

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
        const auto braking_end = checkedAddSeconds(now_s, duration_s);
        if (!braking_end) {
          state_ = MissionControllerState::Paused;
          checkpoint_valid_ = true;
          trajectory_ready_ = false;
          pending_position_control_ = true;
          braking_start_time_s_.reset();
          braking_end_time_s_.reset();
          stopped_start_time_s_.reset();
          return;
        }
        braking_start_time_s_ = now_s;
        braking_end_time_s_ = *braking_end;
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
    const auto braking_end = checkedAddSeconds(now_s, duration_s);
    if (!braking_end) {
      state_ = MissionControllerState::Paused;
      checkpoint_valid_ = true;
      trajectory_ready_ = false;
      pending_position_control_ = true;
      return;
    }
    state_ = MissionControllerState::Braking;
    braking_start_time_s_ = now_s;
    braking_end_time_s_ = *braking_end;
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
  // A terminal BACKUP can be the only certified command that holds the
  // active STOP waypoint. Do not let a later generic readiness callback erase
  // that measured-settling contract before the vehicle is slow enough.
  // The pending latch is cleared by update() after measured settling, or by
  // a new waypoint transition.
  const bool active_terminal_stop =
      active_waypoint_index_ < mission_.waypoints.size() &&
      mission_.waypoints[active_waypoint_index_].behavior ==
          MissionWaypoint::Behavior::Stop;
  if (!active_terminal_stop || !terminal_hold_pending_) {
    terminal_hold_pending_ = false;
  }
}

void MissionController::onNativeSafetyTrajectoryObserved() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == MissionControllerState::Idle ||
      state_ == MissionControllerState::Complete ||
      state_ == MissionControllerState::Failed) {
    return;
  }
  // BACKUP/EMERGENCY owns the current command until a certified stop. It may
  // not be used as the nominal trajectory-ready witness for pass-through
  // progress. Keep terminal_hold_pending_ unchanged: a terminal STOP still
  // needs its own bounded hold contract.
  trajectory_ready_ = false;
  checkpoint_valid_ = true;
}

void MissionController::onNativeTerminalHoldObserved() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == MissionControllerState::Idle ||
      state_ == MissionControllerState::Complete ||
      state_ == MissionControllerState::Failed ||
      active_waypoint_index_ >= mission_.waypoints.size()) {
    return;
  }

  const auto& waypoint = mission_.waypoints[active_waypoint_index_];
  if (waypoint.behavior == MissionWaypoint::Behavior::Stop) {
    terminal_hold_pending_ = true;
    // A terminal safety suffix is not nominal readiness for a pass-through
    // checkpoint, but it is the certified command that owns this STOP
    // endpoint. Keep the readiness witness so update() can enter the normal
    // measured confirmation window when the speed gate settles.
    trajectory_ready_ = true;
    checkpoint_valid_ = true;
    return;
  }
  if (navigation_mission::passThroughNextWaypointIsCoincidentStop(
          route_progress_.snapshot(
              mission_.id, mission_.frame, 1U, request_id_,
              active_waypoint_index_))) {
    // The active pass-through checkpoint is still accepted by the normal
    // measured pass-through path, but the following STOP owns the hold. Do
    // not request an outgoing velocity for the zero-length boundary.
    terminal_hold_pending_ = true;
    // onNativeSafetyTrajectoryObserved() deliberately clears generic
    // readiness while BACKUP owns the command. Restore readiness only for
    // this exact terminal witness so update() can perform the measured
    // pass-through acceptance and publish the following STOP goal.
    trajectory_ready_ = true;
    checkpoint_valid_ = true;
    return;
  }
  if (active_waypoint_index_ == 0U ||
      active_waypoint_index_ + 1U >= mission_.waypoints.size()) {
    return;
  }
  const auto& previous = mission_.waypoints[active_waypoint_index_ - 1U];
  const auto& next = mission_.waypoints[active_waypoint_index_ + 1U];
  const auto incoming = waypoint.position_enu - previous.position_enu;
  const auto outgoing = next.position_enu - waypoint.position_enu;
  if (!incoming.allFinite() || !outgoing.allFinite() || incoming.norm() <= 1e-6 ||
      outgoing.norm() <= 1e-6) {
    return;
  }
  // Match passThroughCornerReady(): a genuine turn must settle before the
  // next mission request is allowed to change the velocity direction.
  if (incoming.normalized().dot(outgoing.normalized()) <= 0.7) {
    terminal_hold_pending_ = true;
  }
}

MissionControllerEvent MissionController::update(
    double now_s, const std::optional<Eigen::Vector3d>& position, bool airborne,
    const std::optional<Eigen::Vector3d>& velocity,
    const std::optional<CertifiedContinuation>& continuation,
    const bool certified_suffix_stop) {
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

  // Keep the last valid mission-update sample for pass-through crossing
  // detection. The local copy is deliberately captured before replacing the
  // sample so a single update can certify the measured segment that crossed a
  // waypoint acceptance ball.
  const auto previous_position = previous_position_;
  const double previous_position_time_s = previous_position_time_s_;
  if (position.has_value() && position->allFinite()) {
    previous_position_ = *position;
    previous_position_time_s_ = now_s;
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
        if (!advanceRequestId()) {
          state_ = MissionControllerState::Failed;
          return {MissionControllerEvent::Type::Failure, active_waypoint_index_, request_id_};
        }
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
  if (position.has_value()) {
    (void)route_progress_.update(*position);
  }
  const auto insideAcceptance = [&]() {
    return position.has_value() &&
           route_progress_.insideAcceptance(active_waypoint_index_, *position);
  };
  // A pass-through waypoint is still a mission waypoint, not a planner
  // horizon marker. Advance only from measured positions: either the current
  // sample is inside the acceptance ball, or two recent measured samples form
  // a forward route-ordered segment that intersects that ball. The latter is
  // required at cruise speed because a 50 ms mission timer can otherwise skip
  // a sub-metre ball between samples.
  const auto passThroughAcceptanceError = [&]() -> std::optional<double> {
    if (!position.has_value() || !position->allFinite() ||
        active_waypoint_index_ >= mission_.waypoints.size()) {
      return std::nullopt;
    }
    const double sample_gap_s = previous_position.has_value() &&
                                    std::isfinite(previous_position_time_s) &&
                                    now_s >= previous_position_time_s
        ? now_s - previous_position_time_s
        : std::numeric_limits<double>::quiet_NaN();
    return route_progress_.measuredWaypointCrossingError(
        active_waypoint_index_, *position, previous_position, sample_gap_s,
        kMaximumPassThroughSampleGapS);
  };
  const auto passThroughAcceptance = [&]() {
    return passThroughAcceptanceError().has_value();
  };
  const auto slowEnough = [&]() {
    // Waypoint completion is only valid with a measured, finite velocity
    // sample. Missing velocity must not turn a fly-through into an arrival.
    return velocity.has_value() && velocity->allFinite() &&
           velocity->norm() <= mission_.control.acceptance_speed_mps;
  };
  const auto passThroughCornerReady = [&]() {
    // Position acceptance is still measured and velocity must be finite. The
    // planner owns the incoming leg up to this boundary; the next hot-replan
    // owns the outgoing turn. Requiring outgoing velocity alignment here
    // couples mission progress to one MINCO endpoint and can force a braking
    // stop exactly at a sharp corner.
    if (!velocity.has_value() || !velocity->allFinite()) return false;
    return true;
  };

  if (state_ == MissionControllerState::ExecutingWaypoint) {
    const bool pass_through = waypoint.behavior == MissionWaypoint::Behavior::PassThrough;
    const bool coincident_terminal_stop = pass_through &&
        navigation_mission::passThroughNextWaypointIsCoincidentStop(
            route_progress_.snapshot(
                mission_.id, mission_.frame, 1U, request_id_,
                active_waypoint_index_));
    const bool inside = passThroughAcceptance();
    // Only the initial pass-through checkpoint may be accepted before the
    // first native trajectory is acknowledged.  This is the takeoff/mission
    // handoff exception for a waypoint that is already inside the measured
    // acceptance ball.  Once the route has advanced, trajectory_ready_ is a
    // handoff certificate for the *current* waypoint: allowing a later
    // pass-through to bypass it can advance MissionController while PX4 is
    // still executing the previous waypoint's certified bundle.
    const bool immediate_pass_through =
        pass_through && active_waypoint_index_ == 0U && request_id_ == 1U && inside &&
        slowEnough();
    const bool acceptance_ready = pass_through ? passThroughCornerReady() : slowEnough();
    // Capture this before the coincident PASS/STOP hold block clears its
    // pending marker at a measured stop. The captured readiness is the
    // terminal MAIN witness for the same update that accepts PASS.
    const bool coincident_terminal_hold_ready = pass_through &&
        coincident_terminal_stop && terminal_hold_pending_ && trajectory_ready_;
    if (terminal_hold_pending_ && !pass_through) {
      // A terminal STOP requires a continuous measured confirmation window,
      // not a single low-speed sample. Keep the certified endpoint hold while
      // either position or speed temporarily leaves the gate, and restart
      // the confirmation timer. This prevents a brief speed dip from
      // releasing the hold and allowing a moving replan before the vehicle is
      // actually settled.
      if (!inside || !acceptance_ready) {
        arrival_start_time_s_.reset();
        return {};
      }
      if (!arrival_start_time_s_.has_value()) {
        arrival_start_time_s_ = now_s;
      }
      if (now_s - *arrival_start_time_s_ >= mission_.control.acceptance_confirmation_s) {
        terminal_hold_pending_ = false;
        state_ = MissionControllerState::Holding;
        hold_start_time_s_ = now_s;
      }
      return {};
    }
    if (terminal_hold_pending_) {
      // A certified terminal MAIN sample already supplies a position hold.
      // While the vehicle is inside the active acceptance ball but still
      // moving, keep that hold and wait for measured settling. Re-publishing
      // the same goal here would clear the hold and recreate the oscillation
      // at a corner. If the vehicle has not yet entered the ball, the same
      // bounded hold is still safe. A later generic readiness callback cannot
      // clear this latch while the active waypoint is a STOP; only measured
      // settling or a waypoint transition may release it.
      // A coincident PASS_THROUGH/STOP boundary is owned by the following
      // STOP. Do not publish that STOP goal while the vehicle is still
      // moving through the shared acceptance ball; runtime may still be
      // draining the certified suffix and cannot safely replace its identity
      // until the measured stop boundary.
      const bool terminal_hold_ready = coincident_terminal_stop
          ? slowEnough() : acceptance_ready;
      if (inside && terminal_hold_ready) {
        terminal_hold_pending_ = false;
      } else {
        return {};
      }
    }
    const bool certified_main_continuation = pass_through && continuation.has_value() &&
        continuation->valid() && continuation->mission_id == mission_.id &&
        continuation->waypoint_index == active_waypoint_index_ &&
        continuation->request_id == request_id_;
    // PASS_THROUGH requires measured crossing plus the current accepted MAIN
    // command witness. Generic readiness remains for STOP/legacy hold paths.
    const bool progression_ready = pass_through
        ? (certified_main_continuation || certified_suffix_stop ||
           coincident_terminal_hold_ready || immediate_pass_through)
        : trajectory_ready_;
    if (progression_ready && inside && acceptance_ready) {
      if (pass_through) {
        const auto acceptance_error = passThroughAcceptanceError();
        if (!acceptance_error.has_value() || !position.has_value() ||
            !velocity.has_value() || !velocity->allFinite()) {
          return {};
        }
        const double acceptance_speed = velocity->norm();
        ++active_waypoint_index_;
        if (active_waypoint_index_ >= mission_.waypoints.size()) {
          state_ = MissionControllerState::Complete;
          checkpoint_valid_ = false;
          MissionControllerEvent event{MissionControllerEvent::Type::Complete,
                                       active_waypoint_index_ - 1U, request_id_};
          event.waypoint_accepted = true;
          event.accepted_waypoint_index = active_waypoint_index_ - 1U;
          event.acceptance_position_error_m = *acceptance_error;
          event.acceptance_speed_mps = acceptance_speed;
          return event;
        }
        state_ = MissionControllerState::ExecutingWaypoint;
        trajectory_ready_ = false;
        arrival_start_time_s_.reset();
        next_goal_time_s_ = std::numeric_limits<double>::infinity();
        if (!advanceRequestId()) {
          state_ = MissionControllerState::Failed;
          return {MissionControllerEvent::Type::Failure, active_waypoint_index_, request_id_};
        }
        MissionControllerEvent event{MissionControllerEvent::Type::PublishGoal,
                                     active_waypoint_index_, request_id_};
        event.waypoint_accepted = true;
        event.accepted_waypoint_index = active_waypoint_index_ - 1U;
        event.acceptance_position_error_m = *acceptance_error;
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
      if (!advanceRequestId()) {
        state_ = MissionControllerState::Failed;
        return {MissionControllerEvent::Type::Failure, active_waypoint_index_, request_id_};
      }
      return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
    }
    return {};
  }

  if (state_ == MissionControllerState::Holding) {
    if (!insideAcceptance()) {
      state_ = MissionControllerState::ExecutingWaypoint;
      trajectory_ready_ = false;
      arrival_start_time_s_.reset();
      next_goal_time_s_ = std::numeric_limits<double>::infinity();
      if (!advanceRequestId()) {
        state_ = MissionControllerState::Failed;
        return {MissionControllerEvent::Type::Failure, active_waypoint_index_, request_id_};
      }
      return {MissionControllerEvent::Type::PublishGoal, active_waypoint_index_, request_id_};
    }
    if (!slowEnough()) {
      // Keep the certified position hold while the vehicle is still inside
      // the acceptance ball. A single noisy/overshooting velocity sample must
      // restart the measured hold timer, not publish the same goal again: the
      // latter sends the planner back into a stop/replan loop at a waypoint.
      hold_start_time_s_ = now_s;
      return {};
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
      if (!advanceRequestId()) {
        state_ = MissionControllerState::Failed;
        return {MissionControllerEvent::Type::Failure, active_waypoint_index_, request_id_};
      }
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

std::optional<MissionWaypoint> MissionController::activeWaypoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_waypoint_index_ >= mission_.waypoints.size()) return std::nullopt;
  return mission_.waypoints[active_waypoint_index_];
}

bool MissionController::advanceRequestId() noexcept {
  if (request_id_ == std::numeric_limits<std::uint64_t>::max()) return false;
  ++request_id_;
  return request_id_ != 0U;
}

std::optional<MissionWaypoint> MissionController::waypointAt(std::size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= mission_.waypoints.size()) return std::nullopt;
  return mission_.waypoints[index];
}

std::optional<MissionWaypoint> MissionController::nextWaypoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto next = active_waypoint_index_ + 1U;
  if (next >= mission_.waypoints.size()) return std::nullopt;
  return mission_.waypoints[next];
}

bool MissionController::nativeTrajectoryReady() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return trajectory_ready_;
}

bool MissionController::terminalHoldPending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_hold_pending_;
}

double MissionController::acceptanceSpeedMps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mission_.control.acceptance_speed_mps;
}

}  // namespace px4_navigation_external_mode

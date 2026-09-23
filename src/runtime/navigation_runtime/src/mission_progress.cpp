#include "navigation_runtime/mission_progress.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace navigation_runtime {
namespace {
constexpr std::int64_t kMaximumCrossingSampleGapNs = 250'000'000;

bool sameRoute(const MissionRouteIdentity& a,
               const MissionRouteIdentity& b) noexcept {
  return a.mission_id == b.mission_id && a.revision == b.revision &&
      a.localization_epoch == b.localization_epoch;
}

bool sameGate(const MissionGateIdentity& a,
              const MissionGateIdentity& b) noexcept {
  return a.waypoint_index == b.waypoint_index && a.request_id == b.request_id;
}

std::optional<std::int64_t> secondsToNs(const double seconds) noexcept {
  if (!std::isfinite(seconds) || seconds < 0.0 ||
      seconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(std::ceil(seconds * 1e9));
}
}  // namespace

bool MissionMeasuredSample::valid() const noexcept {
  return position.allFinite() && velocity.allFinite() && source_stamp_ns > 0 &&
      sequence > 0U && localization_epoch > 0U;
}

MissionProgress::MissionProgress(navigation_mission::Mission mission)
    : mission_(std::move(mission)), measured_route_(mission_) {
  if (!mission_.valid()) {
    throw std::invalid_argument("Core mission progress requires valid mission");
  }
  identity_.mission_id = mission_.id;
}

void MissionProgress::resetIdentity(const std::uint64_t route_revision,
                                    const std::uint64_t localization_epoch) {
  if (route_revision == 0U || localization_epoch == 0U) {
    throw std::invalid_argument("mission identity requires nonzero route and localization");
  }
  if (identity_.revision != route_revision) {
    gate_.waypoint_index = 0U;
    accepted_.reset();
  }
  identity_.revision = route_revision;
  identity_.localization_epoch = localization_epoch;
  measured_route_.reset();
  previous_.reset();
  latest_.reset();
  crossing_.reset();
  continuation_.reset();
  stop_confirmation_start_ns_.reset();
  stop_hold_start_ns_.reset();
  active_ = false;
}

MissionProgressDecision MissionProgress::activate() {
  if (identity_.localization_epoch == 0U || complete()) return {};
  active_ = true;
  crossing_.reset();
  continuation_.reset();
  stop_confirmation_start_ns_.reset();
  stop_hold_start_ns_.reset();
  if (!latest_.has_value() || !measured_route_.state().valid) return {};
  if (gate_.request_id == std::numeric_limits<std::uint64_t>::max()) return {};
  ++gate_.request_id;
  return {MissionProgressDecision::Kind::Goal, gate_, std::nullopt, 0.0, 0.0};
}

void MissionProgress::deactivate() {
  active_ = false;
  crossing_.reset();
  continuation_.reset();
  stop_confirmation_start_ns_.reset();
  stop_hold_start_ns_.reset();
  previous_.reset();
}

bool MissionProgress::continuationMatches() const noexcept {
  return continuation_.has_value() && sameRoute(continuation_->route, identity_) &&
      sameGate(continuation_->gate, gate_) &&
      continuation_->bundle_generation > 0U &&
      continuation_->boundary_stamp_ns > 0 &&
      continuation_->valid_until_ns >= continuation_->boundary_stamp_ns;
}

bool MissionProgress::isCoincidentTerminalStop() const {
  return navigation_mission::passThroughNextWaypointIsCoincidentStop(routeSnapshot());
}

MissionProgressDecision MissionProgress::observeContinuation(
    const MissionContinuationWitness& witness, const std::int64_t now_ns) {
  if (!active_ || complete() || now_ns <= 0 || now_ns > witness.valid_until_ns ||
      !sameRoute(witness.route, identity_) || !sameGate(witness.gate, gate_) ||
      witness.bundle_generation == 0U || witness.boundary_stamp_ns <= 0 ||
      witness.boundary_stamp_ns > witness.valid_until_ns) {
    return {};
  }
  if (continuation_.has_value() &&
      continuation_->bundle_generation > witness.bundle_generation) {
    return {};
  }
  continuation_ = witness;
  if (gate_.waypoint_index >= mission_.waypoints.size()) return {};
  const auto& waypoint = mission_.waypoints[gate_.waypoint_index];
  if (waypoint.behavior != navigation_mission::MissionWaypoint::Behavior::PassThrough ||
      !crossing_.has_value() || !latest_.has_value() ||
      (witness.kind != MissionContinuationWitness::Kind::Main &&
       witness.kind != MissionContinuationWitness::Kind::SafetySuffixStop &&
       !(witness.kind == MissionContinuationWitness::Kind::TerminalHold &&
         isCoincidentTerminalStop()))) {
    return {};
  }
  if ((isCoincidentTerminalStop() ||
       witness.kind == MissionContinuationWitness::Kind::SafetySuffixStop) &&
      latest_->velocity.norm() >
      mission_.control.acceptance_speed_mps) {
    return {};
  }
  if (witness.kind == MissionContinuationWitness::Kind::SafetySuffixStop &&
      !measured_route_.insideAcceptance(gate_.waypoint_index, latest_->position)) {
    return {};
  }
  return accept(crossing_->crossing_error_m, crossing_->after.velocity.norm());
}

MissionProgressDecision MissionProgress::observeMeasured(
    const MissionMeasuredSample& sample, const std::int64_t now_ns) {
  if (!sample.valid() || now_ns < sample.source_stamp_ns ||
      sample.localization_epoch != identity_.localization_epoch ||
      complete()) {
    return {};
  }
  if (previous_.has_value() &&
      (sample.source_stamp_ns <= previous_->source_stamp_ns ||
       sample.sequence <= previous_->sequence)) {
    // An unorderable stream cannot extend the physical crossing witness.
    crossing_.reset();
    previous_.reset();
    return {};
  }
  if (previous_.has_value() &&
      sample.source_stamp_ns - previous_->source_stamp_ns >
          kMaximumCrossingSampleGapNs) {
    // The old physical segment is no longer a continuously observed route
    // crossing. Execution readiness has its own finite command lease.
    crossing_.reset();
    previous_.reset();
  }
  const auto prior = previous_;
  const auto before_cursor = measured_route_.state().projection;
  const auto measured = measured_route_.update(sample.position);
  previous_ = sample;
  latest_ = sample;
  const auto tangent = measured_route_.incomingTangent(gate_.waypoint_index).value_or(
      measured_route_.outgoingTangent(gate_.waypoint_index).value_or(
          Eigen::Vector3d::Zero()));
  const bool reverse_crossing = crossing_.has_value() && prior.has_value() &&
      tangent.allFinite() && tangent.norm() > 0.0 &&
      (sample.position - prior->position).dot(tangent) <
          -navigation_mission::RouteProgressConfig{}.backtrack_tolerance_m;
  if (measured.backtracking_exceeded || reverse_crossing) crossing_.reset();
  if (!active_) return {};
  if (gate_.request_id == 0U) {
    gate_.request_id = 1U;
    return {MissionProgressDecision::Kind::Goal, gate_, std::nullopt, 0.0, 0.0};
  }
  if (gate_.waypoint_index >= mission_.waypoints.size()) return {};
  const auto& waypoint = mission_.waypoints[gate_.waypoint_index];
  const double error_m = (sample.position - waypoint.position_enu).norm();
  if (waypoint.behavior == navigation_mission::MissionWaypoint::Behavior::PassThrough) {
    if (!measured.backtracking_exceeded && !reverse_crossing) {
      const bool gap_valid = prior.has_value() &&
          sample.source_stamp_ns - prior->source_stamp_ns <= kMaximumCrossingSampleGapNs;
      const auto crossing_error = measured_route_.measuredWaypointCrossingError(
          gate_.waypoint_index, sample.position,
          gap_valid ? std::optional<Eigen::Vector3d>{prior->position} : std::nullopt,
          gap_valid ? static_cast<double>(sample.source_stamp_ns - prior->source_stamp_ns) / 1e9
                    : std::numeric_limits<double>::quiet_NaN(),
          static_cast<double>(kMaximumCrossingSampleGapNs) / 1e9);
      if (crossing_error.has_value()) {
        crossing_ = MissionCrossingWitness{
            identity_, gate_, prior.value_or(sample), sample, before_cursor,
            measured.projection, *crossing_error};
      }
    }
    if (!crossing_.has_value()) return {};
    const bool initial_inside = gate_.waypoint_index == 0U && gate_.request_id == 1U &&
        error_m <= waypoint.acceptance_radius_m &&
        sample.velocity.norm() <= mission_.control.acceptance_speed_mps;
    const bool ready = continuationMatches() &&
        continuation_->valid_until_ns >= now_ns &&
        (continuation_->kind == MissionContinuationWitness::Kind::Main ||
         continuation_->kind == MissionContinuationWitness::Kind::SafetySuffixStop ||
         (continuation_->kind == MissionContinuationWitness::Kind::TerminalHold &&
          isCoincidentTerminalStop()));
    if (!ready && !initial_inside) return {};
    if ((isCoincidentTerminalStop() ||
         (ready && continuation_->kind == MissionContinuationWitness::Kind::SafetySuffixStop)) &&
        sample.velocity.norm() > mission_.control.acceptance_speed_mps) return {};
    if (ready && continuation_->kind == MissionContinuationWitness::Kind::SafetySuffixStop &&
        !measured_route_.insideAcceptance(gate_.waypoint_index, sample.position)) return {};
    return accept(crossing_->crossing_error_m, sample.velocity.norm());
  }

  const bool inside = measured_route_.insideAcceptance(gate_.waypoint_index,
                                                       sample.position);
  const bool slow = sample.velocity.norm() <= mission_.control.acceptance_speed_mps;
  const bool has_execution_hold = continuationMatches() &&
      continuation_->valid_until_ns >= now_ns;
  if (!inside || !slow || !has_execution_hold) {
    stop_confirmation_start_ns_.reset();
    stop_hold_start_ns_.reset();
    return {};
  }
  if (!stop_confirmation_start_ns_.has_value()) {
    stop_confirmation_start_ns_ = sample.source_stamp_ns;
  }
  const auto confirmation_ns = secondsToNs(mission_.control.acceptance_confirmation_s);
  const auto hold_ns = secondsToNs(waypoint.hold_s);
  if (!confirmation_ns || !hold_ns ||
      sample.source_stamp_ns - *stop_confirmation_start_ns_ < *confirmation_ns) {
    return {};
  }
  if (!stop_hold_start_ns_.has_value()) stop_hold_start_ns_ = sample.source_stamp_ns;
  if (sample.source_stamp_ns - *stop_hold_start_ns_ < *hold_ns) return {};
  return accept(error_m, sample.velocity.norm());
}

MissionProgressDecision MissionProgress::accept(const double error_m,
                                                const double speed_mps) {
  const MissionGateIdentity just_accepted = gate_;
  accepted_ = just_accepted;
  crossing_.reset();
  continuation_.reset();
  stop_confirmation_start_ns_.reset();
  stop_hold_start_ns_.reset();
  if (gate_.waypoint_index + 1U >= mission_.waypoints.size()) {
    active_ = false;
    return {MissionProgressDecision::Kind::Complete, gate_, just_accepted,
            error_m, speed_mps};
  }
  ++gate_.waypoint_index;
  if (gate_.request_id == std::numeric_limits<std::uint64_t>::max()) return {};
  ++gate_.request_id;
  return {MissionProgressDecision::Kind::Goal, gate_, just_accepted,
          error_m, speed_mps};
}

navigation_mission::ImmutableRouteSnapshot MissionProgress::routeSnapshot() const {
  return measured_route_.snapshot(mission_.id, mission_.frame, identity_.revision,
                                  gate_.request_id, gate_.waypoint_index);
}

}  // namespace navigation_runtime

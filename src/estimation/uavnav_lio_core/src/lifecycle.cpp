#include "uavnav/lio/lifecycle.hpp"

namespace uavnav::lio {

namespace {

constexpr bool is_listed(LioState from, LioState to, LioReason reason) {
  for (const TransitionEdge& edge : kLioTransitions) {
    if (edge.from == from && edge.to == to && edge.reason == reason) return true;
  }
  return false;
}

constexpr bool is_scan(LioEvent::Kind k) {
  return k == LioEvent::Kind::kScanGood || k == LioEvent::Kind::kScanDegenerate || k == LioEvent::Kind::kScanEmpty;
}

constexpr bool is_bad_scan(LioEvent::Kind k) {
  return k == LioEvent::Kind::kScanDegenerate || k == LioEvent::Kind::kScanEmpty;
}

// A NaN sigma is "exceeded": the comparison is written so that anything that is not a
// finite-or-infinite number at or below the limit fails closed.
constexpr bool sigma_exceeded(double sigma, double limit) { return !(sigma <= limit); }

static_assert(is_listed(LioState::kLost, LioState::kRestarting, LioReason::kGeometryReturned));
static_assert(!is_listed(LioState::kLost, LioState::kTracking, LioReason::kConfirmationReached));

}  // namespace

Transition LioLifecycle::go(LioState to, LioReason reason) noexcept {
  const LioState from = state_;
  // Every state change must be a listed edge; an unlisted one is a bug here and is refused.
  if (!is_listed(from, to, reason)) return unchanged();

  state_ = to;
  good_run_ = 0;
  bad_run_ = 0;
  map_ready_at_.reset();
  // The degenerate run that degraded TRACKING is still the current run in DEGRADED.
  if (!(from == LioState::kTracking && to == LioState::kDegraded)) degenerate_since_.reset();
  return Transition{from, to, reason, true};
}

Transition LioLifecycle::on(const LioEvent& e) noexcept {
  if (is_scan(e.kind)) last_scan_ = e.t;
  switch (state_) {
    case LioState::kInitializing: return on_initializing(e);
    case LioState::kTracking: return on_tracking(e);
    case LioState::kDegraded: return on_degraded(e);
    case LioState::kLost: return on_lost(e);
    case LioState::kRestarting: return on_restarting(e);
  }
  return unchanged();
}

Transition LioLifecycle::on_initializing(const LioEvent& e) noexcept {
  using Kind = LioEvent::Kind;
  if (e.kind == Kind::kMapReady) {
    if (map_ready_at_) return unchanged();  // already counting; do not restart the confirmation
    map_ready_at_ = e.t;
    good_run_ = 0;
    return Transition{state_, state_, LioReason::kMapReady, false};
  }
  if (!map_ready_at_) return unchanged();  // scans are not counted before the map is ready
  if (e.kind == Kind::kScanGood) {
    if (++good_run_ >= cfg_.confirm_scans) return go(LioState::kTracking, LioReason::kConfirmationReached);
  } else if (is_bad_scan(e.kind)) {
    good_run_ = 0;  // confirmation needs consecutive good scans
  }
  return unchanged();
}

Transition LioLifecycle::on_tracking(const LioEvent& e) noexcept {
  using Kind = LioEvent::Kind;
  if (sigma_exceeded(e.position_sigma_m, cfg_.position_sigma_lost_m)) {
    return go(LioState::kLost, LioReason::kCovarianceExceeded);
  }
  if (e.kind == Kind::kScanGood) {
    bad_run_ = 0;
    degenerate_since_.reset();
  } else if (is_bad_scan(e.kind)) {
    if (!degenerate_since_) degenerate_since_ = e.t;
    if (++bad_run_ >= cfg_.degenerate_scans) return go(LioState::kDegraded, LioReason::kDegenerateScans);
  } else if (e.kind == Kind::kImuTick) {
    if (e.t - last_scan_ > cfg_.gap_degraded) return go(LioState::kDegraded, LioReason::kLidarGapDegraded);
  }
  return unchanged();
}

Transition LioLifecycle::on_degraded(const LioEvent& e) noexcept {
  using Kind = LioEvent::Kind;
  if (sigma_exceeded(e.position_sigma_m, cfg_.position_sigma_lost_m)) {
    return go(LioState::kLost, LioReason::kCovarianceExceeded);
  }
  if (e.kind == Kind::kScanGood) {
    degenerate_since_.reset();
    if (++good_run_ >= cfg_.confirm_scans) return go(LioState::kTracking, LioReason::kConfirmationReached);
  } else if (is_bad_scan(e.kind)) {
    good_run_ = 0;
    if (!degenerate_since_) degenerate_since_ = e.t;
    if (e.t - *degenerate_since_ > cfg_.degeneracy_lost) return go(LioState::kLost, LioReason::kDegeneracyPersisted);
  } else if (e.kind == Kind::kImuTick) {
    if (e.t - last_scan_ > cfg_.gap_lost) return go(LioState::kLost, LioReason::kLidarGapLost);
  }
  return unchanged();
}

Transition LioLifecycle::on_lost(const LioEvent& e) noexcept {
  if (e.kind == LioEvent::Kind::kScanGood) return go(LioState::kRestarting, LioReason::kGeometryReturned);
  return unchanged();
}

Transition LioLifecycle::on_restarting(const LioEvent& e) noexcept {
  if (e.kind == LioEvent::Kind::kRestartSeeded) return go(LioState::kInitializing, LioReason::kRestartSeeded);
  return unchanged();
}

}  // namespace uavnav::lio

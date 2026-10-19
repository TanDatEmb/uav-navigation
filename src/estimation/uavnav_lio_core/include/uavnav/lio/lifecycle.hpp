#pragma once

// D32 boundary marker (SYSTEM_DESIGN §3.5): every source file of the LIO backend ends with
// `#ifdef UAVNAV_LIO_LIFECYCLE_HPP_INCLUDED` / `#error`, so including this header into the backend, even
// through another header, fails the build. The frontend is the single writer of the lifecycle.
#define UAVNAV_LIO_LIFECYCLE_HPP_INCLUDED 1

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"

// LIO lifecycle state machine (SYSTEM_DESIGN §3.1, D20, D12 rule 1).
//
// One state enum (LioState), one transition table (kLioTransitions), one writer
// (LioLifecycle::on). Pure logic: no clock reads, no threads, no ROS. All time comes
// from the events' sensor stamps, so a test drives it with synthetic values.
namespace uavnav::lio {

/// Numeric values equal the LioHealth.msg constants (INITIALIZING=0 .. RESTARTING=4).
enum class LioState : std::uint8_t { kInitializing = 0, kTracking = 1, kDegraded = 2, kLost = 3, kRestarting = 4 };

static_assert(static_cast<std::uint8_t>(LioState::kInitializing) == 0);
static_assert(static_cast<std::uint8_t>(LioState::kTracking) == 1);
static_assert(static_cast<std::uint8_t>(LioState::kDegraded) == 2);
static_assert(static_cast<std::uint8_t>(LioState::kLost) == 3);
static_assert(static_cast<std::uint8_t>(LioState::kRestarting) == 4);

enum class LioReason : std::uint8_t {
  kNone,
  kScanAccepted,  // reserved for estimator/health output; never returned by LioLifecycle::on()
  kConfirmationReached,
  kDegenerateScans,
  kScanEmpty,  // reserved for estimator/health output; never returned by LioLifecycle::on()
  kLidarGapDegraded,
  kLidarGapLost,
  kDegeneracyPersisted,
  kCovarianceExceeded,
  kGeometryReturned,
  kRestartSeeded,
  kMapReady,
  kCorrectionRejected  // D31: a run of kScanGood results the output predictor did not apply
};

constexpr std::string_view to_string(LioState s) {
  switch (s) {
    case LioState::kInitializing: return "INITIALIZING";
    case LioState::kTracking: return "TRACKING";
    case LioState::kDegraded: return "DEGRADED";
    case LioState::kLost: return "LOST";
    case LioState::kRestarting: return "RESTARTING";
  }
  return "";
}

constexpr std::string_view to_string(LioReason r) {
  switch (r) {
    case LioReason::kNone: return "NONE";
    case LioReason::kScanAccepted: return "SCAN_ACCEPTED";
    case LioReason::kConfirmationReached: return "CONFIRMATION_REACHED";
    case LioReason::kDegenerateScans: return "DEGENERATE_SCANS";
    case LioReason::kScanEmpty: return "SCAN_EMPTY";
    case LioReason::kLidarGapDegraded: return "LIDAR_GAP_DEGRADED";
    case LioReason::kLidarGapLost: return "LIDAR_GAP_LOST";
    case LioReason::kDegeneracyPersisted: return "DEGENERACY_PERSISTED";
    case LioReason::kCovarianceExceeded: return "COVARIANCE_EXCEEDED";
    case LioReason::kGeometryReturned: return "GEOMETRY_RETURNED";
    case LioReason::kRestartSeeded: return "RESTART_SEEDED";
    case LioReason::kMapReady: return "MAP_READY";
    case LioReason::kCorrectionRejected: return "CORRECTION_REJECTED";
  }
  return "";
}

static_assert(ReasonEnum<LioState>);
static_assert(ReasonEnum<LioReason>);

/// One per scan result or IMU-time tick.
///
/// kScanGood: a scan whose correction the output predictor APPLIED (D31). It always moves last_scan_time() and
/// clears the degenerate run; it counts toward confirm_scans only when fresh (see LioLifecycle::on, D40, AR4).
///
/// kScanCorrectionRejected (D31, D40, SYSTEM_DESIGN §3.5): a kScanGood scan result whose correction the output
/// predictor could not place (OutputPredictor::on_correction: older than its 300 ms buffer, newer than the
/// output, not finite). Its age alone never makes it one (D40). Only a scan the predictor applied counts as a scan
/// with a correction, so this kind is handled exactly like kScanDegenerate by the transition rules (it counts
/// in the TRACKING degenerate run, whose transition then has reason kCorrectionRejected, counts toward
/// "degenerate since > degeneracy_lost" in DEGRADED, breaks the confirmation runs, is ignored in LOST and
/// RESTARTING), but it does NOT advance last_scan_time(): a backend whose results cannot be placed ends in
/// DEGRADED and LOST.
struct LioEvent {
  enum class Kind : std::uint8_t {
    kScanGood,
    kScanDegenerate,
    kScanEmpty,
    kImuTick,
    kMapReady,
    kRestartSeeded,
    kScanCorrectionRejected
  } kind;
  /// IMU sample stamp, or for scan events the scan END time (what the §3.1 gap rule measures from).
  /// Must come from the range-checked converters (time::from_stamp<SensorTag>: non-negative and
  /// at most time::kMaxStampSeconds), so `t - last_scan_time()` cannot overflow int64.
  time::SensorTime t;
  double position_sigma_m{0.0};  ///< sqrt(max diag of position covariance). NaN counts as exceeded.
};

/// Tier (b), loaded from kLioSpecs (see config.hpp). Beta values in parentheses.
struct LifecycleConfig {
  std::uint32_t confirm_scans;      ///< consecutive fresh good scans to enter TRACKING (5)
  std::uint32_t degenerate_scans;   ///< consecutive degenerate/empty/correction-rejected scans TRACKING -> DEGRADED (3)
  time::Duration gap_degraded;      ///< LiDAR gap TRACKING -> DEGRADED (0.25 s)
  time::Duration gap_lost;          ///< LiDAR gap DEGRADED -> LOST (0.5 s)
  time::Duration degeneracy_lost;   ///< degenerate run length DEGRADED -> LOST (1.0 s)
  double position_sigma_lost_m;     ///< position sigma TRACKING/DEGRADED -> LOST (0.5 m)
};

struct Transition {
  LioState before;
  LioState after;
  LioReason reason;
  bool changed;  ///< false when the state did not change (reason is kNone except for the first kMapReady)
};

/// A legal state change. Anything not listed here cannot happen; in particular there is
/// no LOST -> TRACKING edge: every way back to TRACKING goes through RESTARTING,
/// INITIALIZING and a confirmation (SYSTEM_DESIGN §3.1: every entry to TRACKING is confirmed).
struct TransitionEdge {
  LioState from;
  LioState to;
  LioReason reason;
};

inline constexpr std::array kLioTransitions = std::to_array<TransitionEdge>({
    {LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached},
    {LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans},
    {LioState::kTracking, LioState::kDegraded, LioReason::kLidarGapDegraded},
    {LioState::kTracking, LioState::kDegraded, LioReason::kCorrectionRejected},
    {LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached},
    {LioState::kDegraded, LioState::kLost, LioReason::kLidarGapLost},
    {LioState::kDegraded, LioState::kLost, LioReason::kDegeneracyPersisted},
    {LioState::kTracking, LioState::kLost, LioReason::kCovarianceExceeded},
    {LioState::kDegraded, LioState::kLost, LioReason::kCovarianceExceeded},
    {LioState::kLost, LioState::kRestarting, LioReason::kGeometryReturned},
    {LioState::kRestarting, LioState::kInitializing, LioReason::kRestartSeeded},
});

class LioLifecycle {
 public:
  explicit LioLifecycle(const LifecycleConfig& cfg) noexcept : cfg_(cfg) {}

  /// The only writer of the state. Applies at most one transition per event.
  /// Guards (in each state, in this order): position sigma first, then the event itself.
  /// LiDAR gaps are evaluated on kImuTick only; scans never evaluate a gap.
  ///
  /// Freshness guard on confirmation (D40, AR4; SYSTEM_DESIGN §3.1). In INITIALIZING (confirming) and DEGRADED a
  /// kScanGood counts toward confirm_scans only when (newest kImuTick stamp seen - e.t) <= gap_degraded (equal
  /// counts); a stale one sets the run back to 0 (the run is of consecutive fresh scans). It still moves
  /// last_scan_time() and clears the degenerate run, because the output WAS corrected up to e.t. Harm prevented:
  /// confirming TRACKING on an output whose IMU-only horizon already breaks the gap rule; the next tick would
  /// degrade again (O15 flapping), and odometry that old would go to PX4. No IMU tick yet: every scan is fresh.
  ///
  /// Preconditions: every `e.t` was built by the range-checked converters (see LioEvent::t);
  /// a raw or garbage stamp can overflow the gap subtraction (undefined behaviour). "Newest scan"
  /// means most recently received (arrival order): last_scan_time() is plain assignment, so an
  /// out-of-order older stamp moves it backward.
  Transition on(const LioEvent& e) noexcept;

  LioState state() const noexcept { return state_; }
  /// Stamp of the most recently received scan event of kind good, degenerate or empty (never
  /// kScanCorrectionRejected, D31); the reference for gap checks.
  time::SensorTime last_scan_time() const noexcept { return last_scan_; }

 private:
  Transition unchanged() const noexcept { return Transition{state_, state_, LioReason::kNone, false}; }
  Transition go(LioState to, LioReason reason) noexcept;
  Transition on_initializing(const LioEvent& e) noexcept;
  Transition on_tracking(const LioEvent& e) noexcept;
  Transition on_degraded(const LioEvent& e) noexcept;
  Transition on_lost(const LioEvent& e) noexcept;
  Transition on_restarting(const LioEvent& e) noexcept;
  /// The freshness guard (see on()): a kScanGood whose end is at most gap_degraded before the newest tick.
  bool fresh(const LioEvent& e) const noexcept { return last_imu_t_ - e.t <= cfg_.gap_degraded; }

  /// Progress inside INITIALIZING. Reset to kWaitingForMap by every state change.
  enum class InitPhase : std::uint8_t { kWaitingForMap, kConfirming };

  LifecycleConfig cfg_;
  LioState state_{LioState::kInitializing};  // the one and only state
  time::SensorTime last_scan_{};
  time::SensorTime last_imu_t_{};  // newest kImuTick stamp seen (data, not a flag); never reset by go()

  // Counters and time points below are data about recent events, not state. Each is reset
  // by go() (a state change), except degenerate_since_, which survives TRACKING -> DEGRADED
  // because that transition does not end the degenerate run that caused it.
  InitPhase init_phase_{InitPhase::kWaitingForMap};    // INITIALIZING only: scans count once the map is ready
  std::uint32_t good_run_{0};                         // INITIALIZING (map ready) / DEGRADED: consecutive fresh good scans
  std::uint32_t bad_run_{0};                          // TRACKING: consecutive degenerate/empty/rejected scans
  std::optional<time::SensorTime> degenerate_since_;  // first degenerate/empty/rejected scan of the current run
};

}  // namespace uavnav::lio

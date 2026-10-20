#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "uavnav/core/time.hpp"
#include "uavnav/lio/lifecycle.hpp"

using namespace uavnav;
using namespace uavnav::lio;
using Kind = LioEvent::Kind;

namespace {

constexpr LifecycleConfig kCfg{5,   3, time::milliseconds(250), time::milliseconds(500), time::seconds(1), 0.5,
                               time::seconds(2)};
/// kCfg with the largest restart dwell (30 s): for tests of the confirmation rules that run longer than 2 s.
constexpr LifecycleConfig kLongDwellCfg{5,   3, time::milliseconds(250), time::milliseconds(500), time::seconds(1),
                                        0.5, time::seconds(30)};

// Synthetic sensor time: no wall clock anywhere.
constexpr time::SensorTime T(std::int64_t ms) { return time::SensorTime{ms * 1'000'000}; }

LioEvent Ev(Kind k, std::int64_t ms, double sigma = 0.0) { return LioEvent{k, T(ms), sigma}; }

void ExpectChange(const Transition& tr, LioState before, LioState after, LioReason reason) {
  EXPECT_EQ(tr.before, before) << to_string(tr.before);
  EXPECT_EQ(tr.after, after) << to_string(tr.after);
  EXPECT_EQ(tr.reason, reason) << to_string(tr.reason);
  EXPECT_TRUE(tr.changed);
}

void ExpectUnchanged(const Transition& tr, LioState state) {
  EXPECT_EQ(tr.before, state);
  EXPECT_EQ(tr.after, state);
  EXPECT_EQ(tr.reason, LioReason::kNone);
  EXPECT_FALSE(tr.changed);
}

// Drives a lifecycle to a given state with scan stamps at 100 ms spacing starting at `t0` ms.
// Returns the stamp (ms) of the last scan fed.
struct Rig {
  explicit Rig(const LifecycleConfig& cfg = kCfg) : lc(cfg) {}
  LioLifecycle lc;
  std::int64_t now_ms{0};

  Transition Scan(Kind k, double sigma = 0.0) {
    now_ms += 100;
    return lc.on(Ev(k, now_ms, sigma));
  }
  void ToTracking() {
    ASSERT_EQ(lc.state(), LioState::kInitializing);
    lc.on(Ev(Kind::kMapReady, now_ms));
    for (int i = 0; i < 5; ++i) Scan(Kind::kScanGood);
    ASSERT_EQ(lc.state(), LioState::kTracking);
  }
  void ToDegraded() {
    ToTracking();
    for (int i = 0; i < 3; ++i) Scan(Kind::kScanDegenerate);
    ASSERT_EQ(lc.state(), LioState::kDegraded);
  }
  void ToLost() {
    ToDegraded();
    lc.on(Ev(Kind::kImuTick, now_ms + 600));
    ASSERT_EQ(lc.state(), LioState::kLost);
  }
  /// One 10 Hz scan of kind `k` with its 19 IMU ticks (5 ms) before it; returns the scan's transition, or the
  /// first tick's that changed the state.
  Transition PacedScan(Kind k) {
    for (int i = 1; i < 20; ++i) {
      const Transition tick = lc.on(Ev(Kind::kImuTick, now_ms + 5 * i));
      if (tick.changed) {
        now_ms += 5 * i;
        return tick;
      }
    }
    return Scan(k);
  }
  /// LOST, then paced non-empty scans from 0.1 s after the LOST tick: RESTARTING at the IMU tick restart_dwell
  /// (2 s) after the first of them.
  void ToRestarting() {
    ToLost();
    now_ms += 600;  // the LOST tick
    for (int i = 0; i < 25 && lc.state() == LioState::kLost; ++i) PacedScan(Kind::kScanGood);
    ASSERT_EQ(lc.state(), LioState::kRestarting);
  }
};

}  // namespace

// --- names and numbering ---------------------------------------------------------------

TEST(LioLifecycleNames, StateValuesMatchLioHealthConstants) {
  static_assert(static_cast<std::uint8_t>(LioState::kInitializing) == 0);
  static_assert(static_cast<std::uint8_t>(LioState::kTracking) == 1);
  static_assert(static_cast<std::uint8_t>(LioState::kDegraded) == 2);
  static_assert(static_cast<std::uint8_t>(LioState::kLost) == 3);
  static_assert(static_cast<std::uint8_t>(LioState::kRestarting) == 4);
  SUCCEED();
}

TEST(LioLifecycleNames, StateNames) {
  static_assert(ReasonEnum<LioState>);
  EXPECT_EQ(to_string(LioState::kInitializing), "INITIALIZING");
  EXPECT_EQ(to_string(LioState::kTracking), "TRACKING");
  EXPECT_EQ(to_string(LioState::kDegraded), "DEGRADED");
  EXPECT_EQ(to_string(LioState::kLost), "LOST");
  EXPECT_EQ(to_string(LioState::kRestarting), "RESTARTING");
}

TEST(LioLifecycleNames, ReasonNames) {
  static_assert(ReasonEnum<LioReason>);
  EXPECT_EQ(to_string(LioReason::kNone), "NONE");
  EXPECT_EQ(to_string(LioReason::kScanAccepted), "SCAN_ACCEPTED");
  EXPECT_EQ(to_string(LioReason::kConfirmationReached), "CONFIRMATION_REACHED");
  EXPECT_EQ(to_string(LioReason::kDegenerateScans), "DEGENERATE_SCANS");
  EXPECT_EQ(to_string(LioReason::kScanEmpty), "SCAN_EMPTY");
  EXPECT_EQ(to_string(LioReason::kLidarGapDegraded), "LIDAR_GAP_DEGRADED");
  EXPECT_EQ(to_string(LioReason::kLidarGapLost), "LIDAR_GAP_LOST");
  EXPECT_EQ(to_string(LioReason::kDegeneracyPersisted), "DEGENERACY_PERSISTED");
  EXPECT_EQ(to_string(LioReason::kCovarianceExceeded), "COVARIANCE_EXCEEDED");
  EXPECT_EQ(to_string(LioReason::kStableDataDwell), "STABLE_DATA_DWELL");
  EXPECT_EQ(to_string(LioReason::kRestartSeeded), "RESTART_SEEDED");
  EXPECT_EQ(to_string(LioReason::kMapReady), "MAP_READY");
  EXPECT_EQ(to_string(LioReason::kCorrectionRejected), "CORRECTION_REJECTED");
  EXPECT_EQ(to_string(LioReason::kReinitNotConfirmed), "REINIT_NOT_CONFIRMED");
  EXPECT_EQ(to_string(LioReason::kRestartRejected), "RESTART_REJECTED");
}

TEST(LioLifecycleTable, NoEdgeFromLostToTracking) {
  for (const auto& edge : kLioTransitions) {
    EXPECT_FALSE(edge.from == LioState::kLost && edge.to == LioState::kTracking);
  }
  EXPECT_EQ(kLioTransitions.size(), 13U);
}

// D41, AR4: LIO never gives up. Every state has a way out (no terminal state), LOST leaves only to RESTARTING (no
// LOST -> TRACKING, LIO-4), and the only edges of the recovery chain are the two dwell rules and the restart answer.
TEST(LioLifecycleTable, NoTerminalStateAndNoLostToTracking) {
  for (const LioState s : {LioState::kInitializing, LioState::kTracking, LioState::kDegraded, LioState::kLost,
                           LioState::kRestarting}) {
    std::size_t out = 0;
    for (const auto& edge : kLioTransitions) out += edge.from == s ? 1U : 0U;
    EXPECT_GE(out, 1U) << to_string(s) << " is terminal";
  }
  std::vector<TransitionEdge> from_lost;
  for (const auto& edge : kLioTransitions) {
    if (edge.from == LioState::kLost) from_lost.push_back(edge);
    EXPECT_FALSE(edge.from == LioState::kLost && edge.to == LioState::kTracking);
  }
  ASSERT_EQ(from_lost.size(), 1U);
  EXPECT_EQ(from_lost[0].to, LioState::kRestarting);
  EXPECT_EQ(from_lost[0].reason, LioReason::kStableDataDwell);
  std::size_t reinit = 0;
  for (const auto& edge : kLioTransitions) {
    if (edge.reason != LioReason::kReinitNotConfirmed) continue;
    ++reinit;
    EXPECT_EQ(edge.from, LioState::kInitializing);
    EXPECT_EQ(edge.to, LioState::kLost);
  }
  EXPECT_EQ(reinit, 1U);
  // Review T3 I2: a seed the backend refused returns to LOST, so the next attempt waits a full dwell.
  std::size_t refused = 0;
  for (const auto& edge : kLioTransitions) {
    if (edge.reason != LioReason::kRestartRejected) continue;
    ++refused;
    EXPECT_EQ(edge.from, LioState::kRestarting);
    EXPECT_EQ(edge.to, LioState::kLost);
  }
  EXPECT_EQ(refused, 1U);
}

// D31: the one new edge, TRACKING -> DEGRADED for a run of corrections the output predictor rejected.
TEST(LioLifecycleTable, CorrectionRejectedIsATrackingToDegradedEdge) {
  std::size_t n = 0;
  for (const auto& edge : kLioTransitions) {
    if (edge.reason != LioReason::kCorrectionRejected) continue;
    ++n;
    EXPECT_EQ(edge.from, LioState::kTracking);
    EXPECT_EQ(edge.to, LioState::kDegraded);
  }
  EXPECT_EQ(n, 1U);
}

// --- one test per table row -------------------------------------------------------------

// Row 1: INITIALIZING + kMapReady -> INITIALIZING (counter starts).
TEST(LioLifecycle, StartsInitializing) {
  LioLifecycle lc(kCfg);
  EXPECT_EQ(lc.state(), LioState::kInitializing);
  EXPECT_EQ(lc.last_scan_time(), T(0));
}

TEST(LioLifecycle, MapReadyInInitializingStaysAndStartsCounter) {
  LioLifecycle lc(kCfg);
  const auto tr = lc.on(Ev(Kind::kMapReady, 100));
  EXPECT_EQ(tr.before, LioState::kInitializing);
  EXPECT_EQ(tr.after, LioState::kInitializing);
  EXPECT_EQ(tr.reason, LioReason::kMapReady);
  EXPECT_FALSE(tr.changed);
  EXPECT_EQ(lc.state(), LioState::kInitializing);
}

TEST(LioLifecycle, GoodScansBeforeMapReadyAreIgnored) {
  Rig r;
  for (int i = 0; i < 20; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  // The ignored scans did not pre-load the counter: confirmation needs five scans AFTER kMapReady.
  r.lc.on(Ev(Kind::kMapReady, r.now_ms));
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached);
}

// Row 2: INITIALIZING, confirm_scans consecutive good scans after kMapReady -> TRACKING.
TEST(LioLifecycle, ConfirmScansAfterMapReadyReachTracking) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached);
  EXPECT_EQ(r.lc.state(), LioState::kTracking);
}

TEST(LioLifecycle, DegenerateOrEmptyScanBreaksInitializingConfirmation) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kInitializing);
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kInitializing);
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached);
}

// Review T3 m6: a repeated kMapReady keeps the restart-dwell clock of the first one.
TEST(LioLifecycle, RepeatedMapReadyDoesNotRestartTheDwellClock) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, 1'500)), LioState::kInitializing);
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, 2'000)), LioState::kInitializing);
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, 2'001)), LioState::kInitializing, LioState::kLost,
               LioReason::kReinitNotConfirmed);
}

TEST(LioLifecycle, RepeatedMapReadyDoesNotResetTheCounter) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
  ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, r.now_ms)), LioState::kInitializing);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached);
}

TEST(LioLifecycle, InitializingIgnoresImuGapAndSigma) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  // 1.9 s without a scan and a sigma of 99 m: neither is a rule of INITIALIZING (only the restart dwell is).
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, 1'900, 99.0)), LioState::kInitializing);
  ExpectUnchanged(r.lc.on(Ev(Kind::kRestartSeeded, 1'905)), LioState::kInitializing);
  // Before the map is ready nothing is timed: no data, no attempt to judge.
  Rig waiting;
  ExpectUnchanged(waiting.lc.on(Ev(Kind::kImuTick, 600'000, 99.0)), LioState::kInitializing);
}

// Row 3: TRACKING, degenerate_scans consecutive degenerate scans -> DEGRADED.
TEST(LioLifecycle, TrackingDegradesAfterConsecutiveDegenerateScans) {
  Rig r;
  r.ToTracking();
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kTracking);
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kTracking);
  ExpectChange(r.Scan(Kind::kScanDegenerate), LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
}

TEST(LioLifecycle, GoodScanResetsTrackingDegenerateRun) {
  Rig r;
  r.ToTracking();
  r.Scan(Kind::kScanDegenerate);
  r.Scan(Kind::kScanDegenerate);
  ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kTracking);
  r.Scan(Kind::kScanDegenerate);
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kTracking);
  ExpectChange(r.Scan(Kind::kScanDegenerate), LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
}

TEST(LioLifecycle, EmptyScansCountAsDegenerate) {
  Rig r;
  r.ToTracking();
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kTracking);
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kTracking);
  ExpectChange(r.Scan(Kind::kScanEmpty), LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
}

TEST(LioLifecycle, MixedDegenerateAndEmptyScansCountTogether) {
  Rig r;
  r.ToTracking();
  r.Scan(Kind::kScanEmpty);
  r.Scan(Kind::kScanDegenerate);
  ExpectChange(r.Scan(Kind::kScanEmpty), LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
}

// Row 4: TRACKING, kImuTick with t - last_scan_time > gap_degraded -> DEGRADED.
TEST(LioLifecycle, TrackingGapDegradesOnImuTickStrictlyAboveThreshold) {
  Rig r;
  r.ToTracking();
  const std::int64_t last = r.now_ms;
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, last + 250)), LioState::kTracking);
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, last + 251)), LioState::kTracking, LioState::kDegraded,
               LioReason::kLidarGapDegraded);
}

TEST(LioLifecycle, ScansNeverEvaluateAGap) {
  Rig r;
  r.ToTracking();
  // A good scan arriving 2 s late is a good scan, not a gap: gaps are judged on IMU ticks.
  r.now_ms += 2'000;
  ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, r.now_ms)), LioState::kTracking);
}

// Row 5: DEGRADED, confirm_scans consecutive good scans -> TRACKING.
TEST(LioLifecycle, DegradedReturnsToTrackingAfterConfirmation) {
  Rig r;
  r.ToDegraded();
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
}

TEST(LioLifecycle, DegenerateScanResetsDegradedConfirmation) {
  Rig r;
  r.ToDegraded();
  for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kDegraded);
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
}

TEST(LioLifecycle, ConfirmationCounterRestartsAfterEachTransition) {
  Rig r;
  r.ToDegraded();
  for (int i = 0; i < 5; ++i) r.Scan(Kind::kScanGood);
  ASSERT_EQ(r.lc.state(), LioState::kTracking);
  for (int i = 0; i < 3; ++i) r.Scan(Kind::kScanDegenerate);
  ASSERT_EQ(r.lc.state(), LioState::kDegraded);
  // The earlier five good scans must not carry over: a full new confirmation is needed.
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
}

// Row 6: DEGRADED, kImuTick with gap > gap_lost -> LOST.
TEST(LioLifecycle, DegradedGapLostOnImuTick) {
  Rig r;
  r.ToDegraded();
  const std::int64_t last = r.now_ms;
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, last + 500)), LioState::kDegraded);
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, last + 501)), LioState::kDegraded, LioState::kLost,
               LioReason::kLidarGapLost);
}

// Row 7: DEGRADED, degenerate since > degeneracy_lost -> LOST. Measured from the first
// degenerate scan of the run (sensor time).
TEST(LioLifecycle, DegradedDegeneracyPersistedMeasuredFromFirstDegenerateScan) {
  LioLifecycle lc(kCfg);
  lc.on(Ev(Kind::kMapReady, 9'900));
  for (int i = 0; i < 5; ++i) lc.on(Ev(Kind::kScanGood, 10'000 + 100 * i));  // TRACKING at t=10.4 s
  ASSERT_EQ(lc.state(), LioState::kTracking);
  // First degenerate scan at 10.5 s; the run is 3 scans long when DEGRADED is entered.
  lc.on(Ev(Kind::kScanDegenerate, 10'500));
  lc.on(Ev(Kind::kScanDegenerate, 10'600));
  ExpectChange(lc.on(Ev(Kind::kScanDegenerate, 10'700)), LioState::kTracking, LioState::kDegraded,
               LioReason::kDegenerateScans);
  for (std::int64_t ms = 10'800; ms <= 11'500; ms += 100) {
    ExpectUnchanged(lc.on(Ev(Kind::kScanDegenerate, ms)), LioState::kDegraded);  // 11.5 - 10.5 = 1.0 s, not above
  }
  ExpectChange(lc.on(Ev(Kind::kScanDegenerate, 11'600)), LioState::kDegraded, LioState::kLost,
               LioReason::kDegeneracyPersisted);
}

TEST(LioLifecycle, GoodScanRestartsTheDegenerateRunClock) {
  Rig r;
  r.ToDegraded();
  const std::int64_t run1_start = r.now_ms;
  r.Scan(Kind::kScanGood);  // clears the run
  // New run starts 2 s later; 0.9 s into it must not be LOST although > 1.0 s after run 1.
  r.now_ms = run1_start + 2'000;
  r.lc.on(Ev(Kind::kScanDegenerate, r.now_ms));
  ExpectUnchanged(r.lc.on(Ev(Kind::kScanDegenerate, r.now_ms + 900)), LioState::kDegraded);
  ExpectChange(r.lc.on(Ev(Kind::kScanEmpty, r.now_ms + 1'001)), LioState::kDegraded, LioState::kLost,
               LioReason::kDegeneracyPersisted);
}

// Row 8: TRACKING/DEGRADED, any event with sigma > position_sigma_lost_m -> LOST.
TEST(LioLifecycle, CovarianceExceededInTrackingOnAnyEvent) {
  for (const Kind k : {Kind::kScanGood, Kind::kScanDegenerate, Kind::kScanEmpty, Kind::kImuTick, Kind::kMapReady,
                       Kind::kRestartSeeded, Kind::kScanCorrectionRejected, Kind::kRestartRejected}) {
    Rig r;
    r.ToTracking();
    ExpectChange(r.lc.on(Ev(k, r.now_ms + 10, 0.51)), LioState::kTracking, LioState::kLost,
                 LioReason::kCovarianceExceeded);
  }
}

TEST(LioLifecycle, CovarianceExceededInDegradedOnAnyEvent) {
  for (const Kind k : {Kind::kScanGood, Kind::kScanDegenerate, Kind::kScanEmpty, Kind::kImuTick, Kind::kMapReady,
                       Kind::kRestartSeeded, Kind::kScanCorrectionRejected, Kind::kRestartRejected}) {
    Rig r;
    r.ToDegraded();
    ExpectChange(r.lc.on(Ev(k, r.now_ms + 10, 0.51)), LioState::kDegraded, LioState::kLost,
                 LioReason::kCovarianceExceeded);
  }
}

TEST(LioLifecycle, SigmaAtThresholdIsNotExceeded) {
  Rig r;
  r.ToTracking();
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 10, 0.5)), LioState::kTracking);
}

TEST(LioLifecycle, NotANumberSigmaFailsClosed) {
  Rig r;
  r.ToTracking();
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 10, std::numeric_limits<double>::quiet_NaN())),
               LioState::kTracking, LioState::kLost, LioReason::kCovarianceExceeded);
}

TEST(LioLifecycle, SigmaIsIgnoredOutsideTrackingAndDegraded) {
  Rig r;
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, 10, 9.0)), LioState::kInitializing);
  r.ToLost();
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 1'000, 9.0)), LioState::kLost);
}

// --- D41, AR4: LOST waits, without limit, for non-empty scans arriving without a gap for restart_dwell -------

// Row 9a: LOST is a wait. Empty scans (no geometry at all) for 600 s, with 200 Hz ticks: LOST throughout.
TEST(LioLifecycle, LostWaitsWithoutLimitOnEmptyScans) {
  Rig r;
  r.ToLost();
  r.now_ms += 600;
  for (int i = 0; i < 6'000; ++i) {  // 600 s of 10 Hz scans
    for (int k = 1; k < 20; ++k) ASSERT_FALSE(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5 * k)).changed);
    ASSERT_FALSE(r.Scan(Kind::kScanEmpty).changed) << r.now_ms;
  }
  EXPECT_EQ(r.lc.state(), LioState::kLost);
  // An IMU-only stretch (the LiDAR gone) is a wait too: 600 s of ticks.
  for (std::int64_t ms = r.now_ms; ms <= r.now_ms + 600'000; ms += 5) {
    ASSERT_FALSE(r.lc.on(Ev(Kind::kImuTick, ms)).changed) << ms;
  }
  EXPECT_EQ(r.lc.state(), LioState::kLost);
}

// Row 9: LOST -> RESTARTING exactly when restart_dwell has passed since the first non-empty scan, with no gap and no
// empty scan in between. Good, degenerate and rejected-correction scans all count as non-empty: the result of the
// registration against the frozen map is not read (critique §1).
TEST(LioLifecycle, NonEmptyScansForTheDwellRestart) {
  for (const Kind k : {Kind::kScanGood, Kind::kScanDegenerate}) {
    Rig r;
    r.ToLost();
    const std::int64_t first = r.now_ms + 700;  // the first non-empty scan after LOST: the dwell starts here
    r.now_ms = first - 100;
    for (int i = 0; i < 20; ++i) {  // scans 0 .. 1.9 s after the first: never long enough
      for (int t = 1; t < 20; ++t) ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5 * t)), LioState::kLost);
      ExpectUnchanged(r.Scan(i % 3 == 2 ? Kind::kScanCorrectionRejected : k), LioState::kLost);
    }
    ASSERT_EQ(r.now_ms, first + 1'900);
    // The tick 1 ns before first + restart_dwell does not restart; the tick at first + restart_dwell does (>=).
    ExpectUnchanged(r.lc.on(LioEvent{Kind::kImuTick, T(first + 1'995), 0.0}), LioState::kLost);
    ExpectUnchanged(r.lc.on(LioEvent{Kind::kImuTick, T(first + 2'000) - time::nanoseconds(1), 0.0}), LioState::kLost);
    ExpectChange(r.lc.on(Ev(Kind::kImuTick, first + 2'000)), LioState::kLost, LioState::kRestarting,
                 LioReason::kStableDataDwell);
  }
  // A scan event can complete the dwell too (any event is evaluated).
  Rig r;
  r.ToLost();
  r.lc.on(Ev(Kind::kScanDegenerate, r.now_ms + 700));
  r.lc.on(Ev(Kind::kImuTick, r.now_ms + 750));
  ExpectChange(r.lc.on(Ev(Kind::kScanGood, r.now_ms + 700 + 2'000)), LioState::kLost, LioState::kRestarting,
               LioReason::kStableDataDwell);
}

// An empty scan, or an IMU tick with a LiDAR gap above gap_degraded, inside the dwell clears it: the wait restarts at
// the next non-empty scan. A gap of exactly gap_degraded does not clear it.
TEST(LioLifecycle, AnEmptyScanOrAGapInsideTheDwellRestartsTheWait) {
  // An empty scan 1.5 s into the dwell.
  {
    Rig r;
    r.ToLost();
    const std::int64_t first = r.now_ms + 700;
    r.now_ms = first - 100;
    for (int i = 0; i < 15; ++i) ExpectUnchanged(r.PacedScan(Kind::kScanGood), LioState::kLost);
    ExpectUnchanged(r.PacedScan(Kind::kScanEmpty), LioState::kLost);  // at first + 1.5 s
    ASSERT_EQ(r.now_ms, first + 1'500);
    const std::int64_t again = first + 1'600;  // the next non-empty scan: the wait starts again here
    Transition tr{};
    while (r.lc.state() == LioState::kLost && r.now_ms < again + 3'000) tr = r.PacedScan(Kind::kScanDegenerate);
    ExpectChange(tr, LioState::kLost, LioState::kRestarting, LioReason::kStableDataDwell);
    EXPECT_EQ(r.now_ms, again + 2'000) << "restarted at the tick restart_dwell after the scan after the empty one";
  }
  // A LiDAR gap (> gap_degraded on a tick) 1 s into the dwell.
  {
    Rig r;
    r.ToLost();
    const std::int64_t first = r.now_ms + 700;
    r.now_ms = first - 100;
    for (int i = 0; i < 11; ++i) ExpectUnchanged(r.PacedScan(Kind::kScanGood), LioState::kLost);
    ASSERT_EQ(r.now_ms, first + 1'000);
    // Exactly gap_degraded after the last scan: not a gap.
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, first + 1'250)), LioState::kLost);
    // The scans resume after exactly gap_degraded: the dwell still runs from `first`.
    r.now_ms = first + 1'250;
    for (int i = 0; i < 7; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kLost);  // up to first + 1.95 s
    ExpectChange(r.lc.on(Ev(Kind::kImuTick, first + 2'000)), LioState::kLost, LioState::kRestarting,
                 LioReason::kStableDataDwell);
  }
  {
    Rig r;
    r.ToLost();
    const std::int64_t first = r.now_ms + 700;
    r.now_ms = first - 100;
    for (int i = 0; i < 11; ++i) ExpectUnchanged(r.PacedScan(Kind::kScanGood), LioState::kLost);
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, first + 1'251)), LioState::kLost);  // a gap: the wait is cleared
    // No restart at first + 2 s although scans resumed at once; the dwell runs from the first scan after the gap.
    r.now_ms = first + 1'300;
    const std::int64_t again = r.now_ms;
    r.lc.on(Ev(Kind::kScanGood, again));
    for (std::int64_t ms = again + 5; ms < again + 2'000; ms += 5) {
      ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, ms)), LioState::kLost);
      if ((ms - again) % 100 == 0) ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, ms)), LioState::kLost);
    }
    ExpectChange(r.lc.on(Ev(Kind::kImuTick, again + 2'000)), LioState::kLost, LioState::kRestarting,
                 LioReason::kStableDataDwell);
  }
  // Other events (a map-ready result, a restart answer) neither start the wait nor leave LOST.
  {
    Rig r;
    r.ToLost();
    ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, r.now_ms + 650)), LioState::kLost);
    ExpectUnchanged(r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms + 650)), LioState::kLost);
  }
}

// F27: LOST + 40 good scans: RESTARTING after the dwell, never TRACKING; RESTARTING stays without its answer.
TEST(LioLifecycle, LostNeverJumpsToTracking) {
  Rig r;
  r.ToLost();
  r.now_ms += 600;
  for (int i = 0; i < 40; ++i) {
    r.PacedScan(Kind::kScanGood);
    EXPECT_NE(r.lc.state(), LioState::kTracking);
  }
  EXPECT_EQ(r.lc.state(), LioState::kRestarting);
}

// Row 9b: INITIALIZING with a ready map but no confirmation restart_dwell after kMapReady -> LOST (the attempt
// answered itself). The same rule at boot; strictly above the dwell.
TEST(LioLifecycle, MapReadyButNotConfirmedWithinTheDwellReturnsToLost) {
  // After a restart: degenerate scans in the new map.
  {
    Rig r;
    r.ToRestarting();
    ExpectChange(r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms)), LioState::kRestarting, LioState::kInitializing,
                 LioReason::kRestartSeeded);
    const std::int64_t ready = r.now_ms + 100;
    r.lc.on(Ev(Kind::kMapReady, ready));
    r.now_ms = ready;
    for (int i = 0; i < 20; ++i) ExpectUnchanged(r.PacedScan(Kind::kScanDegenerate), LioState::kInitializing);
    ASSERT_EQ(r.now_ms, ready + 2'000);
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, ready + 2'000)), LioState::kInitializing);  // exactly the dwell
    ExpectChange(r.lc.on(LioEvent{Kind::kImuTick, T(ready + 2'000) + time::nanoseconds(1), 0.0}),
                 LioState::kInitializing, LioState::kLost, LioReason::kReinitNotConfirmed);
  }
  // At boot, the same rule: good scans that never make a run of five (a degenerate one every fifth).
  {
    Rig r;
    r.lc.on(Ev(Kind::kMapReady, 0));
    Transition tr{};
    for (int i = 0; i < 30 && r.lc.state() == LioState::kInitializing; ++i) {
      tr = r.PacedScan(i % 5 == 4 ? Kind::kScanDegenerate : Kind::kScanGood);
    }
    ExpectChange(tr, LioState::kInitializing, LioState::kLost, LioReason::kReinitNotConfirmed);
    EXPECT_EQ(r.now_ms, 2'005) << "the first tick strictly after the dwell";
  }
  // Confirmation inside the dwell: TRACKING, and the dwell is over (TRACKING has no such rule).
  {
    Rig r;
    r.lc.on(Ev(Kind::kMapReady, 0));
    r.now_ms = 1'400;
    for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
    ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking,
                 LioReason::kConfirmationReached);
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5)), LioState::kTracking);
    for (int i = 0; i < 30; ++i) ExpectUnchanged(r.PacedScan(Kind::kScanGood), LioState::kTracking);
  }
}

// Row 10: RESTARTING + kRestartSeeded -> INITIALIZING.
TEST(LioLifecycle, RestartSeededReturnsToInitializing) {
  Rig r;
  r.ToRestarting();
  ExpectChange(r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms + 10)), LioState::kRestarting, LioState::kInitializing,
               LioReason::kRestartSeeded);
}

// Review T3 I2: a refused seed (the backend's kRestartRejected) is RESTARTING -> LOST (RESTART_REJECTED); the
// next attempt waits restart_dwell of stable data again, so a backend that refuses every seed gets one per dwell.
TEST(LioLifecycle, RestartRejectedReturnsToLostAndWaitsADwell) {
  Rig r;
  r.ToRestarting();
  const std::int64_t refused_at = r.now_ms + 20;
  ExpectChange(r.lc.on(Ev(Kind::kRestartRejected, refused_at)), LioState::kRestarting, LioState::kLost,
               LioReason::kRestartRejected);
  r.now_ms = refused_at;
  const std::int64_t first = r.now_ms + 100;  // the first scan after the refusal: the wait starts there
  Transition tr{};
  for (int i = 0; i < 30 && r.lc.state() == LioState::kLost; ++i) tr = r.PacedScan(Kind::kScanDegenerate);
  ExpectChange(tr, LioState::kLost, LioState::kRestarting, LioReason::kStableDataDwell);
  EXPECT_EQ(r.now_ms, first + 2'000);
  // kRestartRejected outside RESTARTING changes nothing.
  Rig t;
  t.ToTracking();
  ExpectUnchanged(t.lc.on(Ev(Kind::kRestartRejected, t.now_ms + 5)), LioState::kTracking);
  Rig l;
  l.ToLost();
  ExpectUnchanged(l.lc.on(Ev(Kind::kRestartRejected, l.now_ms + 650)), LioState::kLost);
}

TEST(LioLifecycle, RestartingIgnoresOtherEvents) {
  Rig r;
  r.ToRestarting();
  ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kRestarting);
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kRestarting);
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kRestarting);
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5'000)), LioState::kRestarting);
  ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, r.now_ms + 5'005)), LioState::kRestarting);
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 600'000, 99.0)), LioState::kRestarting);
}

TEST(LioLifecycle, RestartRequiresMapReadyAndFullConfirmationAgain) {
  Rig r;
  r.ToRestarting();
  r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms + 10));
  ASSERT_EQ(r.lc.state(), LioState::kInitializing);
  // The old map-ready and the old counters are gone: scans before the new kMapReady are ignored.
  for (int i = 0; i < 10; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  r.lc.on(Ev(Kind::kMapReady, r.now_ms));
  for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking, LioReason::kConfirmationReached);
}

// --- named tests from the brief -----------------------------------------------------------

// F22: IMU samples alone drive TRACKING -> DEGRADED -> LOST.
TEST(LioLifecycle, LidarGapOnImuTicksAloneReachesLost) {
  LioLifecycle lc(kCfg);
  lc.on(Ev(Kind::kMapReady, 9'000));
  for (int i = 0; i < 5; ++i) lc.on(Ev(Kind::kScanGood, 9'600 + 100 * i));
  // The last scan is at t = 10.0 s.
  ASSERT_EQ(lc.state(), LioState::kTracking);
  ASSERT_EQ(lc.last_scan_time(), T(10'000));

  std::int64_t degraded_ms = -1;
  std::int64_t lost_ms = -1;
  for (std::int64_t ms = 10'005; ms <= 11'000 && lost_ms < 0; ms += 5) {
    const auto tr = lc.on(Ev(Kind::kImuTick, ms));
    if (tr.changed && tr.after == LioState::kDegraded) {
      EXPECT_EQ(tr.reason, LioReason::kLidarGapDegraded);
      EXPECT_EQ(degraded_ms, -1);
      degraded_ms = ms;
    } else if (tr.changed && tr.after == LioState::kLost) {
      EXPECT_EQ(tr.reason, LioReason::kLidarGapLost);
      lost_ms = ms;
    }
  }
  EXPECT_EQ(degraded_ms, 10'255);  // first tick with t > 10.25 s
  EXPECT_EQ(lost_ms, 10'505);      // first tick with t > 10.5 s
  EXPECT_EQ(lc.state(), LioState::kLost);
}

TEST(LioLifecycle, EmptyScansCountAsDegenerateNamed) {
  Rig r;
  r.ToTracking();
  r.Scan(Kind::kScanEmpty);
  r.Scan(Kind::kScanEmpty);
  const auto tr = r.Scan(Kind::kScanEmpty);
  ExpectChange(tr, LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
}

// --- last_scan_time -----------------------------------------------------------------------

TEST(LioLifecycle, LastScanTimeFollowsEveryScanKindAndNothingElse) {
  LioLifecycle lc(kCfg);
  lc.on(Ev(Kind::kScanGood, 100));
  EXPECT_EQ(lc.last_scan_time(), T(100));
  lc.on(Ev(Kind::kScanDegenerate, 200));
  EXPECT_EQ(lc.last_scan_time(), T(200));
  lc.on(Ev(Kind::kScanEmpty, 300));
  EXPECT_EQ(lc.last_scan_time(), T(300));
  lc.on(Ev(Kind::kImuTick, 400));
  lc.on(Ev(Kind::kMapReady, 500));
  lc.on(Ev(Kind::kRestartSeeded, 600));
  EXPECT_EQ(lc.last_scan_time(), T(300));
}

TEST(LioLifecycle, EmptyScanRefreshesTheGapReference) {
  Rig r;
  r.ToTracking();
  const std::int64_t last = r.now_ms;
  r.lc.on(Ev(Kind::kScanEmpty, last + 200));
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, last + 400)), LioState::kTracking);  // 0.2 s since the empty scan
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, last + 451)), LioState::kTracking, LioState::kDegraded,
               LioReason::kLidarGapDegraded);
}

TEST(LioLifecycle, TransitionsAlwaysFollowTheTable) {
  // Fuzz-lite: a fixed pseudo-random event sequence; every reported change must be a listed edge.
  LioLifecycle lc(kCfg);
  std::uint32_t seed = 12345;
  std::int64_t ms = 0;
  for (int i = 0; i < 5'000; ++i) {
    seed = seed * 1664525U + 1013904223U;
    const auto k = static_cast<Kind>((seed >> 16) % 8);  // every kind, kScanCorrectionRejected (D31) included
    ms += 5 + static_cast<std::int64_t>((seed >> 8) % 120);
    const double sigma = ((seed >> 4) % 50 == 0) ? 0.9 : 0.1;
    const LioState before = lc.state();
    const auto tr = lc.on(Ev(k, ms, sigma));
    EXPECT_EQ(tr.before, before);
    EXPECT_EQ(tr.after, lc.state());
    EXPECT_EQ(tr.changed, tr.before != tr.after);
    if (tr.changed) {
      bool listed = false;
      for (const auto& edge : kLioTransitions) {
        listed = listed || (edge.from == tr.before && edge.to == tr.after && edge.reason == tr.reason);
      }
      EXPECT_TRUE(listed) << to_string(tr.before) << " -> " << to_string(tr.after) << " " << to_string(tr.reason);
    }
  }
}

// --- D31: a kScanGood whose correction the output predictor rejected --------------------------

TEST(LioLifecycle, CorrectionRejectedCountsLikeDegenerateButKeepsTheGapReference) {
  // TRACKING: three consecutive rejected corrections -> DEGRADED, reason CORRECTION_REJECTED; none of them
  // moves the gap reference.
  {
    Rig r;
    r.ToTracking();
    const time::SensorTime last = r.lc.last_scan_time();
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking);
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking);
    EXPECT_EQ(r.lc.last_scan_time(), last);
    ExpectChange(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking, LioState::kDegraded,
                 LioReason::kCorrectionRejected);
    EXPECT_EQ(r.lc.last_scan_time(), last);
    // DEGRADED: the gap still runs from the last real scan, so IMU ticks reach LOST at last + gap_lost.
    const std::int64_t last_ms = last.ns / 1'000'000;
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, last_ms + 500)), LioState::kDegraded);
    ExpectChange(r.lc.on(Ev(Kind::kImuTick, last_ms + 501)), LioState::kDegraded, LioState::kLost,
                 LioReason::kLidarGapLost);
  }
  // A good scan between them resets the run.
  {
    Rig r;
    r.ToTracking();
    r.Scan(Kind::kScanCorrectionRejected);
    r.Scan(Kind::kScanCorrectionRejected);
    ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kTracking);
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking);
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking);
    ExpectChange(r.Scan(Kind::kScanCorrectionRejected), LioState::kTracking, LioState::kDegraded,
                 LioReason::kCorrectionRejected);
  }
  // Mixed with degenerate scans it counts in the same run; the event that completes the run names the reason.
  {
    Rig r;
    r.ToTracking();
    r.Scan(Kind::kScanCorrectionRejected);
    r.Scan(Kind::kScanCorrectionRejected);
    ExpectChange(r.Scan(Kind::kScanDegenerate), LioState::kTracking, LioState::kDegraded, LioReason::kDegenerateScans);
  }
  // INITIALIZING: breaks the confirmation run.
  {
    Rig r;
    r.lc.on(Ev(Kind::kMapReady, 0));
    for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kInitializing);
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
    ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking,
                 LioReason::kConfirmationReached);
  }
  // DEGRADED: breaks the confirmation run, and counts toward "degenerate since > degeneracy_lost".
  {
    Rig r;
    r.ToDegraded();
    for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kDegraded);
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
    ExpectChange(r.Scan(Kind::kScanGood), LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
  }
  {
    Rig r;
    r.ToDegraded();
    r.Scan(Kind::kScanGood);  // clears the degenerate run
    const std::int64_t first = r.now_ms + 100;
    r.lc.on(Ev(Kind::kScanCorrectionRejected, first));
    ExpectUnchanged(r.lc.on(Ev(Kind::kScanCorrectionRejected, first + 1'000)), LioState::kDegraded);
    ExpectChange(r.lc.on(Ev(Kind::kScanCorrectionRejected, first + 1'001)), LioState::kDegraded, LioState::kLost,
                 LioReason::kDegeneracyPersisted);
  }
  // LOST and RESTARTING ignore it.
  {
    Rig r;
    r.ToLost();
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kLost);
  }
  {
    Rig r;
    r.ToRestarting();
    ExpectUnchanged(r.Scan(Kind::kScanCorrectionRejected), LioState::kRestarting);
  }
}

TEST(LioLifecycle, LastScanTimeIgnoresRejectedCorrections) {
  LioLifecycle lc(kCfg);
  lc.on(Ev(Kind::kScanGood, 100));
  lc.on(Ev(Kind::kScanCorrectionRejected, 900));
  EXPECT_EQ(lc.last_scan_time(), T(100));
}

// --- D40, AR4: the freshness guard on confirmation ---------------------------------------------
//
// A kScanGood (a correction the predictor applied) counts toward confirm_scans only when (newest kImuTick stamp
// seen - scan end) + kScanPeriod <= gap_degraded: the gap rule must still hold when the next scan is as late as this
// one (review T2 I1, the DV1 band). With the beta 0.25 s the boundary is 0.15 s. A stale one still moves the gap
// reference and clears the degenerate run, and it breaks the consecutive confirmation run.

TEST(LioLifecycle, StaleGoodScanMovesTheGapReferenceButNeverConfirms) {
  // DEGRADED: a saturated backend, every result 0.27 s old. Never TRACKING (10 > confirm_scans results), and never
  // LOST: each stale scan moves the gap reference, so the gap on the next tick is 0.37 s < gap_lost.
  {
    Rig r;
    r.ToDegraded();
    const std::int64_t base = r.now_ms;
    for (int i = 1; i <= 10; ++i) {
      const std::int64_t end = base + 100 * i;
      ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, end + 270)), LioState::kDegraded);
      ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, end)), LioState::kDegraded);
      EXPECT_EQ(r.lc.last_scan_time(), T(end)) << i;
    }
    // The gap runs from the newest stale scan end: LOST only once the ticks are gap_lost past it.
    const std::int64_t last = base + 1'000;
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, last + 500)), LioState::kDegraded);
    ExpectChange(r.lc.on(Ev(Kind::kImuTick, last + 501)), LioState::kDegraded, LioState::kLost,
                 LioReason::kLidarGapLost);
  }
  // DEGRADED: a stale good scan clears the degenerate run, like a fresh one.
  {
    Rig r;
    r.ToDegraded();
    const std::int64_t first = r.now_ms - 200;  // first degenerate scan of the run
    ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, first + 700)), LioState::kDegraded);
    ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, first + 400)), LioState::kDegraded);  // 0.3 s old
    ExpectUnchanged(r.lc.on(Ev(Kind::kScanDegenerate, first + 1'001)), LioState::kDegraded);  // a new run
  }
  // DEGRADED: a stale good scan breaks the confirmation run (4 fresh, 1 stale, 4 fresh: not yet).
  {
    Rig r;
    r.ToDegraded();
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
    r.lc.on(Ev(Kind::kImuTick, r.now_ms + 360));
    ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, r.now_ms + 100)), LioState::kDegraded);  // 0.26 s old
    r.now_ms += 360;
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kDegraded);
    ExpectChange(r.Scan(Kind::kScanGood), LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
  }
  // INITIALIZING (map ready): stale good scans never confirm, and one breaks the run. 11 s with a ready map: the
  // 30 s dwell keeps the restart-dwell rule out of this test.
  {
    Rig r(kLongDwellCfg);
    r.lc.on(Ev(Kind::kMapReady, 0));
    for (int i = 1; i <= 10; ++i) {
      r.lc.on(Ev(Kind::kImuTick, 1'000 * i));
      ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, 1'000 * i - 300)), LioState::kInitializing);
    }
    r.now_ms = 10'000;
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
    r.lc.on(Ev(Kind::kImuTick, r.now_ms + 400));
    ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, r.now_ms + 100)), LioState::kInitializing);  // 0.3 s old
    r.now_ms += 400;
    for (int i = 0; i < 4; ++i) ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kInitializing);
    ExpectChange(r.Scan(Kind::kScanGood), LioState::kInitializing, LioState::kTracking,
                 LioReason::kConfirmationReached);
  }
  // The boundary: one nanosecond older than gap_degraded - kScanPeriod (0.15 s) is stale (the fifth scan does not
  // confirm), and so is a scan 0.2 s old, inside gap_degraded (the DV1 band).
  for (const std::int64_t age_ms : {150, 200}) {
    Rig r;
    r.ToDegraded();
    for (int i = 0; i < 4; ++i) r.Scan(Kind::kScanGood);
    const std::int64_t end = r.now_ms + 100;
    r.lc.on(Ev(Kind::kImuTick, end + age_ms));
    const LioEvent stale{Kind::kScanGood, T(end) - time::nanoseconds(age_ms == 150 ? 1 : 0), 0.0};
    ExpectUnchanged(r.lc.on(stale), LioState::kDegraded);
  }
}

TEST(LioLifecycle, FreshGoodScansStillConfirm) {
  // Age exactly gap_degraded - kScanPeriod (0.15 s) counts (the rule is <=), in DEGRADED and in INITIALIZING.
  {
    Rig r;
    r.ToDegraded();
    const std::int64_t base = r.now_ms;
    for (int i = 1; i <= 4; ++i) {
      const std::int64_t end = base + 100 * i;
      ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, end + 150)), LioState::kDegraded);  // gap 0.25 s < gap_lost
      ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, end)), LioState::kDegraded);
    }
    const std::int64_t end = base + 500;
    r.lc.on(Ev(Kind::kImuTick, end + 150));
    ExpectChange(r.lc.on(Ev(Kind::kScanGood, end)), LioState::kDegraded, LioState::kTracking,
                 LioReason::kConfirmationReached);
  }
  {
    Rig r;
    r.lc.on(Ev(Kind::kMapReady, 0));
    for (int i = 1; i <= 4; ++i) {
      r.lc.on(Ev(Kind::kImuTick, 100 * i + 150));
      ExpectUnchanged(r.lc.on(Ev(Kind::kScanGood, 100 * i)), LioState::kInitializing);
    }
    r.lc.on(Ev(Kind::kImuTick, 500 + 150));
    ExpectChange(r.lc.on(Ev(Kind::kScanGood, 500)), LioState::kInitializing, LioState::kTracking,
                 LioReason::kConfirmationReached);
  }
  // Paced 10 Hz scans with 200 Hz ticks (scan end after the newest tick): five confirm, as before the guard.
  {
    Rig r;
    r.ToDegraded();
    Transition tr{};
    for (int i = 0; i < 5; ++i) {
      for (int k = 1; k < 20; ++k) r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5 * k));
      tr = r.Scan(Kind::kScanGood);
      if (i < 4) ExpectUnchanged(tr, LioState::kDegraded);
    }
    ExpectChange(tr, LioState::kDegraded, LioState::kTracking, LioReason::kConfirmationReached);
  }
}

// --- rule precedence (sigma first) ---------------------------------------------------------

TEST(LioLifecyclePrecedence, SigmaBeatsGapLostOnTheSameImuTick) {
  Rig r;
  r.ToDegraded();
  // Gap 0.6 s > gap_lost AND sigma 0.9 > limit: both rules fire; sigma is checked first.
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 600, 0.9)), LioState::kDegraded, LioState::kLost,
               LioReason::kCovarianceExceeded);
}

TEST(LioLifecyclePrecedence, SigmaBeatsGapDegradedOnTheSameImuTick) {
  Rig r;
  r.ToTracking();
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 300, 0.9)), LioState::kTracking, LioState::kLost,
               LioReason::kCovarianceExceeded);
}

TEST(LioLifecyclePrecedence, SigmaBeatsDegenerateScanThatWouldDegrade) {
  Rig r;
  r.ToTracking();
  r.Scan(Kind::kScanDegenerate);
  r.Scan(Kind::kScanDegenerate);
  // The third degenerate scan would degrade, but its sigma is exceeded.
  ExpectChange(r.Scan(Kind::kScanDegenerate, 0.9), LioState::kTracking, LioState::kLost,
               LioReason::kCovarianceExceeded);
}

TEST(LioLifecyclePrecedence, SigmaBeatsDegeneracyPersistedAndConfirmation) {
  Rig r;
  r.ToDegraded();
  r.Scan(Kind::kScanDegenerate);
  r.now_ms += 1'500;  // far beyond degeneracy_lost since the first degenerate scan
  ExpectChange(r.lc.on(Ev(Kind::kScanDegenerate, r.now_ms, 0.9)), LioState::kDegraded, LioState::kLost,
               LioReason::kCovarianceExceeded);
  Rig r2;
  r2.ToDegraded();
  for (int i = 0; i < 4; ++i) r2.Scan(Kind::kScanGood);
  // The fifth good scan would confirm TRACKING, but its sigma is exceeded.
  ExpectChange(r2.Scan(Kind::kScanGood, 0.9), LioState::kDegraded, LioState::kLost, LioReason::kCovarianceExceeded);
}

// --- IMU ticks between scans must not touch the scan counters ---------------------------------

namespace {

// 19 IMU ticks at 5 ms (200 Hz) between 10 Hz scans, then the scan itself.
Transition ScanWithTicks(Rig& r, Kind k) {
  for (int i = 1; i < 20; ++i) {
    const auto tick = r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5 * i));
    EXPECT_FALSE(tick.changed) << "tick changed state at " << r.now_ms + 5 * i;
  }
  return r.Scan(k);
}

}  // namespace

TEST(LioLifecycleTicks, TicksDoNotResetInitializingConfirmation) {
  Rig r;
  r.lc.on(Ev(Kind::kMapReady, 0));
  for (int i = 0; i < 4; ++i) ExpectUnchanged(ScanWithTicks(r, Kind::kScanGood), LioState::kInitializing);
  ExpectChange(ScanWithTicks(r, Kind::kScanGood), LioState::kInitializing, LioState::kTracking,
               LioReason::kConfirmationReached);
}

TEST(LioLifecycleTicks, TicksDoNotResetTrackingDegenerateRun) {
  Rig r;
  r.ToTracking();
  ExpectUnchanged(ScanWithTicks(r, Kind::kScanDegenerate), LioState::kTracking);
  ExpectUnchanged(ScanWithTicks(r, Kind::kScanEmpty), LioState::kTracking);
  ExpectChange(ScanWithTicks(r, Kind::kScanDegenerate), LioState::kTracking, LioState::kDegraded,
               LioReason::kDegenerateScans);
}

TEST(LioLifecycleTicks, TicksDoNotResetDegradedConfirmation) {
  Rig r;
  r.ToDegraded();
  for (int i = 0; i < 4; ++i) ExpectUnchanged(ScanWithTicks(r, Kind::kScanGood), LioState::kDegraded);
  ExpectChange(ScanWithTicks(r, Kind::kScanGood), LioState::kDegraded, LioState::kTracking,
               LioReason::kConfirmationReached);
}

TEST(LioLifecycleTicks, TicksDoNotResetTheDegenerateClockInDegraded) {
  Rig r;
  r.ToDegraded();
  // Degenerate scans with ticks in between: the run clock keeps running to LOST.
  Transition tr{};
  for (int i = 0; i < 20 && r.lc.state() == LioState::kDegraded; ++i) tr = ScanWithTicks(r, Kind::kScanDegenerate);
  ExpectChange(tr, LioState::kDegraded, LioState::kLost, LioReason::kDegeneracyPersisted);
}

// --- sigma fails closed ---------------------------------------------------------------------

TEST(LioLifecycleSigma, OnlyZeroToLimitIsAcceptable) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  struct Case { double sigma; bool lost; };
  for (const Case c : {Case{0.0, false}, Case{0.5, false}, Case{0.25, false}, Case{-0.0, false},
                       Case{std::nextafter(0.5, 1.0), true}, Case{-1e-12, true}, Case{-1.0, true}, Case{-inf, true},
                       Case{inf, true}, Case{nan, true}}) {
    Rig r;
    r.ToTracking();
    const auto tr = r.lc.on(Ev(Kind::kImuTick, r.now_ms + 10, c.sigma));
    if (c.lost) {
      ExpectChange(tr, LioState::kTracking, LioState::kLost, LioReason::kCovarianceExceeded);
    } else {
      ExpectUnchanged(tr, LioState::kTracking);
    }
  }
}

TEST(LioLifecycleSigma, NegativeSigmaAlsoFailsClosedInDegraded) {
  Rig r;
  r.ToDegraded();
  ExpectChange(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 10, -0.1)), LioState::kDegraded, LioState::kLost,
               LioReason::kCovarianceExceeded);
}

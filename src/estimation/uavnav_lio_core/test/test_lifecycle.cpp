#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "uavnav/core/time.hpp"
#include "uavnav/lio/lifecycle.hpp"

using namespace uavnav;
using namespace uavnav::lio;
using Kind = LioEvent::Kind;

namespace {

constexpr LifecycleConfig kCfg{5, 3, time::milliseconds(250), time::milliseconds(500), time::seconds(1), 0.5};

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
  LioLifecycle lc{kCfg};
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
  void ToRestarting() {
    ToLost();
    now_ms += 700;
    lc.on(Ev(Kind::kScanGood, now_ms));
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
  EXPECT_EQ(to_string(LioReason::kGeometryReturned), "GEOMETRY_RETURNED");
  EXPECT_EQ(to_string(LioReason::kRestartSeeded), "RESTART_SEEDED");
  EXPECT_EQ(to_string(LioReason::kMapReady), "MAP_READY");
}

TEST(LioLifecycleTable, NoEdgeFromLostToTracking) {
  for (const auto& edge : kLioTransitions) {
    EXPECT_FALSE(edge.from == LioState::kLost && edge.to == LioState::kTracking);
  }
  EXPECT_EQ(kLioTransitions.size(), 10U);
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
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, 60'000, 99.0)), LioState::kInitializing);
  ExpectUnchanged(r.lc.on(Ev(Kind::kRestartSeeded, 60'005)), LioState::kInitializing);
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
  lc.on(Ev(Kind::kMapReady, 0));
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
                       Kind::kRestartSeeded}) {
    Rig r;
    r.ToTracking();
    ExpectChange(r.lc.on(Ev(k, r.now_ms + 10, 0.51)), LioState::kTracking, LioState::kLost,
                 LioReason::kCovarianceExceeded);
  }
}

TEST(LioLifecycle, CovarianceExceededInDegradedOnAnyEvent) {
  for (const Kind k : {Kind::kScanGood, Kind::kScanDegenerate, Kind::kScanEmpty, Kind::kImuTick, Kind::kMapReady,
                       Kind::kRestartSeeded}) {
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

// Row 9: LOST + kScanGood -> RESTARTING.
TEST(LioLifecycle, LostGoodScanStartsRestarting) {
  Rig r;
  r.ToLost();
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kLost, LioState::kRestarting, LioReason::kGeometryReturned);
}

TEST(LioLifecycle, LostIgnoresEverythingButGoodScan) {
  Rig r;
  r.ToLost();
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kLost);
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kLost);
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5'000)), LioState::kLost);
  ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, r.now_ms + 5'005)), LioState::kLost);
  ExpectUnchanged(r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms + 5'010)), LioState::kLost);
}

// F27: LOST + 20 good scans -> RESTARTING after the first, stays RESTARTING.
TEST(LioLifecycle, LostNeverJumpsToTracking) {
  Rig r;
  r.ToLost();
  ExpectChange(r.Scan(Kind::kScanGood), LioState::kLost, LioState::kRestarting, LioReason::kGeometryReturned);
  for (int i = 0; i < 19; ++i) {
    ExpectUnchanged(r.Scan(Kind::kScanGood), LioState::kRestarting);
    EXPECT_EQ(r.lc.state(), LioState::kRestarting);
  }
}

// Row 10: RESTARTING + kRestartSeeded -> INITIALIZING.
TEST(LioLifecycle, RestartSeededReturnsToInitializing) {
  Rig r;
  r.ToRestarting();
  ExpectChange(r.lc.on(Ev(Kind::kRestartSeeded, r.now_ms + 10)), LioState::kRestarting, LioState::kInitializing,
               LioReason::kRestartSeeded);
}

TEST(LioLifecycle, RestartingIgnoresOtherEvents) {
  Rig r;
  r.ToRestarting();
  ExpectUnchanged(r.Scan(Kind::kScanDegenerate), LioState::kRestarting);
  ExpectUnchanged(r.Scan(Kind::kScanEmpty), LioState::kRestarting);
  ExpectUnchanged(r.lc.on(Ev(Kind::kImuTick, r.now_ms + 5'000)), LioState::kRestarting);
  ExpectUnchanged(r.lc.on(Ev(Kind::kMapReady, r.now_ms + 5'005)), LioState::kRestarting);
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
    const auto k = static_cast<Kind>((seed >> 16) % 6);
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

#pragma once

#include <cstddef>
#include <cstdint>

#include "uavnav/core/time.hpp"

// Tier-(a) constants owned by uavnav_lio_core (SYSTEM_DESIGN §6.2). Changing one is a
// design-review change; each states how its value was derived.
namespace uavnav::lio::limits {

/// Points accepted in one scan. Mid-360 at 10 Hz ≈ 20k points per scan; 10× headroom, guards memory.
inline constexpr std::size_t kMaxScanPoints = 200'000;

// --- output predictor (SYSTEM_DESIGN §3.3, F35, D20) ------------------------------------------------

/// History the output predictor corrects across: the largest scan-to-output delay (SYSTEM_DESIGN §3.3,
/// "độ trễ tối đa 300 ms"). A correction stamped earlier than newest output - span is rejected
/// (kOlderThanBuffer) and changes nothing; so is an ImuDelta whose dt_s exceeds it.
inline constexpr time::Duration kPredictorBufferSpan = time::milliseconds(300);

/// Fastest IMU the buffer is sized for: the Mid-360's internal IMU at 200 Hz.
inline constexpr time::Duration kPredictorDesignImuPeriod = time::milliseconds(5);

/// Output samples held (one per accepted ImuDelta, plus the aligned sample). 64 samples at 200 Hz span
/// 63 x 5 ms = 315 ms >= kPredictorBufferSpan. A faster IMU shortens the covered history below the span;
/// corrections beyond it are then rejected as older than the buffer, never matched to a wrong sample.
inline constexpr std::size_t kPredictorBufferCapacity = 64;
static_assert(static_cast<std::int64_t>(kPredictorBufferCapacity - 1) * kPredictorDesignImuPeriod.ns >=
                  kPredictorBufferSpan.ns,
              "the predictor buffer must cover kPredictorBufferSpan at the design IMU rate");

/// Clamp of every dt that enters a gain (averaged IMU dt, averaged correction interval, the lookup
/// tolerance): PX4 ekf2 OutputPredictor constrain(dt, 0.0001f, 0.03f). Keeps 0.5*dt/delay and dt/tau
/// finite and bounded under jitter (dt 0 or a burst after a stall).
inline constexpr time::Duration kPredictorDtMin = time::nanoseconds(100'000);
inline constexpr time::Duration kPredictorDtMax = time::milliseconds(30);

// --- estimator facade (SYSTEM_DESIGN §3, LioEstimator) ----------------------------------------------

/// IMU samples kept for the ESKF prediction between scans. 1024 samples = 5.1 s at 200 Hz, which covers the
/// largest allowed lifecycle_gap_lost_s (5 s), so the first scan after a LiDAR outage that ended in LOST can
/// still be predicted from the last scan. When history is dropped, that scan's prediction fails (an event,
/// treated as a degenerate scan) and the ESKF time is moved to the scan end; it never blocks.
inline constexpr std::size_t kImuHistoryCapacity = 1024;

/// Period of the HealthOutput produced from IMU time (sensor time), besides one on every transition.
/// 10 Hz: one health sample per scan period at the 10 Hz scan rate (SYSTEM_DESIGN §6.4).
inline constexpr time::Duration kHealthPeriod = time::milliseconds(100);

/// ESKF rebase after a failed prediction (IMU gap, IMU history dropped) or a math
/// exception: the state is kept, its time moves to the scan end and the covariance is multiplied by this
/// factor (sigma x 3.2), then each eigenvalue is capped at kRebaseMaxCovarianceEigenvalue. Both values are
/// main's pipeline (PROPAGATION_STATE_TIME_REBASED: tracking.discontinuity_covariance_inflation = 10 in
/// config/runtime/sim.yaml, kMaximumRecoveryCovarianceEigenvalue = 100). Starting from the initial position
/// sigma 0.03 m, the third consecutive rebase exceeds lifecycle_position_sigma_lost_m (0.5 m) -> LOST, so
/// repeated unpredicted scans fail closed through the covariance rule.
inline constexpr double kRebaseCovarianceInflation = 10.0;
/// Numerical guard only (not a claim of certainty): keeps the IKFoM normal equations from overflowing.
inline constexpr double kRebaseMaxCovarianceEigenvalue = 100.0;

/// Per-sample facade events (ImuDuplicate, LioInputRejected) are emitted on the 1st occurrence per reason
/// and epoch and then on every kEventSummaryEvery-th, so a misbehaving driver cannot flood the event ring
/// (10 Hz worth of events at a 10 kHz fault rate).
inline constexpr std::uint64_t kEventSummaryEvery = 1000;

// --- frontend/backend split (SYSTEM_DESIGN §3.5, D30/O14) -------------------------------------------

/// Requests (IMU copies, scans, restarts) queued frontend -> backend. 512 is about 2.4 s of 200 Hz IMU plus
/// its 10 Hz scans (210 requests/s), more than 4x the beta lifecycle_gap_lost_s (0.5 s): a backend that
/// falls this far behind is already LOST by the gap rule. A full queue drops the newest IMU copy with an
/// event (the backend then sees an IMU gap: that scan's prediction fails, fail closed).
inline constexpr std::size_t kBackendRequestCapacity = 512;

/// Scans admitted to the backend with no result yet. Two = one in ICP plus one waiting at most one ICP time
/// (20-50 ms, §3.5), so the waiting scan's correction still lands inside the 300 ms predictor buffer
/// (kPredictorBufferSpan). A third is refused (drop-newest, EstimatorReason::kBackendBusy).
inline constexpr std::uint32_t kMaxScansInFlight = 2;

/// Results queued backend -> frontend. Pending results are at most kMaxScansInFlight scan results + one
/// restart answer + one IMU initialisation = 4; 8 leaves 2x headroom, so a full result queue cannot happen
/// by construction (the backend still fails closed on it: event, result dropped).
inline constexpr std::size_t kBackendResultCapacity = 8;
static_assert(kBackendResultCapacity >= kMaxScansInFlight + 2,
              "the result queue must hold every scan in flight, a restart answer and the IMU initialisation");

/// OpenMP threads of one ICP correction (fast_lio_core ResidualBuilderConfig::parallel_thread_count), used by
/// the backend only (§3.5: the libgomp team runs only inside the backend's correction call). 3 = fast_lio_core's
/// default, the value main's pipeline and the S1a facade ran with; the residuals are computed per point, so
/// the count changes the ICP time, not its result.
inline constexpr std::size_t kIcpThreads = 3;

}  // namespace uavnav::lio::limits

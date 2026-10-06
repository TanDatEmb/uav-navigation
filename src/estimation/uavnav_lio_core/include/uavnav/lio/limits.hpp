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

}  // namespace uavnav::lio::limits

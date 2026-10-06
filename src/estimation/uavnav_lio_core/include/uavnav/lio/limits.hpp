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

}  // namespace uavnav::lio::limits

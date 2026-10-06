#pragma once

#include <cstddef>
#include <cstdint>

#include "uavnav/core/time.hpp"

// Tier-(a) constants owned by uavnav_px4_bridge (SYSTEM_DESIGN §6.2). Changing one is a design-review
// change; each states how its value was derived.
namespace uavnav::px4bridge::limits {

// --- alignment T_px4<-lio (SYSTEM_DESIGN §4.1, D7, D21) ---------------------------------------------

/// Largest distance in time between a LIO scan stamp and the PX4 sample used on EACH side of it for the
/// interpolation. PX4 odometry >= 50 Hz: two consecutive samples are at most 20 ms apart, so 50 ms is
/// 2.5 periods (one lost sample on a side still pairs). A scan without a sample this close on both sides
/// (or an exact stamp match) is not paired (kNoPx4Sample).
inline constexpr time::Duration kPairingWindow = time::milliseconds(50);

/// History of PX4 samples the alignment pairs against. A scan reaches the bridge after the LIO
/// scan-to-output delay (at most 300 ms, SYSTEM_DESIGN §3.3) plus transport; 1 s covers that delay
/// plus kPairingWindow with about 3x margin. Samples older than (newest PX4 sample - span) are never used.
inline constexpr time::Duration kPx4BufferSpan = time::seconds(1);

/// Fastest PX4 local-position stream the ring is sized for: 250 Hz (EKF2 output at the SITL IMU rate).
inline constexpr time::Duration kPx4DesignPeriod = time::milliseconds(4);

/// Fixed ring of PX4 samples (no heap growth, no allocation per sample). 256 samples at 250 Hz span
/// 255 x 4 ms = 1.02 s >= kPx4BufferSpan. A faster stream shortens the covered history below the span;
/// older scans are then reported kNoPx4Sample, never paired with a wrong sample.
inline constexpr std::size_t kPx4BufferCapacity = 256;
static_assert(static_cast<std::int64_t>(kPx4BufferCapacity - 1) * kPx4DesignPeriod.ns >= kPx4BufferSpan.ns,
              "the PX4 ring must cover kPx4BufferSpan at the design rate");
static_assert(kPx4BufferSpan > kPairingWindow + kPairingWindow);

/// Clamp of the filter step dt (LIO sensor time between two accepted pairs). The minimum (1 ms) only keeps
/// the step well defined; LIO stamps are strictly increasing so dt > 0 anyway. The maximum (200 ms = two
/// 10 Hz scan periods) bounds what one pair may do after a gap (FROZEN, rejected or unpaired scans): at
/// most alpha = 0.2 / tau of the error and max_rate x 0.2 s of motion. It equals the smallest allowed
/// alignment_tau_s, so alpha = dt / tau <= 1 for every loadable config (checked in config.hpp).
inline constexpr time::Duration kFilterDtMin = time::milliseconds(1);
inline constexpr time::Duration kFilterDtMax = time::milliseconds(200);

/// Largest alignment_consistent_pairs (its spec max): the INIT accumulation is a fixed array of this size.
inline constexpr std::uint32_t kMaxConsistentPairs = 200;

}  // namespace uavnav::px4bridge::limits

#pragma once

#include <cstddef>
#include <cstdint>

#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/px4_time.hpp"

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
/// most alpha = 0.2 / tau of the error measured at the vehicle, i.e. at most jump_position_m x 0.2 / tau of
/// motion of T at the vehicle and jump_yaw_rad x 0.2 / tau of yaw (there is no separate rate limiter, D30).
/// It equals the smallest allowed alignment_tau_s, so alpha = dt / tau <= 1 for every loadable config
/// (checked in config.hpp).
inline constexpr time::Duration kFilterDtMin = time::milliseconds(1);
inline constexpr time::Duration kFilterDtMax = time::milliseconds(200);

/// Largest |component| [m] of a position the alignment accepts: a tracking LIO pose and a PX4 position
/// (a PX4 reset delta, the difference of two such positions, up to 2x). Valid data never comes close: a
/// mission stays within tens of km of both origins, and 1e6 m is 1000 km. The bound keeps every sum finite
/// and meaningful: the INIT accumulation adds at most kMaxConsistentPairs positions (<= 2e8 m), T, q and e
/// are sums of a few such terms, so no overflow to inf and no inf - inf = NaN can reach a gate; a double
/// still resolves 1.2e-10 m at 1e6 m. A larger component is rejected like a non-finite one (kInputRejected).
inline constexpr double kMaxPositionAbsM = 1e6;

/// Largest alignment_consistent_pairs (its spec max): the INIT accumulation is a fixed array of this size.
inline constexpr std::uint32_t kMaxConsistentPairs = 200;

// --- Px4Bridge input shell (SYSTEM_DESIGN §4.1, D29, D33/O22) ----------------------------------------------

/// Period of Px4Bridge::on_tick (O22, the node's ROS-clock timer). It drives the alignment timeouts without
/// LIO input, so a timeout is seen at most one period late: 0.1 s against valid_stale >= 0.2 s and
/// frozen_max >= 1 s (their spec minimums, checked in bridge.cpp).
inline constexpr time::Duration kTickPeriod = time::milliseconds(100);

/// An input stamp later than the newest tick time + this is refused (BridgeReason::kStampInFuture). In SITL every
/// stamp (LIO sensor time, PX4 time) is the simulation /clock (D29), and a message is stamped before it is
/// received; the newest tick lags ROS now by at most one kTickPeriod plus executor latency, so a correct stamp is
/// at most about 0.1 s ahead of it. 0.5 s (the lio node's O20 bound) leaves four tick periods of margin and still
/// refuses garbage before it reaches the alignment, where an accepted future stamp would make every later true
/// sample out of order. Before the first tick there is no reference and nothing is refused as ahead.
inline constexpr time::Duration kMaxStampAhead = time::milliseconds(500);
static_assert(kMaxStampAhead > kTickPeriod + kTickPeriod);

/// SensorTime -> PX4 time of the EV encoder. The beta is SITL only (SYSTEM_DESIGN §0, D29): the sensor clock is
/// the simulation /clock, which is PX4's, so the conversion is the identity. kRealtime is refused by to_px4.
inline constexpr ClockMode kClockMode = ClockMode::kSimulationIdentity;

/// Per-input rejection events of Px4Bridge (EvGated, EvRejected, AlignmentPairRejected, Px4SampleRejected,
/// LioInputRejected) are emitted on the 1st occurrence per event and reason and then on every kEventSummaryEvery-th
/// (the lio frontend's rule and value, uavnav/lio/limits.hpp), so a 50 Hz PX4 fault or a 10 Hz gated EV stream
/// reports once and then every 20 s / 100 s instead of flooding the event ring.
inline constexpr std::uint64_t kEventSummaryEvery = 1000;

}  // namespace uavnav::px4bridge::limits

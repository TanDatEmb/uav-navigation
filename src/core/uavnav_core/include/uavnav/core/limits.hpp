#pragma once

#include <cstddef>

#include "uavnav/core/time.hpp"

// Tier-(a) constants owned by uavnav_core (SYSTEM_DESIGN §6.2). Changing one is a
// design-review change; each states how its value was derived.
namespace uavnav::limits {

/// Values carried by one EventRecord. SYSTEM_DESIGN §6.1 fixes `values[≤16]`.
inline constexpr std::size_t kMaxEventValues = 16;

/// Records the event ring holds before emit() starts dropping. Sized for about 4 s
/// of 1 kHz event bursts (4096 / 1000 Hz ≈ 4.1 s), so a sink stall of a few
/// seconds loses nothing at the highest per-process event rate we expect.
inline constexpr std::size_t kEventRingCapacity = 4096;

/// Longest the event writer sleeps between drains. 20 ms = one 50 Hz command
/// cycle (§6.4): batches stay small (≈20 records at 1 kHz), the ring is drained
/// far faster than it fills, and the writer wakes at most 50 times a second.
inline constexpr time::Duration kEventWriterPeriod = time::milliseconds(20);

}  // namespace uavnav::limits

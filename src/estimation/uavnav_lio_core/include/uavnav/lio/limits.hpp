#pragma once

#include <cstddef>

// Tier-(a) constants owned by uavnav_lio_core (SYSTEM_DESIGN §6.2). Changing one is a
// design-review change; each states how its value was derived.
namespace uavnav::lio::limits {

/// Points accepted in one scan. Mid-360 at 10 Hz ≈ 20k points per scan; 10× headroom, guards memory.
inline constexpr std::size_t kMaxScanPoints = 200'000;

}  // namespace uavnav::lio::limits

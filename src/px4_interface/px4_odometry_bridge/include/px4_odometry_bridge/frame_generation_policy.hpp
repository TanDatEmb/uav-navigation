#pragma once

#include <cstdint>

namespace px4_odometry_bridge {

// A source-time restart only creates a new public frame after at least one
// public sample existed.  During startup it reinitializes the pending origin
// without manufacturing a frame transition that downstream consumers could
// observe.
inline std::uint64_t frame_generation_after_source_restart(
    std::uint64_t current_generation, bool public_frame_exists) {
  return public_frame_exists ? current_generation + 1U : current_generation;
}

}  // namespace px4_odometry_bridge

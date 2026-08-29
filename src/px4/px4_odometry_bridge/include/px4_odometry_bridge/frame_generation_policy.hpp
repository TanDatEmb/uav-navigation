#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace px4_odometry_bridge {

// A source-time restart only creates a new public frame after at least one
// public sample existed.  During startup it reinitializes the pending origin
// without manufacturing a frame transition that downstream consumers could
// observe.
inline std::optional<std::uint64_t> frame_generation_after_source_restart(
    std::uint64_t current_generation, bool public_frame_exists) {
  if (current_generation == 0U) return std::nullopt;
  if (!public_frame_exists) return current_generation;
  if (current_generation == std::numeric_limits<std::uint64_t>::max()) {
    return std::nullopt;
  }
  return current_generation + 1U;
}

}  // namespace px4_odometry_bridge

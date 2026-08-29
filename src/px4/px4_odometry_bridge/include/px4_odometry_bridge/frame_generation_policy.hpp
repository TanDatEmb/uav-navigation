#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace px4_odometry_bridge {

// Source samples are accepted only when their producer-owned identity is
// strictly newer than the highest identity already received. This high-water
// check must happen before any publication gate: a gated newer sample still
// invalidates delayed older samples.
inline bool strictly_newer_source_identity(
    std::uint64_t highest_epoch, std::uint64_t highest_sequence,
    std::uint64_t epoch, std::uint64_t sequence) noexcept {
  if (epoch == 0U || sequence == 0U) return false;
  if (highest_epoch == 0U) return true;
  return epoch > highest_epoch ||
         (epoch == highest_epoch && sequence > highest_sequence);
}

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

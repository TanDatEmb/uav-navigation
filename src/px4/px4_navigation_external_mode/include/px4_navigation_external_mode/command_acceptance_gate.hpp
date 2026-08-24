#pragma once

#include <cstdint>

#include <execution_state_freshness.hpp>

namespace px4_navigation_external_mode {

enum class CommandAcceptanceGate : std::uint8_t {
  kAccept = 0,
  kOdometryStale,
  kNonIncreasingMessageId,
};

inline CommandAcceptanceGate classifyCommandAcceptance(
    const navigation_interfaces::ExecutionStateFreshness& odometry_freshness,
    std::uint32_t incoming_message_id, std::uint32_t previous_message_id) noexcept {
  if (!odometry_freshness.valid()) return CommandAcceptanceGate::kOdometryStale;
  if (previous_message_id != 0U && incoming_message_id <= previous_message_id) {
    return CommandAcceptanceGate::kNonIncreasingMessageId;
  }
  return CommandAcceptanceGate::kAccept;
}

}  // namespace px4_navigation_external_mode

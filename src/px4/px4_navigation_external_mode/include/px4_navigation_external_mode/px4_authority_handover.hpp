#pragma once

#include <cstdint>

namespace px4_navigation_external_mode::handover {

// Identifies one callback-producing request within one executor activation.
// PX4 callbacks can outlive the request/activation that scheduled them.
struct AttemptToken {
  std::uint64_t activation_generation{0U};
  std::uint32_t attempt{0U};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return activation_generation != 0U && attempt != 0U;
  }
};

enum class CallbackDisposition : std::uint8_t {
  kSuperseded,
  kAlreadyConfirmed,
  kAwaitVehicleStatus,
};

enum class CallbackResult : std::uint8_t {
  kSuccess,
  kDeactivated,
  kFailure,
};

// scheduleMode callback results describe command/mode-executor processing.
// They do not replace VehicleStatus as proof of the current PX4 nav state.
[[nodiscard]] constexpr CallbackDisposition assessCallback(
    const AttemptToken expected, const AttemptToken current,
    const bool pending, const bool in_flight,
    const bool px4_hold_confirmed, const CallbackResult result) noexcept {
  if (!expected.valid() || expected.activation_generation != current.activation_generation ||
      expected.attempt != current.attempt) {
    return CallbackDisposition::kSuperseded;
  }
  if (px4_hold_confirmed) return CallbackDisposition::kAlreadyConfirmed;
  if (!pending || !in_flight) return CallbackDisposition::kSuperseded;
  // No callback result proves current nav_state. All current callback outcomes
  // leave the episode waiting on VehicleStatus; callers may distinguish their
  // diagnostics, but must not treat Success/Deactivated as Hold confirmation.
  (void)result;
  return CallbackDisposition::kAwaitVehicleStatus;
}

[[nodiscard]] constexpr bool retryDue(
    const std::int64_t now_steady_ns, const std::int64_t retry_at_steady_ns,
    const bool pending, const bool px4_hold_confirmed) noexcept {
  // This is also the timeout for an accepted schedule request whose
  // ModeCompleted callback has not arrived. The executor retires that attempt
  // before scheduling a replacement; the old callback token then becomes stale.
  return pending && !px4_hold_confirmed && now_steady_ns >= retry_at_steady_ns;
}

}  // namespace px4_navigation_external_mode::handover

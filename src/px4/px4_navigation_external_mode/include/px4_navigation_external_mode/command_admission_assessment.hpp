#pragma once

#include <cstdint>
#include <optional>
#include <type_traits>
#include <variant>

#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/navigation_command_contract.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>

#include "px4_navigation_external_mode/command_acceptance_gate.hpp"

namespace px4_navigation_external_mode {

enum class AdmissionStage : std::uint8_t {
  kPresence = 0,
  kContract,
  kTemporalLease,
  kTerminalOwnership,
  kSessionIdentity,
  kOdometryFreshness,
  kSampleOrdering,
  kTrackingEnvelope,
  kCommit,
};

enum class AdmissionDisposition : std::uint8_t {
  kAccept = 0,
  kRejectRetainPrevious,
  kRejectFailNavigation,
  kRejectSafetyStop,
  kIgnoreAfterTerminal,
};

enum class PresenceReason : std::uint8_t { kMessageMissing = 1 };
enum class TerminalReason : std::uint8_t { kAuthorityClosed = 1 };
enum class TrackingReason : std::uint8_t { kEnvelopeExceeded = 1 };

enum class SessionIdentityReason : std::uint8_t {
  kValid = 0,
  kHealthEpochMismatch,
  kModeActivationMismatch,
  kMissionSessionMismatch,
  kRequestRegression,
  kWorldIdentityRegression,
};

using AdmissionReason = std::variant<std::monostate, PresenceReason,
    navigation_contracts::CommandContractReason,
    navigation_contracts::CommandTemporalReason, TerminalReason,
    SessionIdentityReason, navigation_contracts::ExecutionStateFreshnessReason,
    CommandAcceptanceGate, TrackingReason>;

struct CommandAdmissionAssessment final {
  AdmissionStage stage{AdmissionStage::kCommit};
  AdmissionDisposition disposition{AdmissionDisposition::kAccept};
  AdmissionReason reason{};

  [[nodiscard]] bool accepted() const noexcept {
    return disposition == AdmissionDisposition::kAccept;
  }
};

[[nodiscard]] inline SessionIdentityReason assessCommandSessionIdentity(
    const navigation_contracts::msg::NavigationCommand& incoming,
    const std::optional<navigation_contracts::msg::NavigationCommand>& previous,
    bool typed_health_seen, bool health_valid, std::uint64_t health_localization_epoch,
    std::uint64_t mode_activation_id) noexcept {
  if (!navigation_contracts::estimatorHealthAllowsCommand(
          typed_health_seen, health_valid, incoming.localization_epoch,
          health_localization_epoch)) {
    return SessionIdentityReason::kHealthEpochMismatch;
  }
  if (mode_activation_id != 0U && incoming.mode_activation_id != mode_activation_id) {
    return SessionIdentityReason::kModeActivationMismatch;
  }
  if (previous && incoming.mission_id != previous->mission_id) {
    return SessionIdentityReason::kMissionSessionMismatch;
  }
  if (previous &&
      (incoming.goal_epoch == previous->goal_epoch
           ? (incoming.waypoint_index != previous->waypoint_index ||
              incoming.request_id != previous->request_id)
           : incoming.request_id < previous->request_id)) {
    return SessionIdentityReason::kRequestRegression;
  }
  if (previous && !navigation_contracts::commandWorldIdentityNonRegressing(
                      incoming, *previous)) {
    return SessionIdentityReason::kWorldIdentityRegression;
  }
  return SessionIdentityReason::kValid;
}

[[nodiscard]] constexpr const char* admissionStageName(AdmissionStage stage) noexcept {
  switch (stage) {
    case AdmissionStage::kPresence: return "PRESENCE";
    case AdmissionStage::kContract: return "CONTRACT";
    case AdmissionStage::kTemporalLease: return "TEMPORAL_LEASE";
    case AdmissionStage::kTerminalOwnership: return "TERMINAL_OWNERSHIP";
    case AdmissionStage::kSessionIdentity: return "SESSION_IDENTITY";
    case AdmissionStage::kOdometryFreshness: return "ODOMETRY_FRESHNESS";
    case AdmissionStage::kSampleOrdering: return "SAMPLE_ORDERING";
    case AdmissionStage::kTrackingEnvelope: return "TRACKING_ENVELOPE";
    case AdmissionStage::kCommit: return "COMMIT";
  }
  return "UNRECOGNIZED_STAGE";
}

[[nodiscard]] constexpr const char* admissionDispositionName(
    AdmissionDisposition disposition) noexcept {
  switch (disposition) {
    case AdmissionDisposition::kAccept: return "ACCEPT";
    case AdmissionDisposition::kRejectRetainPrevious: return "REJECT_RETAIN_PREVIOUS";
    case AdmissionDisposition::kRejectFailNavigation: return "REJECT_FAIL_NAVIGATION";
    case AdmissionDisposition::kRejectSafetyStop: return "REJECT_SAFETY_STOP";
    case AdmissionDisposition::kIgnoreAfterTerminal: return "IGNORE_AFTER_TERMINAL";
  }
  return "UNRECOGNIZED_DISPOSITION";
}

[[nodiscard]] inline std::uint16_t admissionReasonCode(const AdmissionReason& reason) noexcept {
  return std::visit([](const auto& value) -> std::uint16_t {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, std::monostate>) return 0U;
    else return static_cast<std::uint16_t>(value);
  }, reason);
}

[[nodiscard]] inline const char* admissionReasonName(const AdmissionReason& reason) noexcept {
  return std::visit([](const auto& value) -> const char* {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, std::monostate>) return "NONE";
    else if constexpr (std::is_same_v<T, PresenceReason>) return "MESSAGE_MISSING";
    else if constexpr (std::is_same_v<T, navigation_contracts::CommandContractReason>) {
      return navigation_contracts::commandContractReasonName(value);
    } else if constexpr (std::is_same_v<T, navigation_contracts::CommandTemporalReason>) {
      return navigation_contracts::commandTemporalReasonName(value);
    } else if constexpr (std::is_same_v<T, TerminalReason>) return "TERMINAL_AUTHORITY_CLOSED";
    else if constexpr (std::is_same_v<T, TrackingReason>) return "TRACKING_ENVELOPE_EXCEEDED";
    else if constexpr (std::is_same_v<T, navigation_contracts::ExecutionStateFreshnessReason>) {
      return navigation_contracts::executionStateFreshnessReasonName(value);
    }
    else if constexpr (std::is_same_v<T, CommandAcceptanceGate>) {
      switch (value) {
        case CommandAcceptanceGate::kAccept: return "ACCEPT";
        case CommandAcceptanceGate::kOdometryStale: return "ODOMETRY_STALE";
        case CommandAcceptanceGate::kNonIncreasingMessageId: return "SAMPLE_NONINCREASING";
      }
      return "UNRECOGNIZED_ACCEPTANCE_GATE";
    } else {
      switch (value) {
        case SessionIdentityReason::kValid: return "VALID";
        case SessionIdentityReason::kHealthEpochMismatch: return "HEALTH_EPOCH_MISMATCH";
        case SessionIdentityReason::kModeActivationMismatch: return "MODE_ACTIVATION_MISMATCH";
        case SessionIdentityReason::kMissionSessionMismatch: return "MISSION_SESSION_MISMATCH";
        case SessionIdentityReason::kRequestRegression: return "REQUEST_REGRESSION";
        case SessionIdentityReason::kWorldIdentityRegression: return "WORLD_IDENTITY_REGRESSION";
      }
      return "UNRECOGNIZED_SESSION_REASON";
    }
  }, reason);
}

}  // namespace px4_navigation_external_mode

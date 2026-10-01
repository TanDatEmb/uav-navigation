#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

#include <builtin_interfaces/msg/time.hpp>
#include <navigation_common/time.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>

namespace navigation_contracts {

using msg::NavigationCommand;

enum class CommandContractReason : std::uint8_t {
  kValid,
  kExpectedFrameMissing,
  kMissionIdEmpty,
  kFrameMismatch,
  kHeaderStampInvalid,
  kValidityWindowInvalid,
  kWorldStampInvalid,
  kStateStampInvalid,
  kLocalizationIdentityInvalid,
  kGoalIdentityInvalid,
  kWorldIdentityInvalid,
  kSampleIdentityInvalid,
  kStatusInvalid,
  kRoleStatusMismatch,
  kPvajOrYawNonFinite,
  kContinuationContractInvalid,
  kTrajectoryTimeInvalid,
};

enum class CommandTemporalReason : std::uint8_t {
  kValid,
  kNowInvalid,
  kInvalidWindow,
  kNotYetValid,
  kExpired,
};

[[nodiscard]] constexpr const char* commandContractReasonName(
    CommandContractReason reason) noexcept {
  switch (reason) {
    case CommandContractReason::kValid: return "VALID";
    case CommandContractReason::kExpectedFrameMissing: return "EXPECTED_FRAME_MISSING";
    case CommandContractReason::kMissionIdEmpty: return "MISSION_ID_EMPTY";
    case CommandContractReason::kFrameMismatch: return "FRAME_MISMATCH";
    case CommandContractReason::kHeaderStampInvalid: return "HEADER_STAMP_INVALID";
    case CommandContractReason::kValidityWindowInvalid: return "VALIDITY_WINDOW_INVALID";
    case CommandContractReason::kWorldStampInvalid: return "WORLD_STAMP_INVALID";
    case CommandContractReason::kStateStampInvalid: return "STATE_STAMP_INVALID";
    case CommandContractReason::kLocalizationIdentityInvalid: return "LOCALIZATION_ID_INVALID";
    case CommandContractReason::kGoalIdentityInvalid: return "GOAL_ID_INVALID";
    case CommandContractReason::kWorldIdentityInvalid: return "WORLD_ID_INVALID";
    case CommandContractReason::kSampleIdentityInvalid: return "SAMPLE_ID_INVALID";
    case CommandContractReason::kStatusInvalid: return "STATUS_INVALID";
    case CommandContractReason::kRoleStatusMismatch: return "ROLE_STATUS_MISMATCH";
    case CommandContractReason::kPvajOrYawNonFinite: return "PVAJ_OR_YAW_NONFINITE";
    case CommandContractReason::kContinuationContractInvalid: return "CONTINUATION_CONTRACT_INVALID";
    case CommandContractReason::kTrajectoryTimeInvalid: return "TRAJECTORY_TIME_INVALID";
  }
  return "UNRECOGNIZED_CONTRACT_REASON";
}

[[nodiscard]] constexpr const char* commandTemporalReasonName(
    CommandTemporalReason reason) noexcept {
  switch (reason) {
    case CommandTemporalReason::kValid: return "VALID";
    case CommandTemporalReason::kNowInvalid: return "NOW_INVALID";
    case CommandTemporalReason::kInvalidWindow: return "INVALID_WINDOW";
    case CommandTemporalReason::kNotYetValid: return "NOT_YET_VALID";
    case CommandTemporalReason::kExpired: return "EXPIRED";
  }
  return "UNRECOGNIZED_TEMPORAL_REASON";
}

inline std::int64_t commandStampNanoseconds(
    const builtin_interfaces::msg::Time& stamp) noexcept {
  return navigation_common::rosTimeToNanoseconds(stamp).value_or(0);
}

[[nodiscard]] inline CommandTemporalReason assessCommandTemporalLease(
    const NavigationCommand& command, std::int64_t now_ns) noexcept {
  if (now_ns <= 0) return CommandTemporalReason::kNowInvalid;
  const auto header_ns = commandStampNanoseconds(command.header.stamp);
  const auto valid_until_ns = commandStampNanoseconds(command.valid_until);
  if (header_ns <= 0 || valid_until_ns <= header_ns) {
    return CommandTemporalReason::kInvalidWindow;
  }
  if (header_ns > now_ns) return CommandTemporalReason::kNotYetValid;
  if (now_ns > valid_until_ns) return CommandTemporalReason::kExpired;
  return CommandTemporalReason::kValid;
}

inline bool commandValidAt(
    const NavigationCommand& command, std::int64_t now_ns) noexcept {
  return assessCommandTemporalLease(command, now_ns) == CommandTemporalReason::kValid;
}

// Pure lease predicate shared by the PX4 update path and its focused tests.
// All inputs are sampled by the caller; this function owns no freshness or
// lifecycle state and fails closed on any missing lease component.
inline bool continuationWitnessLeaseValid(
    const bool command_valid_at, const std::int64_t now_ns,
    const std::int64_t last_receive_ns, const std::int64_t stale_after_ns,
    const ExecutionStateFreshness& odometry_freshness,
    const ExecutionStateFreshness& health_freshness, const bool health_matches,
    const bool failure_reported,
    const bool mission_terminal, const bool handover_requested) noexcept {
  return command_valid_at && now_ns > 0 && last_receive_ns > 0 &&
         stale_after_ns >= 0 && now_ns >= last_receive_ns &&
         now_ns - last_receive_ns <= stale_after_ns &&
         odometry_freshness.valid() && health_freshness.valid() && health_matches &&
         !failure_reported &&
         !mission_terminal && !handover_requested;
}

inline bool commandMissionIdentityMatches(
    const NavigationCommand& command, std::string_view mission_id,
    std::uint32_t waypoint_index, std::uint64_t request_id) noexcept {
  return !mission_id.empty() && command.mission_id == mission_id &&
         command.waypoint_index == waypoint_index && command.request_id == request_id;
}

inline bool certifiedMainContinuationFieldsValid(
    const NavigationCommand& command) noexcept {
  const auto boundary_ns = command.continuation_boundary_stamp_ns;
  if (!command.certified_main_continuation) return boundary_ns == 0U;
  return command.role == NavigationCommand::ROLE_MAIN &&
         command.status == NavigationCommand::STATUS_READY &&
         boundary_ns > 0U;
}

inline bool commandWorldIdentityNonRegressing(
    const NavigationCommand& current,
    const NavigationCommand& previous) noexcept {
  if (current.localization_epoch < previous.localization_epoch ||
      current.goal_epoch < previous.goal_epoch ||
      current.world_generation < previous.world_generation) {
    return false;
  }
  if (current.world_generation == previous.world_generation &&
      (current.world_revision < previous.world_revision ||
       commandStampNanoseconds(current.world_observation_stamp) <
           commandStampNanoseconds(previous.world_observation_stamp))) {
    return false;
  }
  if (current.localization_epoch == previous.localization_epoch &&
      commandStampNanoseconds(current.state_source_stamp) <
          commandStampNanoseconds(previous.state_source_stamp)) {
    return false;
  }
  return true;
}

[[nodiscard]] inline CommandContractReason assessCommandContract(
    const NavigationCommand& command, std::string_view expected_frame) noexcept {
  const auto header_ns = commandStampNanoseconds(command.header.stamp);
  const auto valid_until_ns = commandStampNanoseconds(command.valid_until);
  const auto world_stamp_ns = commandStampNanoseconds(command.world_observation_stamp);
  const auto state_stamp_ns = commandStampNanoseconds(command.state_source_stamp);
  const bool rejected = command.status == NavigationCommand::STATUS_REJECTED;
  const bool normal = command.status == NavigationCommand::STATUS_READY ||
                      command.status == NavigationCommand::STATUS_COMPLETED;
  const bool braking = command.status == NavigationCommand::STATUS_BRAKING;
  const bool pvaj_finite = std::isfinite(command.position.x) &&
                           std::isfinite(command.position.y) &&
                           std::isfinite(command.position.z) &&
                           std::isfinite(command.velocity.x) &&
                           std::isfinite(command.velocity.y) &&
                           std::isfinite(command.velocity.z) &&
                           std::isfinite(command.acceleration.x) &&
                           std::isfinite(command.acceleration.y) &&
                           std::isfinite(command.acceleration.z) &&
                           std::isfinite(command.jerk.x) &&
                           std::isfinite(command.jerk.y) &&
                           std::isfinite(command.jerk.z) &&
                           std::isfinite(command.yaw) &&
                           std::isfinite(command.yaw_rate) &&
                           std::isfinite(command.trajectory_time_s);
  const bool role_valid = (normal &&
                           (command.role == NavigationCommand::ROLE_MAIN ||
                            command.role == NavigationCommand::ROLE_BACKUP) &&
                           command.bundle_generation != 0U) ||
                          (braking && command.role == NavigationCommand::ROLE_EMERGENCY &&
                           command.bundle_generation != 0U) ||
                          (rejected && command.role == NavigationCommand::ROLE_EMERGENCY);
  if (expected_frame.empty()) return CommandContractReason::kExpectedFrameMissing;
  if (command.mission_id.empty()) return CommandContractReason::kMissionIdEmpty;
  if (command.header.frame_id != expected_frame) return CommandContractReason::kFrameMismatch;
  if (header_ns <= 0) return CommandContractReason::kHeaderStampInvalid;
  if (valid_until_ns <= header_ns) return CommandContractReason::kValidityWindowInvalid;
  if (world_stamp_ns <= 0) return CommandContractReason::kWorldStampInvalid;
  if (state_stamp_ns <= 0) return CommandContractReason::kStateStampInvalid;
  if (command.localization_epoch == 0U) {
    return CommandContractReason::kLocalizationIdentityInvalid;
  }
  if (command.goal_epoch == 0U) return CommandContractReason::kGoalIdentityInvalid;
  if (command.world_generation == 0U || command.world_revision == 0U) {
    return CommandContractReason::kWorldIdentityInvalid;
  }
  if (command.sample_id == 0U) return CommandContractReason::kSampleIdentityInvalid;
  if (!(normal || braking || rejected)) return CommandContractReason::kStatusInvalid;
  if (!role_valid) return CommandContractReason::kRoleStatusMismatch;
  if (!pvaj_finite) return CommandContractReason::kPvajOrYawNonFinite;
  if (!certifiedMainContinuationFieldsValid(command)) {
    return CommandContractReason::kContinuationContractInvalid;
  }
  if (command.trajectory_time_s < 0.0) {
    return CommandContractReason::kTrajectoryTimeInvalid;
  }
  return CommandContractReason::kValid;
}

inline bool commandContractValid(
    const NavigationCommand& command, std::string_view expected_frame) noexcept {
  return assessCommandContract(command, expected_frame) == CommandContractReason::kValid;
}

}  // namespace navigation_contracts

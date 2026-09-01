#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

#include <builtin_interfaces/msg/time.hpp>
#include <navigation_common/time.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>

namespace navigation_contracts {

using msg::NavigationCommand;

inline std::int64_t commandStampNanoseconds(
    const builtin_interfaces::msg::Time& stamp) noexcept {
  return navigation_common::rosTimeToNanoseconds(stamp).value_or(0);
}

inline bool commandValidAt(
    const NavigationCommand& command, std::int64_t now_ns) noexcept {
  if (now_ns <= 0) return false;
  const auto header_ns = commandStampNanoseconds(command.header.stamp);
  const auto valid_until_ns = commandStampNanoseconds(command.valid_until);
  return header_ns > 0 && header_ns <= now_ns && valid_until_ns > header_ns &&
         now_ns <= valid_until_ns;
}

inline bool commandMissionIdentityMatches(
    const NavigationCommand& command, std::string_view mission_id,
    std::uint32_t waypoint_index, std::uint64_t request_id) noexcept {
  return !mission_id.empty() && command.mission_id == mission_id &&
         command.waypoint_index == waypoint_index && command.request_id == request_id;
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

inline bool commandContractValid(
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
  return !expected_frame.empty() && !command.mission_id.empty() &&
         command.header.frame_id == expected_frame && header_ns > 0 &&
         valid_until_ns > header_ns && world_stamp_ns > 0 && state_stamp_ns > 0 &&
         command.localization_epoch != 0U && command.goal_epoch != 0U &&
         command.world_generation != 0U && command.world_revision != 0U &&
         command.sample_id != 0U && command.status != NavigationCommand::STATUS_EMPTY &&
         (normal || braking || rejected) && role_valid && pvaj_finite &&
         command.trajectory_time_s >= 0.0;
}

}  // namespace navigation_contracts

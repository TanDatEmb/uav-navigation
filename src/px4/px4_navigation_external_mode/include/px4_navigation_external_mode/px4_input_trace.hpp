#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace px4_navigation_external_mode {

enum class Px4InputTraceBoundary : std::uint8_t {
  kTracking = 0U,
  kVelocityOnly = 1U,
  kVelocityHold = 2U,
  kPositionHold = 3U,
};

// Fixed-size, trivially-copyable producer record.  It deliberately contains
// no ROS objects or heap-owned strings so enqueueing it cannot allocate or
// invoke serialization on the setpoint callback path.
struct Px4InputTraceRecord final {
  static constexpr std::size_t kMissionIdCapacity = 64U;
  static constexpr std::size_t kReasonCapacity = 96U;

  std::uint64_t trace_sequence{0U};
  std::uint64_t sample_id{0U};
  std::uint64_t request_id{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t localization_epoch{0U};
  std::uint64_t bundle_generation{0U};
  std::uint64_t causal_planning_cycle_id{0U};
  std::uint64_t world_generation{0U};
  std::uint64_t world_revision{0U};
  std::uint64_t velocity_only_limited_count{0U};
  std::uint32_t waypoint_index{0U};
  std::uint8_t role{0U};
  Px4InputTraceBoundary boundary{Px4InputTraceBoundary::kTracking};
  bool command_present{false};
  bool position_present{false};
  bool velocity_present{false};
  bool acceleration_present{false};
  bool yaw_present{false};
  bool yaw_rate_present{false};
  std::int64_t update_start_ros_ns{0};
  std::int64_t update_end_ros_ns{0};
  std::int64_t update_start_steady_ns{0};
  std::int64_t update_end_steady_ns{0};
  std::int64_t world_observation_stamp_ns{0};
  float position_ned[3]{0.0F, 0.0F, 0.0F};
  float velocity_ned[3]{0.0F, 0.0F, 0.0F};
  float acceleration_ned[3]{0.0F, 0.0F, 0.0F};
  float yaw_ned{0.0F};
  float yaw_rate_ned{0.0F};
  std::array<char, kMissionIdCapacity> mission_id{};
  std::array<char, kReasonCapacity> velocity_only_reason{};
};

static_assert(std::is_trivially_copyable_v<Px4InputTraceRecord>);

}  // namespace px4_navigation_external_mode

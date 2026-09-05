#pragma once

#include <cstdint>
#include <string>

namespace navigation_runtime {

// A completion is an observation about one immutable execution bundle.  It is
// deliberately a value rather than a global readiness bit: callers must match
// every identity component before consuming it.
struct TrajectoryCompletionWitness final {
  std::uint64_t bundle_generation{0U};
  std::uint64_t timeline_version{0U};
  std::uint64_t localization_epoch{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  std::string mission_id;
  std::uint32_t waypoint_index{0U};

  [[nodiscard]] bool valid() const noexcept {
    return bundle_generation != 0U && timeline_version != 0U &&
           localization_epoch != 0U &&
           goal_epoch != 0U && request_id != 0U && !mission_id.empty();
  }
};

[[nodiscard]] inline bool completionWitnessMatches(
    const TrajectoryCompletionWitness& witness,
    const std::uint64_t bundle_generation,
    const std::uint64_t timeline_version,
    const std::uint64_t localization_epoch,
    const std::uint64_t goal_epoch,
    const std::uint64_t request_id,
    const std::string& mission_id,
    const std::uint32_t waypoint_index) noexcept {
  return witness.valid() && witness.bundle_generation == bundle_generation &&
         witness.timeline_version == timeline_version &&
         witness.localization_epoch == localization_epoch &&
         witness.goal_epoch == goal_epoch && witness.request_id == request_id &&
         witness.mission_id == mission_id && witness.waypoint_index == waypoint_index;
}

[[nodiscard]] inline bool operator==(
    const TrajectoryCompletionWitness& lhs,
    const TrajectoryCompletionWitness& rhs) noexcept {
  return lhs.bundle_generation == rhs.bundle_generation &&
         lhs.timeline_version == rhs.timeline_version &&
         lhs.localization_epoch == rhs.localization_epoch &&
         lhs.goal_epoch == rhs.goal_epoch && lhs.request_id == rhs.request_id &&
         lhs.mission_id == rhs.mission_id && lhs.waypoint_index == rhs.waypoint_index;
}

}  // namespace navigation_runtime

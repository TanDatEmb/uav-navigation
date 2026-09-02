#pragma once

#include <cstdint>

#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_planning {

// Immutable state sampled from the execution-owned command at the planned
// splice time.  This lives in the planning contract package so a request can
// carry the exact handoff witness without making planning depend on runtime.
struct ExecutionAnchor final {
  std::uint64_t active_bundle_generation{0U};
  std::uint64_t localization_epoch{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  std::int64_t request_stamp_ns{0};
  std::int64_t activation_stamp_ns{0};
  TrajectoryPoint state{};
  CandidateRole active_role{CandidateRole::kMain};
  std::int64_t active_main_end_ns{0};
  std::int64_t active_bundle_end_ns{0};
  navigation_world_model::WorldSnapshotIdentity command_world{};

  [[nodiscard]] bool valid() const noexcept {
    return active_bundle_generation != 0U && localization_epoch != 0U &&
           goal_epoch != 0U && request_id != 0U && request_stamp_ns > 0 &&
           activation_stamp_ns >= request_stamp_ns && state.finite() &&
           candidateRoleValid(active_role) &&
           active_main_end_ns >= activation_stamp_ns &&
           active_bundle_end_ns >= active_main_end_ns &&
           command_world.localization_epoch == localization_epoch &&
           command_world.generation != 0U && command_world.revision != 0U &&
           command_world.observation_stamp_ns > 0;
  }
};

}  // namespace navigation_planning

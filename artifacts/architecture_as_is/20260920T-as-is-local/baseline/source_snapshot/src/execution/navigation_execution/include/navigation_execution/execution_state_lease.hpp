#pragma once

#include <cstdint>

#include <navigation_planning/kinematic_state.hpp>

namespace navigation_execution {

// Immutable ingress metadata kept beside, but outside, the physical planner
// state. The sequence identifies an accepted runtime delivery; it is not a
// substitute for source/receive freshness or localization-epoch validation.
struct ExecutionStateLease final {
  navigation_planning::KinematicState state;
  std::uint64_t ingress_sequence{0};
};

}  // namespace navigation_execution

#pragma once

#include <cstddef>
#include <cstdint>

#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_runtime {

// Producer facts reduced to the immutable values needed to decide whether a
// pass-through boundary can carry a nominal MAIN continuation certificate.
// Sampling finiteness and route decoding remain producer checks at the call
// site; this predicate owns no state and cannot authorize a command by itself.
struct CertifiedMainContinuationBoundaryFacts final {
  bool pass_through_goal{false};
  bool coincident_terminal_stop{false};
  bool trajectory_finished{false};
  bool has_boundary_event{false};
  bool has_boundary_constraint{false};
  std::uint64_t candidate_localization_epoch{0U};
  std::uint64_t expected_localization_epoch{0U};
  std::uint64_t candidate_goal_epoch{0U};
  std::uint64_t expected_goal_epoch{0U};
  std::uint64_t candidate_request_id{0U};
  std::uint64_t expected_request_id{0U};
  navigation_planning::RouteBoundaryEventKind boundary_kind{
      navigation_planning::RouteBoundaryEventKind::kPassThrough};
  navigation_planning::CandidateRole boundary_role{
      navigation_planning::CandidateRole::kEmergency};
  std::size_t boundary_junction_index{0U};
  std::size_t constraint_junction_index{0U};
  std::size_t expected_junction_index{0U};
  std::int64_t boundary_stamp_ns{0};
  std::int64_t declared_start_ns{0};
  std::int64_t declared_end_ns{0};
  std::int64_t main_interval_begin_ns{-1};
  std::int64_t main_interval_end_ns{-1};
};

[[nodiscard]] inline bool certifiedMainContinuationBoundaryEligible(
    const CertifiedMainContinuationBoundaryFacts& facts) noexcept {
  if (!facts.pass_through_goal || facts.coincident_terminal_stop ||
      facts.trajectory_finished ||
      !facts.has_boundary_event || !facts.has_boundary_constraint ||
      facts.candidate_localization_epoch == 0U ||
      facts.candidate_localization_epoch != facts.expected_localization_epoch ||
      facts.candidate_goal_epoch == 0U ||
      facts.candidate_goal_epoch != facts.expected_goal_epoch ||
      facts.candidate_request_id == 0U ||
      facts.candidate_request_id != facts.expected_request_id ||
      facts.boundary_kind != navigation_planning::RouteBoundaryEventKind::kPassThrough ||
      facts.boundary_role != navigation_planning::CandidateRole::kMain ||
      facts.boundary_junction_index != facts.expected_junction_index ||
      facts.constraint_junction_index != facts.expected_junction_index ||
      facts.boundary_stamp_ns <= 0 ||
      facts.declared_end_ns <= facts.declared_start_ns ||
      facts.boundary_stamp_ns < facts.declared_start_ns ||
      facts.boundary_stamp_ns >= facts.declared_end_ns) {
    return false;
  }
  const auto boundary_offset_ns = facts.boundary_stamp_ns - facts.declared_start_ns;
  return facts.main_interval_begin_ns >= 0 &&
         facts.main_interval_end_ns > facts.main_interval_begin_ns &&
         boundary_offset_ns >= facts.main_interval_begin_ns &&
         boundary_offset_ns < facts.main_interval_end_ns &&
         facts.main_interval_end_ns > boundary_offset_ns;
}

}  // namespace navigation_runtime

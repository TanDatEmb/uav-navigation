#include "navigation_runtime/certified_continuation.hpp"

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

CertifiedMainContinuationBoundaryFacts validFacts() {
  CertifiedMainContinuationBoundaryFacts facts;
  facts.pass_through_goal = true;
  facts.has_boundary_event = true;
  facts.has_boundary_constraint = true;
  facts.candidate_localization_epoch = 3U;
  facts.expected_localization_epoch = 3U;
  facts.candidate_goal_epoch = 5U;
  facts.expected_goal_epoch = 5U;
  facts.candidate_request_id = 7U;
  facts.expected_request_id = 7U;
  facts.boundary_role = navigation_planning::CandidateRole::kMain;
  facts.boundary_junction_index = 2U;
  facts.constraint_junction_index = 2U;
  facts.expected_junction_index = 2U;
  facts.boundary_stamp_ns = 1'500'000'000LL;
  facts.declared_start_ns = 1'000'000'000LL;
  facts.declared_end_ns = 3'000'000'000LL;
  facts.main_interval_begin_ns = 0LL;
  facts.main_interval_end_ns = 2'000'000'000LL;
  return facts;
}

TEST(CertifiedContinuation, AcceptsCurrentMainBoundaryWithContinuation) {
  EXPECT_TRUE(certifiedMainContinuationBoundaryEligible(validFacts()));
}

TEST(CertifiedContinuation, RejectsBoundaryAtMainToBackupBoundary) {
  auto facts = validFacts();
  facts.boundary_stamp_ns = 3'000'000'000LL;
  facts.main_interval_end_ns = 2'000'000'000LL;
  facts.declared_end_ns = 4'000'000'000LL;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
}

TEST(CertifiedContinuation, RejectsBoundarySampledInBackup) {
  auto facts = validFacts();
  facts.boundary_role = navigation_planning::CandidateRole::kBackup;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
}

TEST(CertifiedContinuation, RejectsTerminalAndCoincidentPassThrough) {
  auto terminal = validFacts();
  terminal.boundary_kind = navigation_planning::RouteBoundaryEventKind::kTerminalStop;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(terminal));

  auto coincident = validFacts();
  coincident.coincident_terminal_stop = true;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(coincident));

  auto finished = validFacts();
  finished.trajectory_finished = true;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(finished));
}

TEST(CertifiedContinuation, RejectsAnyStaleIdentityOrJunction) {
  auto facts = validFacts();
  facts.candidate_request_id = 8U;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
  facts = validFacts();
  facts.candidate_goal_epoch = 6U;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
  facts = validFacts();
  facts.candidate_localization_epoch = 4U;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
  facts = validFacts();
  facts.boundary_junction_index = 1U;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
  facts = validFacts();
  facts.constraint_junction_index = 1U;
  EXPECT_FALSE(certifiedMainContinuationBoundaryEligible(facts));
}

}  // namespace
}  // namespace navigation_runtime

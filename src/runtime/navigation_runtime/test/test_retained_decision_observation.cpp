#include "navigation_runtime/retained_decision_observation.hpp"

#include <algorithm>
#include <stdexcept>

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

std::string value(const diagnostic_msgs::msg::DiagnosticStatus& status, const char* key) {
  const auto entry = std::find_if(status.values.begin(), status.values.end(),
      [key](const auto& item) { return item.key == key; });
  return entry == status.values.end() ? "MISSING" : entry->value;
}

TEST(RetainedDecisionObservation, RevocationIsVisibleWithoutACommandOrOptimizerJob) {
  ExecutionTraceSnapshot trace;
  trace.planning_cycle_id = 307U;
  trace.solve_generation = 0U;
  trace.execution_localization_epoch = 1U;
  trace.execution_goal_epoch = 2U;
  trace.execution_request_id = 3U;
  trace.execution_bundle_generation = 21U;
  trace.anchor_error_raw_m = 0.750003332;
  RetainedDecisionObservation decision;
  decision.purpose = 2U;  // terminal monitor, not an optimizer attempt
  decision.disposition = RetainedDecisionDisposition::kFailClosed;
  decision.delivery_evaluated = true;
  decision.world_validation_attempted = true;
  decision.world_validation.evaluated_generation = 21U;
  decision.world_validation.validated_world = {1U, 2U, 10U, 52'528'000'000LL};
  decision.after_failure_latched = true;
  RetainedObservationAccounting accounting;
  bool received = false;
  ASSERT_TRUE(tryEmitRetainedDecision(trace, decision, true, accounting, [&](auto status) {
    received = true;
    EXPECT_EQ(value(status, "solve_generation"), "0");
    EXPECT_EQ(value(status, "expected_bundle_generation"), "21");
    EXPECT_EQ(value(status, "evaluated_bundle_generation"), "21");
    EXPECT_EQ(value(status, "after_bundle_generation"), "0");
    EXPECT_EQ(value(status, "after_failure_latched"), "1");
    EXPECT_EQ(value(status, "aligned_error_m"), "NOT_EVALUABLE");
    EXPECT_DOUBLE_EQ(std::stod(value(status, "raw_error_m")), trace.anchor_error_raw_m);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  }));
  EXPECT_TRUE(received);
  EXPECT_EQ(accounting.attempted, 1U);
  EXPECT_EQ(accounting.published, 1U);
}

TEST(RetainedDecisionObservation, DistinguishesPreparationAdmissionAndIdentityDelivery) {
  ExecutionTraceSnapshot trace;
  RetainedDecisionObservation decision;
  decision.emergency_preparation_attempted = true;
  decision.emergency_prepared = true;
  decision.emergency_admission_attempted = true;
  RetainedObservationAccounting accounting;
  auto rejected = retainedDecisionDiagnostic(trace, decision, accounting);
  EXPECT_EQ(value(rejected, "emergency_prepared"), "1");
  EXPECT_EQ(value(rejected, "emergency_store_admitted"), "0");
  decision.emergency_store_admitted = true;
  auto admitted_but_superseded = retainedDecisionDiagnostic(trace, decision, accounting);
  EXPECT_EQ(value(admitted_but_superseded, "emergency_store_admitted"), "1");
  EXPECT_EQ(value(admitted_but_superseded, "emergency_identity_delivered"), "0");
}

TEST(RetainedDecisionObservation, KeepsEvaluatedOwnerAndWorldSeparateFromDelivery) {
  ExecutionTraceSnapshot trace;
  trace.execution_bundle_generation = 21U;
  RetainedDecisionObservation decision;
  decision.captured_world = {1U, 2U, 10U, 10'000'000'000LL};
  decision.world_validation_attempted = true;
  decision.world_validation.evaluated_generation = 20U;  // lost ACK is observable
  decision.world_validation.validated_world = {1U, 2U, 11U, 10'100'000'000LL};
  decision.world_validation.blocking_cell_observed = false;
  decision.world_validation.first_blocked_cell_state = 1;  // not a cell witness
  decision.world_validation.evaluated_unknown_policy = 0;
  decision.after_bundle_generation = 22U;
  decision.delivery_evaluated = true;
  decision.disposition = RetainedDecisionDisposition::kDiscarded;
  auto status = retainedDecisionDiagnostic(trace, decision, {});
  EXPECT_EQ(value(status, "expected_bundle_generation"), "21");
  EXPECT_EQ(value(status, "evaluated_bundle_generation"), "20");
  EXPECT_EQ(value(status, "after_bundle_generation"), "22");
  EXPECT_EQ(value(status, "captured_world_revision"), "10");
  EXPECT_EQ(value(status, "validated_world_revision"), "11");
  EXPECT_EQ(value(status, "world_cell_observed"), "0");
  EXPECT_EQ(value(status, "world_cell_state"), "NOT_EVALUABLE");
  EXPECT_EQ(value(status, "world_unknown_policy"), "0");
}

TEST(RetainedDecisionObservation, AnUnattemptedBoundaryDoesNotInventZeroAgesOrSuccess) {
  ExecutionTraceSnapshot trace;
  RetainedDecisionObservation decision;
  decision.disposition = RetainedDecisionDisposition::kEntryRejected;
  auto status = retainedDecisionDiagnostic(trace, decision, {});
  EXPECT_EQ(value(status, "delivery_evaluated"), "0");
  EXPECT_EQ(value(status, "source_age_ms"), "NOT_EVALUABLE");
  EXPECT_EQ(value(status, "final_source_age_ms"), "NOT_EVALUABLE");
  EXPECT_EQ(value(status, "evaluated_bundle_generation"), "NOT_EVALUABLE");
  EXPECT_EQ(value(status, "world_failure_code"), "NOT_EVALUABLE");
  EXPECT_EQ(value(status, "after_failure_latched"), "NOT_EVALUABLE");
}

TEST(RetainedDecisionObservation, SupersededEntryKeepsExpectedAndCapturedOwnersSeparate) {
  ExecutionTraceSnapshot trace;
  trace.execution_bundle_generation = 21U;
  RetainedDecisionObservation decision;
  decision.disposition = RetainedDecisionDisposition::kEntryRejected;
  decision.captured_bundle_generation = 22U;
  decision.expected_world = {1U, 2U, 10U, 10'000'000'000LL};
  decision.captured_world = {1U, 2U, 11U, 10'100'000'000LL};
  decision.expected_end_ns = 10'400'000'000LL;
  decision.declared_end_ns = 11'500'000'000LL;
  auto status = retainedDecisionDiagnostic(trace, decision, {});
  EXPECT_EQ(value(status, "expected_bundle_generation"), "21");
  EXPECT_EQ(value(status, "captured_bundle_generation"), "22");
  EXPECT_EQ(value(status, "expected_world_revision"), "10");
  EXPECT_EQ(value(status, "captured_world_revision"), "11");
  EXPECT_EQ(value(status, "expected_end_ros_ns"), "10400000000");
  EXPECT_EQ(value(status, "captured_end_ros_ns"), "11500000000");
  EXPECT_EQ(value(status, "evaluated_bundle_generation"), "NOT_EVALUABLE");
}

TEST(RetainedDecisionObservation, DisabledOrThrowingSinkCannotChangeDecision) {
  ExecutionTraceSnapshot trace;
  trace.execution_bundle_generation = 21U;
  RetainedDecisionObservation decision;
  decision.disposition = RetainedDecisionDisposition::kCertifiedCommandPreserved;
  decision.after_bundle_generation = 21U;
  RetainedObservationAccounting accounting;
  bool sink_called = false;
  EXPECT_FALSE(tryEmitRetainedDecision(trace, decision, false, accounting,
      [&](auto) { sink_called = true; }));
  EXPECT_FALSE(sink_called);
  EXPECT_EQ(accounting.suppressed, 1U);
  EXPECT_FALSE(tryEmitRetainedDecision(trace, decision, true, accounting,
      [](auto) { throw std::runtime_error("observer delivery failed"); }));
  EXPECT_EQ(accounting.failed, 1U);
  EXPECT_EQ(decision.after_bundle_generation, 21U);
  EXPECT_EQ(decision.disposition, RetainedDecisionDisposition::kCertifiedCommandPreserved);
  ASSERT_TRUE(tryEmitRetainedDecision(trace, decision, true, accounting, [&](auto status) {
    EXPECT_EQ(value(status, "event_sequence"), "3");
    EXPECT_EQ(value(status, "failed_before_event"), "1");
    EXPECT_EQ(value(status, "suppressed_before_event"), "1");
  }));
  EXPECT_EQ(accounting.attempted, 3U);
  EXPECT_EQ(accounting.published, 1U);
}

}  // namespace
}  // namespace navigation_runtime

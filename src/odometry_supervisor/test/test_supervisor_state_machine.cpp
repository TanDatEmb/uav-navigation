#include <gtest/gtest.h>

#include <stdexcept>

#include "odometry_supervisor/supervisor_state_machine.hpp"

namespace {
odometry_supervisor::EvaluationInput healthy_input(std::int64_t time) {
  odometry_supervisor::EvaluationInput input;
  input.evaluation_time_ns = time;
  input.lio_valid = true;
  input.propagated_fresh = true;
  input.corrected_fresh = true;
  input.px4_available = true;
  input.px4_fresh = true;
  input.px4_continuity_valid = true;
  input.px4_post_reset_stable = true;
  input.origin_aligned = true;
  input.lio_diagnostics_valid = true;
  input.px4_diagnostics_valid = true;
  input.lio_diagnostics_schema_valid = true;
  input.px4_diagnostics_schema_valid = true;
  input.alignment_gap_ns = 0;
  input.comparison_epoch_ns = time;
  input.new_comparison_sample = true;
  input.aligned_comparison_fresh = true;
  input.residual.valid = true;
  input.px4_reset_generation = 1;
  input.px4_time_generation = 1;
  return input;
}

odometry_supervisor::SupervisorOutput evaluate(
    odometry_supervisor::SupervisorStateMachine& machine,
    std::int64_t time, double position_error = 0.0) {
  auto input = healthy_input(time);
  input.residual.position_error_m = position_error;
  return machine.evaluate(input);
}
}  // namespace

TEST(OdometrySupervisorStateMachine, HealthyAndPx4UnavailableDoesNotInvalidateLio) {
  odometry_supervisor::SupervisorStateMachine machine;
  auto input = healthy_input(1'000'000'000);
  input.px4_available = false;
  input.px4_fresh = false;
  input.px4_continuity_valid = false;
  input.origin_aligned = false;
  input.lio_diagnostics_valid = false;
  input.px4_diagnostics_valid = false;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kHealthy);
  EXPECT_FALSE(output.monitoring_available);
  EXPECT_TRUE(output.external_odometry_allowed);
  EXPECT_FALSE(output.reinitialization_requested);
}

TEST(OdometrySupervisorStateMachine, SingleOutlierDoesNotLeaveHealthy) {
  odometry_supervisor::SupervisorStateMachine machine;
  ASSERT_EQ(evaluate(machine, 1'000'000'000).health,
            odometry_supervisor::HealthState::kHealthy);
  const auto output = evaluate(machine, 1'100'000'000, 2.0);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kHealthy);
}

TEST(OdometrySupervisorStateMachine, PersistentDriftTransitionsThroughStates) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  EXPECT_EQ(evaluate(machine, 1'100'000'000, 0.4).health,
            odometry_supervisor::HealthState::kHealthy);
  EXPECT_EQ(evaluate(machine, 1'400'000'000, 0.4).health,
            odometry_supervisor::HealthState::kSuspect);
  EXPECT_EQ(evaluate(machine, 1'500'000'000, 1.0).health,
            odometry_supervisor::HealthState::kSuspect);
  EXPECT_EQ(evaluate(machine, 2'300'000'000, 1.0).health,
            odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(evaluate(machine, 2'400'000'000, 1.0).external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, PersistentDivergenceLatchesAndRequestsIndependentReinit) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  evaluate(machine, 1'100'000'000, 1.6);
  const auto output = evaluate(machine, 2'600'000'000, 1.6);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_TRUE(output.reinitialization_requested);
  EXPECT_EQ(output.reinitialization_request_sequence, 1U);
  EXPECT_TRUE(output.hover_or_failsafe_requested);
  const auto latched = evaluate(machine, 3'000'000'000, 0.0);
  EXPECT_EQ(latched.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_EQ(latched.reinitialization_request_sequence, 1U);
}

TEST(OdometrySupervisorStateMachine, CorrelatedDivergenceDoesNotRequestReinit) {
  odometry_supervisor::SupervisorConfig config;
  config.reference_mode = odometry_supervisor::ReferenceMode::kCorrelated;
  odometry_supervisor::SupervisorStateMachine machine(config);
  evaluate(machine, 1'000'000'000);
  evaluate(machine, 1'100'000'000, 1.6);
  const auto output = evaluate(machine, 2'600'000'000, 1.6);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_FALSE(output.reinitialization_requested);
}

TEST(OdometrySupervisorStateMachine, LioLostImmediatelyClosesGateInBothReferenceModes) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.lio_lost = true;
  input.lio_valid = false;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_FALSE(output.external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, LioDegradedClosesGate) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.lio_valid = false;
  input.lio_degraded = true;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(output.external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, StalePropagatedLioEventuallyClosesGate) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.propagated_fresh = false;
  input.lio_valid = false;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  input.evaluation_time_ns = 1'900'000'000;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(output.external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, StalePropagatedLioCannotRecoverOnHeldResidual) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.propagated_fresh = false;
  input.lio_valid = false;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  input.evaluation_time_ns = 1'900'000'000;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kDegraded);
  input.evaluation_time_ns = 5'000'000'000;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(output.comparison_valid);
  EXPECT_FALSE(output.external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, CorrelatedAgreementCannotOverrideLioLost) {
  odometry_supervisor::SupervisorConfig config;
  config.reference_mode = odometry_supervisor::ReferenceMode::kCorrelated;
  odometry_supervisor::SupervisorStateMachine machine(config);
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.lio_valid = false;
  input.lio_lost = true;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_FALSE(output.external_odometry_allowed);
  EXPECT_FALSE(output.reinitialization_requested);
}

TEST(OdometrySupervisorStateMachine, ResetGraceClearsPersistenceWithoutFalseDivergence) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  evaluate(machine, 1'100'000'000, 1.6);
  auto input = healthy_input(1'200'000'000);
  input.px4_reset_generation = 2;
  input.residual.position_error_m = 1.6;
  const auto output = machine.evaluate(input);
  EXPECT_NE(output.health, odometry_supervisor::HealthState::kDiverged);
  EXPECT_EQ(output.reason, "PX4_RESET_GRACE");
}

TEST(OdometrySupervisorStateMachine, Px4ResetDoesNotReopenDegradedGate) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  evaluate(machine, 1'100'000'000, 1.0);
  ASSERT_EQ(evaluate(machine, 1'900'000'000, 1.0).health,
            odometry_supervisor::HealthState::kDegraded);
  auto input = healthy_input(2'000'000'000);
  input.px4_reset_generation = 2;
  input.residual.position_error_m = 1.0;
  const auto output = machine.evaluate(input);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(output.external_odometry_allowed);
  EXPECT_EQ(output.reason, "PX4_RESET_GRACE");
}

TEST(OdometrySupervisorStateMachine, TimeGenerationChangeInvalidatesComparison) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.px4_time_generation = 2;
  input.time_generation_changed = true;
  input.residual.position_error_m = 1.6;
  const auto output = machine.evaluate(input);
  EXPECT_FALSE(output.comparison_valid);
  EXPECT_NE(output.health, odometry_supervisor::HealthState::kDiverged);
}

TEST(OdometrySupervisorStateMachine, ClockPauseDoesNotAdvancePersistence) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  EXPECT_EQ(evaluate(machine, 1'100'000'000, 0.4).health,
            odometry_supervisor::HealthState::kHealthy);
  EXPECT_EQ(evaluate(machine, 1'100'000'000, 0.4).health,
            odometry_supervisor::HealthState::kHealthy);
}

TEST(OdometrySupervisorStateMachine, HeldComparisonEpochCannotAdvanceResidualPersistence) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.residual.position_error_m = 0.4;
  ASSERT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  input.evaluation_time_ns = 1'500'000'000;
  input.new_comparison_sample = false;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  input.comparison_epoch_ns = 1'400'000'000;
  input.new_comparison_sample = true;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kSuspect);
}

TEST(OdometrySupervisorStateMachine, StaleComparisonCannotTriggerResidualState) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.residual.position_error_m = 2.0;
  input.aligned_comparison_fresh = false;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  EXPECT_EQ(machine.evaluate(input).reason, "ALIGNED_COMPARISON_STALE");
}

TEST(OdometrySupervisorStateMachine, PersistentLioDiagnosticInvalidityDegradesAfterConfiguredWindow) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.lio_diagnostics_valid = false;
  input.lio_diagnostics_schema_valid = true;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kHealthy);
  input.evaluation_time_ns = 1'600'000'000;
  EXPECT_EQ(machine.evaluate(input).health, odometry_supervisor::HealthState::kDegraded);
  EXPECT_FALSE(machine.evaluate(input).external_odometry_allowed);
}

TEST(OdometrySupervisorStateMachine, InvalidComparisonDoesNotBecomeZeroResidual) {
  odometry_supervisor::SupervisorStateMachine machine;
  evaluate(machine, 1'000'000'000);
  auto input = healthy_input(1'100'000'000);
  input.residual.valid = false;
  input.origin_aligned = false;
  const auto output = machine.evaluate(input);
  EXPECT_FALSE(output.comparison_valid);
  EXPECT_EQ(output.health, odometry_supervisor::HealthState::kHealthy);
}

TEST(OdometrySupervisorStateMachine, ConfigurationRejectsInvalidOrdering) {
  odometry_supervisor::SupervisorConfig config;
  config.degraded.position_m = config.suspect.position_m;
  EXPECT_THROW({ odometry_supervisor::SupervisorStateMachine machine(config); },
               std::invalid_argument);
}

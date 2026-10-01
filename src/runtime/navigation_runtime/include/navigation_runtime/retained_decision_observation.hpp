#pragma once

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <navigation_planning/planner_diagnostics.hpp>

#include "navigation_runtime/execution_trace_snapshot.hpp"

namespace navigation_runtime {

// Factual delivery, not a transition request or a second execution owner.
enum class RetainedDecisionDisposition : std::uint8_t {
  kNotDelivered,
  kEntryRejected,
  kDiscarded,
  kSuperseded,
  kFailClosed,
  kEmergencyDelivered,
  kRecoveryBridgePreserved,
  kMainBridgePreserved,
  kCertifiedCommandPreserved,
};

struct RetainedDecisionObservation final {
  // The planning attempt can target desired N+1 while the validated active
  // execution still belongs to predecessor N. Keep both identities explicit.
  std::uint64_t desired_request_id{0U};
  std::uint64_t desired_goal_epoch{0U};
  std::uint8_t purpose{0U};
  RetainedDecisionDisposition disposition{RetainedDecisionDisposition::kNotDelivered};
  std::uint64_t captured_timeline_version{0U};
  std::uint64_t captured_bundle_generation{0U};
  navigation_world_model::WorldSnapshotIdentity expected_world{};
  navigation_world_model::WorldSnapshotIdentity captured_world{};
  std::int64_t declared_end_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t expected_end_ns{0};
  std::int64_t expected_valid_until_ns{0};
  bool source_sample_valid{false};
  bool initial_bridge_usable{false};
  std::int64_t initial_freshness_ros_ns{0};
  std::int64_t initial_freshness_steady_ns{0};
  int initial_freshness_reason{-1};
  bool world_validation_attempted{false};
  navigation_planning::TrajectoryValidationResult world_validation{};
  bool emergency_preparation_attempted{false};
  bool emergency_prepared{false};
  bool emergency_admission_attempted{false};
  bool emergency_store_admitted{false};
  bool emergency_identity_delivered{false};
  bool delivery_evaluated{false};
  std::int64_t lock_recheck_ros_ns{0};
  std::int64_t lock_recheck_steady_ns{0};
  std::int64_t final_state_receive_ns{0};
  double final_source_age_ms{std::numeric_limits<double>::quiet_NaN()};
  double final_receive_age_ms{std::numeric_limits<double>::quiet_NaN()};
  int final_freshness_reason{-1};
  bool final_witness_age_bounded{false};
  int final_phase_status{-1};
  int final_path_status{-1};
  bool final_experimental_support_valid{false};
  bool final_body_known_free{false};
  bool final_command_anchor_valid{false};
  bool owner_snapshot_current{false};
  bool callback_request_current{false};
  bool monitor_window_current{false};
  std::uint64_t after_timeline_version{0U};
  std::uint64_t after_bundle_generation{0U};
  std::uint64_t after_episode_generation{0U};
  bool after_command_available{false};
  bool after_failure_latched{false};
};

// Serial PlanningWorker accounting. A successful ROS publish is not a receipt
// from the evidence writer. Receiver coverage/loss must still be evaluated.
struct RetainedObservationAccounting final {
  std::uint64_t attempted{0U};
  std::uint64_t published{0U};
  std::uint64_t suppressed{0U};
  std::uint64_t failed{0U};
  std::int64_t previous_emit_duration_us{0};
};

inline diagnostic_msgs::msg::DiagnosticStatus retainedDecisionDiagnostic(
    const ExecutionTraceSnapshot& trace,
    const RetainedDecisionObservation& decision,
    const RetainedObservationAccounting& accounting) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_runtime/retained_command_decision";
  status.hardware_id = "planning_worker";
  status.level = decision.disposition == RetainedDecisionDisposition::kFailClosed
      ? diagnostic_msgs::msg::DiagnosticStatus::WARN
      : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "diagnostic_only; actual retained evaluation; not an optimizer job";
  status.values.reserve(128U);
  const auto add = [&](const char* key, const auto value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    if constexpr (std::is_floating_point_v<std::decay_t<decltype(value)>>) {
      if (!std::isfinite(value)) {
        entry.value = "NOT_EVALUABLE";
      } else {
        char buffer[64];
        const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
            std::chars_format::general, std::numeric_limits<double>::max_digits10);
        entry.value = result.ec == std::errc{}
            ? std::string(buffer, result.ptr) : std::string("NOT_EVALUABLE");
      }
    } else {
      entry.value = std::to_string(value);
    }
    status.values.push_back(std::move(entry));
  };
  const auto addObserved = [&](const char* key, const auto value, const bool observed) {
    if (observed) {
      add(key, value);
    } else {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = "NOT_EVALUABLE";
      status.values.push_back(std::move(entry));
    }
  };
  add("schema_version", 1);
  add("event_sequence", accounting.attempted);
  add("published_before_event", accounting.published);
  add("suppressed_before_event", accounting.suppressed);
  add("failed_before_event", accounting.failed);
  add("previous_emit_duration_us", accounting.previous_emit_duration_us);
  add("purpose", decision.purpose);
  add("disposition", static_cast<std::uint8_t>(decision.disposition));
  add("planning_cycle_id", trace.planning_cycle_id);
  add("desired_request_id", decision.desired_request_id);
  add("desired_goal_epoch", decision.desired_goal_epoch);
  add("solve_generation", trace.solve_generation);
  add("evaluation_ros_ns", trace.timestamp_ns);
  add("localization_epoch", trace.execution_localization_epoch);
  add("goal_epoch", trace.execution_goal_epoch);
  add("request_id", trace.execution_request_id);
  add("expected_bundle_generation", trace.execution_bundle_generation);
  add("captured_timeline_version", decision.captured_timeline_version);
  add("captured_bundle_generation", decision.captured_bundle_generation);
  add("expected_world_localization_epoch", decision.expected_world.localization_epoch);
  add("expected_world_generation", decision.expected_world.generation);
  add("expected_world_revision", decision.expected_world.revision);
  add("expected_world_observation_ns", decision.expected_world.observation_stamp_ns);
  add("captured_world_localization_epoch", decision.captured_world.localization_epoch);
  add("captured_world_generation", decision.captured_world.generation);
  add("captured_world_revision", decision.captured_world.revision);
  add("captured_world_observation_ns", decision.captured_world.observation_stamp_ns);
  add("state_ingress_sequence", trace.execution_state_ingress_sequence);
  add("state_source_ros_ns", trace.execution_state_source_stamp_ns);
  add("state_receive_steady_ns", trace.execution_state_receive_stamp_ns);
  const bool state_stamps_observed = trace.execution_state_source_stamp_ns > 0 &&
      trace.execution_state_receive_stamp_ns > 0;
  addObserved("source_age_ms", trace.execution_state_source_age_ms, state_stamps_observed);
  addObserved("receive_age_ms", trace.execution_state_receive_age_ms, state_stamps_observed);
  addObserved("freshness_ros_ns", decision.initial_freshness_ros_ns,
      decision.initial_freshness_ros_ns > 0);
  addObserved("freshness_steady_ns", decision.initial_freshness_steady_ns,
      decision.initial_freshness_steady_ns > 0);
  addObserved("freshness_reason", decision.initial_freshness_reason,
      decision.initial_freshness_reason >= 0);
  add("source_sample_valid", decision.source_sample_valid);
  add("start_ros_ns", trace.committed_bundle_start_stamp_ns);
  add("captured_end_ros_ns", decision.declared_end_ns);
  add("captured_valid_until_ros_ns", decision.valid_until_ns);
  add("expected_end_ros_ns", decision.expected_end_ns);
  add("expected_valid_until_ros_ns", decision.expected_valid_until_ns);
  add("duration_s", trace.committed_bundle_duration_s);
  add("elapsed_s", trace.retained_elapsed_s);
  add("state_fresh", trace.retained_fresh_vehicle_state);
  add("body_known_free", trace.current_vehicle_state_known_free);
  add("anchor_valid", trace.retained_command_anchor_valid);
  add("raw_error_m", trace.anchor_error_raw_m);
  add("aligned_error_m", trace.anchor_error_time_aligned_m);
  add("projected_error_m", trace.projected_anchor_error_m);
  add("measured_x_m", trace.measured_position_at_state_source.x());
  add("measured_y_m", trace.measured_position_at_state_source.y());
  add("measured_z_m", trace.measured_position_at_state_source.z());
  add("measured_vx_mps", trace.measured_velocity_at_state_source.x());
  add("measured_vy_mps", trace.measured_velocity_at_state_source.y());
  add("measured_vz_mps", trace.measured_velocity_at_state_source.z());
  add("tracking_limit_m", trace.retained_tracking_limit_m);
  add("backup_available", trace.backup_available);
  add("committed_suffix_usable", trace.committed_suffix_usable);
  add("initial_bridge_usable", decision.initial_bridge_usable);
  add("tracking_certificate_exceeded", trace.tracking_certificate_exceeded);
  add("projected_certificate_exceeded", trace.projected_tracking_certificate_exceeded);
  add("world_validation_attempted", decision.world_validation_attempted);
  const auto& validation = decision.world_validation;
  add("world_validation_valid", validation.valid);
  addObserved("world_reused_unchanged_certificate", validation.reused_unchanged_certificate,
      decision.world_validation_attempted && validation.evaluated_generation > 0U);
  addObserved("evaluated_bundle_generation", validation.evaluated_generation,
      decision.world_validation_attempted && validation.evaluated_generation > 0U);
  add("pinned_world_localization_epoch", validation.pinned_world.localization_epoch);
  add("pinned_world_generation", validation.pinned_world.generation);
  add("pinned_world_revision", validation.pinned_world.revision);
  add("pinned_world_observation_ns", validation.pinned_world.observation_stamp_ns);
  add("validated_world_localization_epoch", validation.validated_world.localization_epoch);
  add("validated_world_generation", validation.validated_world.generation);
  add("validated_world_revision", validation.validated_world.revision);
  add("validated_world_observation_ns", validation.validated_world.observation_stamp_ns);
  const bool world_result_observed = decision.world_validation_attempted &&
      validation.evaluated_generation > 0U;
  const bool world_failure_observed = world_result_observed && !validation.valid &&
      validation.failure_code != 0;
  addObserved("world_failure_code", validation.failure_code,
      world_failure_observed || (world_result_observed && validation.valid));
  addObserved("world_blocked_role", validation.blocked_role, world_failure_observed);
  addObserved("world_sample_count", validation.sample_count, world_result_observed);
  addObserved("world_tube_failure_code", validation.tube_failure_code, world_failure_observed);
  addObserved("world_unknown_policy", validation.evaluated_unknown_policy,
      decision.world_validation_attempted && validation.evaluated_unknown_policy >= 0);
  add("world_cell_observed", validation.blocking_cell_observed);
  addObserved("world_cell_state", validation.first_blocked_cell_state,
      decision.world_validation_attempted && validation.blocking_cell_observed);
  addObserved("world_blocked_time_s", validation.first_blocked_time_s, world_failure_observed);
  add("world_blocked_x_m", validation.first_blocked_position.x());
  add("world_blocked_y_m", validation.first_blocked_position.y());
  add("world_blocked_z_m", validation.first_blocked_position.z());
  add("world_unsafe_interval_end_s", validation.unsafe_interval_end_time_s);
  add("world_curve_bound_m", validation.curve_deviation_bound_m);
  add("world_curve_tolerance_m", validation.curve_deviation_tolerance_m);
  add("emergency_authorization_reason", trace.emergency_authorization_reason);
  add("emergency_preparation_attempted", decision.emergency_preparation_attempted);
  add("emergency_prepared", decision.emergency_prepared);
  add("emergency_admission_attempted", decision.emergency_admission_attempted);
  add("emergency_store_admitted", decision.emergency_store_admitted);
  add("emergency_identity_delivered", decision.emergency_identity_delivered);
  add("delivery_evaluated", decision.delivery_evaluated);
  add("lock_recheck_ros_ns", decision.lock_recheck_ros_ns);
  add("lock_recheck_steady_ns", decision.lock_recheck_steady_ns);
  add("final_state_source_ros_ns", trace.phase_execution_final_source_stamp_ns);
  add("final_state_receive_steady_ns", decision.final_state_receive_ns);
  const bool final_state_stamps_observed = trace.phase_execution_final_source_stamp_ns > 0 &&
      decision.final_state_receive_ns > 0;
  addObserved("final_source_age_ms", decision.final_source_age_ms, final_state_stamps_observed);
  addObserved("final_receive_age_ms", decision.final_receive_age_ms, final_state_stamps_observed);
  add("final_freshness_reason", decision.final_freshness_reason);
  add("final_witness_age_bounded", decision.final_witness_age_bounded);
  add("final_phase_status", decision.final_phase_status);
  add("final_path_status", decision.final_path_status);
  add("final_experimental_support_valid", decision.final_experimental_support_valid);
  add("final_body_known_free", decision.final_body_known_free);
  add("final_anchor_valid", decision.final_command_anchor_valid);
  add("final_source_error_m", trace.phase_execution_final_source_error_m);
  add("final_predicted_error_m", trace.phase_execution_final_predicted_error_m);
  add("final_bridge_usable", trace.phase_execution_bridge_usable);
  add("owner_snapshot_current", decision.owner_snapshot_current);
  add("callback_request_current", decision.callback_request_current);
  add("monitor_window_current", decision.monitor_window_current);
  addObserved("after_timeline_version", decision.after_timeline_version, decision.delivery_evaluated);
  addObserved("after_bundle_generation", decision.after_bundle_generation, decision.delivery_evaluated);
  addObserved("after_episode_generation", decision.after_episode_generation, decision.delivery_evaluated);
  addObserved("after_command_available", decision.after_command_available, decision.delivery_evaluated);
  addObserved("after_failure_latched", decision.after_failure_latched, decision.delivery_evaluated);
  return status;
}

// Allocation/formatting/transport failure in this observer never propagates
// into command admission/revocation. Called only after all owner locks are
// released; callers must not use its result to decide execution behavior.
template <class Sink>
bool tryEmitRetainedDecision(
    const ExecutionTraceSnapshot& trace, const RetainedDecisionObservation& decision,
    bool enabled, RetainedObservationAccounting& accounting, Sink&& sink) noexcept {
  ++accounting.attempted;
  if (!enabled) {
    ++accounting.suppressed;
    return false;
  }
  const auto started = std::chrono::steady_clock::now();
  bool published = false;
  try {
    auto status = retainedDecisionDiagnostic(trace, decision, accounting);
    std::forward<Sink>(sink)(std::move(status));
    ++accounting.published;
    published = true;
  } catch (...) {
    ++accounting.failed;
  }
  accounting.previous_emit_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started).count();
  return published;
}

}  // namespace navigation_runtime

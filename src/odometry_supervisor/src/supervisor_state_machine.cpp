#include "odometry_supervisor/supervisor_state_machine.hpp"

#include <cmath>
#include <stdexcept>

namespace odometry_supervisor {
namespace {
constexpr std::uint16_t kReasonStartup = 1;
constexpr std::uint16_t kReasonHealthy = 2;
constexpr std::uint16_t kReasonMonitoringUnavailable = 3;
constexpr std::uint16_t kReasonOriginNotAligned = 4;
constexpr std::uint16_t kReasonAlignmentFailed = 5;
constexpr std::uint16_t kReasonResidualSuspect = 6;
constexpr std::uint16_t kReasonResidualDegraded = 7;
constexpr std::uint16_t kReasonResidualDiverged = 8;
constexpr std::uint16_t kReasonLioLost = 9;
constexpr std::uint16_t kReasonStateCorruption = 10;
constexpr std::uint16_t kReasonLioResetting = 11;
constexpr std::uint16_t kReasonResetGrace = 12;
constexpr std::uint16_t kReasonSchemaMismatch = 13;
constexpr std::uint16_t kReasonStale = 14;
constexpr std::uint16_t kReasonLioDiagnosticsStale = 15;
constexpr std::uint16_t kReasonLioDiagnosticSchemaMismatch = 16;
constexpr std::uint16_t kReasonPx4DiagnosticsStale = 17;
constexpr std::uint16_t kReasonPx4DiagnosticSchemaMismatch = 18;
constexpr std::uint16_t kReasonAlignedComparisonStale = 19;
}

SupervisorStateMachine::SupervisorStateMachine(SupervisorConfig config)
    : config_(std::move(config)) {
  validateConfig(config_);
}

void SupervisorStateMachine::validateConfig(const SupervisorConfig& config) {
  if (!(config.evaluation_rate_hz > 0.0) || !std::isfinite(config.evaluation_rate_hz)) {
    throw std::invalid_argument("evaluation_rate_hz must be positive and finite");
  }
  for (const auto value : {config.propagated_max_age_ns, config.corrected_max_age_ns,
                           config.px4_max_age_ns, config.diagnostics_max_age_ns,
                           config.maximum_alignment_gap_ns, config.service_timeout_ns}) {
    if (value < 0) throw std::invalid_argument("freshness/alignment limits must be nonnegative");
  }
  if (config.query_epoch_max_age_ns <= 0) {
    throw std::invalid_argument("query epoch maximum age must be positive");
  }
  if (config.maximum_comparison_age_ns <= 0 ||
      config.maximum_comparison_age_ns < config.service_timeout_ns) {
    throw std::invalid_argument("maximum comparison age is incompatible with service timeout");
  }
  if (config.alignment_window_size == 0 || config.alignment_minimum_samples == 0 ||
      config.alignment_minimum_samples > config.alignment_window_size ||
      config.alignment_lock_stable_windows == 0 ||
      !std::isfinite(config.alignment_lock_max_translation_step_m) ||
      config.alignment_lock_max_translation_step_m < 0.0 ||
      !std::isfinite(config.alignment_lock_max_yaw_step_rad) ||
      config.alignment_lock_max_yaw_step_rad < 0.0 ||
      config.alignment_minimum_novel_pairs == 0 ||
      config.alignment_candidate_history_capacity < config.alignment_lock_stable_windows ||
      !std::isfinite(config.alignment_max_cluster_translation_m) ||
      config.alignment_max_cluster_translation_m < 0.0 ||
      !std::isfinite(config.alignment_max_cluster_yaw_rad) ||
      config.alignment_max_cluster_yaw_rad < 0.0 ||
      !std::isfinite(config.alignment_covariance_nis_chi_square) ||
      config.alignment_covariance_nis_chi_square <= 0.0 ||
      config.alignment_revalidation_samples == 0 ||
      config.alignment_revalidation_failure_limit == 0 ||
      !std::isfinite(config.alignment_minimum_horizontal_excitation_m) ||
      config.alignment_minimum_horizontal_excitation_m < 0.0) {
    throw std::invalid_argument("alignment window/excitation configuration is invalid");
  }
  const auto ordered = [](const ResidualThresholds& a, const ResidualThresholds& b) {
    return a.position_m < b.position_m && a.velocity_m_s < b.velocity_m_s &&
           a.orientation_rad < b.orientation_rad && a.yaw_rad < b.yaw_rad;
  };
  if (!ordered(config.suspect, config.degraded) || !ordered(config.degraded, config.diverged)) {
    throw std::invalid_argument("supervisor thresholds must be strictly ordered");
  }
  if (!(config.clear_ratio > 0.0 && config.clear_ratio < 1.0)) {
    throw std::invalid_argument("clear_ratio must be in (0,1)");
  }
  if (config.suspect_enter_ns <= 0 || config.degraded_enter_ns < config.suspect_enter_ns ||
      config.diverged_enter_ns < config.degraded_enter_ns || config.recovery_confirm_ns <= 0 ||
      config.reset_grace_ns < 0 || config.lio_diagnostics_invalid_enter_ns <= 0) {
    throw std::invalid_argument("supervisor persistence durations are not ordered");
  }
  const auto evaluation_period_ns = static_cast<std::int64_t>(1e9 / config.evaluation_rate_hz);
  if (config.service_timeout_ns > 2 * evaluation_period_ns) {
    throw std::invalid_argument("service timeout exceeds evaluation backlog budget");
  }
  if (config.suspect_speed_limit_m_s < 0.0F || config.degraded_speed_limit_m_s < 0.0F) {
    throw std::invalid_argument("planner speed limits must be nonnegative");
  }
}

bool SupervisorStateMachine::exceeds(const Residual& residual,
                                     const ResidualThresholds& thresholds) {
  return residual.position_error_m >= thresholds.position_m ||
         residual.velocity_error_m_s >= thresholds.velocity_m_s ||
         residual.orientation_error_rad >= thresholds.orientation_rad ||
         std::abs(residual.yaw_error_rad) >= thresholds.yaw_rad;
}

bool SupervisorStateMachine::below(const Residual& residual,
                                   const ResidualThresholds& thresholds, double ratio) {
  return residual.position_error_m <= thresholds.position_m * ratio &&
         residual.velocity_error_m_s <= thresholds.velocity_m_s * ratio &&
         residual.orientation_error_rad <= thresholds.orientation_rad * ratio &&
         std::abs(residual.yaw_error_rad) <= thresholds.yaw_rad * ratio;
}

void SupervisorStateMachine::transition(HealthState state, std::uint16_t reason_code,
                                        const char* reason) {
  if (state_ != state) {
    state_ = state;
    ++transition_count_;
  }
  reason_code_ = reason_code;
  reason_ = reason;
}

void SupervisorStateMachine::clearPersistence() {
  suspect_since_ns_.reset();
  degraded_since_ns_.reset();
  diverged_since_ns_.reset();
  recovery_since_ns_.reset();
}

void SupervisorStateMachine::applyActions(SupervisorOutput& output,
                                          const bool safety_gate) const {
  switch (state_) {
    case HealthState::kStartup:
      output.external_odometry_allowed = false;
      break;
    case HealthState::kHealthy:
      output.external_odometry_allowed = safety_gate;
      break;
    case HealthState::kSuspect:
      output.external_odometry_allowed = safety_gate;
      output.planner_speed_limit_active = true;
      output.planner_speed_limit_m_s = config_.suspect_speed_limit_m_s;
      break;
    case HealthState::kDegraded:
      output.external_odometry_allowed = false;
      output.planner_speed_limit_active = true;
      output.planner_speed_limit_m_s = config_.degraded_speed_limit_m_s;
      break;
    case HealthState::kDiverged:
      output.external_odometry_allowed = false;
      output.planner_speed_limit_active = true;
      output.planner_speed_limit_m_s = 0.0F;
      output.hover_or_failsafe_requested = true;
      break;
  }
}

SupervisorOutput SupervisorStateMachine::evaluate(const EvaluationInput& input) {
  ++evaluation_count_;
  if (previous_reset_generation_ &&
      *previous_reset_generation_ != input.px4_reset_generation) {
    clearPersistence();
    reset_grace_until_ns_ = input.evaluation_time_ns + config_.reset_grace_ns;
    reinitialization_latched_ = false;
    if (state_ != HealthState::kDiverged) {
      reason_code_ = kReasonResetGrace;
      reason_ = "PX4_RESET_GRACE";
    }
  }
  const bool time_generation_changed =
      previous_time_generation_ && *previous_time_generation_ != input.px4_time_generation;
  if (time_generation_changed || input.time_generation_changed) {
    clearPersistence();
    ++alignment_failure_count_;
    reset_grace_until_ns_.reset();
  }
  previous_reset_generation_ = input.px4_reset_generation;
  previous_time_generation_ = input.px4_time_generation;

  const bool new_comparison_epoch =
      input.new_comparison_sample && input.comparison_epoch_ns > 0 &&
      (!last_comparison_epoch_ns_ || input.comparison_epoch_ns > *last_comparison_epoch_ns_);
  if (new_comparison_epoch) last_comparison_epoch_ns_ = input.comparison_epoch_ns;

  const bool in_reset_grace = reset_grace_until_ns_ &&
                              input.evaluation_time_ns < *reset_grace_until_ns_;
  const bool comparison_valid = input.px4_available && input.px4_fresh &&
                                input.px4_continuity_valid && input.lio_diagnostics_valid &&
                                input.px4_diagnostics_valid && input.aligned_comparison_fresh &&
                                input.alignment_valid && input.alignment_locked &&
                                !input.alignment_revalidating && input.residual.valid &&
                                input.propagated_fresh && input.corrected_fresh &&
                                input.alignment_gap_ns >= 0 &&
                                input.alignment_gap_ns <= config_.maximum_alignment_gap_ns &&
                                !in_reset_grace && !time_generation_changed &&
                                !input.time_generation_changed;
  const bool monitoring_available = input.px4_available && input.px4_fresh &&
                                    input.px4_diagnostics_valid && input.alignment_valid;
  if (input.px4_available && !comparison_valid) ++alignment_failure_count_;
  if (input.lio_diagnostics_valid) {
    lio_diagnostics_invalid_since_ns_.reset();
  } else if (!lio_diagnostics_invalid_since_ns_) {
    lio_diagnostics_invalid_since_ns_ = input.evaluation_time_ns;
  }

  if (input.lio_lost) {
    transition(HealthState::kDiverged, kReasonLioLost, "LIO_LOST");
  } else if (input.lio_state_corruption) {
    transition(HealthState::kDiverged, kReasonStateCorruption, "STATE_CORRUPTION");
  } else if (state_ != HealthState::kDiverged && !input.lio_diagnostics_valid &&
             lio_diagnostics_invalid_since_ns_ &&
             input.evaluation_time_ns - *lio_diagnostics_invalid_since_ns_ >=
                 config_.lio_diagnostics_invalid_enter_ns) {
    if (input.lio_diagnostics_schema_valid) {
      transition(HealthState::kDegraded, kReasonLioDiagnosticsStale, "LIO_DIAGNOSTICS_STALE");
    } else {
      transition(HealthState::kDegraded, kReasonLioDiagnosticSchemaMismatch,
                 "LIO_DIAGNOSTIC_SCHEMA_MISMATCH");
    }
  } else if (state_ != HealthState::kDiverged && input.lio_resetting) {
    transition(HealthState::kDegraded, kReasonLioResetting, "LIO_RESETTING");
  } else if (state_ != HealthState::kDiverged && input.lio_degraded) {
    transition(HealthState::kDegraded, kReasonResidualDegraded, "LIO_DEGRADED");
  } else if (state_ == HealthState::kStartup) {
    if (input.lio_valid && input.propagated_fresh && input.corrected_fresh) {
      transition(HealthState::kHealthy, kReasonHealthy, "HEALTHY");
    }
  } else if (state_ != HealthState::kDiverged) {
    const bool corrected_stale = input.propagated_fresh && !input.corrected_fresh;
    const bool propagated_stale = !input.propagated_fresh;
    const bool diverged_residual = new_comparison_epoch && comparison_valid &&
                                   exceeds(input.residual, config_.diverged);
    const bool degraded_residual = new_comparison_epoch && comparison_valid &&
                                   exceeds(input.residual, config_.degraded);
    const bool suspect_residual = new_comparison_epoch && comparison_valid &&
                                  exceeds(input.residual, config_.suspect);
    const bool degraded_condition = degraded_residual || corrected_stale || propagated_stale ||
                                    !input.lio_valid;
    if (diverged_residual) {
      if (!diverged_since_ns_) diverged_since_ns_ = input.comparison_epoch_ns;
      if (input.comparison_epoch_ns - *diverged_since_ns_ >= config_.diverged_enter_ns) {
        transition(HealthState::kDiverged, kReasonResidualDiverged, "RESIDUAL_DIVERGED");
      }
    } else if (new_comparison_epoch) {
      diverged_since_ns_.reset();
    }
    if (state_ != HealthState::kDiverged && degraded_condition) {
      if (!degraded_since_ns_) {
        degraded_since_ns_ = degraded_residual ? input.comparison_epoch_ns : input.evaluation_time_ns;
      }
      const auto elapsed = degraded_residual ? input.comparison_epoch_ns - *degraded_since_ns_
                                             : input.evaluation_time_ns - *degraded_since_ns_;
      if (elapsed >= config_.degraded_enter_ns) {
        transition(HealthState::kDegraded, kReasonResidualDegraded, "RESIDUAL_DEGRADED");
      }
    } else if (new_comparison_epoch && !degraded_condition) {
      degraded_since_ns_.reset();
    }
    if ((state_ == HealthState::kHealthy || state_ == HealthState::kSuspect) &&
        suspect_residual) {
      if (!suspect_since_ns_) suspect_since_ns_ = input.comparison_epoch_ns;
      if (input.comparison_epoch_ns - *suspect_since_ns_ >= config_.suspect_enter_ns) {
        transition(HealthState::kSuspect, kReasonResidualSuspect, "RESIDUAL_SUSPECT");
      }
    } else if (new_comparison_epoch && !suspect_residual) {
      suspect_since_ns_.reset();
    }
    if (state_ == HealthState::kSuspect || state_ == HealthState::kDegraded) {
      if (comparison_valid && input.lio_valid && input.propagated_fresh &&
          input.corrected_fresh && new_comparison_epoch &&
          below(input.residual, config_.suspect, config_.clear_ratio)) {
        if (!recovery_since_ns_) recovery_since_ns_ = input.comparison_epoch_ns;
        if (input.comparison_epoch_ns - *recovery_since_ns_ >= config_.recovery_confirm_ns) {
          transition(HealthState::kHealthy, kReasonHealthy, "HEALTHY_RECOVERED");
          recovery_since_ns_.reset();
        }
      } else {
        recovery_since_ns_.reset();
      }
    }
  }

  // Authorization is evaluated after the health transition.  The published
  // health state and the external gate must describe the same evaluation.
  const bool external_measurement_publishable =
      input.lio_valid && input.propagated_fresh && input.corrected_fresh &&
      input.correction_quality_valid && input.timestamp_valid && input.covariance_valid &&
      input.lio_generation_locked && !input.lio_resetting &&
      !input.continuity_unrecoverable;
  const bool correlated_reference_evidence =
      comparison_valid && input.alignment_locked && !input.alignment_revalidating &&
      input.alignment_valid_for_comparison && input.px4_available &&
      input.px4_fresh && input.px4_continuity_valid && input.px4_post_reset_stable;
  const bool health_allows_external = state_ == HealthState::kHealthy ||
                                      state_ == HealthState::kSuspect;
  const bool external_measurement_authorized =
      external_measurement_publishable && input.external_publisher_ready &&
      (config_.reference_mode == ReferenceMode::kIndependent ||
       correlated_reference_evidence) && !in_reset_grace && health_allows_external;

  SupervisorOutput output;
  output.health = state_;
  output.reference_mode = config_.reference_mode;
  output.reason_code = reason_code_;
  output.reason = reason_;
  output.monitoring_available = monitoring_available;
  output.comparison_valid = comparison_valid;
  output.cross_comparison_valid = comparison_valid;
  output.lio_valid = input.lio_valid;
  output.px4_valid = input.px4_available && input.px4_fresh && input.px4_continuity_valid;
  output.time_aligned = comparison_valid;
  output.alignment_valid = input.alignment_valid;
  output.alignment_candidate_valid = input.alignment_candidate_valid;
  output.alignment_locked = input.alignment_locked;
  output.alignment_revalidating = input.alignment_revalidating;
  output.alignment_valid_for_comparison = input.alignment_valid_for_comparison;
  output.alignment_lifecycle = input.alignment_lifecycle;
  output.alignment = input.alignment;
  output.evaluation_time_ns = input.evaluation_time_ns;
  output.lio_propagated_age_ns = input.propagated_age_ns;
  output.lio_corrected_age_ns = input.corrected_age_ns;
  output.px4_age_ns = input.px4_age_ns;
  output.alignment_gap_ns = input.alignment_gap_ns;
  output.aligned_comparison_age_ns =
      input.aligned_comparison_fresh && input.evaluation_time_ns >= input.comparison_epoch_ns
          ? input.evaluation_time_ns - input.comparison_epoch_ns
          : -1;
  output.latest_eligible_epoch_ns = input.latest_eligible_epoch_ns;
  output.comparison_lag_to_latest_eligible_ns = input.comparison_lag_to_latest_eligible_ns;
  output.pending_query_epoch_ns = input.pending_query_epoch_ns;
  output.pending_query_age_ns = input.pending_query_age_ns;
  output.residual = input.residual;
  output.px4_reset_generation = input.px4_reset_generation;
  output.px4_frame_generation = input.px4_frame_generation;
  output.px4_time_generation = input.px4_time_generation;
  output.alignment_frame_generation = input.alignment_frame_generation;
  output.lio_generation = input.lio_generation;
  output.correction_quality_valid = input.correction_quality_valid;
  output.timestamp_valid = input.timestamp_valid;
  output.covariance_valid = input.covariance_valid;
  output.lio_generation_locked = input.lio_generation_locked;
  output.continuity_unrecoverable = input.continuity_unrecoverable;
  output.external_publisher_ready = input.external_publisher_ready;
  output.external_measurement_publishable = external_measurement_publishable;
  output.external_measurement_authorized = external_measurement_authorized;
  output.state_transition_count = transition_count_;
  output.evaluation_count = evaluation_count_;
  output.alignment_failure_count = alignment_failure_count_;
  output.comparison_epoch_ns = input.comparison_epoch_ns;
  output.new_comparison_sample = new_comparison_epoch;
  output.aligned_comparison_fresh = input.aligned_comparison_fresh;
  output.lio_diagnostics_valid = input.lio_diagnostics_valid;
  output.px4_diagnostics_valid = input.px4_diagnostics_valid;
  output.lio_diagnostics_schema_valid = input.lio_diagnostics_schema_valid;
  output.px4_diagnostics_schema_valid = input.px4_diagnostics_schema_valid;
  output.lio_diagnostics_stale = input.lio_diagnostics_stale;
  output.px4_diagnostics_stale = input.px4_diagnostics_stale;
  output.query_sequence = input.query_sequence;
  output.component_validity_mask = input.component_validity_mask;
  output.covariance_availability_mask = input.covariance_availability_mask;
  output.query_invalid_component_count = input.query_invalid_component_count;
  output.query_generation_mismatch_count = input.query_generation_mismatch_count;
  output.query_stale_sequence_count = input.query_stale_sequence_count;
  output.query_timeout_count = input.query_timeout_count;
  output.query_service_unavailable_count = input.query_service_unavailable_count;
  output.query_success_count = input.query_success_count;
  output.query_failure_count = input.query_failure_count;
  output.query_epoch_not_yet_buffered_count = input.query_epoch_not_yet_buffered_count;
  output.query_epoch_expired_count = input.query_epoch_expired_count;
  output.query_duplicate_suppressed_count = input.query_duplicate_suppressed_count;
  output.query_transport_failure_count = input.query_transport_failure_count;
  output.query_geometric_failure_count = input.query_geometric_failure_count;
  output.query_failure_class = input.query_failure_class;
  output.query_last_failure_reason = input.query_last_failure_reason;
  output.query_rtt_count = input.query_rtt_count;
  output.query_rtt_p50_ms = input.query_rtt_p50_ms;
  output.query_rtt_p95_ms = input.query_rtt_p95_ms;
  output.query_rtt_p99_ms = input.query_rtt_p99_ms;
  output.query_rtt_max_ms = input.query_rtt_max_ms;
  output.stale_residual_reuse_count = input.stale_residual_reuse_count;
  output.alignment_candidate_estimate_count = input.alignment_candidate_estimate_count;
  output.alignment_candidate_transition_count = input.alignment_candidate_transition_count;
  output.alignment_revalidation_sample_count = input.alignment_revalidation_sample_count;
  output.alignment_revalidation_success_count = input.alignment_revalidation_success_count;
  output.alignment_revalidation_failure_count = input.alignment_revalidation_failure_count;
  output.alignment_revalidation_start_count = input.alignment_revalidation_start_count;
  output.alignment_revalidation_start_epoch_ns = input.alignment_revalidation_start_epoch_ns;
  output.alignment_locked_transform_age_ns = input.alignment_locked_transform_age_ns;
  if (input.px4_available && !input.alignment_valid &&
             state_ != HealthState::kDiverged) {
    output.reason_code = kReasonAlignmentFailed;
    output.reason = "WORLD_ALIGNMENT_UNAVAILABLE";
  } else if (!input.lio_diagnostics_valid && state_ != HealthState::kDiverged) {
    output.reason_code = input.lio_diagnostics_schema_valid ? kReasonLioDiagnosticsStale
                                                             : kReasonLioDiagnosticSchemaMismatch;
    output.reason = input.lio_diagnostics_schema_valid ? "LIO_DIAGNOSTICS_STALE"
                                                       : "LIO_DIAGNOSTIC_SCHEMA_MISMATCH";
  } else if (input.px4_available && !input.px4_diagnostics_valid &&
             state_ != HealthState::kDiverged) {
    output.reason_code = input.px4_diagnostics_schema_valid ? kReasonPx4DiagnosticsStale
                                                             : kReasonPx4DiagnosticSchemaMismatch;
    output.reason = input.px4_diagnostics_schema_valid ? "PX4_DIAGNOSTICS_STALE"
                                                       : "PX4_DIAGNOSTIC_SCHEMA_MISMATCH";
  } else if (input.px4_available && !input.aligned_comparison_fresh &&
             state_ != HealthState::kDiverged) {
    output.reason_code = kReasonAlignedComparisonStale;
    output.reason = "ALIGNED_COMPARISON_STALE";
  } else if (!input.px4_available && state_ != HealthState::kDiverged) {
    output.reason_code = kReasonMonitoringUnavailable;
    output.reason = "MONITORING_UNAVAILABLE";
  } else if (in_reset_grace && state_ != HealthState::kDiverged) {
    output.reason_code = kReasonResetGrace;
    output.reason = "PX4_RESET_GRACE";
  } else if (input.px4_available && !input.px4_fresh && state_ != HealthState::kDiverged) {
    output.reason_code = kReasonStale;
    output.reason = "PX4_STALE";
  }

  if (state_ == HealthState::kDiverged && config_.reference_mode == ReferenceMode::kIndependent &&
      input.px4_available && input.px4_fresh && input.px4_continuity_valid &&
      input.px4_post_reset_stable && input.alignment_valid && !reinitialization_latched_) {
    reinitialization_latched_ = true;
    ++reinitialization_sequence_;
  }
  output.reinitialization_request_sequence = reinitialization_sequence_;
  output.reinitialization_requested = reinitialization_latched_;
  applyActions(output, external_measurement_authorized);
  previous_residual_ = input.residual.valid ? std::optional<Residual>(input.residual) : std::nullopt;
  return output;
}

void SupervisorStateMachine::reset() {
  state_ = HealthState::kStartup;
  reason_code_ = kReasonStartup;
  reason_ = "STARTUP";
  transition_count_ = 0;
  evaluation_count_ = 0;
  alignment_failure_count_ = 0;
  reinitialization_sequence_ = 0;
  reinitialization_latched_ = false;
  clearPersistence();
  lio_diagnostics_invalid_since_ns_.reset();
  reset_grace_until_ns_.reset();
  previous_reset_generation_.reset();
  previous_time_generation_.reset();
  previous_residual_.reset();
  last_comparison_epoch_ns_.reset();
}

}  // namespace odometry_supervisor

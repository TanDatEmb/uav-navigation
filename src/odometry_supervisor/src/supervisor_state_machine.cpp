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
      config.reset_grace_ns < 0) {
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

void SupervisorStateMachine::applyActions(SupervisorOutput& output) const {
  switch (state_) {
    case HealthState::kStartup:
      output.external_odometry_allowed = false;
      break;
    case HealthState::kHealthy:
      output.external_odometry_allowed = true;
      break;
    case HealthState::kSuspect:
      output.external_odometry_allowed = true;
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

  const bool in_reset_grace = reset_grace_until_ns_ &&
                              input.evaluation_time_ns < *reset_grace_until_ns_;
  const bool comparison_valid = input.px4_available && input.px4_fresh &&
                                input.px4_continuity_valid && input.diagnostics_valid &&
                                input.origin_aligned && input.residual.valid &&
                                input.propagated_fresh && input.corrected_fresh &&
                                input.alignment_gap_ns >= 0 &&
                                input.alignment_gap_ns <= config_.maximum_alignment_gap_ns &&
                                !in_reset_grace && !time_generation_changed &&
                                !input.time_generation_changed;
  const bool monitoring_available = input.px4_available && input.px4_fresh &&
                                    input.diagnostics_valid && input.origin_aligned;
  if (input.px4_available && !comparison_valid) ++alignment_failure_count_;

  if (input.lio_lost) {
    transition(HealthState::kDiverged, kReasonLioLost, "LIO_LOST");
  } else if (input.lio_state_corruption) {
    transition(HealthState::kDiverged, kReasonStateCorruption, "STATE_CORRUPTION");
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
    const bool diverged_condition = comparison_valid && exceeds(input.residual, config_.diverged);
    const bool degraded_condition =
        (comparison_valid && exceeds(input.residual, config_.degraded)) || corrected_stale ||
        propagated_stale || !input.lio_valid;
    const bool suspect_condition = comparison_valid && exceeds(input.residual, config_.suspect);
    if (diverged_condition) {
      if (!diverged_since_ns_) diverged_since_ns_ = input.evaluation_time_ns;
      if (input.evaluation_time_ns - *diverged_since_ns_ >= config_.diverged_enter_ns) {
        transition(HealthState::kDiverged, kReasonResidualDiverged, "RESIDUAL_DIVERGED");
      }
    } else {
      diverged_since_ns_.reset();
    }
    if (state_ != HealthState::kDiverged && degraded_condition) {
      if (!degraded_since_ns_) degraded_since_ns_ = input.evaluation_time_ns;
      if (input.evaluation_time_ns - *degraded_since_ns_ >= config_.degraded_enter_ns) {
        transition(HealthState::kDegraded, kReasonResidualDegraded, "RESIDUAL_DEGRADED");
      }
    } else if (!degraded_condition) {
      degraded_since_ns_.reset();
    }
    if ((state_ == HealthState::kHealthy || state_ == HealthState::kSuspect) &&
        suspect_condition) {
      if (!suspect_since_ns_) suspect_since_ns_ = input.evaluation_time_ns;
      if (input.evaluation_time_ns - *suspect_since_ns_ >= config_.suspect_enter_ns) {
        transition(HealthState::kSuspect, kReasonResidualSuspect, "RESIDUAL_SUSPECT");
      }
    } else if (!suspect_condition) {
      suspect_since_ns_.reset();
    }
    if (state_ == HealthState::kSuspect || state_ == HealthState::kDegraded) {
      if (comparison_valid && input.lio_valid && input.propagated_fresh &&
          input.corrected_fresh && below(input.residual, config_.suspect, config_.clear_ratio)) {
        if (!recovery_since_ns_) recovery_since_ns_ = input.evaluation_time_ns;
        if (input.evaluation_time_ns - *recovery_since_ns_ >= config_.recovery_confirm_ns) {
          transition(HealthState::kHealthy, kReasonHealthy, "HEALTHY_RECOVERED");
          recovery_since_ns_.reset();
        }
      } else {
        recovery_since_ns_.reset();
      }
    }
  }

  SupervisorOutput output;
  output.health = state_;
  output.reference_mode = config_.reference_mode;
  output.reason_code = reason_code_;
  output.reason = reason_;
  output.monitoring_available = monitoring_available;
  output.comparison_valid = comparison_valid;
  output.lio_valid = input.lio_valid;
  output.px4_valid = input.px4_available && input.px4_fresh && input.px4_continuity_valid;
  output.time_aligned = comparison_valid;
  output.evaluation_time_ns = input.evaluation_time_ns;
  output.lio_propagated_age_ns = input.propagated_age_ns;
  output.lio_corrected_age_ns = input.corrected_age_ns;
  output.px4_age_ns = input.px4_age_ns;
  output.alignment_gap_ns = input.alignment_gap_ns;
  output.residual = input.residual;
  output.px4_reset_generation = input.px4_reset_generation;
  output.px4_time_generation = input.px4_time_generation;
  output.state_transition_count = transition_count_;
  output.evaluation_count = evaluation_count_;
  output.alignment_failure_count = alignment_failure_count_;
  if (input.lio_valid && input.px4_available && !input.origin_aligned) {
    output.monitoring_available = false;
    output.comparison_valid = false;
    output.time_aligned = false;
    if (state_ != HealthState::kDiverged) {
      output.reason_code = kReasonOriginNotAligned;
      output.reason = "ODOM_ORIGIN_NOT_ALIGNED";
    }
  } else if (input.px4_available && !input.diagnostics_valid && state_ != HealthState::kDiverged) {
    output.reason_code = kReasonSchemaMismatch;
    output.reason = "DIAGNOSTIC_SCHEMA_MISMATCH";
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
      input.px4_post_reset_stable && input.origin_aligned && !reinitialization_latched_) {
    reinitialization_latched_ = true;
    ++reinitialization_sequence_;
  }
  output.reinitialization_request_sequence = reinitialization_sequence_;
  output.reinitialization_requested = reinitialization_latched_;
  applyActions(output);
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
  reset_grace_until_ns_.reset();
  previous_reset_generation_.reset();
  previous_time_generation_.reset();
  previous_residual_.reset();
}

}  // namespace odometry_supervisor

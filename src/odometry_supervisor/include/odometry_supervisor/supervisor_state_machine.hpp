#pragma once

#include <optional>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

class SupervisorStateMachine {
 public:
  explicit SupervisorStateMachine(SupervisorConfig config = {});

  static void validateConfig(const SupervisorConfig& config);
  SupervisorOutput evaluate(const EvaluationInput& input);
  void reset();

 private:
  static bool exceeds(const Residual& residual, const ResidualThresholds& thresholds);
  static bool below(const Residual& residual, const ResidualThresholds& thresholds,
                    double ratio);
  void transition(HealthState state, std::uint16_t reason_code, const char* reason);
  void clearPersistence();
  void applyActions(SupervisorOutput& output) const;

  SupervisorConfig config_;
  HealthState state_{HealthState::kStartup};
  std::uint16_t reason_code_{1};
  const char* reason_{"STARTUP"};
  std::uint64_t transition_count_{0};
  std::uint64_t evaluation_count_{0};
  std::uint64_t alignment_failure_count_{0};
  std::uint64_t reinitialization_sequence_{0};
  bool reinitialization_latched_{false};
  std::optional<std::int64_t> suspect_since_ns_;
  std::optional<std::int64_t> degraded_since_ns_;
  std::optional<std::int64_t> diverged_since_ns_;
  std::optional<std::int64_t> recovery_since_ns_;
  std::optional<std::int64_t> reset_grace_until_ns_;
  std::optional<std::uint64_t> previous_reset_generation_;
  std::optional<std::uint64_t> previous_time_generation_;
  std::optional<Residual> previous_residual_;
};

}  // namespace odometry_supervisor

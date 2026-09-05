#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>

#include <Eigen/Core>

namespace navigation_runtime {

// One immutable diagnostic record produced at the retained-command decision
// boundary.  The record deliberately carries the identity and both clock
// domains beside the samples it describes; readers must never reconstruct a
// causal observation by loading independent fields.
struct ExecutionTraceSnapshot final {
  std::uint64_t planning_cycle_id{0U};
  std::uint64_t solve_generation{0U};
  std::int64_t timestamp_ns{0};

  std::uint64_t execution_localization_epoch{0U};
  std::uint64_t execution_goal_epoch{0U};
  std::uint64_t execution_request_id{0U};
  std::uint64_t execution_bundle_generation{0U};
  std::uint64_t execution_state_ingress_sequence{0U};

  std::int64_t evaluation_now_ns{0};
  std::int64_t execution_state_source_stamp_ns{0};
  std::int64_t execution_state_receive_stamp_ns{0};
  std::int64_t committed_bundle_start_stamp_ns{0};
  double execution_state_source_age_ms{std::numeric_limits<double>::quiet_NaN()};
  double execution_state_receive_age_ms{std::numeric_limits<double>::quiet_NaN()};

  double anchor_error_m{std::numeric_limits<double>::quiet_NaN()};
  double projected_anchor_error_m{std::numeric_limits<double>::quiet_NaN()};
  double retained_tracking_limit_m{std::numeric_limits<double>::quiet_NaN()};
  double relative_anchor_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool backup_available{false};
  double time_to_backup_start_s{std::numeric_limits<double>::quiet_NaN()};
  bool committed_suffix_usable{false};
  bool sampled_path_clear{false};
  bool tracking_certificate_exceeded{false};
  bool projected_tracking_certificate_exceeded{false};
  std::uint8_t emergency_authorization_reason{0U};
  int emergency_candidate_commit_result{0};

  Eigen::Vector3d measured_position_at_state_source =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d measured_velocity_at_state_source =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d committed_command_position_at_now =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d committed_command_velocity_at_now =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d committed_command_position_at_state_source =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d committed_command_velocity_at_state_source =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

  double anchor_error_raw_m{std::numeric_limits<double>::quiet_NaN()};
  double anchor_error_time_aligned_m{std::numeric_limits<double>::quiet_NaN()};
  double command_motion_over_state_age_m{std::numeric_limits<double>::quiet_NaN()};
  double velocity_residual_time_aligned_mps{std::numeric_limits<double>::quiet_NaN()};
  double retained_elapsed_s{std::numeric_limits<double>::quiet_NaN()};
  double committed_bundle_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double committed_safety_transition_time_s{std::numeric_limits<double>::quiet_NaN()};
  bool validate_without_new_commit{false};
  bool retained_fresh_vehicle_state{false};
  bool retained_committed_command_available{false};
  bool retained_command_anchor_valid{false};
  bool current_vehicle_state_known_free{false};
  bool retained_safety_trajectory_available{false};
  bool retained_terminal_stop{false};
  int retained_committed_role{-1};
  std::uint8_t retained_recovery_state_before{0U};
};

// A retained-command trace describes the execution owner that produced its
// samples.  A command may be published after a successor has been staged or
// after a lifecycle epoch changes, so matching only the planning cycle would
// incorrectly attribute an older trace to the new command.
[[nodiscard]] inline bool executionTraceMatchesCommand(
    const ExecutionTraceSnapshot& trace,
    const std::uint64_t localization_epoch,
    const std::uint64_t goal_epoch,
    const std::uint64_t request_id,
    const std::uint64_t bundle_generation) noexcept {
  return localization_epoch != 0U && goal_epoch != 0U && request_id != 0U &&
         bundle_generation != 0U && trace.execution_localization_epoch == localization_epoch &&
         trace.execution_goal_epoch == goal_epoch &&
         trace.execution_request_id == request_id &&
         trace.execution_bundle_generation == bundle_generation;
}

// Mutex-protected publication of immutable records.  The planner callback is
// the writer; command and diagnostic callbacks each load one shared pointer.
// Older solve completions cannot overwrite a newer trace record.
class ExecutionTraceStore final {
 public:
  ExecutionTraceStore() = default;
  ExecutionTraceStore(const ExecutionTraceStore&) = delete;
  ExecutionTraceStore& operator=(const ExecutionTraceStore&) = delete;

  bool publish(ExecutionTraceSnapshot next) noexcept {
    std::lock_guard lock(mutex_);
    if (next.execution_localization_epoch < minimum_localization_epoch_) return false;
    if (state_ && next.solve_generation < state_->solve_generation) return false;
    try {
      state_ = std::make_shared<const ExecutionTraceSnapshot>(std::move(next));
    } catch (...) {
      return false;
    }
    return true;
  }

  // Lifecycle reset invalidates every record from the previous localization
  // epoch before a stale worker can publish again. Goal transitions do not
  // clear the record because a pass-through successor may still execute the
  // predecessor bundle; command readers use the complete owner identity.
  void advanceLocalizationEpoch(const std::uint64_t localization_epoch) noexcept {
    std::lock_guard lock(mutex_);
    if (localization_epoch <= minimum_localization_epoch_) return;
    minimum_localization_epoch_ = localization_epoch;
    if (state_ && state_->execution_localization_epoch < minimum_localization_epoch_) {
      state_.reset();
    }
  }

  [[nodiscard]] std::shared_ptr<const ExecutionTraceSnapshot> load() const noexcept {
    std::lock_guard lock(mutex_);
    return state_;
  }

 private:
  mutable std::mutex mutex_;
  std::uint64_t minimum_localization_epoch_{0U};
  std::shared_ptr<const ExecutionTraceSnapshot> state_;
};

}  // namespace navigation_runtime

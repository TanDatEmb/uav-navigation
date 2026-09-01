#pragma once

#include <atomic>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/navigation_mode_status.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <navigation_mapping/mapping_worker.hpp>
#include <navigation_mapping/mapping_observation.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>
#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/observation_accounting.hpp>
#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/execution_recovery_state.hpp"
#include "navigation_runtime/planning_worker.hpp"
#include <navigation_execution/execution_state_gate.hpp>
#include <navigation_execution/execution_state_store.hpp>
#include <navigation_execution/committed_bundle_store.hpp>
#include <navigation_execution/command_sampler.hpp>
#include <navigation_mapping/world_snapshot_store.hpp>
#include <navigation_planning/planning_limits.hpp>
#include "navigation_runtime/kinematic_derivative_estimator.hpp"

namespace navigation_planning_backend {
class PlannerFacade;
}

namespace navigation_runtime {

struct MappingTelemetrySnapshot {
  navigation_mapping::RaycastDiagnostics map{};
  std::uint64_t world_generation{1};
  std::uint64_t world_revision{0};
  std::int64_t observation_stamp_ns{0};
  std::int64_t last_update_attempt_stamp_ns{0};
  std::int64_t map_update_us{0};
  std::int64_t snapshot_export_us{0};
  std::int64_t mapping_callback_total_us{0};
  std::int64_t pointcloud_decode_us{0};
  bool world_snapshot_published{false};
  std::uint64_t snapshot_export_mode{0};
  std::uint64_t snapshot_full_export_reason{0};
  std::uint64_t snapshot_export_base_cells{0};
  std::uint64_t snapshot_export_inflated_cells{0};
  std::uint64_t snapshot_patch_depth{0};
  std::uint64_t snapshot_bytes{0};
  std::uint64_t snapshot_owned_bytes{0};
  std::uint64_t snapshot_shared_metadata_bytes{0};
  std::uint64_t snapshot_live_count{0};
  std::uint64_t snapshot_peak_live_count{0};
  std::uint64_t snapshot_live_owned_bytes{0};
  std::uint64_t snapshot_peak_live_owned_bytes{0};
  std::uint64_t discarded_stale{0};
  std::uint64_t discarded_future{0};
  std::uint64_t discarded_invalid{0};
  std::uint64_t outcome_updated{0};
  std::uint64_t outcome_accumulated{0};
  std::uint64_t outcome_slide_only{0};
  std::uint64_t outcome_empty_cloud{0};
  std::uint64_t outcome_callback_owned{0};
  std::uint64_t outcome_below_ground{0};
  std::uint64_t outcome_above_ceiling{0};
  std::uint64_t command_revalidation_fast_path_count{0};
  std::uint64_t command_revalidation_full_count{0};
  std::uint64_t world_snapshot_published_count{0};
  std::uint64_t world_snapshot_deferred_count{0};
  std::uint64_t world_snapshot_full_export_count{0};
  std::uint64_t world_snapshot_patch_export_count{0};
};

class MappingTelemetry {
 public:
  void initialize(MappingTelemetrySnapshot next) {
    std::lock_guard lock(mutex_);
    state_ = std::move(next);
  }
  void recordUpdate(MappingTelemetrySnapshot next) {
    std::lock_guard lock(mutex_);
    next.discarded_stale = state_.discarded_stale;
    next.discarded_future = state_.discarded_future;
    next.discarded_invalid = state_.discarded_invalid;
    next.outcome_updated = state_.outcome_updated;
    next.outcome_accumulated = state_.outcome_accumulated;
    next.outcome_slide_only = state_.outcome_slide_only;
    next.outcome_empty_cloud = state_.outcome_empty_cloud;
    next.outcome_callback_owned = state_.outcome_callback_owned;
    next.outcome_below_ground = state_.outcome_below_ground;
    next.outcome_above_ceiling = state_.outcome_above_ceiling;
    next.world_snapshot_published_count = state_.world_snapshot_published_count;
    next.world_snapshot_deferred_count = state_.world_snapshot_deferred_count;
    next.world_snapshot_full_export_count = state_.world_snapshot_full_export_count;
    next.world_snapshot_patch_export_count = state_.world_snapshot_patch_export_count;
    if (next.world_snapshot_published) {
      ++next.world_snapshot_published_count;
      if (next.snapshot_export_mode ==
          static_cast<std::uint64_t>(navigation_mapping::SnapshotExportMode::kFull)) {
        ++next.world_snapshot_full_export_count;
      } else if (next.snapshot_export_mode ==
                 static_cast<std::uint64_t>(navigation_mapping::SnapshotExportMode::kPatch)) {
        ++next.world_snapshot_patch_export_count;
      }
    } else if (navigation_mapping::worldUpdateAdvanced(next.map.update_outcome)) {
      ++next.world_snapshot_deferred_count;
    }
    // `next` is normally a snapshot copied by the producer. Do not overwrite
    // producer-side increments here: unlike the legacy lifecycle counters,
    // revalidation counters are intentionally updated for this same result.
    switch (next.map.update_outcome) {
      case navigation_mapping::MapUpdateOutcome::kUpdated: ++next.outcome_updated; break;
      case navigation_mapping::MapUpdateOutcome::kAccumulated: ++next.outcome_accumulated; break;
      case navigation_mapping::MapUpdateOutcome::kSlideOnly: ++next.outcome_slide_only; break;
      case navigation_mapping::MapUpdateOutcome::kEmptyCloud: ++next.outcome_empty_cloud; break;
      case navigation_mapping::MapUpdateOutcome::kCallbackOwned: ++next.outcome_callback_owned; break;
      case navigation_mapping::MapUpdateOutcome::kBelowGround: ++next.outcome_below_ground; break;
      case navigation_mapping::MapUpdateOutcome::kAboveCeiling: ++next.outcome_above_ceiling; break;
    }
    state_ = std::move(next);
  }
  [[nodiscard]] MappingTelemetrySnapshot snapshot() const {
    std::lock_guard lock(mutex_);
    return state_;
  }
  void recordDiscard(bool stale, bool future, bool invalid) {
    std::lock_guard lock(mutex_);
    state_.discarded_stale += stale ? 1U : 0U;
    state_.discarded_future += future ? 1U : 0U;
    state_.discarded_invalid += invalid ? 1U : 0U;
  }
  void recordCallbackFailure(std::int64_t callback_total_us) {
    std::lock_guard lock(mutex_);
    state_.mapping_callback_total_us = callback_total_us;
  }
 private:
  mutable std::mutex mutex_;
  MappingTelemetrySnapshot state_;
};

class MappingLifecycleObserver {
 public:
  virtual ~MappingLifecycleObserver() = default;
  virtual void onMutableMapUpdated(std::int64_t observation_stamp_ns) noexcept = 0;
  virtual void onShutdownComplete(
      navigation_mapping::ObservationAccounting::Snapshot lifecycle) noexcept = 0;
};

struct NavigationRuntimeDependencies {
  std::shared_ptr<MappingLifecycleObserver> lifecycle_observer;
};

struct PendingRegisteredScan final {
  navigation_contracts::msg::RegisteredScan::ConstSharedPtr message;
  std::int64_t stamp_ns{0};
  std::uint64_t localization_epoch{0};
  std::uint64_t scan_sequence{0};
};

// Product ROS boundary for the planner backend core. Mapping consumes one atomic
// RegisteredScan containing the registered cloud and its corrected pose.
class NavigationRuntimeNode final : public rclcpp::Node {
 public:
  explicit NavigationRuntimeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  NavigationRuntimeNode(const rclcpp::NodeOptions& options,
                      NavigationRuntimeDependencies dependencies);
  ~NavigationRuntimeNode() override;

 private:
  void onRegisteredScan(
      const navigation_contracts::msg::RegisteredScan::ConstSharedPtr& message);
  void onEstimatorHealth(
      const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message);
  void onPropagatedOdometry(
      const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message);
  void onGoal(const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message);
  // Caller owns input_mutex_. This is the sole goal-transition implementation;
  // promotion and the ROS callback therefore share one linearization path.
  void applyValidatedGoalLocked(
      const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message,
      bool execution_transition_held = false);
  // Caller holds input_mutex_ and transitionMutex() in that order.
  void transitionForeignMissionLocked(bool defer_until_certified_stop);
  bool consumeForeignMissionCancelIfCurrent();
  void onModeStatus(
      const navigation_contracts::msg::NavigationModeStatus::ConstSharedPtr& message);
  void schedulePlanningCycle();
  void runCycle(const PlanningKey& scheduled_key);
  [[nodiscard]] std::optional<PlanningKey> currentPlanningKey();
  void publishCommand();
  bool commitPlannerCandidate(const navigation_contracts::msg::NavigationGoal& goal,
                             std::uint64_t goal_epoch,
                             std::uint64_t localization_epoch,
                             std::int64_t now_ns,
                             const PlanningKey& scheduled_key);
  void suspendCommandForWorldFreshness();
  void resetForLocalizationEpochLocked(std::uint64_t localization_epoch);
  static bool decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                          navigation_mapping::PointCloud& output,
                          bool require_nonempty = true);
  std::string registered_scan_topic_;
  std::string propagated_odometry_topic_;
  std::string goal_topic_;
  std::string status_topic_;
  std::string command_topic_;
  std::string planner_config_path_;
  std::string planning_frame_;
  std::string body_frame_id_;
  std::string deployment_profile_;
  // The planner owns a 180 ms solve budget. Keep one complete timer period
  // available for that budget; command sampling remains independent at 50 Hz.
  double planner_rate_hz_{5.0};
  double command_rate_hz_{50.0};
  double mapping_snapshot_publication_period_s_{0.10};
  double data_freshness_window_s_{0.5};
  std::int64_t data_freshness_window_ns_{500'000'000};
  double command_stream_timeout_s_{0.10};
  std::int64_t command_stream_timeout_ns_{100'000'000};
  double planner_watchdog_timeout_s_{1.0};
  std::int64_t planner_watchdog_timeout_ns_{1'000'000'000};
  double stopped_recovery_timeout_s_{5.0};
  std::uint32_t max_plan_from_rest_failures_{3U};
  std::uint64_t dynamics_hash_{1U};
  navigation_planning::DynamicLimits mission_dynamic_limits_{};

  rclcpp::Subscription<navigation_contracts::msg::RegisteredScan>::SharedPtr
      registered_scan_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::EstimatorHealth>::SharedPtr
      estimator_health_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::PropagatedOdometry>::SharedPtr
      propagated_odometry_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationGoal>::SharedPtr goal_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationModeStatus>::SharedPtr
      status_subscription_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr propagated_state_callback_group_;

  std::mutex input_mutex_;
  std::mutex localization_transition_mutex_;
  navigation_execution::ExecutionStateStore execution_state_store_;
  std::optional<navigation_contracts::msg::NavigationGoal> active_goal_;
  // Sole runtime owner for a goal published while a moving BACKUP/EMERGENCY
  // suffix owns execution.  Do not add another pending optional.
  PendingGoalHandoffOwner pending_goal_owner_;
  // Terminal status observed while the suffix is still draining. It is
  // consumed only after certified stop and exact active-identity matching.
  std::optional<navigation_contracts::msg::NavigationGoal> deferred_terminal_status_;
  // A mission-id change is a control-authority violation, not an ordered goal.
  // Defer the fail-closed Hold transition until measured stop so a certified
  // moving suffix is never cut mid-flight.
  bool foreign_mission_hold_after_stop_{false};
  std::atomic_bool foreign_mission_cancel_pending_{false};
  std::uint64_t foreign_cancel_target_epoch_{0U};
  std::uint64_t foreign_cancel_transition_epoch_{0U};
  std::uint64_t foreign_cancel_localization_epoch_{0U};
  std::atomic_uint64_t active_goal_epoch_{0};
  std::atomic_uint64_t active_localization_epoch_{1U};
  std::atomic_bool localization_epoch_ready_{true};
  std::atomic_uint64_t last_registered_scan_epoch_{1U};
  std::atomic_uint64_t last_registered_scan_sequence_{0U};
  std::atomic_int64_t last_propagated_state_stamp_ns_{0};
  std::atomic_uint64_t last_propagated_state_sequence_{0U};
  std::mutex propagated_derivative_mutex_;
  KinematicDerivativeEstimator propagated_derivative_estimator_;
  bool new_goal_{false};
  // PASS_THROUGH waypoint transitions retarget planner backend through ReplanOnce so the
  // committed polynomial supplies the future PVA initial state.
  bool hot_goal_transition_{false};
  bool restart_from_rest_{false};
  // planner backend's native FSM skips one replan timer callback immediately after a
  // successful PlanFromRest.  Keep that state at the ROS adapter boundary so
  // the first hot replan is not run against a trajectory that has just been
  // committed.
  std::atomic_bool skip_replan_once_{false};
  ConsecutiveFailureBudget plan_from_rest_failure_budget_{3U};
  std::int64_t plan_from_rest_first_failure_steady_ns_{0};
  std::atomic_uint64_t stale_input_count_{0};
  std::atomic_uint64_t stale_mapping_input_count_{0};
  std::atomic_uint64_t future_mapping_input_count_{0};
  std::atomic_uint64_t stale_execution_state_count_{0};
  std::atomic_uint64_t future_execution_state_count_{0};
  std::atomic_uint64_t invalid_corrected_pose_count_{0};
  std::atomic_uint64_t invalid_execution_state_count_{0};
  std::atomic_uint64_t world_snapshot_freshness_rejection_count_{0};
  std::atomic_uint64_t world_freshness_command_suspend_count_{0};
  std::atomic_uint64_t world_freshness_command_recovery_count_{0};
  std::atomic_uint64_t world_freshness_suspended_bundle_generation_{0};
  std::atomic_bool world_freshness_suspended_safety_suffix_active_{false};
  std::atomic_uint64_t command_execution_lease_rejection_count_{0};
  std::atomic_uint64_t command_execution_lease_terminal_latch_count_{0};
  navigation_execution::ExecutionStateFailureLatch command_execution_lease_failure_latch_;
  std::atomic_int command_execution_lease_reason_{0};
  std::atomic_int64_t command_execution_source_age_us_{0};
  std::atomic_int64_t command_execution_receive_age_us_{0};
  std::atomic_uint64_t map_update_exception_count_{0};
  std::atomic_uint64_t command_id_{0};
  std::atomic_uint64_t execution_transaction_id_{0};
  std::atomic_uint64_t command_goal_epoch_{0};
  std::atomic_bool accepting_observations_{true};
  std::atomic_bool planner_command_available_{false};
  std::atomic_bool planner_failure_latched_{false};
  std::atomic_bool safety_suffix_active_{false};
  std::atomic<ExecutionRecoveryState> execution_recovery_state_{
      ExecutionRecoveryState::kInitialHold};
  std::atomic_uint64_t planner_solve_generation_{0U};
  std::atomic_uint64_t active_planner_solve_generation_{0U};
  std::atomic_uint64_t timed_out_planner_solve_generation_{0U};
  std::atomic_int64_t planner_solve_started_steady_ns_{0};
  // A local planner backend trajectory can end at a known-free frontier before the
  // mission goal.  The FSM must call PlanFromRest again after that trajectory
  // finishes instead of holding the completed old trajectory forever.
  std::atomic_bool trajectory_finished_{false};
  std::atomic_bool trajectory_reaches_goal_{false};
  // Non-zero only after the command publisher has observed the terminal sample
  // of a bundle whose endpoint reaches the active mission goal. This is a
  // lifecycle marker for one terminal hold; it never authorizes future samples
  // from an expired trajectory.
  std::atomic_uint64_t terminal_bundle_generation_{0U};
  std::uint64_t cycle_count_{0};
  std::atomic_uint64_t cycle_success_count_{0};
  std::uint64_t optimizer_deferred_count_{0};
  std::uint64_t optimizer_renewal_due_count_{0};
  std::atomic_uint64_t command_publish_count_{0};
  std::int64_t last_planner_us_{0};
  std::atomic_int64_t last_publish_us_{0};
  std::atomic_int64_t last_command_store_publish_us_{0};
  std::atomic_int64_t last_command_transition_lock_wait_us_{0};
  std::int64_t last_input_lock_wait_us_{0};
  std::int64_t last_cycle_started_steady_ns_{0};
  std::int64_t planning_period_us_{0};
  std::int64_t last_planning_scheduling_gap_us_{0};
  navigation_mapping::ObservationAccounting observation_accounting_;
  std::chrono::steady_clock::time_point metrics_log_time_{std::chrono::steady_clock::now()};
  std::vector<double> end_to_end_samples_ms_;

  navigation_mapping::WorldSnapshotStore world_snapshot_store_;
  navigation_execution::CommittedBundleStore command_bundle_store_;
  navigation_execution::CommandSampler command_sampler_;
  std::shared_ptr<MappingTelemetry> mapping_telemetry_;
  std::shared_ptr<MappingLifecycleObserver> mapping_lifecycle_observer_;
  std::unique_ptr<navigation_mapping::MappingWorker<PendingRegisteredScan>> mapping_worker_;
  navigation_planning_backend::PlannerFacade* planner_{nullptr};
  std::unique_ptr<PlanningWorker<navigation_planning_backend::PlannerFacade>> planning_worker_;
};

}  // namespace navigation_runtime

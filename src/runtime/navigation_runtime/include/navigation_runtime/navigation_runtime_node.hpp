#pragma once

#include <atomic>
#include <cstdint>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>
#include <tracking_experiment.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_command_admission.hpp>
#include <navigation_contracts/msg/navigation_execution_diagnostics.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/navigation_mode_status.hpp>
#include <navigation_contracts/msg/navigation_mission_progress.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <navigation_mapping/mapping_worker.hpp>
#include <navigation_mapping/mapping_observation.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>
#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/observation_accounting.hpp>
#include <navigation_planning/planning_outcome.hpp>
#include <navigation_planning/candidate_bundle.hpp>
#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"
#include "navigation_runtime/baseline_refinement.hpp"
#include "navigation_runtime/same_identity_renewal_injection.hpp"
#include "navigation_runtime/execution_recovery_state.hpp"
#include "navigation_runtime/execution_lifecycle_view.hpp"
#include "navigation_runtime/trajectory_completion.hpp"
#include "navigation_runtime/planning_worker.hpp"
#include "navigation_runtime/heading_rebind_worker.hpp"
#include "navigation_runtime/execution_trace_snapshot.hpp"
#include "navigation_runtime/retained_decision_observation.hpp"
#include <navigation_execution/execution_state_gate.hpp>
#include <navigation_execution/execution_state_store.hpp>
#include <navigation_execution/execution_authority.hpp>
#include "navigation_runtime/desired_planning_intent.hpp"
#include <navigation_execution/command_sampler.hpp>
#include <navigation_mapping/world_snapshot_store.hpp>
#include <navigation_planning/planning_limits.hpp>
#include "navigation_runtime/kinematic_derivative_estimator.hpp"
#include "navigation_runtime/mission_progress.hpp"

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
  std::int64_t observation_decode_us{0};
  std::int64_t dirty_region_build_us{0};
  std::int64_t snapshot_export_us{0};
  std::int64_t snapshot_object_build_us{0};
  std::int64_t pending_revalidation_us{0};
  std::int64_t active_revalidation_us{0};
  std::int64_t world_publication_finalize_us{0};
  std::int64_t mapping_callback_total_us{0};
  std::int64_t pointcloud_decode_us{0};
  bool world_snapshot_published{false};
  std::uint64_t snapshot_export_mode{0};
  std::uint64_t snapshot_full_export_reason{0};
  std::uint64_t snapshot_export_base_cells{0};
  std::uint64_t snapshot_export_inflated_cells{0};
  std::uint64_t snapshot_patch_depth{0};
  std::uint64_t dirty_aabb_voxel_count{0};
  std::uint64_t dirty_chunk_count{0};
  std::uint64_t base_planning_state_change_count{0};
  std::uint64_t inflated_planning_state_change_count{0};
  // The exact inflated-state counter was removed from the ROG hot path in
  // 00517182. Keep an explicit validity bit so reports never interpret the
  // retained legacy field (currently zero) as an exact measurement.
  bool inflated_planning_state_change_count_valid{false};
  std::uint64_t full_snapshot_bytes{0};
  std::uint64_t copied_snapshot_bytes{0};
  std::uint64_t reused_snapshot_bytes{0};
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
  std::uint64_t discarded_missing_sensor_origin{0};
  std::uint64_t discarded_sensor_origin_contract{0};
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
  std::uint64_t snapshot_full_reason_no_current_count{0};
  std::uint64_t snapshot_full_reason_whole_world_count{0};
  std::uint64_t snapshot_full_reason_invalid_region_count{0};
  std::uint64_t snapshot_full_reason_patch_depth_count{0};
  std::uint64_t snapshot_full_reason_empty_patch_count{0};
  std::uint64_t snapshot_full_reason_patch_too_large_count{0};
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
    next.discarded_missing_sensor_origin = state_.discarded_missing_sensor_origin;
    next.discarded_sensor_origin_contract = state_.discarded_sensor_origin_contract;
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
    next.snapshot_full_reason_no_current_count = state_.snapshot_full_reason_no_current_count;
    next.snapshot_full_reason_whole_world_count = state_.snapshot_full_reason_whole_world_count;
    next.snapshot_full_reason_invalid_region_count = state_.snapshot_full_reason_invalid_region_count;
    next.snapshot_full_reason_patch_depth_count = state_.snapshot_full_reason_patch_depth_count;
    next.snapshot_full_reason_empty_patch_count = state_.snapshot_full_reason_empty_patch_count;
    next.snapshot_full_reason_patch_too_large_count = state_.snapshot_full_reason_patch_too_large_count;
    if (next.world_snapshot_published) {
      ++next.world_snapshot_published_count;
      if (next.snapshot_export_mode ==
          static_cast<std::uint64_t>(navigation_mapping::SnapshotExportMode::kFull)) {
        ++next.world_snapshot_full_export_count;
      } else if (next.snapshot_export_mode ==
                 static_cast<std::uint64_t>(navigation_mapping::SnapshotExportMode::kPatch)) {
        ++next.world_snapshot_patch_export_count;
      }
      if (next.snapshot_export_mode ==
          static_cast<std::uint64_t>(navigation_mapping::SnapshotExportMode::kFull)) {
        switch (static_cast<navigation_mapping::SnapshotFullExportReason>(
            next.snapshot_full_export_reason)) {
          case navigation_mapping::SnapshotFullExportReason::kNoCurrentSnapshot:
            ++next.snapshot_full_reason_no_current_count; break;
          case navigation_mapping::SnapshotFullExportReason::kWholeWorldChanged:
            ++next.snapshot_full_reason_whole_world_count; break;
          case navigation_mapping::SnapshotFullExportReason::kInvalidChangedRegion:
            ++next.snapshot_full_reason_invalid_region_count; break;
          case navigation_mapping::SnapshotFullExportReason::kPatchDepthLimit:
            ++next.snapshot_full_reason_patch_depth_count; break;
          case navigation_mapping::SnapshotFullExportReason::kEmptyPatch:
            ++next.snapshot_full_reason_empty_patch_count; break;
          case navigation_mapping::SnapshotFullExportReason::kPatchTooLarge:
            ++next.snapshot_full_reason_patch_too_large_count; break;
          case navigation_mapping::SnapshotFullExportReason::kNone: break;
        }
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
  void recordDiscard(
      bool stale, bool future, bool invalid,
      const navigation_mapping::MappingObservationRejectionReason reason =
          navigation_mapping::MappingObservationRejectionReason::kNone) {
    std::lock_guard lock(mutex_);
    state_.discarded_stale += stale ? 1U : 0U;
    state_.discarded_future += future ? 1U : 0U;
    state_.discarded_invalid += invalid ? 1U : 0U;
    state_.discarded_missing_sensor_origin +=
        reason == navigation_mapping::MappingObservationRejectionReason::kMissingSensorOrigin
            ? 1U : 0U;
    state_.discarded_sensor_origin_contract +=
        reason == navigation_mapping::MappingObservationRejectionReason::kSensorOriginContractMismatch
            ? 1U : 0U;
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

struct PendingHeadingRebind final {
  PlanningKey key{};
  navigation_planning::CandidateBundle candidate{};
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
  // Access-only test peer exercises the real callbacks/worker with barriers;
  // it adds no alternate runtime behavior or command-authority path.
  friend class NavigationRuntimeEpochResetTestPeer;
  friend class NavigationRuntimeTerminalMonitorTestPeer;
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
  // Caller holds localization_transition_mutex_ and input_mutex_.
  bool consumeForeignMissionCancelIfCurrentLocked();
  bool consumeForeignMissionCancelIfCurrent();
  void onModeStatus(
      const navigation_contracts::msg::NavigationModeStatus::ConstSharedPtr& message);
  void tickMissionProgress();
  // Caller holds localization_transition_mutex_ and input_mutex_. Record the
  // exact authorized command before it can be delivered to the adapter.
  void rememberMissionCommandIssued(
      const navigation_contracts::msg::NavigationCommand& command,
      bool execution_authorized);
  void onCommandAdmission(
      const navigation_contracts::msg::NavigationCommandAdmission::ConstSharedPtr& message);
  // Caller holds localization_transition_mutex_ and input_mutex_. The
  // decision and internal goal transition share this owner transaction.
  void applyMissionDecisionLocked(const MissionProgressDecision& decision);
  void schedulePlanningCycle();
  void scheduleHeadingRebind(const PlanningKey& key);
  void consumeHeadingRebind(std::int64_t now_ns);
  [[nodiscard]] static navigation_planning::PlanningHistory makePlanningHistory(
      const navigation_execution::ExecutionAuthoritySnapshot& execution,
      const navigation_planning::KinematicState& measured_state);

  [[nodiscard]] bool queueExecutionTimelineActivation(
      std::uint64_t generation) noexcept;
  void applyQueuedExecutionTimelineActivations(
      navigation_planning_backend::PlannerFacade& planner) noexcept;
  void runCycle(const PlanningKey& scheduled_key, std::stop_token stop);
  [[nodiscard]] std::optional<PlanningKey> currentPlanningKey();
  enum class RetainedValidationPurpose {
    kAfterFailedReplacement,
    kPlannerValidationOnly,
    kTerminalMainMonitor,
  };
  // Callback-local compare token, not a second active/pending owner. Both the
  // store cutover and late-failure delivery must still belong to this exact
  // pre-END execution authority snapshot.
  struct TerminalMonitorBoundary {
    navigation_execution::ExecutionAuthoritySnapshot snapshot;
  };
  // Callback-local facts only, never another execution owner. The worker
  // prepares its backend identity/world/cancellation before this transaction;
  // the transaction captures and rechecks the canonical runtime owners itself.
  struct RetainedValidationContext {
    RetainedValidationPurpose purpose;
    bool plan_from_rest_with_transition;
    bool transition_terminal_stop;
    std::uint64_t solve_generation;
    std::uint64_t diagnostic_planning_cycle_id;
    std::optional<navigation_planning::PlannerStatus> planner_result;
    double tracking_limit_m;
    std::optional<TerminalMonitorBoundary> terminal_monitor = std::nullopt;
    std::optional<navigation_execution::ExecutionAuthoritySnapshot>
        expected_execution = std::nullopt;
  };
  void validateRetainedCommand(
      const std::optional<navigation_contracts::msg::NavigationGoal>& goal,
      std::uint64_t goal_epoch, std::uint64_t localization_epoch_at_solve,
      const PlanningKey& effective_scheduled_key,
      const RetainedValidationContext& context);
  void observeRetainedDecision(
      const ExecutionTraceSnapshot& trace,
      const RetainedDecisionObservation& decision) noexcept;
  void publishCommand();
  [[nodiscard]] bool clearCommandForCurrentIdentity(
      const navigation_contracts::msg::NavigationGoal& command_goal,
      std::uint64_t goal_epoch_at_command,
      std::uint64_t localization_epoch_at_command,
      const navigation_execution::ExecutionAuthoritySnapshot& expected);
  bool commitPlannerCandidate(const navigation_contracts::msg::NavigationGoal& goal,
                             std::uint64_t goal_epoch,
                             std::uint64_t localization_epoch,
                             std::int64_t now_ns,
                             const PlanningKey& scheduled_key,
                             const std::optional<navigation_planning::CandidateBundle>&
                                 planned_candidate = std::nullopt,
                             bool* candidate_admitted = nullptr,
                             const std::optional<TerminalMonitorBoundary>&
                                 terminal_monitor = std::nullopt);
  // The one immediate execution cutover: canonical store + runtime identity
  // are delivered under the same owners. Backend ACK never delivers authority.
  navigation_execution::CommitDecision admitImmediateCandidate(
      const navigation_contracts::msg::NavigationGoal& goal,
      const navigation_execution::CommitToken& token,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& candidate,
      const navigation_execution::ExecutionAuthoritySnapshot& predecessor,
      const PlanningKey& key,
      const std::shared_ptr<const navigation_execution::ExecutionStateLease>& measured_state,
      std::int64_t maximum_world_age_ns,
      const std::optional<TerminalMonitorBoundary>& terminal_monitor = std::nullopt);
  void suspendCommandForWorldFreshness(
      const navigation_execution::ExecutionAuthoritySnapshot& expected);
  // Diagnostic-only immutable transaction witness. It is emitted after the
  // owner transaction has linearized and never feeds back into admission.
  void publishWorldTransactionWitness(
      const std::string& event_kind,
      const navigation_world_model::WorldSnapshotIdentity& prior_world,
      const navigation_world_model::WorldSnapshotIdentity& next_world,
      const navigation_execution::ExecutionAuthoritySnapshot& before,
      const navigation_execution::ExecutionAuthoritySnapshot& after,
      const std::string& disposition,
      int active_validation_path,
      int pending_validation_path,
      const std::string& temporal_assessment_reason = "NOT_APPLICABLE");
  // Ingress serialization remains held while this temporarily releases the
  // lifecycle owner lock to drain old mapping work.
  void resetForLocalizationEpochLocked(
      std::uint64_t localization_epoch,
      std::unique_lock<std::mutex>& localization_lock);
  // Caller holds command_execution_lease_failure_latch_.transitionMutex().
  bool applyExecutionRecoveryEventLocked(
      ExecutionRecoveryEvent event,
      const navigation_planning::CandidateBundle& bundle) noexcept;
  // Caller holds command_execution_lease_failure_latch_.transitionMutex().
  // Command-store invalidation remains explicit at call sites because ordinary
  // planner failures must retain a still-certified active command.
  void failClosedLocked() noexcept;
  // Read-only projections from the sole active execution record. Callers
  // needing a coherent bundle/goal pair use execution_authority_.snapshot().
  [[nodiscard]] std::optional<navigation_contracts::msg::NavigationGoal>
  executingGoalSnapshot() const;
  [[nodiscard]] std::uint64_t executionGoalEpoch() const noexcept;
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
  navigation_contracts::TrackingExperimentPolicy tracking_experiment_;
  double data_freshness_window_s_{0.5};
  std::int64_t data_freshness_window_ns_{500'000'000};
  std::int64_t command_stream_timeout_ns_{100'000'000};
  double planner_watchdog_timeout_s_{1.0};
  std::int64_t planner_watchdog_timeout_ns_{1'000'000'000};
  // Diagnostic-only deterministic failure injection. Zero disables it; when
  // matched, exactly one solve is converted to a failed result after the
  // backend returns, leaving the prior committed bundle untouched.
  std::uint64_t inject_failed_replan_cycle_id_{0U};
  bool inject_failed_replan_once_{false};
  bool inject_failed_replan_when_safe_{false};
  bool inject_failed_replan_after_handoff_{false};
  bool inject_failed_replan_repeated_{false};
  bool inject_failed_plan_from_rest_repeated_{false};
  // Diagnostic-only one-shot status substitution; never an execution owner.
  ExactOptimizationFailureInjection exact_optimization_failure_injection_;
  SameIdentityRenewalInjectionController same_identity_renewal_injection_;
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
  rclcpp::Publisher<navigation_contracts::msg::NavigationExecutionDiagnostics>::SharedPtr
      execution_diagnostics_publisher_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationCommandAdmission>::SharedPtr
      command_admission_subscription_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationMissionProgress>::SharedPtr
      mission_progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr propagated_state_callback_group_;

  std::mutex input_mutex_;
  // Lock order: ingress -> localization -> input -> command transition.
  // Mapping never takes ingress: it must finish while an epoch reset drains.
  std::mutex localization_epoch_ingress_mutex_;
  std::mutex localization_transition_mutex_;
  std::optional<MissionProgress> mission_progress_;
  struct ModeMissionBoundary {
    std::uint64_t activation_id{0U};
    std::int64_t source_stamp_ns{0};
    std::int64_t receive_steady_ns{0};
    bool airborne{false};
  };
  std::optional<ModeMissionBoundary> mode_mission_boundary_;
  std::atomic_uint64_t mode_activation_id_seen_{0U};
  // A delayed ACTIVE heartbeat from a terminal PX4 activation cannot restart
  // the same mission after takeover, failure or completion.
  std::uint64_t last_terminal_mode_activation_id_{0U};
  std::optional<std::uint64_t> mission_activation_applied_;
  // Commands awaiting a PX4-local admission receipt. The command's finite
  // lease bounds retention; no received receipt can create Core intent.
  std::deque<navigation_contracts::msg::NavigationCommand>
      issued_mission_commands_;
  navigation_execution::ExecutionStateStore execution_state_store_;
  DesiredPlanningIntent desired_intent_;
  // Mission-start anchor for the planner's first-leg route heading. It is
  // latched per mission/route/localization scope and is never recaptured on a
  // normal waypoint/request handoff.
  std::optional<Eigen::Vector3d> mission_start_position_world_;
  std::string mission_start_mission_id_;
  std::uint64_t mission_start_route_revision_{0U};
  std::uint64_t mission_start_localization_epoch_{0U};
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
  std::atomic_uint64_t active_localization_epoch_{1U};
  std::atomic_bool localization_epoch_ready_{true};
  std::atomic_uint64_t last_registered_scan_epoch_{1U};
  std::atomic_uint64_t last_registered_scan_sequence_{0U};
  std::atomic_int64_t last_propagated_state_stamp_ns_{0};
  std::atomic_uint64_t last_propagated_state_sequence_{0U};
  std::mutex propagated_derivative_mutex_;
  KinematicDerivativeEstimator propagated_derivative_estimator_;
  // PASS_THROUGH retarget disposition is a single typed fact in desired_intent_.
  // The ROS adapter suppresses one scheduler renewal after a successful
  // stopped-state plan, allowing the committed trajectory to establish its
  // continuous command before the next horizon check. This mirror does not own
  // planner recovery state.
  std::atomic_bool skip_replan_once_{false};
  // Accessed only by the serial planning worker, not sampler/mapping callbacks.
  BaselineRefinementOpportunity baseline_refinement_opportunity_;
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
  std::atomic_uint64_t world_transaction_event_sequence_{0};
  std::atomic_uint64_t command_execution_lease_rejection_count_{0};
  std::atomic_uint64_t command_execution_lease_terminal_latch_count_{0};
  std::atomic_uint64_t command_publication_deadline_miss_count_{0};
  navigation_execution::ExecutionStateFailureLatch command_execution_lease_failure_latch_;
  std::atomic_int command_execution_lease_reason_{0};
  // Structured reason for the last candidate rejection at the execution
  // boundary. This is observability only; a nonzero value never authorizes a
  // candidate and the active timeline remains the sole command authority.
  std::atomic_int last_execution_boundary_rejection_{0};
  // These fields describe the latest command-timer activation attempt. They
  // are diagnostic witnesses only; activation authority remains in the
  // execution authority. A nonzero generation is required before a
  // consumer may correlate the event with a candidate.
  std::atomic_uint64_t last_execution_activation_generation_{0U};
  std::atomic_int64_t last_execution_activation_started_steady_ns_{0};
  std::atomic_int64_t last_execution_activation_finished_steady_ns_{0};
  std::atomic_int last_execution_activation_result_{0};
  std::atomic_uint64_t last_command_sampled_generation_{0U};
  std::atomic_int64_t last_command_sampled_steady_ns_{0};
  std::atomic_int last_command_sampled_result_{0};
  // The watchdog witness is correlated by solve generation. It is never
  // treated as a failure for another transaction.
  std::atomic_uint64_t last_watchdog_generation_{0U};
  std::atomic_int64_t last_watchdog_event_steady_ns_{0};
  std::atomic_int64_t command_execution_source_age_us_{0};
  std::atomic_int64_t command_execution_receive_age_us_{0};
  std::atomic_uint64_t map_update_exception_count_{0};
  std::atomic_uint64_t command_id_{0};
  std::atomic_uint64_t execution_transaction_id_{0};
  std::atomic_bool accepting_observations_{true};
  // Diagnostic-only retained-command causal evidence is published as one
  // immutable record. It is copied into the command stream and never
  // consumed by admission or recovery predicates.
  ExecutionTraceStore execution_trace_store_;
  // Observation only; serial PlanningWorker owns these counters. Neither
  // command sampling nor recovery reads them or the enable switch.
  bool retained_decision_diagnostics_enabled_{true};
  RetainedObservationAccounting retained_decision_accounting_;
  std::atomic_uint64_t planner_solve_generation_{0U};
  std::uint64_t active_planner_solve_generation_{0U};
  // Provenance of the one in-flight solve, guarded by planner_solve_activity_mutex_.
  // This is an ephemeral callback witness, not a second execution owner.
  std::optional<PlannerSolveFailureWitness> active_planner_solve_witness_;
  std::atomic_uint64_t timed_out_planner_solve_generation_{0U};
  std::atomic_uint64_t planner_timeline_activation_generation_{0U};
  mutable std::mutex planner_timeline_activation_mutex_;
  std::deque<std::uint64_t> queued_planner_timeline_activations_;
  std::mutex planner_solve_activity_mutex_;
  std::int64_t planner_solve_started_steady_ns_{0};
  std::atomic_int last_planning_outcome_{
      static_cast<int>(navigation_planning::CompletePlanningOutcome::kInvalidRequest)};
  std::atomic_int last_planning_failure_stage_{
      static_cast<int>(navigation_planning::PlanningFailureStage::kInput)};
  std::atomic_int last_planning_failure_reason_{
      static_cast<int>(navigation_planning::PlanningFailureReason::kInvalidInput)};
  // A local planner backend trajectory can end at a known-free frontier before
  // the mission goal. The optional witness is serialized by the execution
  // transition mutex. Goal/timeline matching and lifecycle mutation use the
  // canonical localization -> input -> execution-transition lock order; it is
  // consumed only when its immutable bundle identity is still current.
  std::optional<TrajectoryCompletionWitness> trajectory_completion_witness_;
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
  std::int64_t last_planning_timer_expected_steady_ns_{0};
  std::int64_t last_planning_callback_start_steady_ns_{0};
  std::atomic_uint64_t planning_key_success_count_{0};
  std::atomic_uint64_t planning_key_unavailable_count_{0};
  std::atomic_uint64_t planning_submit_count_{0};
  std::atomic_uint64_t execution_timeline_invariant_violation_count_{0};
  navigation_mapping::ObservationAccounting observation_accounting_;
  std::chrono::steady_clock::time_point metrics_log_time_{std::chrono::steady_clock::now()};
  std::vector<double> end_to_end_samples_ms_;

  navigation_mapping::WorldSnapshotStore world_snapshot_store_;
  navigation_execution::ExecutionAuthority execution_authority_;
  navigation_execution::CommandSampler command_sampler_;
  std::shared_ptr<MappingTelemetry> mapping_telemetry_;
  std::shared_ptr<MappingLifecycleObserver> mapping_lifecycle_observer_;
  std::unique_ptr<navigation_mapping::MappingWorker<PendingRegisteredScan>> mapping_worker_;
  navigation_planning_backend::PlannerFacade* planner_{nullptr};
  std::unique_ptr<PlanningWorker<navigation_planning_backend::PlannerFacade>> planning_worker_;
  std::unique_ptr<HeadingRebindWorker> heading_rebind_worker_;
  mutable std::mutex heading_rebind_mutex_;
  std::optional<PendingHeadingRebind> pending_heading_rebind_;
};

}  // namespace navigation_runtime

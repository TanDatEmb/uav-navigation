"""Cluster prior scanner rows without pretending to resolve per-field disposition."""
import csv
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
PRIOR = HERE.parents[1] / "audit_state_authority_conservation" / "20260923T010729Z-7da3e97c" / "STATE_CONSERVATION_MATRIX.csv"


def cluster(owner, name):
    if owner in {"MappingWorker", "PlanningWorker", "HeadingRebindWorker"}:
        return "WorkerBookkeeping"
    if owner == "BaselineRefinementOpportunity": return "QualityReceipt"
    if owner == "PendingGoalHandoffOwner": return "DesiredMissionIntent"
    if owner == "WorldSnapshotStore": return "LatestWorld"
    if owner == "RouteProgress": return "MeasuredRouteHistory" if name == "state_" else "RouteGeometry"
    if owner == "KinematicDerivativeEstimator": return "MeasuredStateHistory"
    if owner == "ExecutionStateStore": return "MeasuredStateHistory"
    if owner == "ExecutionTimelineStore":
        if name in {"committed_", "active_goal_epoch_", "active_lineage_version_"}: return "ActiveExecutionIdentity"
        if name in {"pending_", "pending_activation_ns_"}: return "StagedExecution"
        if name == "world_identity_": return "CertifiedWorldEvidence"
        return "ExecutionTransactionBookkeeping"
    if owner == "ExecutionEpisodeSnapshot":
        if name in {"goal_epoch", "request_id"}: return "DesiredMissionIntent"
        if name in {"active_command_goal_epoch", "active_command_request_id", "active_generation"}: return "ActiveExecutionIdentity"
        if name == "localization_epoch": return "LocalizationEpoch"
        return "ExecutionSafetyDisposition"
    if owner == "MissionController":
        if name == "mission_": return "DesiredMissionIntent"
        if name in {"route_progress_", "previous_position_", "previous_position_time_s_"}: return "MeasuredRouteHistory"
        if name in {"active_waypoint_index_", "request_id_", "checkpoint_valid_"}: return "AcceptedMissionProgress"
        if name in {"braking_start_time_s_", "braking_end_time_s_", "stopped_start_time_s_", "arrival_start_time_s_", "hold_start_time_s_"}: return "TemporalStopConfirmation"
        if name in {"trajectory_ready_", "terminal_hold_pending_"}: return "SuccessorReadiness"
        if name == "pending_position_control_": return "PX4ModeRequest"
        return "MissionLifecycle"
    if owner == "NavigationModeExecutor":
        if name == "px4_hold_confirmed_": return "PX4VehicleStatus"
        if name == "hold_handover_next_retry_steady_ns_": return "PX4RetryDeadline"
        if name == "hold_handover_complete_navigation_failure_": return "PX4TransferContext"
        return "PX4ModeRequest"
    if owner == "NavigationMode":
        if name in {"mission_", "mission_controller_"}: return "DesiredMissionIntent"
        if name.startswith("last_completed_") or name in {"completion_position_", "mission_complete_published_", "mission_terminal_"}: return "AcceptedMissionProgress"
        if name.startswith("safety_suffix_") or name.startswith("planner_recovery_") or name == "safety_hold_position_": return "ExecutionSafetyDisposition"
        if name.startswith("last_px4_") or name.startswith("px4_local_") or name == "lio_to_px4_local_translation_ned_": return "PX4LocalStateFrame"
        if name.startswith("last_health_") or name.startswith("lio_health_") or name == "typed_health_seen_": return "LocalizationHealth"
        if name.startswith("last_odometry_") or name == "odometry_": return "MeasuredStateHistory"
        if name == "lio_localization_epoch_" or name == "alignment_latch_generation_": return "LocalizationEpoch"
        if name == "navigation_command_" or name.startswith("last_command_") or name.startswith("last_setpoint_") or name.startswith("last_velocity_"): return "CommandLease"
        if name in {"mode_active_", "handover_requested_", "failure_reported_", "last_status_state_"}: return "PX4BoundaryLifecycle"
        if name in {"last_propagated_state_stamp_ns_", "last_propagated_state_sequence_", "last_lio_diagnostics_ns_", "last_state_age_s_"}: return "LocalizationHealth"
        if name in {"last_goal_publish_ns_", "activation_time_"}: return "MissionLifecycle"
        if name.endswith("_s_") or name.endswith("_ns_"): return "TimingConfiguration"
        return "PX4BoundaryBookkeeping"
    if owner == "NavigationRuntimeNode":
        if name.startswith(("data_freshness", "command_stream_timeout", "planner_watchdog_timeout")): return "TimingConfiguration"
        if name in {"dynamics_hash_", "mission_dynamic_limits_"}: return "DynamicsIdentity"
        if name in {"active_goal_", "pending_goal_owner_", "new_goal_", "hot_goal_transition_"}: return "DesiredMissionIntent"
        if name in {"executing_goal_", "command_goal_epoch_", "command_id_", "execution_episode_", "command_bundle_store_", "execution_transaction_id_"}: return "ActiveExecutionIdentity"
        if name.startswith("mission_start_"): return "RouteGeometry"
        if name.startswith("foreign_") or name.startswith("deferred_terminal_"): return "ExecutionSafetyDisposition"
        if name in {"active_goal_epoch_", "active_localization_epoch_", "localization_epoch_ready_"}: return "LocalizationEpoch"
        if name.startswith(("last_registered_scan", "last_propagated_state")) or name in {"execution_state_store_", "propagated_derivative_estimator_"}: return "MeasuredStateHistory"
        if name.startswith("world_freshness_suspended_") or name == "command_execution_lease_failure_latch_": return "CommandLease"
        if name in {"world_snapshot_store_", "mapping_worker_", "mapping_lifecycle_observer_"}: return "LatestWorld"
        if name.startswith("trajectory_") or name.startswith("terminal_bundle_"): return "TemporalStopConfirmation"
        if name.startswith(("planner_", "active_planner", "timed_out_planner", "queued_planner", "last_cycle_", "last_planning_", "pending_heading_")) or name in {"planning_worker_", "heading_rebind_worker_", "skip_replan_once_", "baseline_refinement_opportunity_", "plan_from_rest_first_failure_steady_ns_"}: return "WorkerBookkeeping"
        if name.startswith("last_execution_") or name.startswith("last_command_sampled_") or name.startswith("last_watchdog_"): return "ExecutionTransactionBookkeeping"
        if name in {"mapping_telemetry_", "observation_accounting_", "end_to_end_samples_ms_", "retained_decision_accounting_", "last_execution_boundary_rejection_"}: return "Diagnostics"
        if name in {"accepting_observations_", "command_sampler_", "planner_"}: return "RuntimeInfrastructure"
        return "RuntimeBookkeeping"
    raise AssertionError(owner)


def main():
    with PRIOR.open(newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["Category"] == "behavioral candidate"]
    assert len(rows) == 239, len(rows)
    out = HERE / "FIELD_CLUSTER_MAP.csv"
    with out.open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["Field", "Semantic cluster", "Mapping confidence", "Prior disposition", "Source", "Note"])
        for r in rows:
            owner, name = r["Field"].split(".", 1)
            c = cluster(owner, name)
            confidence = "SOURCE_REVIEWED" if owner in {"MissionController", "ExecutionEpisodeSnapshot", "ExecutionTimelineStore", "NavigationModeExecutor", "RouteProgress"} else "LEXICAL_TENTATIVE"
            w.writerow([r["Field"], c, confidence, r["Target action"], r["Source"], "Cluster is not a field disposition or D_f proof"])
    counts = Counter(cluster(*r["Field"].split(".", 1)) for r in rows)
    print(f"mapped={len(rows)} clusters={len(counts)} tentative={sum(1 for r in rows if r['Current owner'] not in {'MissionController','ExecutionEpisodeSnapshot','ExecutionTimelineStore','NavigationModeExecutor','RouteProgress'})}")
    print(" ".join(f"{k}:{v}" for k, v in sorted(counts.items())))


if __name__ == "__main__": main()

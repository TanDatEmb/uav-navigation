"""Reconcile the pinned 239-field candidate inventory with shadow domains.

KEEP is conservative preservation, not evidence of sole writer. Destructive
choices are restricted to explicit semantic mirrors/derived fields.
"""
import csv
from collections import Counter
from pathlib import Path

CLUSTERS = Path('/home/letandat/Dev/uav-navigation-audit-semantic-closure-20260923/artifacts/audit_semantic_closure/20260923T020220Z-7da3e97c/FIELD_CLUSTER_MAP.csv')
OLD = Path('/home/letandat/Dev/uav-navigation-audit-state-authority-20260923/artifacts/audit_state_authority_conservation/20260923T010729Z-7da3e97c/STATE_CONSERVATION_MATRIX.csv')

# Domain-level default deliberately conserves independent temporal, protocol,
# geometry, localization and worker information rather than deleting by name.
DEFAULT = {
 'AcceptedMissionProgress':('MOVE','MissionProgressShadow','accepted gate / completion receipt'),
 'ActiveExecutionIdentity':('MERGE','ExecutionAuthorityShadow','active trajectory identity / immutable context'),
 'CertifiedWorldEvidence':('KEEP','ExecutionAuthorityShadow','active certificate identity'),
 'CommandLease':('KEEP','Px4AuthorityShadow','receive lease or command witness'),
 'DesiredMissionIntent':('MOVE','MissionProgressShadow','mission intent identity'),
 'Diagnostics':('KEEP','diagnostic sink','observability only'),
 'DynamicsIdentity':('KEEP','PlanningContext','dynamics identity'),
 'ExecutionSafetyDisposition':('MERGE','ExecutionAuthorityShadow','typed execution lifecycle'),
 'ExecutionTransactionBookkeeping':('KEEP','transaction diagnostics','causal transaction evidence'),
 'LatestWorld':('KEEP','WorldModel','immutable latest world / worker object'),
 'LocalizationEpoch':('KEEP','domain boundary','localization identity witness'),
 'LocalizationHealth':('KEEP','Px4AuthorityShadow','local health / source-time witnesses'),
 'MeasuredRouteHistory':('MOVE','MissionProgressShadow','route cursor and crossing witness'),
 'MeasuredStateHistory':('KEEP','measurement boundary','measured state and estimator memory'),
 'MissionLifecycle':('MERGE','MissionProgressShadow','mission lifecycle view'),
 'PX4BoundaryBookkeeping':('KEEP','Px4AuthorityShadow','PX4 output history'),
 'PX4BoundaryLifecycle':('KEEP','Px4AuthorityShadow','external mode lifecycle'),
 'PX4LocalStateFrame':('KEEP','Px4AuthorityShadow','PX4 frame / reset / health witness'),
 'PX4ModeRequest':('MERGE','Px4AuthorityShadow','typed HoldTransfer request'),
 'PX4RetryDeadline':('KEEP','Px4AuthorityShadow','retry deadline'),
 'PX4TransferContext':('KEEP','Px4AuthorityShadow','request cause'),
 'PX4VehicleStatus':('MOVE','Px4AuthorityShadow','fresh status witness'),
 'QualityReceipt':('KEEP','worker boundary','quality opportunity receipt'),
 'RouteGeometry':('KEEP','MissionProgressShadow','route geometry and ordered segments'),
 'RuntimeInfrastructure':('KEEP','runtime boundary','infrastructure, not authority'),
 'StagedExecution':('MOVE','ExecutionAuthorityShadow','staged successor distinct from active'),
 'SuccessorReadiness':('DERIVE','MissionProgressShadow','continuation identity / readiness evidence'),
 'TemporalStopConfirmation':('KEEP','ExecutionAuthorityShadow','source-time stop confirmation'),
 'TimingConfiguration':('KEEP','respective boundary','policy parameter'),
 'WorkerBookkeeping':('KEEP','computation worker','worker lifecycle, no flight authority'),
}
DERIVE_NAMES = {
 'NavigationRuntimeNode.data_freshness_window_s_',
 'NavigationRuntimeNode.planner_watchdog_timeout_s_',
 'NavigationMode.stale_after_s_', 'NavigationMode.state_stale_after_s_',
 'NavigationMode.planner_recovery_wait_timeout_s_',
 'NavigationMode.last_state_age_s_',
 'ExecutionEpisodeSnapshot.command_available',
 'ExecutionEpisodeSnapshot.safety_suffix_active',
 'ExecutionEpisodeSnapshot.restart_from_rest',
 'NavigationMode.mission_terminal_',
 'NavigationModeExecutor.px4_hold_confirmed_',
 'NavigationRuntimeNode.world_freshness_suspended_safety_suffix_active_',
 'NavigationRuntimeNode.trajectory_reaches_goal_',
 'RouteProgress.total_length_m_',
}
DELETE_NAMES = {
 'NavigationMode.planner_recovery_pending_',
 'NavigationMode.planner_recovery_deadline_ns_',
 'NavigationMode.planner_recovery_mission_id_',
 'NavigationMode.planner_recovery_waypoint_index_',
 'NavigationMode.planner_recovery_request_id_',
 'NavigationMode.planner_recovery_bundle_generation_',
 'NavigationMode.safety_suffix_handoff_pending_',
 'NavigationMode.safety_suffix_waypoint_index_',
 'NavigationMode.safety_suffix_request_id_',
 'NavigationMode.mission_controller_',
 'NavigationMode.mission_',
 'MissionController.checkpoint_valid_',
 'MissionController.trajectory_ready_',
 'MissionController.terminal_hold_pending_',
 'NavigationRuntimeNode.new_goal_',
 'NavigationRuntimeNode.hot_goal_transition_',
 'NavigationModeExecutor.hold_handover_in_flight_',
}
UNRESOLVED_NAMES = {
 'NavigationMode.planner_recovery_wait_timeout_ns_',
 'NavigationMode.trajectory_wait_timeout_s_',
 'NavigationMode.safety_hold_position_',
 'NavigationRuntimeNode.foreign_mission_hold_after_stop_',
 'NavigationRuntimeNode.foreign_mission_cancel_pending_',
 'NavigationRuntimeNode.foreign_cancel_target_epoch_',
 'NavigationRuntimeNode.foreign_cancel_transition_epoch_',
 'NavigationRuntimeNode.foreign_cancel_localization_epoch_',
 'NavigationMode.failure_reported_',
 'NavigationMode.last_status_state_',
}

def generate(out: Path):
    clusters={r['Field']:r for r in csv.DictReader(CLUSTERS.open())}
    old=list(csv.DictReader(OLD.open()))
    assert len(old)==239 and len(clusters)==239 and {r['Field'] for r in old}==set(clusters)
    out.parent.mkdir(parents=True,exist_ok=True)
    header=['Field','Cluster','Disposition','Target owner','Replacement / independent information','Confidence','Pinned source','Reason']
    result=[]
    for row in old:
        field=row['Field']; c=clusters[field]; cluster=c['Semantic cluster']
        disposition,owner,replacement=DEFAULT[cluster]
        reason='Conservative cluster disposition; nested product implementation may stay in its boundary.'
        confidence='CLUSTER_CONSERVATIVE' if c['Mapping confidence']=='LEXICAL_TENTATIVE' else 'SOURCE_REVIEWED_CLUSTER'
        if field in DERIVE_NAMES:
            disposition='DERIVE'; reason='Derived from canonical typed witness/value; requires equivalence test at migration.'
            if field=='NavigationModeExecutor.px4_hold_confirmed_':
                replacement='fresh VehicleStatus AUTO_LOITER witness after request; preserve status evidence'
        elif field in DELETE_NAMES:
            disposition='DELETE'; reason='Duplicated policy mirror or boolean lifecycle projection; replacement retains independent fact.'
        elif field in UNRESOLVED_NAMES:
            disposition='UNRESOLVED'; reason='Exact writer/reader or external-owner semantics not proved; do not delete at migration.'
        if field=='MissionController.route_progress_': replacement='ordered measured route cursor, not accepted waypoint'
        if field=='MissionController.previous_position_': replacement='crossing before/after source sample witness'
        if field=='ExecutionTimelineStore.pending_': replacement='staged trajectory separate from active'
        result.append(dict(zip(header,[field,cluster,disposition,owner,replacement,confidence,row['Source'],reason])))
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=header,lineterminator="\n");w.writeheader();w.writerows(result)
    return Counter(r['Disposition'] for r in result)

if __name__=='__main__':
    import sys
    print(dict(generate(Path(sys.argv[1]))))

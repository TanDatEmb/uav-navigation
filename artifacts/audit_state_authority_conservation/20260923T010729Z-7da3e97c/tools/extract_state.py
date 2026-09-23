#!/usr/bin/env python3
"""Conservative declaration inventory for the pinned TARGET; no source edits.

This is a candidate-field scanner, not a C++ semantic analyzer. Rows whose
writer/reader/invalidation have not been manually established stay UNRESOLVED.
"""
import csv
import hashlib
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parents[1]
SPECS = [
    ("NavigationRuntimeNode", "src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp", 384, 594),
    ("ExecutionEpisodeSnapshot", "src/runtime/navigation_runtime/include/navigation_runtime/execution_episode.hpp", 42, 63),
    ("ExecutionTimelineStore", "src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp", 821, 829),
    ("MissionController", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/mission_controller.hpp", 110, 130),
    ("NavigationMode", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/navigation_mode.hpp", 135, 261),
    ("NavigationModeExecutor", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/navigation_mode.hpp", 285, 290),
    ("PlanningWorker", "src/runtime/navigation_runtime/include/navigation_runtime/planning_worker.hpp", 342, 350),
    ("PendingGoalHandoffOwner", "src/runtime/navigation_runtime/include/navigation_runtime/planner_fsm.hpp", 89, 90),
    ("WorldSnapshotStore", "src/mapping/navigation_mapping/include/navigation_mapping/world_snapshot_store.hpp", 176, 178),
    ("RouteProgress", "src/contracts/navigation_mission/include/navigation_mission/route_progress.hpp", 150, 155),
    ("ExecutionStateStore", "src/execution/navigation_execution/include/navigation_execution/execution_state_store.hpp", 44, 47),
    ("KinematicDerivativeEstimator", "src/runtime/navigation_runtime/include/navigation_runtime/kinematic_derivative_estimator.hpp", 86, 91),
    ("HeadingRebindWorker", "src/runtime/navigation_runtime/include/navigation_runtime/heading_rebind_worker.hpp", 113, 121),
    ("BaselineRefinementOpportunity", "src/runtime/navigation_runtime/include/navigation_runtime/baseline_refinement.hpp", 101, 102),
    ("MappingWorker", "src/mapping/navigation_mapping/include/navigation_mapping/mapping_worker.hpp", 280, 293),
    ("ExecutionTraceStore", "src/runtime/navigation_runtime/include/navigation_runtime/execution_trace_snapshot.hpp", 175, 177),
]

BEHAVIORAL_KEEP = {
    ("ExecutionTimelineStore", "committed_"): ("KEEP", "ExecutionAuthority", "active immutable bundle"),
    ("ExecutionTimelineStore", "pending_"): ("KEEP", "ExecutionAuthority", "staged immutable successor"),
    ("ExecutionTimelineStore", "pending_activation_ns_"): ("KEEP", "ExecutionAuthority", "future cutover boundary"),
    ("ExecutionTimelineStore", "world_identity_"): ("KEEP", "WorldAuthority", "last published world identity"),
    ("ExecutionEpisodeSnapshot", "localization_epoch"): ("KEEP", "ExecutionAuthority", "localization reset fence"),
    ("ExecutionEpisodeSnapshot", "goal_epoch"): ("MOVE", "MissionIntent", "desired intent revision"),
    ("ExecutionEpisodeSnapshot", "request_id"): ("MOVE", "MissionIntent", "desired request identity"),
    ("NavigationRuntimeNode", "active_goal_"): ("KEEP", "MissionIntent", "desired goal"),
    ("NavigationRuntimeNode", "executing_goal_"): ("KEEP", "ExecutionAuthority", "executing goal may lag desired"),
    ("MissionController", "active_waypoint_index_"): ("KEEP", "MissionProgress", "accepted mission gate"),
    ("MissionController", "previous_position_"): ("KEEP", "MissionProgress", "previous measured sample for segment crossing"),
    ("MissionController", "previous_position_time_s_"): ("KEEP", "MissionProgress", "measurement interval bound"),
    ("NavigationModeExecutor", "px4_hold_confirmed_"): ("KEEP", "PX4Boundary", "VehicleStatus observation"),
    ("NavigationModeExecutor", "hold_handover_in_flight_"): ("KEEP", "PX4Boundary", "outstanding Hold API operation"),
    ("NavigationModeExecutor", "hold_handover_next_retry_steady_ns_"): ("KEEP", "PX4Boundary", "retry deadline"),
    ("NavigationMode", "navigation_command_"): ("KEEP", "PX4Boundary", "last independently admitted command and lease"),
    ("NavigationMode", "px4_local_frame_aligned_"): ("KEEP", "PX4Boundary", "independent frame alignment latch"),
    ("NavigationMode", "lio_localization_epoch_"): ("KEEP", "PX4Boundary", "independently observed estimator epoch"),
    ("WorldSnapshotStore", "latest_"): ("KEEP", "WorldAuthority", "latest immutable world pointer"),
    ("PendingGoalHandoffOwner", "pending_goal_"): ("MOVE", "MissionIntent", "new intent deferred during safety suffix"),
    ("RouteProgress", "state_"): ("KEEP", "MissionProgress", "measured route cursor/history"),
    ("ExecutionStateStore", "state_"): ("KEEP", "ExecutionStateInput", "source/receive-stamped immutable execution state"),
    ("ExecutionStateStore", "active_epoch_"): ("KEEP", "ExecutionStateInput", "reset fence for state input"),
    ("KinematicDerivativeEstimator", "previous_stamp_ns_"): ("KEEP", "ExecutionStateInput", "temporal derivative history"),
    ("MappingWorker", "ready_"): ("KEEP", "MappingWorker", "latest admitted observation pending processing"),
}

# Selected path evidence only. Empty entries remain explicitly unresolved; this
# is not a whole-repository alias/write proof.
FIELD_EVIDENCE = {
 ("ExecutionTimelineStore", "committed_"): ("commit/activate/revoke methods", "load/publishIfCurrent/anchor checks", "execution until revoke", "replacement or revoke", "yes: active trajectory", "no", "committed_bundle_store.hpp:79-566,821-829"),
 ("ExecutionTimelineStore", "pending_"): ("stage/activate/revoke methods", "activatePendingIfDue and snapshot", "stage to cutover/cancel", "activation/revoke", "yes: successor", "no", "committed_bundle_store.hpp:30-35,821-829"),
 ("ExecutionTimelineStore", "pending_activation_ns_"): ("stage/activate/revoke methods", "activation guard", "stage to cutover/cancel", "activation/revoke", "yes: future time", "possibly from pending bundle; proof pending", "committed_bundle_store.hpp:30-35,821-829"),
 ("ExecutionTimelineStore", "world_identity_"): ("world publish/recertification methods", "commit/admission/world comparison", "published world epoch", "new world/reset", "yes: latest published world", "no", "committed_bundle_store.hpp:821-829"),
 ("ExecutionEpisodeSnapshot", "localization_epoch"): ("ExecutionEpisode::reset/beginGoal", "bundleIdentityMatchesEpisode", "localization epoch", "reset/new epoch", "yes: reset fence", "may derive from core epoch; proof pending", "execution_episode.hpp:74-105,246-260"),
 ("ExecutionEpisodeSnapshot", "goal_epoch"): ("ExecutionEpisode::beginGoal", "desiredPlanningKey and bundleIdentityMatchesEpisode", "desired intent", "new goal/reset", "yes: desired revision", "from canonical desired intent", "execution_episode.hpp:85-105,246-260"),
 ("ExecutionEpisodeSnapshot", "request_id"): ("ExecutionEpisode::beginGoal", "desiredPlanningKey and bundleIdentityMatchesEpisode", "desired intent", "new goal/reset", "yes: request identity", "from canonical desired intent", "execution_episode.hpp:85-105,246-260"),
 ("ExecutionEpisodeSnapshot", "active_command_goal_epoch"): ("commandCommitted/reset/nonretained beginGoal", "activeBundleIdentityMatches", "active bundle episode", "new active/revoke/reset", "active differs from desired", "possibly active bundle key", "execution_episode.hpp:85-115,253-263"),
 ("ExecutionEpisodeSnapshot", "active_command_request_id"): ("commandCommitted/reset/nonretained beginGoal", "activeBundleIdentityMatches", "active bundle episode", "new active/revoke/reset", "active differs from desired", "possibly active bundle key", "execution_episode.hpp:85-115,253-263"),
 ("ExecutionEpisodeSnapshot", "active_generation"): ("commandCommitted/stoppedHold/reset", "activeBundleIdentityMatches/commit guard", "active bundle episode", "new active/revoke/reset", "yes: execution generation", "possibly active bundle key", "execution_episode.hpp:108-116,205-219,253-263"),
 ("ExecutionEpisodeSnapshot", "phase"): ("beginGoal/commit/sample/stoppedHold/failClosed", "planning/publication phase guards", "execution episode", "transition/reset", "policy phase", "possibly sum type with recovery", "execution_episode.hpp:85-243"),
 ("ExecutionEpisodeSnapshot", "command_available"): ("commit/retained/stoppedHold/suspend/failClosed", "publication/retention guards", "execution episode", "suspend/failure/reset", "exposure permission memory", "not from active pointer alone", "execution_episode.hpp:108-243; navigation_runtime_node.cpp:2983-3004"),
 ("ExecutionEpisodeSnapshot", "failure_latched"): ("failClosed/eligible reset", "commit/retention gate", "failure episode", "reset/eligible new goal", "temporal failure memory", "possibly from variant; proof pending", "execution_episode.hpp:85-243"),
 ("ExecutionEpisodeSnapshot", "safety_suffix_active"): ("commit/sample/preserve/stoppedHold/reset", "retarget and publication guard", "safety suffix episode", "stop/new command/reset", "physical role memory", "not from bundle role alone after sample", "execution_episode.hpp:108-219"),
 ("ExecutionEpisodeSnapshot", "restart_from_rest"): ("requestRestartFromRest/clear/stoppedHold", "PlanFromRest selection", "stopped recovery attempt", "consume/fail/reset", "temporal request", "possibly recovery variant; proof pending", "execution_episode.hpp:182-219"),
 ("ExecutionEpisodeSnapshot", "recovery_state"): ("commit/sample/recovery event/failClosed", "nominal admission/retry policy", "execution recovery episode", "transition/reset", "policy state", "cannot equate to phase by name", "execution_episode.hpp:108-243; execution_recovery_state.hpp:7-89"),
 ("NavigationRuntimeNode", "active_goal_"): ("goal/reset callbacks", "planning key/admission/command guard", "desired goal", "new goal/reset", "yes: desired intent", "no", "navigation_runtime_node.hpp:429-463; .cpp:3006-3015"),
 ("NavigationRuntimeNode", "executing_goal_"): ("commit/cutover/reset paths", "command publication/exact identity", "active execution", "cutover/revoke/reset", "yes: executing intent", "possibly from active bundle plus route payload; proof pending", "navigation_runtime_node.hpp:444-450; .cpp:8595-8615"),
 ("MissionController", "active_waypoint_index_"): ("MissionController::update/activate", "activeWaypoint/goal event/progression", "mission session", "accepted gate/new mission", "yes: policy acceptance", "no: cursor is measurement only", "mission_controller.cpp:340-590"),
 ("MissionController", "previous_position_"): ("MissionController::update", "measuredWaypointCrossingError", "adjacent measured samples", "next finite sample/reset", "yes: temporal memory", "no from latest observation alone", "mission_controller.cpp:363-371,465-477"),
 ("MissionController", "previous_position_time_s_"): ("MissionController::update", "sample-gap guard", "adjacent measured samples", "next finite sample/reset", "yes: temporal memory", "no from latest observation alone", "mission_controller.cpp:363-371,465-477"),
 ("NavigationMode", "navigation_command_"): ("onNavigationCommand/lifecycle", "updateMission/updateSetpoint", "last locally accepted command", "new accepted sample/expiry/lifecycle", "yes: receive witness", "no from producer state", "navigation_mode_node.cpp:1328-1410"),
 ("NavigationMode", "px4_local_frame_aligned_"): ("PX4 local-position/reset path", "setpoint conversion gate", "PX4 frame alignment", "reset/lifecycle", "yes: independently observed frame", "no from planner state", "navigation_mode.hpp:152-180"),
 ("NavigationMode", "lio_localization_epoch_"): ("health/odometry input", "command epoch comparison", "estimator epoch", "new estimator epoch/lifecycle", "yes: receiver observation", "no from command alone", "navigation_mode.hpp:181-196; navigation_mode_node.cpp:1384-1388"),
 ("NavigationMode", "mission_terminal_"): ("mission event/lifecycle", "command admission/updateMission/setpoint", "mission terminal observation", "lifecycle", "may mirror MissionController Complete", "not safely without explicit event", "navigation_mode_node.cpp:737-738,1617-1618,2467-2472"),
 ("NavigationMode", "handover_requested_"): ("mission/safety event/lifecycle", "command admission/setpoint", "handover request", "lifecycle/authority release", "protocol request memory", "may encode in PX4 protocol variant", "navigation_mode_node.cpp:1482-1639,2467-2472"),
 ("NavigationMode", "planner_recovery_pending_"): ("completed command/lifecycle/clear helper", "recovery expiry/setpoint/admission", "completed endpoint recovery window", "new command/timeout/lifecycle", "receiver deadline memory", "may replace with explicit protocol event", "navigation_mode_node.cpp:74-92,1234-1241,1340-1355"),
 ("NavigationMode", "planner_recovery_deadline_ns_"): ("completed command/clear helper", "recovery expiry", "completed endpoint recovery window", "new command/timeout/lifecycle", "receiver temporal deadline", "no from producer timer", "navigation_mode_node.cpp:74-92,1234-1241,1340-1355"),
 ("NavigationMode", "safety_suffix_handoff_pending_"): ("retained BACKUP handoff/clear", "adjacent command admission", "lease-bound suffix bridge", "first current command/lifecycle/stop", "receiver handoff memory", "possibly explicit retained-reference protocol", "navigation_mode_node.cpp:918-923,1499-1520,1852-1854"),
 ("NavigationMode", "safety_suffix_waypoint_index_"): ("retained BACKUP handoff/clear", "adjacent command admission", "lease-bound suffix bridge", "first current command/lifecycle/stop", "identity witness", "may derive from retained command if retained", "navigation_mode_node.cpp:918-923,1499-1520"),
 ("NavigationMode", "safety_suffix_request_id_"): ("retained BACKUP handoff/clear", "adjacent command admission", "lease-bound suffix bridge", "first current command/lifecycle/stop", "identity witness", "may derive from retained command if retained", "navigation_mode_node.cpp:918-923,1499-1520"),
 ("NavigationModeExecutor", "px4_hold_confirmed_"): ("onVehicleStatus/lifecycle", "checkHoldHandover", "external nav-state observation", "new status/lifecycle", "yes: external protocol", "no from API callback", "navigation_mode_node.cpp:2846-2872"),
 ("NavigationModeExecutor", "hold_handover_in_flight_"): ("schedulePx4Hold/API callback/status", "duplicate-request guard/retry", "outstanding API call", "callback/status/lifecycle", "yes: protocol operation", "can encode in tagged protocol state", "navigation_mode_node.cpp:2846-2937"),
 ("NavigationModeExecutor", "hold_handover_next_retry_steady_ns_"): ("API failure callback", "checkHoldHandover", "retry window", "new attempt/lifecycle", "yes: temporal deadline", "no from current status alone", "navigation_mode_node.cpp:2857-2863,2917-2923"),
 ("NavigationModeExecutor", "hold_handover_pending_"): ("schedulePx4Hold/API callback/status", "checkHoldHandover", "request through disposition", "Success/Deactivated/status/lifecycle", "protocol request memory", "tagged protocol state candidate", "navigation_mode_node.cpp:2846-2937"),
 ("NavigationModeExecutor", "hold_handover_complete_navigation_failure_"): ("schedulePx4Hold/API failure/lifecycle", "retry context", "handover episode", "lifecycle", "failure context", "tagged protocol payload candidate", "navigation_mode_node.cpp:2894-2923"),
 ("WorldSnapshotStore", "latest_"): ("publish", "load/authorize", "world publication", "new snapshot/reset", "yes: world evidence", "no", "world_snapshot_store.hpp:131-178"),
 ("PendingGoalHandoffOwner", "pending_goal_"): ("enqueueGoal/consumeGoal/clearGoal", "goalSnapshot/goalMatchesStatus", "deferred handoff", "exact consume/clear", "yes: deferred intent", "no from current active", "planner_fsm.hpp:23-90"),
 ("RouteProgress", "state_"): ("RouteProgress::update/reset", "snapshot/projection tie", "route-local measured cursor", "reset/new observation", "yes: physical history", "no from scalar arc alone", "route_progress.cpp:254-345"),
 ("ExecutionStateStore", "state_"): ("publish/reset", "load/current state freshness", "current estimator epoch", "new state/reset", "yes: measured source and receive witness", "no", "execution_state_store.hpp:12-48"),
 ("ExecutionStateStore", "active_epoch_"): ("reset/publish", "epoch admission", "estimator epoch", "new epoch", "yes: reset fence", "maybe from canonical epoch; compare pending input", "execution_state_store.hpp:12-48"),
 ("KinematicDerivativeEstimator", "previous_stamp_ns_"): ("update/reset", "derivative finite difference", "adjacent state samples", "new sample/reset", "yes: physical temporal memory", "no from current sample", "kinematic_derivative_estimator.hpp:20-91"),
 ("MappingWorker", "ready_"): ("submit/worker consume/reset", "worker wake/process", "queued observation", "consume/reset", "yes: input payload", "no", "mapping_worker.hpp:23-293"),
 ("BaselineRefinementOpportunity", "owner_"): ("noteAdmission/consume", "ready", "initial baseline owner", "consume/new goal", "yes: one-shot scheduling receipt", "no from active bundle after consumption", "baseline_refinement.hpp:30-103"),
}

METRIC_WORDS = ("_count_", "_count", "_us_", "_age_us_", "metrics_", "trace_", "diagnostic_", "debug_", "_reason_")
INFRA_TYPES = ("mutex", "Publisher", "Subscription", "TimerBase", "CallbackGroup", "thread", "condition_variable", "function<void()>")
CONFIG_NAMES = {"planning_frame_", "body_frame_", "body_frame_id_", "deployment_profile_", "planner_config_path_", "registered_scan_topic_", "propagated_odometry_topic_", "goal_topic_", "status_topic_", "command_topic_", "navigation_command_topic_", "state_topic_", "tracking_experiment_"}
DECL = re.compile(r"^\s*(?P<type>[^();]+?)\s+(?P<name>[A-Za-z][A-Za-z0-9_]*_?)\s*(?:\{[^;]*\})?\s*;\s*$")

def blob_hash(path):
    return subprocess.check_output(["git", "hash-object", str(ROOT / path)], text=True, cwd=ROOT).strip()

def classify(owner, name, typ):
    if owner == "ExecutionTraceStore":
        return "diagnostic candidate", "", "", ""
    if any(t in typ for t in INFRA_TYPES) or name in CONFIG_NAMES or name.endswith(("_subscription_", "_publisher_", "_timer_", "_callback_group_")):
        return "infrastructure/config", "", "", ""
    if name.startswith("inject_") or name in {"retained_decision_diagnostics_enabled_", "same_identity_renewal_injection_"}:
        return "test/diagnostic control", "", "", ""
    if any(w in name for w in METRIC_WORDS) and name not in {"command_goal_epoch_", "request_id_"}:
        return "diagnostic candidate", "", "", ""
    key = (owner, name)
    if key in BEHAVIORAL_KEEP:
        action, target, replacement = BEHAVIORAL_KEEP[key]
        return "behavioral candidate", action, target, replacement
    return "behavioral candidate", "UNRESOLVED", "UNRESOLVED", "not established"

def main():
    assert subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, cwd=ROOT).strip() == "7da3e97cb399c2e39d62cfe60213a45e8a92300e"
    rows, manifest = [], {}
    for owner, path, start, end in SPECS:
        lines = (ROOT / path).read_text().splitlines()
        manifest[path] = blob_hash(path)
        for lineno in range(start, min(end, len(lines)) + 1):
            line = lines[lineno - 1]
            m = DECL.match(line)
            if not m:
                continue
            typ, name = m.group("type").strip(), m.group("name")
            if name.startswith("k") and "constexpr" in typ:
                continue
            category, action, target, replacement = classify(owner, name, typ)
            details = FIELD_EVIDENCE.get((owner, name))
            rows.append({
                "Field": f"{owner}.{name}", "Current owner": owner,
                "Semantic meaning": replacement if action and action != "UNRESOLVED" else "requires writer/reader proof",
                "Writer(s)": details[0] if details else "UNRESOLVED", "Reader(s)": details[1] if details else "UNRESOLVED",
                "Lifetime": details[2] if details else "UNRESOLVED", "Invalidation": details[3] if details else "UNRESOLVED",
                "Independent info?": details[4] if details else "UNRESOLVED", "Can derive?": details[5] if details else "UNRESOLVED",
                "Target action": action, "Target owner": target, "Replacement": replacement,
                "Equivalence evidence": details[6] + "; target equivalence pending" if details else "source declaration only; transition proof pending",
                "Category": category, "Type": typ, "Source": f"{path}:{lineno}",
                "Blob": manifest[path],
            })
    with (OUT / "STATE_CONSERVATION_MATRIX.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(r for r in rows if r["Category"] == "behavioral candidate")
    with (OUT / "DECLARATION_TRIAGE.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)
    summary = {"target_sha": "7da3e97cb399c2e39d62cfe60213a45e8a92300e", "scoped_declarations": len(rows), "behavioral_candidates": sum(r["Category"] == "behavioral candidate" for r in rows), "diagnostic_candidates": sum("diagnostic" in r["Category"] for r in rows), "file_blobs": manifest}
    (OUT / "extraction_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k:v for k,v in summary.items() if k != "file_blobs"}))

if __name__ == "__main__":
    main()

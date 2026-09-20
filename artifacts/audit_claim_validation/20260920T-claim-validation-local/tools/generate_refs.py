#!/usr/bin/env python3
"""Generate line/hash-backed JSONL refs from frozen A and pinned dependencies."""
import argparse
import hashlib
import json
from pathlib import Path

SPECS = [
    ("E-H0-01", "src/contracts/navigation_contracts/include/navigation_contracts/tracking_experiment.hpp", 29, 54, "TrackingExperimentPolicy::trackingGateEnabled/valid", "Defines the active coefficient predicate and validity constraints."),
    ("E-H0-02", "src/contracts/navigation_contracts/include/navigation_contracts/tracking_experiment.hpp", 57, 90, "loadTrackingExperimentPolicy", "Reads sim clock and coefficients, derives suppression flags, rejects invalid/velocity-only non-sim inputs."),
    ("E-H0-03", "src/navigation_bringup/launch/navigation_runtime.launch.py", 10, 31, "navigation_runtime.launch.py", "Launch defaults and per-node use_sim_time/config overrides."),
    ("E-H0-04", "src/navigation_bringup/launch/avoidance_mission.launch.py", 1, 52, "avoidance_mission.launch.py", "Paired production launch wiring and shared launch substitutions."),
    ("E-H0-05", "config/runtime/mapping.yaml", 1, 21, "navigation_runtime_node ROS parameters", "Checked-in mapping/profile/freshness defaults; not a live parameter dump."),
    ("E-H0-06", "config/runtime/external_mode.yaml", 1, 21, "external mode ROS parameters", "Checked-in PX4 adapter timing/frame/tracking parameters."),
    ("E-H0-07", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 655, 760, "NavigationRuntimeNode constructor parameter load", "Runtime profile, freshness and command-stream timing validation."),
    ("E-H0-08", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 90, 165, "NavigationMode constructor timing/policy load", "Independent adapter parameters and validation."),
    ("E-H1-01", "src/runtime/navigation_runtime/include/navigation_runtime/certified_continuation.hpp", 48, 98, "minimumMainContinuationReserveNs/certifiedMainContinuationBoundaryEligible/certifiedMainContinuationHandoffReady", "Boundary eligibility requires reserve at planned boundary; live witness rechecked against MAIN end."),
    ("E-H1-02", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 350, 390, "certifiedMainContinuationWindow", "Candidate boundary is built and checked against exact identity, route role and MAIN interval."),
    ("E-H1-03", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 8375, 8389, "NavigationRuntimeNode::publishCommand continuation fields", "Only sampled MAIN command with current window and remaining reserve gets continuation flag."),
    ("E-H1-04", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 1328, 1336, "NavigationMode::onNavigationCommand", "Accepted matching continuation immediately invokes updateMission outside mission timer."),
    ("E-H1-05", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 1376, 1408, "NavigationMode::updateMission witness selection", "Rechecks source/receive freshness, health, lifecycle and exact mission/waypoint/request identity."),
    ("E-H1-06", "src/px4/px4_navigation_external_mode/src/mission_controller.cpp", 356, 382, "MissionController::update position sample", "Previous measured position/time are replaced on each valid update."),
    ("E-H1-07", "src/px4/px4_navigation_external_mode/src/mission_controller.cpp", 560, 588, "MissionController::update progression gates", "Pass-through progress requires crossing plus continuation or a specific stop/coincident/initial exception."),
    ("E-H1-08", "src/contracts/navigation_mission/src/route_progress.cpp", 430, 475, "RouteProgress::measuredWaypointCrossingError", "Measured acceptance-ball and segment-crossing constraints."),
    ("E-H1-09", "src/runtime/navigation_runtime/test/test_certified_continuation.cpp", 73, 87, "CertifiedContinuation.ExactPlannedEntryReserveDoesNotAuthorizeLaterMeasuredHandoff", "Existing assertions test planned reserve then false at boundary+1ns/+20ms/+100ms."),
    ("E-H1-10", "src/contracts/navigation_mission/test/test_mission_contract.cpp", 232, 255, "RouteProgress measured crossing assertions", "Existing test assertions for forward segment crossing and lateral miss."),
    ("E-H2-01", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 2981, 3002, "NavigationRuntimeNode::suspendCommandForWorldFreshness", "Suspends exposure while retaining exact execution identity/generation."),
    ("E-H2-02", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 3672, 3689, "planner cycle world freshness gate", "Stale world cancels worker and suspends rather than publishing."),
    ("E-H2-03", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 7702, 7718, "NavigationRuntimeNode::publishCommand world freshness gate", "Stale world callback emits no command and preserves only exact resume path."),
    ("E-H2-04", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 1234, 1286, "world recertification resume block", "Exact generation/epoch/execution identity rechecks before restoring episode."),
    ("E-H2-05", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2628, 2657, "NavigationMode::updateSetpoint command lease checks", "Receive/header lease and exact completed recovery exception."),
    ("E-H2-06", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2210, 2235, "NavigationMode::safetyStopNavigation", "Failure/handover latch, command invalidation, controller deactivation and Hold request."),
    ("E-H2-07", "src/runtime/navigation_runtime/test/test_execution_episode.cpp", 259, 310, "ExecutionEpisode suspension assertions", "Assertions cover suspend identity retention and one-way recovery events; not PX4 adapter behavior."),
    ("E-H3-01", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 8575, 8614, "NavigationRuntimeNode::publishCommand exposure lock prefix", "Three runtime transition locks wrap the store call."),
    ("E-H3-02", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 8655, 8688, "publishIfCurrent finalizer and ROS publish", "Final guards and Publisher::publish run in finalizer before lock scopes end."),
    ("E-H3-03", "src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp", 365, 405, "CommittedBundleStore::publishIfCurrent", "Store lock and finalizer callback scope."),
    ("E-H3-04", "src/execution/navigation_execution/test/test_committed_bundle_store.cpp", 1300, 1345, "CommittedBundleStore.ExposureMustRecheckFreshnessAfterWaitingForStoreLock", "Barrier test checks store freshness after lock contention; it does not publish ROS."),
    ("E-H4-01", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", 5390, 5435, "RetryFromRest failure timer", "First current-identity failure starts steady timer and stopped-state guard."),
    ("E-H4-02", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 1224, 1252, "accepted completed command recovery window", "Adapter recovery timer starts at accepted terminal completion."),
    ("E-H4-03", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2724, 2752, "completed setpoint recovery timer", "Adapter recovery timer can also start when completed endpoint is sampled."),
    ("E-H4-04", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/planner_recovery.hpp", 1, 40, "planner recovery helpers", "Timeout predicates and identity/episode match scope."),
    ("E-H4-05", "src/runtime/navigation_runtime/test/test_planner_fsm.cpp", 978, 1028, "PlannerFsm recovery timer assertions", "Assertions distinguish stopped-only timeout, bounded stopped hold and exact fresh-world resume."),
    ("E-H5-01", "src/planning/navigation_planning_backend/include/planner_core/evidence_speed_governor.hpp", 48, 140, "evidenceAwareSpeedLimitImpl", "Measured PVAJ stop check, 16-point grid, per-candidate abort and failure taxonomy."),
    ("E-H5-02", "src/planning/navigation_planning_backend/src/planner_core/planner.cpp", 4084, 4165, "Planner::plan governor caller", "Support construction and governor failure return before nominal endpoint solve."),
    ("E-H5-03", "src/planning/navigation_planning_backend/src/planner_core/planner.cpp", 5758, 5789, "Planner::PathSearch local support gate", "Partial-route support requires more than two resolution cells and other distance checks."),
    ("E-H5-04", "src/planning/navigation_planning_backend/test/test_planner_config.cpp", 1466, 1539, "PlannerSpeedGovernor tests", "Existing positive, support, measured-PVAJ, envelope and abort assertions."),
    ("E-H6-01", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2846, 2938, "NavigationModeExecutor hold state callbacks", "AUTO_LOITER observer, pending/in-flight/retry writes and Deactivated treatment."),
    ("E-H6-02", "src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/components/mode_executor.cpp", 155, 261, "px4_ros2::ModeExecutorBase::scheduleMode command dispatch", "Pinned dependency ACK request/dispatch behavior."),
    ("E-H6-03", "src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/components/mode_executor.cpp", 480, 520, "ModeCompleted callback path", "Callback waits for matching mode-completion event."),
    ("E-H6-04", "src/external/px4_ros2_interface_lib/px4_ros2_cpp/include/px4_ros2/components/mode_executor.hpp", 85, 108, "ModeExecutorBase scheduleMode API contract", "Pinned API callback description and result type."),
    ("E-TEST-H0", "tests/h0_tracking_loader_truth_table.cpp", 1, 67, "standalone tracking-loader truth table", "Audit fixture calls the production parameter loader with fake parameters."),
    ("E-TEST-H1", "tests/h1_mission_counterexample.cpp", 1, 100, "standalone MissionController harness", "Audit fixture links production MissionController and RouteProgress; continuation input is injected."),
    ("E-TEST-H5", "tests/h5_speed_grid_oracle.cpp", 1, 76, "standalone speed-grid sweep", "Audit fixture sweeps sampled cruise speeds through the production stopping certificate."),
]

def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    repo_root = Path(__file__).resolve().parents[4]
    default_artifact_a = repo_root.parent / "uav-navigation-as-is-audit-20260920/artifacts/architecture_as_is/20260920T-as-is-local"
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot-root", type=Path, default=default_artifact_a / "baseline/source_snapshot")
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "evidence/refs.jsonl")
    args = parser.parse_args()
    records = []
    for evidence_id, rel, start, end, symbol, note in SPECS:
        submodule = rel.startswith("src/external/px4_ros2_interface_lib/")
        local_test = rel.startswith("tests/")
        root = args.repo_root if submodule else (args.output.resolve().parents[1] if local_test else args.snapshot_root)
        path = root / rel
        if not path.is_file():
            raise SystemExit(f"{evidence_id}: missing source {path}")
        raw = path.read_bytes()
        lines = raw.decode("utf-8").splitlines()
        if start < 1 or end < start or end > len(lines):
            raise SystemExit(f"{evidence_id}: invalid source range {start}-{end} (file lines={len(lines)})")
        record = {
            "id": evidence_id,
            "path": rel,
            "symbol": symbol,
            "line_start": start,
            "line_end": end,
            "sha256": hashlib.sha256(raw).hexdigest(),
            "excerpt": "\n".join(f"{i}: {lines[i-1]}" for i in range(start, end + 1)),
            "note": note,
            "provenance": {
                "source": "pinned_dependency_worktree" if submodule else ("audit_generated_test" if local_test else "A/baseline/source_snapshot"),
                "git_revision": "4a3370f084ac6f1ef001a4afa2b007845ffd0837" if submodule else (None if local_test else "f2bd3f46f9936d622377ea4761f733f988273b66"),
            },
        }
        records.append(record)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in records))
    print(f"wrote {len(records)} evidence records: {args.output}")

if __name__ == "__main__":
    main()

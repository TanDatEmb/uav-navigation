#!/usr/bin/env python3
"""Guard the audited Core-to-PX4 command wire and observer separation.

This is a source guard, not a ROS graph proof or a substitute for component
and SITL tests. Historical artifacts are deliberately outside the scan.
"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
CONTRACTS = ROOT / "src/contracts/navigation_contracts"
ADAPTER = ROOT / "src/px4/px4_navigation_external_mode"
RUNTIME = ROOT / "src/runtime/navigation_runtime"
CONTROL_FIELDS = (
    "header", "localization_epoch", "goal_epoch", "mode_activation_id",
    "mission_id", "waypoint_index", "request_id", "world_generation",
    "world_revision", "world_observation_stamp", "bundle_generation",
    "certified_main_continuation", "continuation_boundary_stamp_ns", "sample_id",
    "state_source_stamp", "valid_until", "role", "status", "position",
    "velocity", "acceleration", "jerk", "yaw", "yaw_rate",
    "trajectory_time_s", "emergency_authorization_reason",
    "emergency_candidate_commit_result",
)
REQUIRED_SAFETY_CONSTANTS = (
    "ROLE_MAIN", "ROLE_BACKUP", "ROLE_EMERGENCY", "STATUS_READY",
    "STATUS_BRAKING", "STATUS_COMPLETED", "STATUS_REJECTED",
    "EMERGENCY_AUTHORIZATION_ACTUAL_ANCHOR_CERTIFICATE_EXCEEDED",
    "EMERGENCY_AUTHORIZATION_PROJECTED_MAIN_ONLY_CERTIFICATE_EXCEEDED",
)


def schema_fields(path: Path) -> tuple[set[str], set[str]]:
    fields: set[str] = set()
    constants: set[str] = set()
    for raw in path.read_text(encoding="utf-8").splitlines():
        parts = raw.split("#", 1)[0].split()
        if len(parts) != 2:
            continue
        if "=" in parts[1]:
            constants.add(parts[1].split("=", 1)[0])
        else:
            fields.add(parts[1])
    return fields, constants


def main() -> int:
    errors: list[str] = []
    fields, constants = schema_fields(CONTRACTS / "msg/NavigationCommand.msg")
    expected = set(CONTROL_FIELDS)
    if fields != expected:
        errors.append(f"NavigationCommand field allowlist mismatch: missing={sorted(expected-fields)} extra={sorted(fields-expected)}")
    missing_constants = set(REQUIRED_SAFETY_CONSTANTS) - constants
    if missing_constants:
        errors.append(f"missing safety constants: {sorted(missing_constants)}")
    diagnostic_fields, _ = schema_fields(
        CONTRACTS / "msg/NavigationExecutionDiagnostics.msg")
    if not {"sample_id", "bundle_generation", "execution_authorization",
            "causal_planning_cycle_id", "retained_tracking_limit_m"} <= diagnostic_fields:
        errors.append("observer diagnostic schema lost required provenance")
    adapter_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in ADAPTER.rglob("*")
        if path.suffix in {".hpp", ".cpp"} and "test" not in path.parts
    )
    if re.search(r"NavigationExecutionDiagnostics|execution_diagnostics", adapter_source):
        errors.append("PX4 adapter imports or subscribes to observer-only diagnostics")
    if "NavigationCommandRejection" not in adapter_source or "admissionReasonCode" not in adapter_source:
        errors.append("adapter has no typed rejection evidence")
    if "valid=0" in adapter_source:
        errors.append("ambiguous valid=0 rejection log reintroduced")
    runtime_source = (RUNTIME / "src/navigation_runtime_node.cpp").read_text(encoding="utf-8")
    if "execution_diagnostics_publisher_->publish(diagnostic)" not in runtime_source:
        errors.append("Core does not publish separate observer diagnostics")
    if "rememberMissionCommandIssued(command, true)" not in runtime_source:
        errors.append("Core-local mission issuance incorrectly depends on wire diagnostics")
    if errors:
        print("NAVIGATION_COMMAND_CONTRACT_STATIC_CHECK: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("NAVIGATION_COMMAND_CONTRACT_STATIC_CHECK: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

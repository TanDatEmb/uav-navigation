"""Exact offline join for observer-only execution diagnostics.

The join mutates retained *evidence records* only. It is never used by Core or
PX4 command admission. Missing or conflicting diagnostics stay explicit.
"""

from __future__ import annotations

from typing import Any


IDENTITY_FIELDS = (
    "mode_activation_id", "localization_epoch", "goal_epoch", "mission_id",
    "waypoint_index", "request_id", "bundle_generation", "sample_id",
)
CONTROL_PROTECTED_FIELDS = set(IDENTITY_FIELDS) | {
    "stamp_ns", "source_clock", "frame_id", "valid_until_ns",
    "state_source_stamp_ns", "world_generation", "world_revision",
    "world_observation_stamp_ns", "trajectory_id", "trajectory_generation",
    "trajectory_time_s", "trajectory_status", "trajectory_flag", "role",
    "status", "position", "velocity", "acceleration", "jerk", "yaw",
    "yaw_rate", "executable",
}


def join_execution_diagnostics(events: list[dict[str, Any]]) -> list[str]:
    issues: list[str] = []
    diagnostics_by_key: dict[tuple[Any, ...], dict[str, Any]] = {}
    conflicting: set[tuple[Any, ...]] = set()
    for event in events:
        if event.get("kind") != "execution_diagnostics":
            continue
        payload = event.get("payload")
        if not isinstance(payload, dict):
            continue
        key = tuple(payload.get(field) for field in IDENTITY_FIELDS)
        if any(value is None for value in key):
            issues.append("EXECUTION_DIAGNOSTIC_IDENTITY_MISSING")
            continue
        previous = diagnostics_by_key.setdefault(key, payload)
        if previous != payload:
            conflicting.add(key)
    for event in events:
        if event.get("kind") != "pva_command":
            continue
        payload = event.get("payload")
        if not isinstance(payload, dict):
            continue
        key = tuple(payload.get(field) for field in IDENTITY_FIELDS)
        if key in conflicting:
            payload["execution_diagnostic_missing"] = True
            issues.append("EXECUTION_DIAGNOSTIC_CONFLICT")
            continue
        diagnostic = diagnostics_by_key.get(key)
        if diagnostic is None:
            payload["execution_diagnostic_missing"] = True
            continue
        if diagnostic.get("header_stamp_ns") != payload.get("stamp_ns"):
            payload["execution_diagnostic_missing"] = True
            issues.append("EXECUTION_DIAGNOSTIC_STAMP_MISMATCH")
            continue
        for field, value in diagnostic.items():
            if field not in CONTROL_PROTECTED_FIELDS:
                payload[field] = value
        payload["execution_diagnostic_missing"] = False
    return issues

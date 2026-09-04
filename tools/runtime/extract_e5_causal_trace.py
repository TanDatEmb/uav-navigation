#!/usr/bin/env python3
"""Extract the five-boundary causal trace from the retained E5 artifact.

The extractor intentionally does not synthesize missing timestamps or booleans.
Missing runtime fields are emitted as ``NOT_RECORDED`` so the resulting trace
cannot turn a logging gap into causal evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path
from typing import Any


FIELDS = [
    "boundary",
    "timestamp_ns",
    "planning_cycle_id",
    "bundle_generation",
    "analytic_sample_role",
    "navigation_command_role",
    "trajectory_time_s",
    "backup_available",
    "backup_start_time_s",
    "time_to_backup_start_s",
    "anchor_error_m",
    "projected_anchor_error_m",
    "retained_tracking_limit_m",
    "relative_anchor_speed_mps",
    "sampled_path_clear",
    "current_vehicle_state_known_free",
    "safety_suffix_usable",
    "tracking_certificate_exceeded",
    "projected_tracking_certificate_exceeded",
    "planner_result",
    "planning_failure_stage_reason",
    "safety_suffix_active",
    "execution_recovery_state",
    "emergency_candidate_commit_result",
    "source_trace_timestamp_ns",
    "source_log_timestamp",
]


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            rows.append(item)
    return rows


def value(row: dict[str, Any], key: str, default: Any = "NOT_RECORDED") -> Any:
    item = row.get(key, default)
    return default if item is None else item


def command_role(value_: Any) -> str:
    return {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY"}.get(int(value_), "NOT_RECORDED")


def analytic_role(value_: Any) -> str:
    return {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY", 255: "UNKNOWN"}.get(
        int(value_), "NOT_RECORDED"
    )


def state_name(value_: Any) -> str:
    return {
        0: "INITIAL_HOLD",
        1: "TRACK_MAIN",
        2: "TRACK_BACKUP",
        3: "EMERGENCY_BRAKE",
        4: "STOPPED_RECOVERY",
        5: "PX4_HOLD",
    }.get(int(value_), "NOT_RECORDED")


def planner_trace(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        if row.get("stream") != "mapping_diagnostics":
            continue
        timestamp = row.get("timestamp_ns", "NOT_RECORDED")
        for status in row.get("payload", {}).get("statuses", []):
            if status.get("name") != "navigation_runtime/planner":
                continue
            if status.get("message") != "DECISION_TRACE":
                continue
            values = status.get("values", {})
            if isinstance(values, dict):
                item = dict(values)
                item["source_trace_timestamp_ns"] = timestamp
                result.append(item)
    return result


def commands(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        if row.get("kind") != "pva_command":
            continue
        payload = row.get("payload", {})
        if not isinstance(payload, dict):
            continue
        item = dict(payload)
        item["timestamp_ns"] = row.get("sim_time_ns", "NOT_RECORDED")
        result.append(item)
    return result


def extract(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    sample_rows = read_jsonl(root / "samples.jsonl")
    scenario_rows = read_jsonl(root / "scenario.jsonl")
    traces = planner_trace(sample_rows)
    cmds = commands(scenario_rows)
    injected = next(
        (row for row in traces if str(row.get("injected_replan_failure", "0")) == "1"),
        None,
    )
    if injected is None:
        raise RuntimeError("E5 injected DECISION_TRACE was not found")
    trace_ns = injected.get("source_trace_timestamp_ns")
    try:
        trace_ns_int = int(trace_ns)
    except (TypeError, ValueError):
        trace_ns_int = None
    before = [row for row in cmds if trace_ns_int is not None and int(row["timestamp_ns"]) <= trace_ns_int]
    after = [row for row in cmds if trace_ns_int is not None and int(row["timestamp_ns"]) > trace_ns_int]
    t0 = max(before, key=lambda row: int(row["timestamp_ns"])) if before else None
    t4 = next(
        (row for row in sorted(after, key=lambda row: int(row["timestamp_ns"]))
         if int(row.get("trajectory_flag", -1)) == 2 or int(row.get("analytic_sample_role", -1)) == 2),
        None,
    )

    log_text = ""
    for path in sorted((root / "logs").glob("*")):
        if path.is_file() and path.suffix == ".log":
            log_text += path.read_text(encoding="utf-8", errors="replace") + "\n"
    log_match = re.search(
        r"\[([0-9]+\.[0-9]+)\].*?planner backend replaced the exceeded-anchor command",
        log_text,
    )
    log_timestamp = log_match.group(1) if log_match else "NOT_RECORDED"

    def command_row(boundary: str, row: dict[str, Any] | None) -> dict[str, Any]:
        if row is None:
            return {field: "NOT_RECORDED" for field in FIELDS} | {"boundary": boundary}
        return {
            "boundary": boundary,
            "timestamp_ns": value(row, "timestamp_ns"),
            "planning_cycle_id": "NOT_RECORDED",
            "bundle_generation": value(row, "trajectory_generation"),
            "analytic_sample_role": analytic_role(value(row, "analytic_sample_role")),
            "navigation_command_role": command_role(value(row, "trajectory_flag")),
            "trajectory_time_s": value(row, "trajectory_time_s"),
            "backup_available": value(row, "backup_available"),
            "backup_start_time_s": value(row, "backup_start_time_s"),
            "time_to_backup_start_s": value(row, "time_to_backup_start_s"),
            "anchor_error_m": value(row, "anchor_error_m"),
            "projected_anchor_error_m": value(row, "projected_anchor_error_m"),
            "retained_tracking_limit_m": value(row, "retained_tracking_limit_m"),
            "relative_anchor_speed_mps": value(row, "relative_anchor_speed_mps"),
            "sampled_path_clear": value(row, "sampled_path_clear"),
            "current_vehicle_state_known_free": "NOT_RECORDED",
            "safety_suffix_usable": value(row, "committed_suffix_usable"),
            "tracking_certificate_exceeded": value(row, "tracking_certificate_exceeded"),
            "projected_tracking_certificate_exceeded": value(row, "projected_tracking_certificate_exceeded"),
            "planner_result": "NOT_RECORDED",
            "planning_failure_stage_reason": "NOT_RECORDED",
            "safety_suffix_active": value(row, "safety_suffix_active"),
            "execution_recovery_state": state_name(value(row, "execution_recovery_state")),
            "emergency_candidate_commit_result": "NOT_RECORDED",
            "source_trace_timestamp_ns": "NOT_RECORDED",
            "source_log_timestamp": "NOT_RECORDED",
        }

    t1 = {field: "NOT_RECORDED" for field in FIELDS} | {
        "boundary": "T1_injected_solve_start",
        "planning_cycle_id": value(injected, "planning_cycle_id"),
        "bundle_generation": value(injected, "active_execution_bundle_generation"),
        "source_trace_timestamp_ns": value(injected, "source_trace_timestamp_ns"),
    }
    t2 = {field: "NOT_RECORDED" for field in FIELDS} | {
        "boundary": "T2_injected_solve_failure_return",
        "planning_cycle_id": value(injected, "planning_cycle_id"),
        "bundle_generation": value(injected, "active_execution_bundle_generation"),
        "planner_result": value(injected, "candidate_result"),
        "planning_failure_stage_reason": f"stage={value(injected, 'planning_failure_stage')};reason={value(injected, 'planning_failure_reason')}",
        "source_trace_timestamp_ns": value(injected, "source_trace_timestamp_ns"),
        "source_log_timestamp": log_timestamp,
    }
    t3 = {field: "NOT_RECORDED" for field in FIELDS} | {
        "boundary": "T3_recovery_decision",
        "planning_cycle_id": value(injected, "planning_cycle_id"),
        "bundle_generation": value(injected, "active_execution_bundle_generation"),
        "planner_result": value(injected, "candidate_result"),
        "planning_failure_stage_reason": f"stage={value(injected, 'planning_failure_stage')};reason={value(injected, 'planning_failure_reason')}",
        "source_trace_timestamp_ns": value(injected, "source_trace_timestamp_ns"),
        "source_log_timestamp": log_timestamp,
    }
    rows = [command_row("T0_final_normal_MAIN_sample", t0), t1, t2, t3,
            command_row("T4_first_emergency_sample", t4)]
    measured = {
        "injected_trace": injected,
        "t0": t0,
        "t4": t4,
        "emergency_log_timestamp": log_timestamp,
        "c1_anchor_error_exceeded": "NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT",
        "c2_projected_anchor_error_exceeded": "NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT",
        "c3_backup_available": value(t0, "backup_available") if t0 else "NOT_RECORDED",
        "c4_suffix_usable": "NOT_RECORDED",
        "c5_emergency_authorization_predicate": "NOT_RECORDED",
    }
    return rows, measured


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--trace-output", type=Path, required=True)
    parser.add_argument("--analysis-output", type=Path, required=True)
    args = parser.parse_args()
    rows, measured = extract(args.input)
    args.trace_output.parent.mkdir(parents=True, exist_ok=True)
    with args.trace_output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    injected = measured["injected_trace"]
    t0 = measured["t0"]
    t4 = measured["t4"]
    lines = [
        "# E5 causal analysis",
        "",
        "This analysis uses only the retained pre-telemetry E5 artifact. Missing fields are explicitly marked `NOT_RECORDED`; no causal value is inferred from waypoint geometry or from a later command sample.",
        "",
        "## Measured boundaries",
        "",
        f"- T0: sample_id={t0.get('sample_id') if t0 else 'NOT_RECORDED'}, timestamp_ns={t0.get('timestamp_ns') if t0 else 'NOT_RECORDED'}, generation={t0.get('trajectory_generation') if t0 else 'NOT_RECORDED'}, role={command_role(t0.get('trajectory_flag')) if t0 else 'NOT_RECORDED'}, analytic_role={analytic_role(t0.get('analytic_sample_role')) if t0 else 'NOT_RECORDED'}, time_to_backup={t0.get('time_to_backup_start_s') if t0 else 'NOT_RECORDED'} s.",
        f"- T1: injected solve start timestamp is `NOT_RECORDED`; cycle={injected.get('planning_cycle_id')}, solve_generation={injected.get('solve_generation')}.",
        f"- T2: injected failure is recorded as candidate_result={injected.get('candidate_result')}, failure_stage={injected.get('planning_failure_stage')}, failure_reason={injected.get('planning_failure_reason')}; exact return timestamp_ns is `NOT_RECORDED`.",
        f"- T3: emergency decision log timestamp={measured['emergency_log_timestamp']}; exact ROS timestamp_ns and predicate result are `NOT_RECORDED`.",
        f"- T4: sample_id={t4.get('sample_id') if t4 else 'NOT_RECORDED'}, timestamp_ns={t4.get('timestamp_ns') if t4 else 'NOT_RECORDED'}, generation={t4.get('trajectory_generation') if t4 else 'NOT_RECORDED'}, role={command_role(t4.get('trajectory_flag')) if t4 else 'NOT_RECORDED'}, analytic_role={analytic_role(t4.get('analytic_sample_role')) if t4 else 'NOT_RECORDED'}.",
        "",
        "## C1–C5",
        "",
        "- C1: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The old command stream has no anchor_error_m or retained_tracking_limit_m.",
        "- C2: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The old command stream has no projected_anchor_error_m.",
        f"- C3: measured T0 `backup_available={measured['c3_backup_available']}`; the retained MAIN command reports backup_start_time_s={t0.get('backup_start_time_s') if t0 else 'NOT_RECORDED'} and time_to_backup_start_s={t0.get('time_to_backup_start_s') if t0 else 'NOT_RECORDED'}.",
        "- C4: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. `committedSafetySuffixIsUsable` and `sampled_path_clear` were not serialized. The log proves that the exceeded-anchor emergency path was entered, but cannot distinguish the individual suffix-usability inputs.",
        "- C5: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The exact authorization predicate was not serialized. The source code contains the predicate, but this artifact alone is insufficient to claim its runtime branch without the new telemetry.",
        "",
        "## Required next evidence",
        "",
        "Add the causal fields to DECISION_TRACE and NavigationCommand, then rerun the controlled failure with the retained MAIN margin preconditions. H4 root cause remains unresolved until that run records the authorization reason.",
        "",
    ]
    args.analysis_output.parent.mkdir(parents=True, exist_ok=True)
    args.analysis_output.write_text("\n".join(lines), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

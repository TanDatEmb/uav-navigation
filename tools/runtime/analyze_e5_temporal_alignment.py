#!/usr/bin/env python3
"""Analyze retained-command timestamp alignment from a runtime artifact.

The script consumes direct immutable-polynomial telemetry when available. It
never reconstructs a polynomial from command speed or neighboring samples. A
legacy artifact therefore produces explicit NOT_RECORDED cells instead of
silently becoming temporal evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


FIELDS = [
    "timestamp_ns", "evaluation_now_ns", "execution_state_source_stamp_ns",
    "execution_state_receive_stamp_ns", "source_age_ms", "receive_age_ms",
    "bundle_start_timestamp_ns", "bundle_generation", "measured_position_at_source",
    "measured_velocity_at_source", "command_position_at_now", "command_velocity_at_now",
    "command_position_at_source", "command_velocity_at_source", "raw_anchor_error_m",
    "time_aligned_anchor_error_m", "command_motion_during_state_age_m",
    "velocity_residual_time_aligned_mps", "backup_available", "time_to_backup_start_s",
    "retained_tracking_limit_m", "sampled_path_clear", "committed_suffix_usable",
    "tracking_certificate_exceeded", "projected_tracking_certificate_exceeded",
    "planner_result", "planning_failure_stage_reason", "emergency_authorization_reason",
    "emergency_candidate_commit_result", "analytic_sample_role", "navigation_command_role",
    "execution_recovery_state", "source_record",
]


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    result = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            result.append(value)
    return result


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def stamp(value: Any) -> int | None:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None


def vector(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    result = [finite(item) for item in value]
    return result if all(item is not None for item in result) else None


def norm_delta(lhs: list[float] | None, rhs: list[float] | None) -> float | None:
    if lhs is None or rhs is None:
        return None
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))


def role(value: Any, analytic: bool = False) -> str:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return "NOT_RECORDED"
    return {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY", 255: "UNKNOWN"}.get(
        number, "NOT_RECORDED")


def state(value: Any) -> str:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return "NOT_RECORDED"
    return {0: "INITIAL_HOLD", 1: "TRACK_MAIN", 2: "TRACK_BACKUP",
            3: "EMERGENCY_BRAKE", 4: "STOPPED_RECOVERY", 5: "PX4_HOLD"}.get(
        number, "NOT_RECORDED")


def traces(sample_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in sample_rows:
        if record.get("stream") != "mapping_diagnostics":
            continue
        for status in record.get("payload", {}).get("statuses", []):
            if status.get("name") != "navigation_runtime/planner" or \
                    status.get("message") != "DECISION_TRACE":
                continue
            values = status.get("values", {})
            if isinstance(values, dict):
                item = dict(values)
                item["timestamp_ns"] = record.get("timestamp_ns")
                result.append(item)
    return result


def commands(scenario_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in scenario_rows:
        if record.get("kind") != "pva_command":
            continue
        payload = record.get("payload", {})
        if isinstance(payload, dict):
            item = dict(payload)
            item["timestamp_ns"] = record.get("sim_time_ns")
            result.append(item)
    return result


def direct_row(command: dict[str, Any]) -> dict[str, Any]:
    measured_p = vector(command.get("measured_position_at_state_source"))
    measured_v = vector(command.get("measured_velocity_at_state_source"))
    command_now_p = vector(command.get("committed_command_position_at_now"))
    command_now_v = vector(command.get("committed_command_velocity_at_now"))
    command_source_p = vector(command.get("committed_command_position_at_state_source"))
    command_source_v = vector(command.get("committed_command_velocity_at_state_source"))
    raw = norm_delta(command_now_p, measured_p)
    aligned = norm_delta(command_source_p, measured_p)
    motion = norm_delta(command_now_p, command_source_p)
    velocity_residual = norm_delta(command_source_v, measured_v)
    def val(key: str, default: Any = "NOT_RECORDED") -> Any:
        value = command.get(key, default)
        return default if value is None else value
    return {
        "timestamp_ns": val("timestamp_ns"),
        "evaluation_now_ns": val("evaluation_now_ns"),
        "execution_state_source_stamp_ns": val("execution_state_source_stamp_ns"),
        "execution_state_receive_stamp_ns": val("execution_state_receive_stamp_ns"),
        "source_age_ms": val("execution_state_source_age_ms"),
        "receive_age_ms": val("execution_state_receive_age_ms"),
        "bundle_start_timestamp_ns": val("committed_bundle_start_stamp_ns"),
        "bundle_generation": val("trajectory_generation"),
        "measured_position_at_source": measured_p or "NOT_RECORDED",
        "measured_velocity_at_source": measured_v or "NOT_RECORDED",
        "command_position_at_now": command_now_p or "NOT_RECORDED",
        "command_velocity_at_now": command_now_v or "NOT_RECORDED",
        "command_position_at_source": command_source_p or "NOT_RECORDED",
        "command_velocity_at_source": command_source_v or "NOT_RECORDED",
        "raw_anchor_error_m": raw if raw is not None else val("anchor_error_raw_m"),
        "time_aligned_anchor_error_m": aligned,
        "command_motion_during_state_age_m": motion,
        "velocity_residual_time_aligned_mps": velocity_residual,
        "backup_available": val("backup_available"),
        "time_to_backup_start_s": val("time_to_backup_start_s"),
        "retained_tracking_limit_m": val("retained_tracking_limit_m"),
        "sampled_path_clear": val("sampled_path_clear"),
        "committed_suffix_usable": val("committed_suffix_usable"),
        "tracking_certificate_exceeded": val("tracking_certificate_exceeded"),
        "projected_tracking_certificate_exceeded": val("projected_tracking_certificate_exceeded"),
        "planner_result": val("planner_result"),
        "planning_failure_stage_reason": val("planning_failure_stage_reason"),
        "emergency_authorization_reason": val("emergency_authorization_reason"),
        "emergency_candidate_commit_result": val("emergency_candidate_commit_result"),
        "analytic_sample_role": role(val("analytic_sample_role"), True),
        "navigation_command_role": role(val("trajectory_flag")),
        "execution_recovery_state": state(val("execution_recovery_state")),
        "source_record": "scenario.pva_command",
    }


def attach_trace_context(rows: list[dict[str, Any]], trace_rows: list[dict[str, Any]]) -> None:
    """Annotate nearby failure context without replacing direct command data."""
    if not trace_rows or not rows:
        return
    failures = [row for row in trace_rows
                if str(row.get("injected_replan_failure", "0")) == "1" or
                (finite(row.get("candidate_result")) is not None and
                 int(float(row["candidate_result"])) != 0 and
                 finite(row.get("emergency_authorization_reason")) is not None)]
    if not failures:
        return
    failure = min(failures, key=lambda item: abs(
        (stamp(item.get("timestamp_ns")) or 0) -
        (stamp(rows[0].get("timestamp_ns")) or 0)))
    for row in rows:
        for key, target in (("candidate_result", "planner_result"),
                            ("planning_failure_stage", "planning_failure_stage_reason"),
                            ("emergency_authorization_reason", "emergency_authorization_reason"),
                            ("emergency_candidate_commit_result", "emergency_candidate_commit_result")):
            if row.get(target) == "NOT_RECORDED" and key in failure:
                row[target] = failure[key]


def analyze(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]:
    sample_rows = read_jsonl(root / "samples.jsonl")
    scenario_rows = read_jsonl(root / "scenario.jsonl")
    trace_rows = traces(sample_rows)
    command_rows = commands(scenario_rows)
    # One command record is emitted for every transport sample, while the
    # causal values remain constant until the next planner validation. Keep
    # the first record for each direct evaluation timestamp.
    rows = []
    seen: set[int] = set()
    for command in command_rows:
        if "evaluation_now_ns" not in command:
            continue
        evaluation = stamp(command.get("evaluation_now_ns"))
        if evaluation is None or evaluation in seen:
            continue
        seen.add(evaluation)
        rows.append(direct_row(command))
    attach_trace_context(rows, trace_rows)
    direct_count = len(rows)
    if not rows:
        # Preserve a machine-readable proof that the old artifact cannot
        # answer H7. Do not fall back to speed * age or neighboring samples.
        rows = [{field: "NOT_RECORDED" for field in FIELDS}]
        rows[0]["source_record"] = "NO_DIRECT_TEMPORAL_TELEMETRY"
    usable = [row for row in rows if finite(row.get("raw_anchor_error_m")) is not None and
              finite(row.get("time_aligned_anchor_error_m")) is not None and
              finite(row.get("retained_tracking_limit_m")) is not None]
    return rows, {
        "direct_validation_sample_count": direct_count,
        "usable_alignment_sample_count": len(usable),
        "legacy_artifact_temporal_fields_missing": direct_count == 0,
        "usable_rows": usable,
    }, trace_rows


def hypothetical_predicates(row: dict[str, Any], planning_period_s: float = 0.2) -> dict[str, Any]:
    limit = finite(row.get("retained_tracking_limit_m"))
    raw = finite(row.get("raw_anchor_error_m"))
    aligned = finite(row.get("time_aligned_anchor_error_m"))
    clear = row.get("sampled_path_clear") is True or str(row.get("sampled_path_clear")) == "1"
    backup = row.get("backup_available") is True or str(row.get("backup_available")) == "1"
    current_raw_usable = row.get("committed_suffix_usable") is True or \
        str(row.get("committed_suffix_usable")) == "1"
    raw_exceeded = raw is not None and limit is not None and raw > limit
    aligned_exceeded = aligned is not None and limit is not None and aligned > limit
    # Exact runtime admission also depends on freshness, command availability,
    # role, and validity inputs. Those are not present in the legacy artifact;
    # retain the measured raw decision and expose the aligned branch as a
    # conditional replay of the same non-error inputs.
    aligned_suffix_usable_conditional = (
        aligned is not None and limit is not None and aligned <= limit and clear and
        ((backup and finite(row.get("time_to_backup_start_s")) is not None) or
         (not backup and current_raw_usable)))
    actual_raw_authorized = row.get("emergency_authorization_reason") not in (
        "NOT_RECORDED", None, 0, "0") and raw_exceeded
    actual_aligned_authorized = aligned_exceeded and not aligned_suffix_usable_conditional
    return {
        "raw_retained_suffix_usable": row.get("committed_suffix_usable"),
        "aligned_retained_suffix_usable_conditional": aligned_suffix_usable_conditional,
        "raw_tracking_certificate_exceeded": raw_exceeded,
        "aligned_tracking_certificate_exceeded": aligned_exceeded,
        "raw_emergency_authorized": actual_raw_authorized,
        "aligned_emergency_authorized_actual_anchor_branch": actual_aligned_authorized,
        "predicate_replay_exact": False,
    }


def write_plots(root: Path, rows: list[dict[str, Any]], traces_: list[dict[str, Any]]) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    usable = [row for row in rows if stamp(row.get("timestamp_ns")) and
              finite(row.get("raw_anchor_error_m")) is not None]
    if not usable:
        return []
    output_dir = root / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    times = [(stamp(row["timestamp_ns"]) or 0) * 1.0e-9 for row in usable]
    raw = [finite(row["raw_anchor_error_m"]) for row in usable]
    aligned = [finite(row["time_aligned_anchor_error_m"]) for row in usable]
    limit = [finite(row["retained_tracking_limit_m"]) for row in usable]
    motion = [finite(row["command_motion_during_state_age_m"]) for row in usable]
    age = [finite(row["source_age_ms"]) for row in usable]
    figure = output_dir / "plot_E5_temporal_alignment.png"
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(times, raw, label="raw anchor error", color="tab:red")
    axes[0].plot(times, aligned, label="time-aligned anchor error", color="tab:blue")
    axes[0].plot(times, limit, label="retained tracking limit", color="black", linestyle="--")
    axes[0].plot(times, motion, label="command motion over state age", color="tab:orange")
    axes[0].set_ylabel("distance [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=.25)
    axes[1].plot(times, age, label="source age", color="tab:purple")
    axes[1].set_ylabel("age [ms]")
    axes[1].set_xlabel("simulation time [s]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=.25)
    failure_times = [stamp(row.get("timestamp_ns")) for row in traces_
                     if str(row.get("injected_replan_failure", "0")) == "1"]
    emergency_times = [stamp(row.get("timestamp_ns")) for row in traces_
                       if finite(row.get("emergency_authorization_reason")) not in (None, 0)]
    for event_time, label, color in ((failure_times, "planner failure", "tab:red"),
                                     (emergency_times, "emergency authorization", "tab:green")):
        for event in event_time:
            if event:
                x = event * 1.0e-9
                for axis in axes:
                    axis.axvline(x, color=color, linestyle=":", alpha=.7, label=label)
    # De-duplicate legend entries created by event markers.
    for axis in axes:
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        axis.legend(unique.values(), unique.keys(), loc="best")
    fig.tight_layout(); fig.savefig(figure, dpi=140); plt.close(fig)

    scatter = output_dir / "plot_E5_temporal_alignment_scatter.png"
    ages = [finite(row.get("source_age_ms")) for row in usable]
    differences = [finite(row.get("raw_anchor_error_m")) -
                   finite(row.get("time_aligned_anchor_error_m")) for row in usable]
    fig, axis = plt.subplots(figsize=(8, 5))
    axis.scatter(ages, differences, s=12, alpha=.7)
    axis.axhline(0.0, color="black", linewidth=.8)
    axis.set_xlabel("state source age [ms]")
    axis.set_ylabel("raw anchor error - aligned anchor error [m]")
    axis.set_title("E5 — temporal alignment residual")
    axis.grid(True, alpha=.25); fig.tight_layout(); fig.savefig(scatter, dpi=140); plt.close(fig)
    return [str(figure), str(scatter)]


def write_outputs(root: Path, csv_path: Path, md_path: Path) -> None:
    rows, summary, trace_rows = analyze(root)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader(); writer.writerows(rows)
    plots = write_plots(root, rows, trace_rows)
    usable = summary["usable_rows"]
    predicate = hypothetical_predicates(usable[-1]) if usable else {}
    lines = [
        "# E5 temporal alignment analysis", "",
        "This report uses exact immutable-bundle samples when direct telemetry is present. It does not estimate command position from speed, age, or neighboring command samples.", "",
        f"- Direct retained-validation samples: `{summary['direct_validation_sample_count']}`",
        f"- Usable alignment samples: `{summary['usable_alignment_sample_count']}`",
        f"- Legacy temporal fields missing: `{summary['legacy_artifact_temporal_fields_missing']}`",
        f"- Figures: `{', '.join(plots) if plots else 'NOT_GENERATED'}`", "",
    ]
    if not usable:
        lines += [
            "## Result", "",
            "`NOT_TESTED`: the input artifact has no direct evaluation/state timestamp pair and no exact immutable-polynomial samples at both timestamps. Required fields are emitted as `NOT_RECORDED`; no H7 conclusion is claimed.", "",
            "The legacy `anchor_error_m` field, when present, is not substituted for `raw_anchor_error_m` because it does not prove the command sample and measured state share a timestamp.", "",
        ]
    else:
        emergency = max(usable, key=lambda row: finite(row.get("raw_anchor_error_m")) or -1.0)
        difference = (finite(emergency.get("raw_anchor_error_m")) or 0.0) - (finite(emergency.get("time_aligned_anchor_error_m")) or 0.0)
        lines += [
            "## Measured decomposition", "",
            f"- Boundary sample timestamp: `{emergency.get('timestamp_ns')}`",
            f"- raw anchor error: `{emergency.get('raw_anchor_error_m')}` m",
            f"- time-aligned anchor error: `{emergency.get('time_aligned_anchor_error_m')}` m",
            f"- command motion during state age: `{emergency.get('command_motion_during_state_age_m')}` m",
            f"- raw minus aligned: `{difference}` m",
            f"- source age: `{emergency.get('source_age_ms')}` ms; receive age: `{emergency.get('receive_age_ms')}` ms",
            "",
            "## Offline predicate replay", "",
            "The runtime predicate remains unchanged. The aligned branch below is conditional only where legacy artifacts omit freshness/lease/role inputs; it is not presented as an exact runtime authorization decision unless all inputs are recorded.", "",
            f"```json\n{json.dumps(predicate, indent=2, sort_keys=True)}\n```", "",
        ]
    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()
    write_outputs(args.input, args.output_csv, args.output_md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

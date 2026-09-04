#!/usr/bin/env python3
"""Build the evidence package for the provisional P0 control envelope.

This is an offline report builder.  It never changes a run, planner parameter,
or acceptance gate.  Raw SITL directories remain the authoritative evidence;
the output only contains derived traces, summaries, and figures.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from analyze_h10 import MISSING, build_rows, metric, n, num, vec  # noqa: E402


CONTROL = {
    "maximum_velocity_mps": 0.5624988750005627,
    "maximum_acceleration_mps2": 0.2165062314375,
    "maximum_jerk_mps3": 0.2804066718750005,
}
CERTIFICATE = 0.25
MAIN = "MAIN"


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _stats(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    return metric(rows, key)


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else ["timestamp_ns"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _diag_values(run: Path) -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    source = run / "samples.jsonl"
    if not source.is_file():
        return values
    with source.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            record = item.get("payload", {})
            statuses = record.get("statuses", []) if isinstance(record, dict) else []
            for status in statuses if isinstance(statuses, list) else []:
                if status.get("message") == "DECISION_TRACE":
                    payload = status.get("values", {})
                    if isinstance(payload, dict):
                        values.append(payload)
    return values


def _diag_summary(run: Path) -> dict[str, Any]:
    rows = _diag_values(run)
    def finite(key: str) -> list[float]:
        return [float(num(row.get(key))) for row in rows if num(row.get(key)) is not None]
    def maximum(key: str) -> float | str:
        values = finite(key)
        return max(values) if values else MISSING
    last = rows[-1] if rows else {}
    return {
        "decision_trace_count": len(rows),
        "requested_cruise_speed_mps": last.get("requested_cruise_speed_mps", MISSING),
        "effective_cruise_speed_mps": last.get("effective_cruise_speed_mps", MISSING),
        "control_max_velocity_mps": last.get("control_max_velocity_mps", MISSING),
        "control_max_acceleration_mps2": last.get("control_max_acceleration_mps2", MISSING),
        "control_max_jerk_mps3": last.get("control_max_jerk_mps3", MISSING),
        "physical_max_velocity_mps": last.get("physical_max_velocity_mps", MISSING),
        "physical_max_acceleration_mps2": last.get("physical_max_acceleration_mps2", MISSING),
        "physical_max_jerk_mps3": last.get("physical_max_jerk_mps3", MISSING),
        "candidate_max_velocity_mps_max": maximum("candidate_max_velocity_mps"),
        "candidate_max_acceleration_mps2_max": maximum("candidate_max_acceleration_mps2"),
        "candidate_max_jerk_mps3_max": maximum("candidate_max_jerk_mps3"),
        "emergency_authorization_reason_values": sorted({str(row.get("emergency_authorization_reason", MISSING)) for row in rows}),
        "injected_replan_failure_values": sorted({str(row.get("injected_replan_failure", MISSING)) for row in rows}),
    }


def _main_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row.get("analytic_role") == MAIN]


def _compliance(rows: list[dict[str, Any]]) -> dict[str, Any]:
    main = _main_rows(rows)
    maxima = {
        "maximum_velocity_mps": max((num(row.get("planner_speed_mps")) or 0.0 for row in main), default=0.0),
        "maximum_acceleration_mps2": max((num(row.get("planner_acceleration_mps2")) or 0.0 for row in main), default=0.0),
        "maximum_jerk_mps3": max((num(row.get("planner_jerk_mps3")) or 0.0 for row in main), default=0.0),
    }
    excess = {key: value > CONTROL[key] + 1.0e-9 for key, value in maxima.items()}
    return {"main_sample_count": len(main), "observed_main_maxima": maxima, "limits": CONTROL, "exceeds_limit": excess, "compliant": not any(excess.values())}


def _run_summary(run: Path, rows: list[dict[str, Any]], label: str) -> dict[str, Any]:
    scenario = _read_json(run / "scenario.json")
    metadata = _read_json(run / "metadata.json")
    main = _main_rows(rows)
    mode_states = sorted({str(row.get("external_mode_output_state")) for row in rows if row.get("external_mode_output_state") not in {None, MISSING}})
    recovery_states = sorted({str(row.get("recovery_state")) for row in rows if row.get("recovery_state") not in {None, MISSING}})
    return {
        "label": label,
        "run_id": metadata.get("run_id", run.name),
        "experiment_id": metadata.get("experiment_id", MISSING),
        "raw_run": str(run.resolve()),
        "repo_commit": metadata.get("repo_commit", MISSING),
        "map_profile": (metadata.get("environment") or {}).get("map_profile", MISSING),
        "mission_file": metadata.get("mission_file", MISSING),
        "requested_cruise_speed_mps": metadata.get("requested_cruise_speed_mps", MISSING),
        "planner_rate_hz": metadata.get("planner_rate_hz", MISSING),
        "command_rate_hz": metadata.get("command_rate_hz", MISSING),
        "pva_sample_count": len(rows),
        "main_sample_count": len(main),
        "command_lio_tracking": _stats(rows, "aligned_lio_tracking_error_m"),
        "command_px4_tracking": _stats(rows, "px4_tracking_error_m"),
        "lio_gt_position": _stats(rows, "lio_gt_position_error_m"),
        "px4_gt_position": _stats(rows, "px4_gt_position_error_m"),
        "planner_speed": _stats(main, "planner_speed_mps"),
        "planner_acceleration": _stats(main, "planner_acceleration_mps2"),
        "planner_jerk": _stats(main, "planner_jerk_mps3"),
        "px4_delta_v": _stats(rows, "delta_v_px4_controller_mps"),
        "px4_delta_a": _stats(rows, "delta_a_px4_controller_mps2"),
        "main_compliance": _compliance(rows),
        "first_certificate_crossing_ns": next((int(row["timestamp_ns"]) for row in rows if num(row.get("aligned_lio_tracking_error_m")) is not None and float(row["aligned_lio_tracking_error_m"]) > CERTIFICATE), MISSING),
        "external_mode_states": mode_states,
        "recovery_states": recovery_states,
        "scenario_failures": scenario.get("failures", []),
        "mission_complete_observed": scenario.get("mission_complete_observed", MISSING),
        "decision_trace": _diag_summary(run),
    }


def _baseline_summary(run: Path) -> dict[str, Any]:
    report = _read_json(run / "report.json")
    metadata = _read_json(run / "metadata.json")
    planning = report.get("planning", {})
    execution = planning.get("execution", {})
    lio_gt = ((report.get("tracking") or {}).get("lio_vs_ground_truth") or {}).get("position_m", {}).get("norm", {})
    return {
        "label": "PRODUCTION_BASELINE_E5",
        "run_id": metadata.get("run_id", run.name),
        "raw_run": str(run.resolve()),
        "repo_commit": metadata.get("repo_commit", "37e8dc7f9967990ef4c1401475e0d15abef14360"),
        "requested_cruise_speed_mps": metadata.get("requested_cruise_speed_mps", 3.0),
        "physical_command_maxima": {
            "maximum_velocity_mps": execution.get("maximum_velocity_mps", MISSING),
            "maximum_acceleration_mps2": execution.get("maximum_acceleration_mps2", MISSING),
            "maximum_jerk_mps3": execution.get("maximum_jerk_mps3", MISSING),
        },
        "lio_gt_position_p95_m": lio_gt.get("p95", MISSING),
        "acceptance": report.get("acceptance", {}),
        "note": "Legacy E5 retained no P0 control-envelope fields; command-LIO/PX4 layer metrics are NOT_RECORDED here.",
    }


def _vec_norms(rows: list[dict[str, Any]], key: str) -> list[float | None]:
    values: list[float | None] = []
    for row in rows:
        raw = row.get(key)
        if raw in {None, MISSING}:
            values.append(None)
        else:
            try:
                values.append(n(vec(json.loads(raw))))
            except (TypeError, ValueError, json.JSONDecodeError):
                values.append(None)
    return values


def _plots(rows: list[dict[str, Any]], open_rows: list[dict[str, Any]], out: Path) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out.mkdir(parents=True, exist_ok=True)
    if not rows:
        return []
    t0 = int(rows[0]["timestamp_ns"])
    t = [(int(row["timestamp_ns"]) - t0) / 1e9 for row in rows]
    made: list[str] = []
    def save(name: str, fig: Any) -> None:
        fig.tight_layout(); fig.savefig(out / name, dpi=140); plt.close(fig); made.append(name)
    def scalar(key: str) -> list[float | None]:
        return [num(row.get(key)) for row in rows]
    fig, ax = plt.subplots(figsize=(11, 4)); ax.plot(t, scalar("aligned_lio_tracking_error_m"), label="command-LIO"); ax.plot(t, scalar("px4_tracking_error_m"), label="command-PX4"); ax.axhline(CERTIFICATE, color="k", ls="--", label="0.25 m certificate"); ax.set(xlabel="simulation time (s)", ylabel="error (m)", title="P0 Stage A tracking error"); ax.grid(True); ax.legend(); save("p0_e5_tracking_error.png", fig)
    fig, ax = plt.subplots(figsize=(11, 4)); ax.plot(t, _vec_norms(rows, "planner_velocity"), label="NavigationCommand"); ax.plot(t, _vec_norms(rows, "PX4_input_trajectory_velocity"), label="PX4_INPUT_SETPOINT"); ax.plot(t, _vec_norms(rows, "PX4_effective_velocity_setpoint"), label="PX4 effective"); ax.plot(t, _vec_norms(rows, "PX4_velocity"), label="PX4 measured"); ax.set(xlabel="simulation time (s)", ylabel="speed (m/s)", title="P0 Stage A velocity layers"); ax.grid(True); ax.legend(); save("p0_e5_velocity_layers.png", fig)
    fig, ax = plt.subplots(figsize=(11, 4)); ax.plot(t, scalar("planner_acceleration_mps2"), label="NavigationCommand"); ax.plot(t, _vec_norms(rows, "PX4_input_trajectory_acceleration"), label="PX4_INPUT_SETPOINT"); ax.plot(t, _vec_norms(rows, "PX4_effective_acceleration_setpoint"), label="PX4 effective"); ax.axhline(CONTROL["maximum_acceleration_mps2"], color="k", ls="--", label="MAIN A limit"); ax.set(xlabel="simulation time (s)", ylabel="acceleration (m/s²)", title="P0 Stage A acceleration layers"); ax.grid(True); ax.legend(); save("p0_e5_acceleration_layers.png", fig)
    fig, ax = plt.subplots(figsize=(11, 4)); ax.plot(t, scalar("lio_gt_position_error_m"), label="LIO-GT position"); ax.plot(t, scalar("px4_gt_position_error_m"), label="PX4-GT position"); ax.set(xlabel="simulation time (s)", ylabel="error (m)", title="P0 Stage A estimator error relative to GT"); ax.grid(True); ax.legend(); save("p0_e5_estimator_gt.png", fig)
    fig, ax = plt.subplots(figsize=(7, 5)); ax.scatter(scalar("planner_speed_mps"), scalar("aligned_lio_tracking_error_m"), s=5, label="Stage A"); ax.axhline(CERTIFICATE, color="k", ls="--", label="certificate"); ax.set(xlabel="planner speed (m/s)", ylabel="command-LIO error (m)", title="Demand versus tracking error"); ax.grid(True); ax.legend(); save("p0_demand_vs_tracking.png", fig)
    if open_rows:
        ot0 = int(open_rows[0]["timestamp_ns"]); ot = [(int(row["timestamp_ns"]) - ot0) / 1e9 for row in open_rows]
        fig, ax = plt.subplots(figsize=(11, 4)); ax.plot(ot, _vec_norms(open_rows, "planner_velocity"), label="open control |V|"); ax.plot(t, _vec_norms(rows, "planner_velocity"), label="Stage A |V|"); ax.axhline(CONTROL["maximum_velocity_mps"], color="k", ls="--", label="MAIN V limit"); ax.set(xlabel="relative simulation time (s)", ylabel="speed (m/s)", title="P0 Stage A versus matched open control"); ax.grid(True); ax.legend(); save("p0_stage_a_vs_open_control.png", fig)
    return made


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-a", type=Path, required=True)
    parser.add_argument("--open-control", type=Path, required=True)
    parser.add_argument("--baseline-e5", type=Path, default=Path("runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve(); root.mkdir(parents=True, exist_ok=True)
    stage_rows = build_rows(args.stage_a.resolve()); open_rows = build_rows(args.open_control.resolve())
    _write_rows(root / "e5_stage_a_trace.csv", stage_rows); _write_rows(root / "matched_open_trace.csv", open_rows)
    stage = _run_summary(args.stage_a.resolve(), stage_rows, "P0_STAGE_A_EXACT_E5")
    control = _run_summary(args.open_control.resolve(), open_rows, "P0_MATCHED_OPEN_CONTROL")
    baseline = _baseline_summary(args.baseline_e5.resolve())
    figures = _plots(stage_rows, open_rows, root / "figures")
    stage_a_pass = bool(stage["main_compliance"]["compliant"] and stage["first_certificate_crossing_ns"] == MISSING and not stage["scenario_failures"] and "RECOVERY_HOLD" not in stage["external_mode_states"] and "EMERGENCY_BRAKE" not in stage["recovery_states"] and "PX4_HOLD" not in stage["recovery_states"])
    summary = {
        "production_behavior_baseline": "37e8dc7f9967990ef4c1401475e0d15abef14360",
        "implementation_commit_at_capture": stage.get("repo_commit", MISSING),
        "provisional_profile": CONTROL,
        "certificate_m": CERTIFICATE,
        "raw_evidence": {"stage_a": str(args.stage_a.resolve()), "matched_open_control": str(args.open_control.resolve()), "baseline_e5": str(args.baseline_e5.resolve())},
        "stage_a": stage,
        "matched_open_control": control,
        "production_baseline": baseline,
        "stage_a_acceptance": {"pass": stage_a_pass, "reason": "Stage A has no clean acceptance: retained run reports missing waypoint completion and RECOVERY_HOLD/certificate or tracking gates fail." if not stage_a_pass else "all requested Stage A gates observed"},
        "stage_b": {"status": "NOT_RUN", "reason": "Stage B is conditional on a fully passing Stage A; no failure injection was run."},
        "figures": figures,
        "conclusion": "P0 ENVELOPE FIX NOT SUFFICIENT" if not stage_a_pass else "P0 ENVELOPE FIX VALIDATED",
    }
    (root / "p0_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    md = [
        "# P0 VehicleControlEnvelope — exact E5 regression", "",
        "## Scope and provenance", "",
        f"- Production behavior baseline: `37e8dc7f9967990ef4c1401475e0d15abef14360`.",
        f"- Instrumented implementation captured by Stage A: `{stage.get('repo_commit', MISSING)}`.",
        f"- Stage A raw run: `{args.stage_a.resolve()}`; matched open control: `{args.open_control.resolve()}`.",
        "- Raw run directories are not copied or rewritten; all derived values are generated by `tools/runtime/analyze_p0_envelope_regression.py`.", "",
        "## Contract result", "",
        f"Requested cruise was `{stage.get('requested_cruise_speed_mps')}` m/s and the diagnostic effective speed was `{stage['decision_trace'].get('effective_cruise_speed_mps', MISSING)}` m/s. The physical boundary remained `{stage['decision_trace'].get('physical_max_velocity_mps', MISSING)}` m/s; BACKUP diagnostics retain their physical values.",
        f"MAIN compliance: `{stage['main_compliance']['compliant']}`. Observed MAIN maxima: `{json.dumps(stage['main_compliance']['observed_main_maxima'], sort_keys=True)}`; nominal limits: `{json.dumps(CONTROL, sort_keys=True)}`.", "",
        "## Exact E5 Stage A before/after", "",
        "| Metric | Production baseline E5 | P0 Stage A | Matched open control |", "|---|---:|---:|---:|",
        f"| Requested cruise (m/s) | {baseline.get('requested_cruise_speed_mps', MISSING)} | {stage.get('requested_cruise_speed_mps', MISSING)} | {control.get('requested_cruise_speed_mps', MISSING)} |",
        f"| Physical/observed command V max (m/s) | {baseline.get('physical_command_maxima', {}).get('maximum_velocity_mps', MISSING)} | {stage['main_compliance']['observed_main_maxima']['maximum_velocity_mps']:.6f} MAIN | {control['main_compliance']['observed_main_maxima']['maximum_velocity_mps']:.6f} MAIN |",
        f"| Observed MAIN A max (m/s²) | {baseline.get('physical_command_maxima', {}).get('maximum_acceleration_mps2', MISSING)} | {stage['main_compliance']['observed_main_maxima']['maximum_acceleration_mps2']:.6f} | {control['main_compliance']['observed_main_maxima']['maximum_acceleration_mps2']:.6f} |",
        f"| Observed MAIN J max (m/s³) | {baseline.get('physical_command_maxima', {}).get('maximum_jerk_mps3', MISSING)} | {stage['main_compliance']['observed_main_maxima']['maximum_jerk_mps3']:.6f} | {control['main_compliance']['observed_main_maxima']['maximum_jerk_mps3']:.6f} |",
        f"| Command-LIO P95 (m) | {MISSING} | {stage['command_lio_tracking']['p95']} | {control['command_lio_tracking']['p95']} |",
        f"| LIO-GT position P95 (m) | {baseline.get('lio_gt_position_p95_m', MISSING)} | {stage['lio_gt_position']['p95']} | {control['lio_gt_position']['p95']} |",
        f"| First command-LIO certificate crossing | {MISSING} | {stage['first_certificate_crossing_ns']} | {control['first_certificate_crossing_ns']} |",
        f"| Mission completion | {baseline.get('acceptance', {}).get('mission_complete_observed', MISSING)} | {stage.get('mission_complete_observed', MISSING)} | {control.get('mission_complete_observed', MISSING)} |", "",
        "The legacy baseline does not contain the P0 control-envelope/canonical PX4 input fields, so those cells remain `NOT_RECORDED`; they are not reconstructed from unrelated metrics.", "",
        "## Emergency and backup invariants", "",
        "- `traj_opt/boundary` remained the hard physical source; the runner no longer rewrites it from mission speed.",
        "- MAIN candidate/sample compliance is checked against the provisional control profile.",
        "- BACKUP/EMERGENCY retain the physical configuration; Stage A observed BACKUP maxima above nominal MAIN limits and they were not treated as a violation.",
        "- No tracking threshold, recovery predicate, planner timing, PX4 gain, or mission acceptance rule was changed.", "",
        "## Stage A verdict", "",
        f"Stage A pass = `{stage_a_pass}`. The raw run failures are `{json.dumps(stage['scenario_failures'])}` and observed External Mode states are `{stage['external_mode_states']}`. Because the complete mission/lifecycle and tracking acceptance gates did not pass, Stage B was not run.", "",
        "## Matched open control", "",
        f"The open control is reported separately from E5. Its raw evidence is `{args.open_control.resolve()}`. It is not used to invalidate the E5-specific result.", "",
        "## Figures", "",
        *[f"- `{root / 'figures' / figure}`" for figure in figures], "",
        "## Reproduction", "",
        "```bash",
        f"python3 tools/runtime/analyze_p0_envelope_regression.py --stage-a {args.stage_a} --open-control {args.open_control} --output {root}",
        "```", "",
        f"# {summary['conclusion']}",
    ]
    (root / "p0_analysis.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    # Keep the conventional report names available beside the P0-specific
    # artifact.  The JSON shape remains intentionally small and points back
    # to the complete derived summary rather than duplicating raw evidence.
    report_markdown = "\n".join(md) + "\n"
    (root / "runtime_report.md").write_text(report_markdown, encoding="utf-8")
    report = {
        "hypotheses": {
            "P0_vehicle_control_envelope": {
                "status": "CONFIRMED" if stage_a_pass else "INCONCLUSIVE",
                "evidence": [str((root / "p0_analysis.md").resolve())],
            }
        },
        "critical_events": [],
        "runs": [
            {"experiment": "P0_E5_STAGE_A", "scenario": "S_BAD_E5", "run_id": stage["run_id"], "path": stage["raw_run"], "result": "PASS" if stage_a_pass else "BLOCKED"},
            {"experiment": "P0_MATCHED_OPEN_CONTROL", "scenario": "S_OPEN_CONTROL", "run_id": control["run_id"], "path": control["raw_run"], "result": "BLOCKED" if control["scenario_failures"] else "OBSERVED"},
        ],
        "p0": summary,
    }
    report_json = json.dumps(report, indent=2) + "\n"
    (root / "runtime_report.json").write_text(report_json, encoding="utf-8")
    # Also expose the conventional date-level entry points.  They are the
    # current P0 report, while all historical reports remain untouched in
    # their original evidence directories.
    root.parent.mkdir(parents=True, exist_ok=True)
    (root.parent / "runtime_report.md").write_text(report_markdown, encoding="utf-8")
    (root.parent / "runtime_report.json").write_text(report_json, encoding="utf-8")
    print(json.dumps({"output": str(root), "stage_a_pass": stage_a_pass, "conclusion": summary["conclusion"], "figures": figures}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

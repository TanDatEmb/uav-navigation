#!/usr/bin/env python3
"""Finalize the last H10 evidence package without changing runtime behavior."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from analyze_h10 import (  # noqa: E402
    LIMIT,
    MISSING,
    build_rows,
    metric,
    n,
    num,
    vec,
    write_csv,
)


def write_rows(path: Path, rows: list[dict]) -> None:
    write_csv(path, rows)


def tag_rows(rows: list[dict], segment: str) -> list[dict]:
    return [dict(row, segment_id=segment) for row in rows]


def stats(rows: list[dict], key: str) -> dict:
    return metric(rows, key)


def load_meta(run: Path) -> dict:
    try:
        return json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def finite_values(rows: list[dict], key: str) -> list[float]:
    return [float(row[key]) for row in rows if num(row.get(key)) is not None]


def segment_summary(rows: list[dict], segment_id: str, run: Path, valid: bool) -> dict:
    return {
        "segment_id": segment_id,
        "run_id": load_meta(run).get("run_id", run.name),
        "run_path": str(run),
        "valid_for_envelope": bool(valid),
        "sample_count": len(rows),
        "planner_velocity_mps": stats(rows, "planner_speed_mps"),
        "planner_acceleration_mps2": stats(rows, "planner_acceleration_mps2"),
        "planner_jerk_mps3": stats(rows, "planner_jerk_mps3"),
        "planner_lateral_acceleration_mps2": stats(rows, "planner_lateral_acceleration_mps2"),
        "gt_tracking_error_m": stats(rows, "gt_tracking_error_m"),
        "lio_gt_position_error_m": stats(rows, "lio_gt_position_error_m"),
        "px4_gt_position_error_m": stats(rows, "px4_gt_position_error_m"),
        "px4_delta_v_mps": stats(rows, "delta_v_px4_controller_mps"),
        "px4_delta_a_mps2": stats(rows, "delta_a_px4_controller_mps2"),
        "recovery_states": sorted({row.get("recovery_state", MISSING) for row in rows}),
    }


def add_gt_tracking(rows: list[dict]) -> None:
    for row in rows:
        p = vec(json.loads(row["planner_position"])) if row.get("planner_position") != MISSING else None
        g = vec(json.loads(row["LIO_position"])) if row.get("LIO_position") != MISSING else None
        gt = vec(json.loads(row["PX4_position"])) if row.get("PX4_position") != MISSING else None
        # The existing H10 trace contains planner-LIO and planner-PX4.  GT
        # command error is recomputed from the planner and independently
        # sampled ground truth when the source columns are present.
        if row.get("ground_truth_position") not in (None, MISSING):
            q = vec(json.loads(row["ground_truth_position"]))
            if p is not None and q is not None:
                row["gt_tracking_error_m"] = n([x - y for x, y in zip(p, q)])
        elif row.get("aligned_lio_tracking_error_m") not in (None, MISSING):
            row["gt_tracking_error_m"] = MISSING


def split_lat(rows: list[dict]) -> list[dict]:
    active = [row for row in rows if num(row.get("planner_lateral_acceleration_mps2")) is not None]
    if not active:
        return [dict(row, segment_id="LAT_NOT_RECORDED") for row in rows]
    vals = sorted(float(row["planner_lateral_acceleration_mps2"]) for row in active)
    cut = vals[len(vals) // 2]
    out = []
    for row in rows:
        value = num(row.get("planner_lateral_acceleration_mps2"))
        label = "LAT_ACTIVE" if value is not None and value >= cut else "LAT_LOW"
        out.append(dict(row, segment_id=label))
    return out


def plot_final(exact: list[dict], long_rows: list[dict], lat_rows: list[dict], out: Path, events: dict[str, int | None]) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out.mkdir(parents=True, exist_ok=True)
    made: list[str] = []

    def values(rows: list[dict], key: str) -> list[float | None]:
        return [num(row.get(key)) for row in rows]

    def vec_norm(rows: list[dict], key: str) -> list[float | None]:
        result = []
        for row in rows:
            raw = row.get(key)
            result.append(n(vec(json.loads(raw))) if raw not in (None, MISSING) else None)
        return result

    def time(rows: list[dict]) -> list[float]:
        if not rows:
            return []
        t0 = int(rows[0]["timestamp_ns"])
        return [(int(row["timestamp_ns"]) - t0) / 1e9 for row in rows]

    def save(name: str, fig) -> None:
        fig.tight_layout()
        fig.savefig(out / name, dpi=150)
        plt.close(fig)
        made.append(name)

    t = time(exact)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, values(exact, "aligned_lio_tracking_error_m"), label="command-LIO")
    ax.plot(t, values(exact, "px4_tracking_error_m"), label="command-PX4")
    ax.axhline(LIMIT, color="k", linestyle="--", label="certificate 0.25 m")
    for label, stamp in events.items():
        if stamp is not None and exact:
            ax.axvline((stamp - int(exact[0]["timestamp_ns"])) / 1e9, linestyle=":", label=label)
    ax.set(xlabel="simulation time (s)", ylabel="tracking error (m)", title="Exact E5 tracking error and event ordering")
    ax.grid(True); ax.legend(ncol=3, fontsize=8)
    save("fig1_e5_tracking_error_events.png", fig)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, vec_norm(exact, "planner_velocity"), label="NavigationCommand")
    ax.plot(t, vec_norm(exact, "PX4_input_trajectory_velocity"), label="PX4_INPUT_SETPOINT")
    ax.plot(t, vec_norm(exact, "PX4_effective_velocity_setpoint"), label="PX4 effective")
    ax.plot(t, vec_norm(exact, "PX4_velocity"), label="PX4 measured")
    ax.set(xlabel="simulation time (s)", ylabel="speed (m/s)", title="E5 velocity layers"); ax.grid(True); ax.legend()
    save("fig2_e5_velocity_layers.png", fig)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, values(exact, "planner_acceleration_mps2"), label="NavigationCommand")
    ax.plot(t, vec_norm(exact, "PX4_input_trajectory_acceleration"), label="PX4_INPUT_SETPOINT")
    ax.plot(t, vec_norm(exact, "PX4_effective_acceleration_setpoint"), label="PX4 effective")
    ax.set(xlabel="simulation time (s)", ylabel="acceleration (m/s²)", title="E5 acceleration layers"); ax.grid(True); ax.legend()
    save("fig3_e5_acceleration_layers.png", fig)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, values(exact, "lio_gt_position_error_m"), label="LIO-GT position")
    ax.plot(t, values(exact, "px4_gt_position_error_m"), label="PX4-GT position")
    ax.set(xlabel="simulation time (s)", ylabel="error (m)", title="E5 estimator error relative to ground truth"); ax.grid(True); ax.legend()
    save("fig4_e5_estimator_gt_error.png", fig)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(values(long_rows, "planner_speed_mps"), values(long_rows, "aligned_lio_tracking_error_m"), s=5, label="DYN-LONG")
    ax.set(xlabel="planner speed (m/s)", ylabel="command-LIO error (m)", title="DYN-LONG demand vs tracking error"); ax.grid(True); ax.legend()
    save("fig5_dyn_long_demand_tracking.png", fig)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(values(lat_rows, "planner_lateral_acceleration_mps2"), values(lat_rows, "aligned_lio_tracking_error_m"), s=5, label="DYN-LAT pre-recovery")
    ax.set(xlabel="commanded lateral acceleration (m/s²)", ylabel="command-LIO error (m)", title="DYN-LAT lateral demand vs tracking error"); ax.grid(True); ax.legend()
    save("fig6_dyn_lat_lateral_tracking.png", fig)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(values(exact, "planner_speed_mps"), values(exact, "aligned_lio_tracking_error_m"), s=5, label="S_BAD_E5")
    ax.axhline(0.10, color="k", linestyle="--", label="P95 target 0.10 m")
    ax.axhline(0.175, color="r", linestyle=":", label="MAX target 0.175 m")
    ax.set(xlabel="planner speed (m/s)", ylabel="command-LIO error (m)", title="E5 demand vs identified nominal envelope"); ax.grid(True); ax.legend()
    save("fig7_e5_demand_vs_nominal_envelope.png", fig)
    return made


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--date-root", type=Path, required=True)
    parser.add_argument("--exact", type=Path, required=True)
    parser.add_argument("--open", type=Path, required=True)
    parser.add_argument("--dyn-long", nargs="+", type=Path, required=True)
    parser.add_argument("--dyn-lat", type=Path, required=True)
    args = parser.parse_args()
    root = args.date_root
    final_dir = root / "H10_FINAL"
    e5_dir = final_dir / "S_BAD_E5"
    long_dir = final_dir / "S_DYNAMIC_ID" / "DYN-LONG"
    lat_dir = final_dir / "S_DYNAMIC_ID" / "DYN-LAT"
    for path in (e5_dir, long_dir, lat_dir):
        path.mkdir(parents=True, exist_ok=True)

    exact = build_rows(args.exact)
    control = build_rows(args.open)
    for rows in (exact, control):
        for row in rows:
            row.setdefault("gt_tracking_error_m", MISSING)
        add_gt_tracking(rows)
    exact_path = e5_dir / "h10_exact_e5_final.csv"
    write_rows(exact_path, exact)
    write_rows(root / "h10_exact_e5_final.csv", exact)
    write_rows(root / "h10_open_control_final.csv", control)

    long_all: list[dict] = []
    long_segments = []
    for index, run in enumerate(args.dyn_long, 1):
        rows = build_rows(run)
        for row in rows:
            row.setdefault("gt_tracking_error_m", MISSING)
        add_gt_tracking(rows)
        tagged = tag_rows(rows, f"LONG_RUN_{index}")
        long_all.extend(tagged)
        long_segments.append(segment_summary(tagged, f"LONG_RUN_{index}", run, False))
    write_rows(long_dir / "dyn_long.csv", long_all)
    write_rows(root / "h10_dynamic_long.csv", long_all)

    lat_base = build_rows(args.dyn_lat)
    for row in lat_base:
        row.setdefault("gt_tracking_error_m", MISSING)
    add_gt_tracking(lat_base)
    lat_all = split_lat(lat_base)
    pre_recovery = [row for row in lat_all if row.get("recovery_state") not in {"EMERGENCY_BRAKE", "PX4_HOLD", "STOPPED_RECOVERY"}]
    write_rows(lat_dir / "dyn_lat.csv", lat_all)
    write_rows(root / "h10_dynamic_lat.csv", lat_all)
    lat_segments = []
    for segment_id in sorted({row["segment_id"] for row in lat_all}):
        segment_rows = [row for row in pre_recovery if row["segment_id"] == segment_id]
        lat_segments.append(segment_summary(segment_rows, segment_id, args.dyn_lat, False))

    injection_count = sum(1 for line in (args.exact / "samples.jsonl").read_text(encoding="utf-8", errors="replace").splitlines() if '"injected_replan_failure": 1' in line)
    events = {}
    for row in __import__("csv").DictReader((root / "h10_exact_e5_events.csv").open(encoding="utf-8")):
        try: events[row["event"]] = int(row["timestamp_ns"])
        except (KeyError, TypeError, ValueError): events[row.get("event", "unknown")] = None
    figures = plot_final(exact, long_all, pre_recovery, final_dir / "figures", events)

    envelope = {
        "status": "NOT_IDENTIFIED",
        "criterion": {
            "gt_tracking_p95_m_max": 0.10,
            "gt_tracking_max_m_max": 0.175,
            "no_emergency_or_recovery": True,
            "estimator_health_valid": True,
        },
        "v_nominal_max_mps": "NOT_IDENTIFIED",
        "a_long_nominal_max_mps2": "NOT_IDENTIFIED",
        "decel_nominal_max_mps2": "NOT_IDENTIFIED",
        "jerk_nominal_max_mps3": "NOT_IDENTIFIED",
        "a_lateral_nominal_max_mps2": "NOT_IDENTIFIED",
        "evidence": {
            "DYN-LONG": long_segments,
            "DYN-LAT": lat_segments,
        },
        "reason": "No controlled increasing-demand matrix with a clean no-recovery pass satisfied the stated criterion. DYN-LAT entered RECOVERY_HOLD before mission completion.",
    }
    (root / "h10_final_dynamic_envelope.json").write_text(json.dumps(envelope, indent=2) + "\n", encoding="utf-8")
    (final_dir / "h10_final_dynamic_envelope.json").write_text(json.dumps(envelope, indent=2) + "\n", encoding="utf-8")

    def m(rows, key): return stats(rows, key)
    exact_metrics = {
        "tracking_error": m(exact, "aligned_lio_tracking_error_m"),
        "gt_tracking_error": m(exact, "gt_tracking_error_m"),
        "lio_gt_position": m(exact, "lio_gt_position_error_m"),
        "px4_gt_position": m(exact, "px4_gt_position_error_m"),
        "planner_speed": m(exact, "planner_speed_mps"),
        "planner_acceleration": m(exact, "planner_acceleration_mps2"),
        "planner_jerk": m(exact, "planner_jerk_mps3"),
        "planner_lateral_acceleration": m(exact, "planner_lateral_acceleration_mps2"),
        "delta_v": m(exact, "delta_v_px4_controller_mps"),
        "delta_a": m(exact, "delta_a_px4_controller_mps2"),
    }
    control_metrics = {
        "tracking_error": m(control, "aligned_lio_tracking_error_m"),
        "gt_tracking_error": m(control, "gt_tracking_error_m"),
        "lio_gt_position": m(control, "lio_gt_position_error_m"),
        "px4_gt_position": m(control, "px4_gt_position_error_m"),
        "planner_acceleration": m(control, "planner_acceleration_mps2"),
        "planner_jerk": m(control, "planner_jerk_mps3"),
        "delta_v": m(control, "delta_v_px4_controller_mps"),
        "delta_a": m(control, "delta_a_px4_controller_mps2"),
    }
    status = {
        "H10a_planner_demand_exceeds_usable_closed_loop_envelope": "INCONCLUSIVE",
        "H10b_px4_controller_materially_reshapes_command": "INCONCLUSIVE",
        "H10c_lio_estimator_material_contributor": "INCONCLUSIVE",
        "H10d_px4_estimator_material_contributor": "INCONCLUSIVE",
        "H10e_vehicle_cannot_follow_px4_effective_setpoint": "INCONCLUSIVE",
    }
    final = {
        "scenario_scope": {
            "S_BAD_E5": {"run": str(args.exact), "map": "sanity_open", "route": "external_mode_open_route", "speed_mps": 3.0},
            "S_OPEN_CONTROL": {"run": str(args.open), "map": "sanity_open", "route": "external_mode_open_route", "speed_mps": 3.0},
            "S_DYNAMIC_ID": {"DYN-LONG": [str(x) for x in args.dyn_long], "DYN-LAT": str(args.dyn_lat)},
        },
        "exact_e5": {"injected_failure_count": injection_count, "valid_exact_reproduction": injection_count == 1, "metrics": exact_metrics},
        "open_control": control_metrics,
        "dynamic_long_segments": long_segments,
        "dynamic_lat_segments": lat_segments,
        "dynamic_envelope": envelope,
        "h10": status,
        "figures": [str(final_dir / "figures" / name) for name in figures],
        "decision": {"first_fix": "PLANNER_CLOSED_LOOP_ENVELOPE", "confidence": "PROVISIONAL"},
    }
    (root / "h10_final_summary.json").write_text(json.dumps(final, indent=2) + "\n", encoding="utf-8")

    def fmt(x):
        return MISSING if x is None else f"{x:.6f}"
    md = [
        "# H10-Final — Closed-loop attribution and usable dynamic envelope", "",
        "## Scenario scope", "",
        f"- `S_BAD_E5`: `{args.exact}`; sanity_open / external_mode_open_route / 3.0 m/s. Raw bag retained; exact final injection validity is `{injection_count == 1}` because the immutable marker count is `{injection_count}`.",
        f"- `S_OPEN_CONTROL`: `{args.open}`; matched open control at 3.0 m/s. Statistics are separate and never pooled.",
        f"- `S_DYNAMIC_ID`: DYN-LONG `{[str(x) for x in args.dyn_long]}` and DYN-LAT `{args.dyn_lat}`. Segment IDs are analysis labels; no production mission behavior was changed.", "",
        "## Evidence validity", "",
        "The exact E5 final SITL artifact contains all canonical layers and independent `/sim/ground_truth/odometry`, but the requested cycle-5 hook produced zero immutable `injected_replan_failure=1` events. It is therefore not a valid exact injected-failure reproduction. DYN-LAT is layer-valid only before the observed RECOVERY_HOLD and is not a clean envelope pass.", "",
        "## Exact E5 ordering", "",
        "| Event | timestamp_ns | evidence |", "|---|---:|---|",
    ]
    for name in ["bundle_activation", "sustained_error_growth_start", "tracking_error_0.10_m", "tracking_error_0.20_m", "tracking_error_0.25_m", "planner_failure_return", "emergency_authorization", "emergency_activation"]:
        stamp = events.get(name)
        row = min(exact, key=lambda r: abs(int(r["timestamp_ns"]) - stamp)) if stamp is not None and exact else None
        evidence = f"e_lio={row.get('aligned_lio_tracking_error_m', MISSING)}, planner_a={row.get('planner_acceleration_mps2', MISSING)}, recovery={row.get('recovery_state', MISSING)}" if row else MISSING
        md.append(f"| {name} | {stamp if stamp is not None else MISSING} | {evidence} |")
    md += [
        "", "The observed ordering is tracking degradation/growth before the natural planner failure and emergency. Because the injected marker is absent, this run cannot support a causal claim that the injected planner failure caused the loss.", "",
        "## Exact E5 vs matched open control", "",
        "| Metric | S_BAD_E5 | S_OPEN_CONTROL |", "|---|---:|---:|",
        f"| command-LIO tracking P95 | {fmt(exact_metrics['tracking_error']['p95'])} | {fmt(control_metrics['tracking_error']['p95'])} |",
        f"| command-GT tracking P95 | {fmt(exact_metrics['gt_tracking_error']['p95'])} | {fmt(control_metrics['gt_tracking_error']['p95'])} |",
        f"| LIO-GT position P95 | {fmt(exact_metrics['lio_gt_position']['p95'])} | {fmt(control_metrics['lio_gt_position']['p95'])} |",
        f"| PX4-GT position P95 | {fmt(exact_metrics['px4_gt_position']['p95'])} | {fmt(control_metrics['px4_gt_position']['p95'])} |",
        f"| planner acceleration P95 | {fmt(exact_metrics['planner_acceleration']['p95'])} | {fmt(control_metrics['planner_acceleration']['p95'])} |",
        f"| planner jerk P95 | {fmt(exact_metrics['planner_jerk']['p95'])} | {fmt(control_metrics['planner_jerk']['p95'])} |",
        f"| PX4 controller delta-V P95 | {fmt(exact_metrics['delta_v']['p95'])} | {fmt(control_metrics['delta_v']['p95'])} |",
        f"| PX4 controller delta-A P95 | {fmt(exact_metrics['delta_a']['p95'])} | {fmt(control_metrics['delta_a']['p95'])} |", "",
        "## DYN-LONG / DYN-LAT", "",
        "The available low-demand runs do not constitute the requested deterministic increasing-demand matrix. DYN-LAT entered `RECOVERY_HOLD` at the second lateral waypoint. Per-segment metrics are in `h10_dynamic_long.csv` and `h10_dynamic_lat.csv`; no nominal envelope value is claimed.", "",
        "## Usable closed-loop envelope", "",
        "`h10_final_dynamic_envelope.json` is `NOT_IDENTIFIED`. The required P95 <= 0.10 m, MAX <= 0.175 m, no-recovery, estimator-health criterion was not demonstrated by a clean controlled matrix. E5 demand/envelope ratios are therefore `NOT_IDENTIFIED`.", "",
        "## H10 scenario-scoped classification", "",
        "| Hypothesis | S_BAD_E5 | DYN-LONG | DYN-LAT |", "|---|---|---|---|",
        "| H10a planner demand exceeds usable closed-loop envelope | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |",
        "| H10b PX4 materially reshapes planner command | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |",
        "| H10c LIO materially contributes | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |",
        "| H10d PX4 estimator materially contributes | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |",
        "| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |", "",
        "No H10 hypothesis is promoted to CONFIRMED by an invalid injection or a recovery-contaminated dynamic-ID run. This is an evidence limitation, not evidence that the previously established H8b result is false.", "",
        "## Figures", "",
    ]
    md += [f"- `{final_dir / 'figures' / name}`" for name in figures]
    md += ["", "# FIRST FIX RECOMMENDATION", "", "- Selected first fix: `PLANNER_CLOSED_LOOP_ENVELOPE` (provisional; implementation deferred).", "- Measured basis: the prior exact-E5 family already established dynamic tracking insufficiency, while this final run confirms the requested observability layers but not a valid injected failure or clean envelope boundary.", "- Competing fixes are deferred because PX4 reshaping, LIO contribution, PX4 estimator contribution, and plant-following attribution remain INCONCLUSIVE in the scenario-scoped final package.", "- Preserve unchanged: 0.25 m retained-command certificate, fail-closed recovery, estimator/world freshness and validity gates, backup/emergency safety checks, planner timing, and mission acceptance rules.", "- Exact E5 regression: rerun the same map/route/speed and require exactly one `injected_replan_failure=1`, then verify demand/tracking/error ordering and that the selected fix removes unwanted loss without weakening the certificate.", "- Matched open-control regression: same vehicle, PX4 parameters, planner/command rates, map, route, and speed; compare tracking and controller-correction distributions separately; no easier control run may invalidate an E5 finding."]
    (root / "h10_final_analysis.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    (final_dir / "h10_final_analysis.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    report_path = root / "runtime_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {"hypotheses": {}, "critical_events": [], "runs": []}
    report.setdefault("hypotheses", {}).update({key: {"status": value, "evidence": [str(root / "h10_final_analysis.md")]} for key, value in status.items()})
    report["h10_final"] = final
    report["runs"] = [row for row in report.get("runs", []) if not str(row.get("experiment", "")).startswith("H10_FINAL_")]
    report.setdefault("runs", []).append({"experiment": "H10_FINAL_E5", "scenario": "S_BAD_E5", "run_id": args.exact.name, "path": str(args.exact), "result": "INCONCLUSIVE", "injected_failure_count": injection_count})
    report.setdefault("runs", []).append({"experiment": "H10_FINAL_DYN_LONG", "scenario": "S_DYNAMIC_ID", "run_id": "multiple", "path": str(long_dir), "result": "INCONCLUSIVE", "segments": long_segments})
    report.setdefault("runs", []).append({"experiment": "H10_FINAL_DYN_LAT", "scenario": "S_DYNAMIC_ID", "run_id": args.dyn_lat.name, "path": str(lat_dir), "result": "BLOCKED_RECOVERY_HOLD", "segments": lat_segments})
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    report_md = root / "runtime_report.md"
    previous = report_md.read_text(encoding="utf-8") if report_md.is_file() else ""
    marker = "\n## 15. H10-Final — Closed-loop attribution and usable dynamic envelope"
    if marker in previous:
        previous = previous.split(marker, 1)[0].rstrip() + "\n"
    report_md.write_text(previous + "\n## 15. H10-Final — Closed-loop attribution and usable dynamic envelope\n\n" + (root / "h10_final_analysis.md").read_text(encoding="utf-8"), encoding="utf-8")
    print(json.dumps({"exact_injection_count": injection_count, "dynamic_envelope": envelope["status"], "figures": figures}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

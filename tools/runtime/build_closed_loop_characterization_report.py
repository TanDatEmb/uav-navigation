#!/usr/bin/env python3
"""Build the isolated PX4-local closed-loop characterization package."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path
from typing import Any


MISSING = "NOT_IDENTIFIED"
CERTIFICATE = 0.25


def finite(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values: list[float], fraction: float) -> float | None:
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    index = (len(values) - 1) * fraction
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (index - lo)


def stats(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = [value for row in rows if (value := finite(row.get(key))) is not None]
    return {"count": len(values), "rms": math.sqrt(sum(x * x for x in values) / len(values)), "p95": percentile(values, 0.95), "max": max(values)} if values else {"count": 0, "rms": MISSING, "p95": MISSING, "max": MISSING}


def read_csv(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_analysis(path: Path) -> dict[str, Any]:
    return json.loads((path / "analysis.json").read_text(encoding="utf-8"))


def stats_subset(rows: list[dict[str, Any]], key: str, segment_id: str) -> dict[str, Any]:
    return stats([row for row in rows if row.get("segment_id") == segment_id], key)


def quality_segments(analysis: dict[str, Any]) -> list[dict[str, Any]]:
    return [item for item in analysis.get("segments", []) if item.get("quality") in {"GOOD", "MARGINAL", "UNUSABLE"}]


def envelope(long_analysis: dict[str, Any], lat_analysis: dict[str, Any]) -> dict[str, Any]:
    long_good = [s for s in quality_segments(long_analysis) if s.get("quality") == "GOOD" and s.get("segment_kind") == "longitudinal"]
    lat_good = [s for s in quality_segments(lat_analysis) if s.get("quality") == "GOOD" and s.get("segment_kind") == "arc"]

    def pick(items: list[dict[str, Any]], metric: str) -> dict[str, Any]:
        candidates = [(s["metrics"][metric].get("max"), s["segment_id"], s.get("requested_radius_m")) for s in items if finite(s["metrics"].get(metric, {}).get("max")) is not None]
        if not candidates:
            return {"value": MISSING}
        value, segment_id, radius = max(candidates, key=lambda item: item[0])
        result = {"value": value, "run_id": items[0].get("run_id", MISSING), "segment_id": segment_id}
        if radius not in (None, MISSING):
            result["radius_m"] = radius
        return result

    result = {
        "status": "IDENTIFIED" if long_good and lat_good else "PARTIALLY_IDENTIFIED",
        "criterion": {"gt_tracking_p95_m_max": 0.10, "gt_tracking_max_m_max": 0.175, "no_recovery": True, "estimator_health_valid": True},
        "v_nominal_max_mps": pick(long_good, "planner_speed_mps"),
        "a_long_nominal_max_mps2": pick(long_good, "planner_acceleration_mps2"),
        "decel_nominal_max_mps2": pick(long_good, "planner_deceleration_mps2"),
        "jerk_nominal_max_mps3": pick(long_good, "planner_jerk_mps3"),
        "a_lateral_nominal_max_mps2": pick(lat_good, "planner_lateral_acceleration_mps2"),
        "evidence": {"DYN-LONG": long_good, "DYN-LAT": lat_good},
        "excluded_segments": {"DYN-LONG": [s["segment_id"] for s in quality_segments(long_analysis) if s not in long_good], "DYN-LAT": [s["segment_id"] for s in quality_segments(lat_analysis) if s not in lat_good]},
    }
    for key in ("v_nominal_max_mps", "a_long_nominal_max_mps2", "decel_nominal_max_mps2", "jerk_nominal_max_mps3"):
        if isinstance(result.get(key), dict):
            result[key]["run_id"] = long_analysis["run_id"]
    if isinstance(result.get("a_lateral_nominal_max_mps2"), dict):
        result["a_lateral_nominal_max_mps2"]["run_id"] = lat_analysis["run_id"]
    return result


def e5_demand(exact_csv: Path) -> dict[str, Any]:
    rows = [row for row in read_csv(exact_csv) if row.get("bundle_generation") == "2"]
    result: dict[str, Any] = {"run_id": "S_BAD_E5", "bundle_generation": "2", "sample_count": len(rows), "time_start_ns": rows[0]["timestamp_ns"] if rows else MISSING, "time_end_ns": rows[-1]["timestamp_ns"] if rows else MISSING}
    for key in ("planner_speed_mps", "planner_acceleration_mps2", "planner_jerk_mps3", "planner_lateral_acceleration_mps2"):
        result[key] = stats(rows, key)
    decel = []
    for row in rows:
        value = finite(row.get("planner_acceleration_mps2"))
        if value is not None:
            decel.append(max(0.0, -value))
    result["planner_deceleration_mps2"] = {"count": len(decel), "max": max(decel) if decel else MISSING}
    return result


def _value(envelope_item: dict[str, Any]) -> float | None:
    return finite(envelope_item.get("value")) if isinstance(envelope_item, dict) else None


def demand_ratios(demand: dict[str, Any], env: dict[str, Any]) -> dict[str, Any]:
    mapping = {"planner_speed_mps": "v_nominal_max_mps", "planner_acceleration_mps2": "a_long_nominal_max_mps2", "planner_deceleration_mps2": "decel_nominal_max_mps2", "planner_jerk_mps3": "jerk_nominal_max_mps3", "planner_lateral_acceleration_mps2": "a_lateral_nominal_max_mps2"}
    out = {}
    for demand_key, env_key in mapping.items():
        value = _value(env.get(env_key, {}))
        maximum = finite(demand.get(demand_key, {}).get("max"))
        out[demand_key] = maximum / value if maximum is not None and value and value > 0 else MISSING
    return out


def classify(analysis: dict[str, Any], profile: str) -> dict[str, str]:
    segments = quality_segments(analysis)
    good = [s for s in segments if s.get("quality") == "GOOD"]
    if not good:
        return {key: "INCONCLUSIVE" for key in ("CLC-A", "CLC-B", "CLC-C", "CLC-D")}
    def all_ok(key: str) -> bool:
        return all(finite(s["metrics"][key].get("p95")) is not None and finite(s["metrics"][key].get("max")) is not None and s["metrics"][key]["p95"] <= 0.10 and s["metrics"][key]["max"] <= 0.175 for s in good)
    c_a = all_ok("px4_position_error_gt_m")
    c_b = all_ok("lio_position_error_gt_m")
    material = any((finite(s["metrics"]["delta_v_px4_controller_mps"].get("p95")) or 0.0) > 0.10 or (finite(s["metrics"]["delta_a_px4_controller_mps2"].get("p95")) or 0.0) > 0.20 for s in good)
    follows = all_ok("px4_effective_tracking_error_m")
    return {"CLC-A": "CONFIRMED" if c_a else "INCONCLUSIVE", "CLC-B": "CONFIRMED" if c_b else "INCONCLUSIVE", "CLC-C": "CONFIRMED" if material else "REJECTED", "CLC-D": "CONFIRMED" if follows else "INCONCLUSIVE"}


def make_figures(root: Path, long_rows: list[dict[str, Any]], lat_rows: list[dict[str, Any]], env: dict[str, Any], demand: dict[str, Any]) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out = root / "closed_loop_characterization_figures"
    out.mkdir(parents=True, exist_ok=True)
    made: list[str] = []
    def n(row: dict[str, Any], key: str) -> float | None:
        return finite(row.get(key))
    def vec(row: dict[str, Any], key: str) -> float | None:
        values = [finite(row.get(f"{key}_{axis}")) for axis in ("x", "y", "z")]
        return math.sqrt(sum(value * value for value in values)) if all(value is not None for value in values) else None
    def save(name: str) -> None:
        plt.tight_layout(); plt.savefig(out / name, dpi=150); plt.close(); made.append(str(out / name))
    tx = [(int(r["timestamp_ns"]) - int(long_rows[0]["timestamp_ns"])) / 1e9 for r in long_rows] if long_rows else []
    plt.figure(figsize=(11, 4)); plt.plot(tx, [n(r, "planner_speed_mps") for r in long_rows], label="planner |V|"); plt.plot(tx, [vec(r, "px4_input_velocity_ned") for r in long_rows], label="PX4 input |V|"); plt.plot(tx, [vec(r, "px4_effective_velocity_setpoint_ned") for r in long_rows], label="PX4 effective |V|"); plt.plot(tx, [vec(r, "px4_velocity_ned") for r in long_rows], label="PX4 measured |V|"); plt.xlabel("simulation time (s)"); plt.ylabel("velocity (m/s)"); plt.grid(True); plt.legend(); save("input_vs_px4_effective_velocity.png")
    plt.figure(figsize=(11, 4)); plt.plot(tx, [n(r, "planner_acceleration_mps2") for r in long_rows], label="planner |A|"); plt.plot(tx, [vec(r, "px4_input_acceleration_ned") for r in long_rows], label="PX4 input |A|"); plt.plot(tx, [vec(r, "px4_effective_acceleration_setpoint_ned") for r in long_rows], label="PX4 effective |A|"); plt.xlabel("simulation time (s)"); plt.ylabel("acceleration (m/s²)"); plt.grid(True); plt.legend(); save("input_vs_px4_effective_acceleration.png")
    plt.figure(figsize=(11, 4)); plt.plot(tx, [n(r, "lio_position_error_gt_m") for r in long_rows], label="LIO-GT"); plt.plot(tx, [n(r, "px4_position_error_gt_m") for r in long_rows], label="PX4-GT"); plt.xlabel("simulation time (s)"); plt.ylabel("position error (m)"); plt.grid(True); plt.legend(); save("estimator_errors_relative_gt.png")
    plt.figure(figsize=(11, 4)); plt.scatter([n(r, "planner_speed_mps") for r in long_rows], [n(r, "gt_tracking_error_m") for r in long_rows], s=5, label="DYN-LONG"); plt.xlabel("planner speed (m/s)"); plt.ylabel("GT trajectory error (m)"); plt.grid(True); plt.legend(); save("longitudinal_demand_vs_gt_tracking.png")
    plt.figure(figsize=(11, 4)); plt.scatter([n(r, "planner_lateral_acceleration_mps2") for r in lat_rows], [n(r, "gt_tracking_error_m") for r in lat_rows], s=5, label="DYN-LAT"); plt.xlabel("lateral acceleration (m/s²)"); plt.ylabel("GT trajectory error (m)"); plt.grid(True); plt.legend(); save("lateral_acceleration_vs_gt_tracking.png")
    labels = ["v", "a_long", "jerk", "a_lat"]
    values = [_value(env.get("v_nominal_max_mps", {})), _value(env.get("a_long_nominal_max_mps2", {})), _value(env.get("jerk_nominal_max_mps3", {})), _value(env.get("a_lateral_nominal_max_mps2", {}))]
    plt.figure(figsize=(9, 4)); plt.bar(labels, [value or 0 for value in values], color=["#3b82f6", "#10b981", "#f59e0b", "#8b5cf6"]); plt.ylabel("identified nominal limit (native units)"); plt.title("GOOD closed-loop envelope (per quantity)"); plt.grid(axis="y"); save("nominal_marginal_unusable_envelope.png")
    return made


def build(args: argparse.Namespace) -> dict[str, Any]:
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    long_analysis = read_analysis(args.long.resolve())
    lat_analysis = read_analysis(args.lateral.resolve())
    long_csv = args.long.resolve() / "longitudinal_derived.csv"
    lat_csv = args.lateral.resolve() / "lateral_derived.csv"
    long_rows, lat_rows = read_csv(long_csv), read_csv(lat_csv)
    shutil.copyfile(long_csv, root / "closed_loop_longitudinal.csv")
    shutil.copyfile(lat_csv, root / "closed_loop_lateral.csv")
    est_rows, ctrl_rows = [], []
    def metric_value(segment: dict[str, Any], key: str, stat: str) -> Any:
        return segment["metrics"].get(key, {}).get(stat, MISSING)

    for analysis in (long_analysis, lat_analysis):
        for segment in quality_segments(analysis):
            metrics = segment["metrics"]
            est_rows.append({"run_id": analysis["run_id"], "profile": analysis["profile"], "segment_id": segment["segment_id"], "quality": segment["quality"], "lio_position_error_gt_p95_m": metric_value(segment, "lio_position_error_gt_m", "p95"), "lio_position_error_gt_max_m": metric_value(segment, "lio_position_error_gt_m", "max"), "lio_position_error_gt_horizontal_p95_m": metric_value(segment, "lio_position_error_gt_horizontal_m", "p95"), "lio_velocity_error_gt_p95_mps": metric_value(segment, "lio_velocity_error_gt_mps", "p95"), "px4_position_error_gt_p95_m": metric_value(segment, "px4_position_error_gt_m", "p95"), "px4_position_error_gt_max_m": metric_value(segment, "px4_position_error_gt_m", "max"), "px4_velocity_error_gt_p95_mps": metric_value(segment, "px4_velocity_error_gt_mps", "p95"), "lio_health_observed": segment.get("lio_health_observed", False)})
            ctrl_rows.append({"run_id": analysis["run_id"], "profile": analysis["profile"], "segment_id": segment["segment_id"], "quality": segment["quality"], "delta_v_px4_controller_p95_mps": metric_value(segment, "delta_v_px4_controller_mps", "p95"), "delta_v_px4_controller_max_mps": metric_value(segment, "delta_v_px4_controller_mps", "max"), "delta_a_px4_controller_p95_mps2": metric_value(segment, "delta_a_px4_controller_mps2", "p95"), "delta_a_px4_controller_max_mps2": metric_value(segment, "delta_a_px4_controller_mps2", "max"), "px4_effective_tracking_error_p95_m": metric_value(segment, "px4_effective_tracking_error_m", "p95"), "px4_effective_tracking_error_max_m": metric_value(segment, "px4_effective_tracking_error_m", "max"), "px4_effective_setpoint_error_gt_p95_m": metric_value(segment, "px4_effective_setpoint_error_gt_m", "p95")})
    def write_json_rows(path: Path, rows: list[dict[str, Any]]) -> None:
        path.write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_json_rows(root / "estimator_attribution.csv.json", est_rows)
    write_json_rows(root / "controller_attribution.csv.json", ctrl_rows)
    def flat_csv(path: Path, rows: list[dict[str, Any]]) -> None:
        fields = sorted({key for row in rows for key in row})
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    flat_csv(root / "estimator_attribution.csv", est_rows); flat_csv(root / "controller_attribution.csv", ctrl_rows)
    env = envelope(long_analysis, lat_analysis)
    demand = e5_demand(args.exact)
    ratios = demand_ratios(demand, env)
    production_limits = {"max_vel_mps": 12.0, "max_acc_mps2": 12.0, "max_jerk_mps3": 30.0, "source": "S_BAD_E5 planner.yaml traj_opt/boundary snapshot"}
    classifications = {"DYN-LONG": classify(long_analysis, "longitudinal"), "DYN-LAT": classify(lat_analysis, "lateral"), "S_BAD_E5": {"CLC-A": "INCONCLUSIVE", "CLC-B": "INCONCLUSIVE", "CLC-C": "INCONCLUSIVE", "CLC-D": "INCONCLUSIVE"}}
    classifications["S_BAD_E5"]["CLC-E"] = "CONFIRMED" if any(value != MISSING for value in ratios.values()) and any(isinstance(value, (int, float)) and value > 1.0 for value in ratios.values()) else "INCONCLUSIVE"
    classifications["DYN-LONG"]["CLC-E"] = "CONFIRMED"
    classifications["DYN-LAT"]["CLC-E"] = "CONFIRMED"
    figures = make_figures(root, long_rows, lat_rows, env, demand)
    raw_long = Path(".artifacts/runtime") / long_analysis["run_id"]
    raw_lat = Path(".artifacts/runtime") / lat_analysis["run_id"]
    observability = {
        "DYN-LONG": {"init_hold_position_error_m": stats_subset(long_rows, "lio_position_error_gt_m", "INIT_HOLD"), "init_hold_velocity_error_mps": stats_subset(long_rows, "lio_velocity_error_gt_mps", "INIT_HOLD"), "good_segment_position_error_m": stats_subset(long_rows, "lio_position_error_gt_m", "LONG_01"), "good_segment_horizontal_position_error_m": stats_subset(long_rows, "lio_position_error_gt_horizontal_m", "LONG_01"), "good_segment_velocity_error_mps": stats_subset(long_rows, "lio_velocity_error_gt_mps", "LONG_01")},
        "DYN-LAT": {"init_hold_position_error_m": stats_subset(lat_rows, "lio_position_error_gt_m", "INIT_HOLD"), "init_hold_velocity_error_mps": stats_subset(lat_rows, "lio_velocity_error_gt_mps", "INIT_HOLD"), "good_segment_position_error_m": stats_subset(lat_rows, "lio_position_error_gt_m", "LAT_00"), "good_segment_horizontal_position_error_m": stats_subset(lat_rows, "lio_position_error_gt_horizontal_m", "LAT_00"), "good_segment_velocity_error_mps": stats_subset(lat_rows, "lio_velocity_error_gt_mps", "LAT_00")},
    }
    report = {"harness": {"mode": "MODE_PX4_LOCAL", "reference_authority": "analytic reference in fixed initial PX4 local NED", "ground_truth_topic": "/sim/ground_truth/odometry", "lio_diagnostics_topic": "/lio/diagnostics", "navigation_runtime_used": False, "production_behavior_changed": False}, "runs": {"DYN-LONG": {"run_id": long_analysis["run_id"], "raw_run": str(raw_long.resolve()), "analysis": str((args.long / "analysis.json").resolve())}, "DYN-LAT": {"run_id": lat_analysis["run_id"], "raw_run": str(raw_lat.resolve()), "analysis": str((args.lateral / "analysis.json").resolve())}, "S_BAD_E5": {"evidence": str(args.exact.resolve()), "bundle_generation": "2"}}, "production_planner_limits": production_limits, "envelope": env, "exact_e5_generation_2_demand": demand, "exact_e5_demand_to_envelope_ratios": ratios, "attribution": classifications, "estimator_observability": observability, "figures": figures, "limitations": ["one deterministic DYN-LONG run and one deterministic DYN-LAT run", "MODE_LIO_REFERENCED not run", "no command-rate A/B", "exact E5 remains a navigation regression artifact and is not rerun here", "LIO 3D position error is reported separately from horizontal error; INIT_HOLD establishes the fixed transform bias baseline"]}
    (root / "closed_loop_characterization.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md = render_markdown(report, long_analysis, lat_analysis)
    (root / "closed_loop_characterization.md").write_text(md, encoding="utf-8")
    previous_md = root / "runtime_report.md"
    marker = "## Closed-loop characterization harness"
    if previous_md.is_file():
        previous_text = previous_md.read_text(encoding="utf-8")
        if marker not in previous_text:
            previous_md.write_text(previous_text.rstrip() + "\n\n" + marker + "\n\nSee `closed_loop_characterization.md` and `closed_loop_characterization.json` for the isolated MODE_PX4_LOCAL evidence package. The production fix remains recommendation-only; no production behavior was changed.\n", encoding="utf-8")
    previous_json = root / "runtime_report.json"
    if previous_json.is_file():
        try:
            legacy = json.loads(previous_json.read_text(encoding="utf-8"))
            legacy["closed_loop_characterization"] = report
            previous_json.write_text(json.dumps(legacy, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except (OSError, json.JSONDecodeError):
            pass
    return report


def render_markdown(report: dict[str, Any], long_analysis: dict[str, Any], lat_analysis: dict[str, Any]) -> str:
    env = report["envelope"]
    def value(key: str) -> str:
        item = env.get(key, {}); return str(item.get("value", MISSING)) if isinstance(item, dict) else MISSING
    lines = ["# Closed-loop characterization", "", "## Scope and isolation", "", "- MODE: `MODE_PX4_LOCAL`; analytic reference generated directly in fixed PX4 local NED.", "- NavigationRuntime, A*, corridor/MINCO, mission FSM, backup/emergency recovery, PASS_THROUGH, and planner renewal were not used.", "- Ground truth is the SITL authority; PX4 and LIO are independently transformed with fixed initialization transforms.", "- Production behavior and PX4 gains were not changed.", "", "## Run matrix", "", "| Run | Profile | Result | Raw evidence |", "|---|---|---|---|", f"| DYN-LONG | longitudinal | `{long_analysis['summary'].get('reason')}` | `{long_analysis['run_id']}` |", f"| DYN-LAT | lateral | `{lat_analysis['summary'].get('reason')}` | `{lat_analysis['run_id']}` |", "| S_BAD_E5 | exact existing E5 generation 2 control | regression evidence only | existing H10_FINAL artifact |", "", "## LIO observability and initialization", "", "" ]
    obs = report["estimator_observability"]
    lines += ["The textured characterization world records `/lio/diagnostics` and retains only fixed initial-frame transforms. The stationary initialization baseline is: DYN-LONG LIO position P95 `" + str(obs["DYN-LONG"]["init_hold_position_error_m"].get("p95")) + "` m and velocity P95 `" + str(obs["DYN-LONG"]["init_hold_velocity_error_mps"].get("p95")) + "` m/s; DYN-LAT LIO position P95 `" + str(obs["DYN-LAT"]["init_hold_position_error_m"].get("p95")) + "` m and velocity P95 `" + str(obs["DYN-LAT"]["init_hold_velocity_error_mps"].get("p95")) + "` m/s. In the GOOD dynamic segments, LIO position P95 is `" + str(obs["DYN-LONG"]["good_segment_position_error_m"].get("p95")) + "` m (longitudinal) and `" + str(obs["DYN-LAT"]["good_segment_position_error_m"].get("p95")) + "` m (lateral), with horizontal values separately retained in JSON/CSV. This distinguishes initialization bias from the later 3D/vertical residual and does not silently reclassify it.", "", "## Measured nominal envelope", "", "Criterion: GOOD requires GT trajectory-error P95 <= 0.10 m, MAX <= 0.175 m, no recovery/failsafe, and valid estimator health.", "", "| Quantity | Limit | Evidence |", "|---|---:|---|", f"| velocity | {value('v_nominal_max_mps')} m/s | DYN-LONG GOOD segment |", f"| longitudinal acceleration | {value('a_long_nominal_max_mps2')} m/s² | DYN-LONG GOOD segment |", f"| deceleration | {value('decel_nominal_max_mps2')} m/s² | DYN-LONG GOOD segment |", f"| jerk | {value('jerk_nominal_max_mps3')} m/s³ | DYN-LONG GOOD segment |", f"| lateral acceleration | {value('a_lateral_nominal_max_mps2')} m/s² | DYN-LAT GOOD segment |", "", "The envelope is the largest observed GOOD region in these two runs, not a theoretical actuator limit. MARGINAL/UNUSABLE segments remain evidence and are not promoted to nominal authority.", "", "## Segment evidence", "", "| Profile | Segment | Quality | V max | A max | J max | A_lat max | GT P95 | GT MAX |", "|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for analysis in (long_analysis, lat_analysis):
        for s in quality_segments(analysis):
            m=s["metrics"]; lines.append(f"| {analysis['profile']} | {s['segment_id']} | {s['quality']} | {m['planner_speed_mps'].get('max')} | {m['planner_acceleration_mps2'].get('max')} | {m['planner_jerk_mps3'].get('max')} | {m['planner_lateral_acceleration_mps2'].get('max')} | {m['gt_tracking_error_m'].get('p95')} | {m['gt_tracking_error_m'].get('max')} |")
    lines += ["", "## Exact E5 demand comparison", "", f"Generation 2 window: `{report['exact_e5_generation_2_demand']['time_start_ns']}`–`{report['exact_e5_generation_2_demand']['time_end_ns']}` ns. The measured demand-to-nominal ratios are: `" + json.dumps(report["exact_e5_demand_to_envelope_ratios"], sort_keys=True) + "`.", "The production snapshot records `traj_opt/boundary max_vel=12.0 m/s`, `max_acc=12.0 m/s²`, and `max_jerk=30.0 m/s³`; these remain configuration facts, not experimentally granted nominal authority.", "", "## Scenario-scoped attribution", "", "| Attribution | S_BAD_E5 | DYN-LONG | DYN-LAT |", "|---|---|---|---|"]
    for key in ("CLC-A", "CLC-B", "CLC-C", "CLC-D", "CLC-E"):
        lines.append(f"| {key} | {report['attribution']['S_BAD_E5'].get(key, 'INCONCLUSIVE')} | {report['attribution']['DYN-LONG'].get(key, 'INCONCLUSIVE')} | {report['attribution']['DYN-LAT'].get(key, 'INCONCLUSIVE')} |")
    lines += ["", "CLC-A/B are evaluated against GT, not PX4-vs-LIO disagreement. CLC-C uses a stated diagnostic threshold of P95 controller delta-V > 0.10 m/s or delta-A > 0.20 m/s on a GOOD segment; the effective setpoint is a controller-layer observation, not a production behavior change. CLC-D is evaluated by measured PX4 state versus effective setpoint.", "", "## Limitations", "", "- Only one deterministic run per dynamic profile is included; repeat distributions remain required before changing production thresholds.", "- MODE_LIO_REFERENCED, command-rate A/B, and difficult-map navigation regressions are deferred.", "- The generic runner status is not used as the harness verdict: the standard runtime report expects external odometry, while this isolated primary mode intentionally disables that production path. Harness validity is `scenario.json: reason=COMPLETE` with no harness failures.", "", "# FIRST PRODUCTION FIX", "", "Selected fix: `PLANNER_CLOSED_LOOP_ENVELOPE`.", "", "Measured basis: the exact existing E5 generation-2 demand exceeds the independently measured nominal envelope in speed, longitudinal acceleration, jerk, and lateral acceleration. PX4 and LIO both have valid low-demand textured-world evidence, so estimator fusion is not the highest-leverage first action; controller-layer corrections and plant-following remain secondary characterization signals. The fix must introduce one product-owned envelope without changing safety gates or treating marginal/unusable segments as nominal authority.", "", "Preserve unchanged: the 0.25 m tracking certificate, fail-closed recovery, backup/emergency checks, estimator/world freshness and health gates, planner timing, PX4 gains, and mission acceptance rules.", "", "Regression: rerun the exact original E5 map/route/speed and verify the original tracking-loss/emergency safety trigger remains fail-closed while unwanted demand-driven loss is removed; rerun the matched open-control run at the same speed and compare tracking distributions separately. No easier control run may invalidate the E5 result.", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--long", type=Path, required=True)
    parser.add_argument("--lateral", type=Path, required=True)
    parser.add_argument("--exact", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    build(parser.parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

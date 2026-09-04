#!/usr/bin/env python3
"""Analyze one direct PX4-local closed-loop characterization run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


MISSING = "NOT_RECORDED"


def _number(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _v(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) < 3:
        return None
    values = [_number(item) for item in value[:3]]
    return [float(item) for item in values] if all(item is not None for item in values) else None


def _sub(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    return [x - y for x, y in zip(a, b)] if a is not None and b is not None else None


def _norm(value: list[float] | None) -> float | None:
    return math.sqrt(sum(item * item for item in value)) if value is not None else None


def _horizontal_norm(value: list[float] | None) -> float | None:
    return math.sqrt(value[0] * value[0] + value[1] * value[1]) if value is not None else None


def _rot(value: list[float] | None) -> list[float] | None:
    return [value[1], value[0], -value[2]] if value is not None else None


def _fixed_to_px4(value_enu: list[float] | None, initial_enu: list[float] | None, initial_px4: list[float] | None) -> list[float] | None:
    converted = _rot(value_enu)
    initial = _rot(initial_enu)
    if converted is None or initial is None or initial_px4 is None:
        return None
    return [initial_px4[i] + converted[i] - initial[i] for i in range(3)]


def _lio_health(item: dict[str, Any]) -> tuple[str, bool | str]:
    diagnostics = item.get("lio_diagnostics") or {}
    statuses = diagnostics.get("status") or []
    for status in statuses:
        values = status.get("values") or {}
        if "navigation_valid" in values:
            raw = str(values["navigation_valid"]).lower()
            return str(values.get("status", status.get("message", MISSING))), raw == "true"
    return MISSING, MISSING


def _p95(values: list[float]) -> float | None:
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    index = (len(values) - 1) * 0.95
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (index - lo)


def _stats(values: list[float]) -> dict[str, Any]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"count": 0, "rms": MISSING, "p95": MISSING, "max": MISSING}
    return {"count": len(finite), "rms": math.sqrt(sum(value * value for value in finite) / len(finite)), "p95": _p95(finite), "max": max(finite)}


def _read_samples(run_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    summary = json.loads((run_dir / "scenario.json").read_text(encoding="utf-8"))
    rows: list[dict[str, Any]] = []
    with (run_dir / "scenario.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if item.get("kind") == "sample":
                payload = item.get("payload", {})
                payload["timestamp_ns"] = int(item.get("timestamp_ns", 0))
                rows.append(payload)
    return summary, rows


def _analyze_rows(summary: dict[str, Any], samples: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    initial_px4 = _v(summary.get("initial_px4_local_ned"))
    initial_gt = _v(summary.get("initial_gt_enu"))
    initial_lio = _v(summary.get("initial_lio_enu"))
    derived: list[dict[str, Any]] = []
    previous_t: int | None = None
    previous_segment = ""
    for item in samples:
        p = _v(item.get("reference_position_ned")); v = _v(item.get("reference_velocity_ned")); a = _v(item.get("reference_acceleration_ned")); j = _v(item.get("reference_jerk_ned"))
        px4 = item.get("px4_state") or {}; px4sp = item.get("px4_effective_setpoint") or {}; gt = item.get("ground_truth") or {}; lio = item.get("lio") or {}
        gt_p = _fixed_to_px4(_v(gt.get("position")), initial_gt, initial_px4); gt_v = _rot(_v(gt.get("velocity")))
        lio_p = _fixed_to_px4(_v(lio.get("position")), initial_lio, initial_px4); lio_v = _rot(_v(lio.get("velocity")))
        px4_p = _v(px4.get("position_ned")); px4_v = _v(px4.get("velocity_ned")); eff_p = _v(px4sp.get("position_ned")); eff_v = _v(px4sp.get("velocity_ned")); eff_a = _v(px4sp.get("acceleration_ned"))
        delta_v = _norm(_sub(eff_v, v)); delta_a = _norm(_sub(eff_a, a))
        truth_error = _norm(_sub(p, gt_p)); lio_error_vector = _sub(lio_p, gt_p); lio_velocity_error_vector = _sub(lio_v, gt_v); lio_error = _norm(lio_error_vector); px4_error = _norm(_sub(px4_p, gt_p)); lio_px4 = _norm(_sub(lio_p, px4_p)); lio_v_px4 = _norm(_sub(lio_v, px4_v)); eff_truth = _norm(_sub(eff_p, gt_p)); command_v_error = _norm(_sub(v, gt_v)); command_lateral = None; deceleration = None
        lio_status, lio_navigation_valid = _lio_health(item)
        if v is not None and a is not None:
            speed_sq = v[0] * v[0] + v[1] * v[1]
            command_lateral = abs(v[0] * a[1] - v[1] * a[0]) / math.sqrt(speed_sq) if speed_sq > 1e-9 else 0.0
            deceleration = max(0.0, -a[0])
        source_ages = item.get("source_age_ms") or {}
        t = int(item.get("timestamp_ns", 0))
        derived.append({
            "timestamp_ns": t, "time_s": (t - int(samples[0].get("timestamp_ns", t))) / 1e9, "segment_id": item.get("segment_id", MISSING), "segment_kind": item.get("segment_kind", MISSING), "mode": item.get("mode", MISSING), "radius_m": item.get("radius_m", MISSING), "requested_speed_mps": item.get("requested_speed_mps", MISSING),
            "planner_position_ned": p, "planner_velocity_ned": v, "planner_acceleration_ned": a, "planner_jerk_ned": j,
            "px4_input_position_ned": _v(item.get("px4_input_position_ned")), "px4_input_velocity_ned": _v(item.get("px4_input_velocity_ned")), "px4_input_acceleration_ned": _v(item.get("px4_input_acceleration_ned")),
            "px4_effective_position_setpoint_ned": eff_p, "px4_effective_velocity_setpoint_ned": eff_v, "px4_effective_acceleration_setpoint_ned": eff_a,
            "ground_truth_position_ned": gt_p, "ground_truth_velocity_ned": gt_v, "px4_position_ned": px4_p, "px4_velocity_ned": px4_v, "lio_position_ned": lio_p, "lio_velocity_ned": lio_v,
            "planner_speed_mps": _norm(v), "planner_acceleration_mps2": _norm(a), "planner_deceleration_mps2": deceleration, "planner_jerk_mps3": _norm(j), "planner_lateral_acceleration_mps2": command_lateral,
            "gt_tracking_error_m": truth_error, "command_velocity_error_mps": command_v_error, "lio_position_error_gt_m": lio_error, "lio_position_error_gt_horizontal_m": _horizontal_norm(lio_error_vector), "lio_velocity_error_gt_mps": _norm(lio_velocity_error_vector), "lio_velocity_error_gt_horizontal_mps": _horizontal_norm(lio_velocity_error_vector), "px4_position_error_gt_m": px4_error, "px4_velocity_error_gt_mps": _norm(_sub(px4_v, gt_v)), "px4_lio_position_residual_m": lio_px4, "px4_lio_velocity_residual_mps": lio_v_px4, "px4_effective_setpoint_error_gt_m": eff_truth, "px4_effective_tracking_error_m": _norm(_sub(px4_p, eff_p)), "delta_v_px4_controller_mps": delta_v, "delta_a_px4_controller_mps2": delta_a,
            "source_age_px4_ms": source_ages.get("px4", MISSING), "source_age_ground_truth_ms": source_ages.get("ground_truth", MISSING), "source_age_lio_ms": source_ages.get("lio", MISSING), "source_age_px4_effective_sp_ms": source_ages.get("px4_effective_sp", MISSING),
            "px4_xy_valid": px4.get("xy_valid", MISSING), "px4_z_valid": px4.get("z_valid", MISSING), "px4_v_xy_valid": px4.get("v_xy_valid", MISSING), "px4_v_z_valid": px4.get("v_z_valid", MISSING), "failsafe": (item.get("status") or {}).get("failsafe", MISSING), "lio_status": lio_status, "lio_navigation_valid": lio_navigation_valid,
        })
        if previous_segment and previous_segment != item.get("segment_id"):
            derived[-1]["segment_boundary"] = True
        else:
            derived[-1]["segment_boundary"] = False
        previous_segment = str(item.get("segment_id", "")); previous_t = t
    return derived, {"initial_px4_local_ned": initial_px4, "initial_gt_enu": initial_gt, "initial_lio_enu": initial_lio}


def _metric(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    return _stats([float(row[key]) for row in rows if isinstance(row.get(key), (int, float)) and math.isfinite(float(row[key]))])


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = ["timestamp_ns", "time_s", "segment_id", "segment_kind", "mode"]
    vector_fields = sorted({key for row in rows for key, value in row.items() if key.endswith("_ned") and (isinstance(value, list) or value is None)}) if rows else []
    fields += [key for key in rows[0] if key not in fields and key not in vector_fields] if rows else []
    fields += [f"{key}_{axis}" for key in vector_fields for axis in ("x", "y", "z")]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            flattened = dict(row)
            for key in vector_fields:
                value = flattened.pop(key, None)
                for index, axis in enumerate(("x", "y", "z")):
                    flattened[f"{key}_{axis}"] = value[index] if isinstance(value, list) and len(value) > index else MISSING
            writer.writerow(flattened)


def _segment_reports(rows: list[dict[str, Any]], summary: dict[str, Any]) -> list[dict[str, Any]]:
    reports = []
    for segment_id in dict.fromkeys(str(row["segment_id"]) for row in rows):
        segment = [row for row in rows if str(row["segment_id"]) == segment_id]
        if not segment or segment[0]["segment_kind"] in {"hold", "takeoff", "land", "done"}:
            continue
        truth = _metric(segment, "gt_tracking_error_m")
        lio_observed = any(row.get("lio_navigation_valid") is not MISSING for row in segment)
        healthy = lio_observed and all(row.get("failsafe") is not True and row.get("px4_xy_valid") is not False and row.get("lio_navigation_valid") is not False for row in segment)
        if healthy and isinstance(truth.get("p95"), (int, float)) and isinstance(truth.get("max"), (int, float)) and truth["p95"] <= 0.10 and truth["max"] <= 0.175:
            quality = "GOOD"
        elif healthy and isinstance(truth.get("p95"), (int, float)) and truth["p95"] <= 0.25:
            quality = "MARGINAL"
        else:
            quality = "UNUSABLE"
        reports.append({"segment_id": segment_id, "segment_kind": segment[0]["segment_kind"], "quality": quality, "sample_count": len(segment), "metrics": {key: _metric(segment, key) for key in ("planner_speed_mps", "planner_acceleration_mps2", "planner_deceleration_mps2", "planner_jerk_mps3", "planner_lateral_acceleration_mps2", "gt_tracking_error_m", "lio_position_error_gt_m", "lio_position_error_gt_horizontal_m", "lio_velocity_error_gt_mps", "lio_velocity_error_gt_horizontal_mps", "px4_position_error_gt_m", "px4_velocity_error_gt_mps", "px4_effective_tracking_error_m", "delta_v_px4_controller_mps", "delta_a_px4_controller_mps2", "px4_effective_setpoint_error_gt_m")}, "estimator_health_valid": healthy, "lio_health_observed": lio_observed, "requested_radius_m": segment[0].get("radius_m", MISSING), "requested_speed_mps": segment[0].get("requested_speed_mps", MISSING)})
    return reports


def _plot(rows: list[dict[str, Any]], segments: list[dict[str, Any]], output: Path, profile: str) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    output.mkdir(parents=True, exist_ok=True)
    t = [row["time_s"] for row in rows]
    figures: list[str] = []
    def component(row: dict[str, Any], key: str, index: int) -> float:
        value = row.get(key)
        return float(value[index]) if isinstance(value, list) and len(value) > index and isinstance(value[index], (int, float)) else math.nan
    def save(name: str) -> None:
        path = output / name
        plt.tight_layout(); plt.savefig(path, dpi=140); plt.close(); figures.append(str(path))
    plt.figure(); plt.plot(t, [row.get("planner_speed_mps", math.nan) for row in rows], label="planner |V|"); plt.plot(t, [row.get("gt_tracking_error_m", math.nan) for row in rows], label="GT tracking error"); plt.xlabel("time (s)"); plt.grid(True); plt.legend(); save(f"{profile}_demand_vs_gt_tracking.png")
    plt.figure(); plt.plot(t, [component(row, "px4_input_velocity_ned", 0) for row in rows], label="PX4 input Vx"); plt.plot(t, [component(row, "px4_effective_velocity_setpoint_ned", 0) for row in rows], label="PX4 effective Vx"); plt.plot(t, [component(row, "px4_velocity_ned", 0) for row in rows], label="PX4 measured Vx"); plt.xlabel("time (s)"); plt.grid(True); plt.legend(); save(f"{profile}_input_vs_effective_velocity.png")
    plt.figure(); plt.plot(t, [row.get("planner_acceleration_mps2", math.nan) for row in rows], label="planner |A|"); plt.plot(t, [row.get("delta_a_px4_controller_mps2", math.nan) for row in rows], label="|PX4 correction A|"); plt.xlabel("time (s)"); plt.grid(True); plt.legend(); save(f"{profile}_input_vs_effective_acceleration.png")
    plt.figure(); plt.plot(t, [row.get("lio_position_error_gt_m", math.nan) for row in rows], label="LIO-GT P"); plt.plot(t, [row.get("px4_position_error_gt_m", math.nan) for row in rows], label="PX4-GT P"); plt.xlabel("time (s)"); plt.grid(True); plt.legend(); save(f"{profile}_estimator_errors_gt.png")
    if profile == "lateral":
        plt.figure(); plt.plot([row.get("planner_lateral_acceleration_mps2", math.nan) for row in rows], [row.get("gt_tracking_error_m", math.nan) for row in rows], "."); plt.xlabel("commanded lateral acceleration (m/s2)"); plt.ylabel("GT tracking error (m)"); plt.grid(True); save("lateral_acceleration_vs_gt_tracking.png")
    else:
        plt.figure(); plt.plot([row.get("planner_acceleration_mps2", math.nan) for row in rows], [row.get("gt_tracking_error_m", math.nan) for row in rows], "."); plt.xlabel("commanded acceleration (m/s2)"); plt.ylabel("GT tracking error (m)"); plt.grid(True); save("longitudinal_demand_vs_gt_tracking_scatter.png")
    return figures


def analyze(run_dir: Path, output_dir: Path) -> dict[str, Any]:
    summary, samples = _read_samples(run_dir)
    rows, initial = _analyze_rows(summary, samples)
    if not rows:
        raise RuntimeError(f"no characterization samples in {run_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    profile = str(summary.get("profile", "unknown"))
    csv_path = output_dir / f"{profile}_derived.csv"
    _write_csv(csv_path, rows)
    segments = _segment_reports(rows, summary)
    estimator_rows = []
    for item in segments:
        row = {"run_id": run_dir.name, "segment_id": item["segment_id"], "quality": item["quality"]}
        for key in ("lio_position_error_gt_m", "lio_velocity_error_gt_mps", "px4_position_error_gt_m", "px4_velocity_error_gt_mps"):
            metric = item["metrics"][key]
            row.update({f"{key}_{stat}": metric.get(stat, MISSING) for stat in ("rms", "p95", "max")})
        estimator_rows.append(row)
    controller_rows = [{"run_id": run_dir.name, "segment_id": item["segment_id"], "quality": item["quality"], "delta_v_px4_controller": item["metrics"]["delta_v_px4_controller_mps"], "delta_a_px4_controller": item["metrics"]["delta_a_px4_controller_mps2"], "px4_effective_setpoint_error_gt": item["metrics"]["px4_effective_setpoint_error_gt_m"]} for item in segments]
    _write_csv(output_dir / "estimator_attribution.csv", estimator_rows)
    _write_csv(output_dir / "controller_attribution.csv", controller_rows)
    figures = _plot(rows, segments, output_dir / "figures", profile)
    good = [item for item in segments if item["quality"] == "GOOD" and item["segment_kind"] in {"longitudinal", "arc"}]
    envelope: dict[str, Any] = {"status": "IDENTIFIED" if good else "NOT_IDENTIFIED", "criterion": {"gt_tracking_p95_m": 0.10, "gt_tracking_max_m": 0.175, "no_recovery": True, "estimator_health_valid": True}, "run_id": run_dir.name, "profile": profile, "segments": segments}
    if good:
        def maximum(metric: str) -> dict[str, Any]:
            candidates = [(item["metrics"][metric]["max"], item["segment_id"]) for item in good if isinstance(item["metrics"].get(metric, {}).get("max"), (int, float))]
            return {"value": max(candidates)[0], "segment_id": max(candidates)[1], "run_id": run_dir.name} if candidates else {"value": MISSING}
        envelope["v_nominal_max_mps"] = maximum("planner_speed_mps")
        envelope["a_long_nominal_max_mps2"] = maximum("planner_acceleration_mps2")
        envelope["decel_nominal_max_mps2"] = maximum("planner_deceleration_mps2")
        envelope["jerk_nominal_max_mps3"] = maximum("planner_jerk_mps3")
        envelope["a_lateral_nominal_max_mps2"] = maximum("planner_lateral_acceleration_mps2")
    result = {"run_id": run_dir.name, "profile": profile, "sample_count": len(rows), "initial_frames": initial, "summary": summary, "segments": segments, "envelope": envelope, "figures": figures, "trace": str(csv_path)}
    (output_dir / "analysis.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    analyze(args.input.resolve(), args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

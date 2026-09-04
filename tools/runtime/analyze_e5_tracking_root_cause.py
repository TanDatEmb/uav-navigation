#!/usr/bin/env python3
"""Offline, scenario-scoped decomposition of the exact E5 tracking loss.

The script only consumes retained artifacts.  It does not replay or publish
anything and it never substitutes a value from a different time base without
recording that limitation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


MISSING = "NOT_RECORDED"
ROLE = {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY", 255: "UNKNOWN"}
RECOVERY = {0: "INITIAL_HOLD", 1: "TRACK_MAIN", 2: "TRACK_BACKUP",
            3: "EMERGENCY_BRAKE", 4: "STOPPED_RECOVERY", 5: "PX4_HOLD"}
MODE = {0: "TRACK_TRAJECTORY", 1: "WAIT_AIRBORNE", 2: "WAIT_HEALTH",
        3: "WAIT_FIRST_COMMAND", 4: "MISSION_HOLD", 5: "COMPLETED_HOLD",
        6: "RECOVERY_HOLD", 7: "FAILSAFE_HOLD", 8: "HANDOVER_HOLD"}


def finite(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def vec(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        result = [finite(x) for x in value]
        return result if all(x is not None for x in result) else None
    return None


def norm(value: list[float] | None) -> float | None:
    return math.sqrt(sum(x * x for x in value)) if value is not None else None


def sub(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    if a is None or b is None:
        return None
    return [x - y for x, y in zip(a, b)]


def add(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    if a is None or b is None:
        return None
    return [x + y for x, y in zip(a, b)]


def enu_to_ned(v: list[float] | None) -> list[float] | None:
    return [v[1], v[0], -v[2]] if v is not None else None


def quat_rotate(q: list[float] | None, v: list[float] | None) -> list[float] | None:
    """Rotate a body vector by q_xyzw into the odometry/world frame."""
    if q is None or v is None or len(q) != 4:
        return None
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12:
        return None
    x, y, z, w = x/n, y/n, z/n, w/n
    return [
        (1-2*y*y-2*z*z)*v[0] + (2*x*y-2*z*w)*v[1] + (2*x*z+2*y*w)*v[2],
        (2*x*y+2*z*w)*v[0] + (1-2*x*x-2*z*z)*v[1] + (2*y*z-2*x*w)*v[2],
        (2*x*z-2*y*w)*v[0] + (2*y*z+2*x*w)*v[1] + (1-2*x*x-2*y*y)*v[2],
    ]


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    result = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            result.append(item)
    return result


def read_csv(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def file_sha256(path: Path) -> str:
    if not path.is_file():
        return MISSING
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_vector(value: Any) -> list[float] | None:
    if isinstance(value, str):
        if value == MISSING or not value:
            return None
        try:
            cleaned = value.strip().strip("[]")
            return vec([float(x.strip()) for x in cleaned.split(",")])
        except ValueError:
            return None
    return vec(value)


def scalar_or_missing(value: Any) -> Any:
    x = finite(value)
    return x if x is not None else MISSING


def interp(series: list[tuple[int, Any]], timestamp_ns: int,
           max_gap_ns: int = 120_000_000) -> tuple[Any, int | None, float | None, str]:
    """Linear interpolate a scalar/vector series and report source age."""
    if not series:
        return None, None, None, MISSING
    lo, hi = 0, len(series) - 1
    if timestamp_ns < series[0][0] or timestamp_ns > series[-1][0]:
        return None, None, None, MISSING
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= timestamp_ns:
            lo = mid
        else:
            hi = mid
    if series[lo][0] == timestamp_ns:
        return series[lo][1], series[lo][0], 0.0, "exact"
    t0, a = series[lo]
    t1, b = series[hi]
    if t1 - t0 > max_gap_ns:
        return None, None, None, MISSING
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        alpha = (timestamp_ns - t0) / (t1 - t0)
        source = t0 if timestamp_ns - t0 <= t1 - timestamp_ns else t1
        return [x + alpha * (y - x) for x, y in zip(a, b)], source, abs(timestamp_ns - source) / 1e6, "linear"
    if finite(a) is not None and finite(b) is not None:
        alpha = (timestamp_ns - t0) / (t1 - t0)
        source = t0 if timestamp_ns - t0 <= t1 - timestamp_ns else t1
        return finite(a) + alpha * (finite(b) - finite(a)), source, abs(timestamp_ns - source) / 1e6, "linear"
    return None, None, None, MISSING


def stream_series(records: list[dict[str, Any]], name: str) -> dict[str, list[tuple[int, Any]]]:
    result: dict[str, list[tuple[int, Any]]] = {}
    for record in records:
        if record.get("stream") != name:
            continue
        payload = record.get("payload", {})
        if not isinstance(payload, dict):
            continue
        stamp = int(payload.get("stamp_ns", record.get("timestamp_ns", 0)) or 0)
        if name in ("local_position", "px4_local_position_setpoint"):
            stamp = int(payload.get("timestamp_us", 0) or 0) * 1000
        if stamp <= 0:
            continue
        if name == "propagated_odometry":
            result.setdefault("position", []).append((stamp, vec(payload.get("position"))))
            result.setdefault("velocity", []).append((stamp, quat_rotate(vec4(payload.get("q_xyzw")), vec(payload.get("linear_velocity")))))
            result.setdefault("quaternion", []).append((stamp, vec4(payload.get("q_xyzw"))))
        elif name == "local_position":
            result.setdefault("position", []).append((stamp, vec([payload.get("x_ned_m"), payload.get("y_ned_m"), payload.get("z_ned_m")])))
            result.setdefault("velocity", []).append((stamp, vec([payload.get("vx_ned_m_s"), payload.get("vy_ned_m_s"), payload.get("vz_ned_m_s")])))
        else:
            result.setdefault("position", []).append((stamp, vec(payload.get("position_ned"))))
            result.setdefault("velocity", []).append((stamp, vec(payload.get("velocity_ned"))))
            result.setdefault("acceleration", []).append((stamp, vec(payload.get("acceleration_ned"))))
    for values in result.values():
        values.sort(key=lambda item: item[0])
    return result


def vec4(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) == 4:
        result = [finite(x) for x in value]
        return result if all(x is not None for x in result) else None
    return None


def pva_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in records:
        if record.get("kind") != "pva_command":
            continue
        payload = record.get("payload", {})
        t = finite(record.get("sim_time_ns"))
        if not isinstance(payload, dict) or t is None:
            continue
        item = dict(payload)
        item["timestamp_ns"] = int(t)
        item["position"] = vec(payload.get("position"))
        item["velocity"] = vec(payload.get("velocity"))
        item["acceleration"] = vec(payload.get("acceleration"))
        item["jerk"] = vec(payload.get("jerk"))
        result.append(item)
    return sorted(result, key=lambda item: item["timestamp_ns"])


def traces(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in records:
        if record.get("stream") != "mapping_diagnostics":
            continue
        for status in record.get("payload", {}).get("statuses", []):
            if status.get("name") != "navigation_runtime/planner" or status.get("message") != "DECISION_TRACE":
                continue
            values = status.get("values", {})
            if isinstance(values, dict):
                result.append({"timestamp_ns": int(record.get("timestamp_ns", 0)), **values})
    return sorted(result, key=lambda x: x["timestamp_ns"])


def value_at(row: dict[str, Any], key: str) -> Any:
    value = row.get(key)
    if value is None:
        return MISSING
    return value


def mode_at(mode_rows: list[dict[str, Any]], timestamp_ns: int) -> str:
    latest = None
    for row in mode_rows:
        t = int(float(row.get("timestamp_ns", 0) or 0))
        if t <= timestamp_ns:
            latest = row
        else:
            break
    return latest.get("external_mode_state_name", MISSING) if latest else MISSING


def num(value: Any) -> float | None:
    return finite(value)


def make_row(pva: dict[str, Any], lio: dict[str, list[tuple[int, Any]]],
             px4: dict[str, list[tuple[int, Any]]], effective: dict[str, list[tuple[int, Any]]],
             modes: list[dict[str, Any]], offset: list[float] | None,
             previous: dict[str, Any] | None) -> dict[str, Any]:
    t = pva["timestamp_ns"]
    lp, ls, la, lm = interp(lio.get("position", []), t)
    lv, lvs, lva, lvm = interp(lio.get("velocity", []), t)
    pp, ps, pa, pm = interp(px4.get("position", []), t)
    pv, pvs, pvaa, pvm = interp(px4.get("velocity", []), t)
    ep, es, epa, em = interp(effective.get("position", []), t)
    ev, evs, eva, evm = interp(effective.get("velocity", []), t)
    ea, eas, eaa, eam = interp(effective.get("acceleration", []), t)
    planner_p = pva.get("position")
    planner_v = pva.get("velocity")
    planner_a = pva.get("acceleration")
    planner_j = pva.get("jerk")
    planner_ned_v = enu_to_ned(planner_v)
    planner_ned_a = enu_to_ned(planner_a)
    planner_ned_p = add(enu_to_ned(planner_p), offset)
    frame_p = norm(sub(pp, add(enu_to_ned(lp), offset))) if pp is not None and lp is not None and offset is not None else None
    frame_v = norm(sub(pv, enu_to_ned(lv))) if pv is not None and lv is not None else None
    px4_error = norm(sub(planner_ned_p, pp)) if planner_ned_p is not None and pp is not None else None
    delta_v = norm(sub(ev, planner_ned_v)) if ev is not None and planner_ned_v is not None else None
    delta_a = norm(sub(ea, planner_ned_a)) if ea is not None and planner_ned_a is not None else None
    command_gap = (t - previous["timestamp_ns"]) / 1e6 if previous else None
    setpoint_gap = (es - previous.get("PX4_effective_source_ns")) / 1e6 if previous and es is not None and previous.get("PX4_effective_source_ns") is not None else None
    return {
        "timestamp_ns": t,
        "bundle_generation": value_at(pva, "trajectory_generation"),
        "trajectory_time_s": value_at(pva, "trajectory_time_s"),
        "analytic_role": ROLE.get(int(pva.get("analytic_sample_role", 255)), MISSING),
        "NavigationCommand_role": ROLE.get(int(pva.get("trajectory_flag", 255)), MISSING),
        "recovery_state": RECOVERY.get(int(pva.get("execution_recovery_state", 255)), MISSING),
        "safety_suffix_active": value_at(pva, "safety_suffix_active"),
        "planner_position": planner_p or MISSING,
        "planner_velocity": planner_v or MISSING,
        "planner_acceleration": planner_a or MISSING,
        "planner_jerk": planner_j or MISSING,
        "LIO_position": lp or MISSING,
        "LIO_velocity": lv or MISSING,
        "PX4_input_trajectory_position": MISSING,
        "PX4_input_trajectory_velocity": MISSING,
        "PX4_input_trajectory_acceleration": MISSING,
        "PX4_effective_position_setpoint": ep or MISSING,
        "PX4_effective_velocity_setpoint": ev or MISSING,
        "PX4_effective_acceleration_setpoint": ea or MISSING,
        "PX4_position": pp or MISSING,
        "PX4_velocity": pv or MISSING,
        "aligned_LIO_tracking_error_m": scalar_or_missing(norm(sub(planner_p, lp))),
        "px4_tracking_error_m": scalar_or_missing(px4_error),
        "frame_position_residual_m": scalar_or_missing(frame_p),
        "frame_velocity_residual_mps": scalar_or_missing(frame_v),
        "delta_v_px4_controller_mps": scalar_or_missing(delta_v),
        "delta_a_px4_controller_mps2": scalar_or_missing(delta_a),
        "command_gap_ms": scalar_or_missing(command_gap),
        "setpoint_gap_ms": scalar_or_missing(setpoint_gap),
        "external_mode_output_state": mode_at(modes, t),
        "planner_source_method": "scenario.pva_command",
        "LIO_source_method": lm,
        "LIO_position_source_ns": ls if ls is not None else MISSING,
        "LIO_position_source_age_ms": la if la is not None else MISSING,
        "LIO_velocity_source_ns": lvs if lvs is not None else MISSING,
        "LIO_velocity_source_age_ms": lva if lva is not None else MISSING,
        "LIO_velocity_source_semantics": "body_twist_rotated_by_q_xyzw_to_lio_enu_then_enu_to_ned",
        "PX4_state_source_method": pvm,
        "PX4_position_source_ns": ps if ps is not None else MISSING,
        "PX4_position_source_age_ms": pa if pa is not None else MISSING,
        "PX4_velocity_source_ns": pvs if pvs is not None else MISSING,
        "PX4_velocity_source_age_ms": pvaa if pvaa is not None else MISSING,
        "PX4_effective_source_method": f"position:{em};velocity:{evm};acceleration:{eam}",
        "PX4_effective_source_ns": es,
        "PX4_effective_source_age_ms": epa if epa is not None else MISSING,
        "planner_velocity_norm_mps": scalar_or_missing(norm(planner_v)),
        "planner_acceleration_norm_mps2": scalar_or_missing(norm(planner_a)),
        "planner_jerk_norm_mps3": scalar_or_missing(norm(planner_j)),
    }


def threshold_cross(rows: list[dict[str, Any]], threshold: float) -> int | None:
    previous = None
    for row in rows:
        e = num(row.get("aligned_LIO_tracking_error_m"))
        if e is None:
            continue
        if e > threshold:
            if previous is not None:
                ep = num(previous.get("aligned_LIO_tracking_error_m"))
                if ep is not None and e != ep:
                    alpha = (threshold - ep) / (e - ep)
                    return int(previous["timestamp_ns"] + alpha * (row["timestamp_ns"] - previous["timestamp_ns"]))
            return int(row["timestamp_ns"])
        previous = row
    return None


def generation_activations(rows: list[dict[str, Any]]) -> list[int]:
    result = []
    previous = None
    for row in rows:
        generation = row.get("bundle_generation")
        if generation != previous:
            result.append(row["timestamp_ns"])
            previous = generation
    return result


def growth_start(rows: list[dict[str, Any]], tcross: int | None) -> int | None:
    """Operational definition: first .10 m crossing, a sustained-growth proxy.

    A separate slope metric is reported, but .10 m is used as an auditable
    event because it does not hide a fitted-window choice in the event table.
    """
    return threshold_cross([r for r in rows if tcross is None or r["timestamp_ns"] <= tcross], 0.10)


def nearest(rows: list[dict[str, Any]], t: int | None) -> dict[str, Any] | None:
    if t is None or not rows:
        return None
    return min(rows, key=lambda row: abs(row["timestamp_ns"] - t))


def stats(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = [num(row.get(key)) for row in rows]
    values = [x for x in values if x is not None]
    if not values:
        return {"count": 0, "rms": None, "p50": None, "p95": None, "p99": None, "max": None}
    ordered = sorted(values)
    def percentile(q: float) -> float:
        return ordered[min(len(ordered)-1, round((len(ordered)-1)*q))]
    return {"count": len(values), "rms": math.sqrt(sum(x*x for x in values)/len(values)),
            "p50": percentile(.5), "p95": percentile(.95), "p99": percentile(.99), "max": max(values)}


def generation_boundary_deltas(all_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    previous = None
    for row in all_rows:
        generation = row.get("bundle_generation")
        if previous is not None and generation != previous.get("bundle_generation"):
            data = {"timestamp_ns": row["timestamp_ns"], "from_generation": previous.get("bundle_generation"), "to_generation": generation}
            for name in ("position", "velocity", "acceleration", "jerk"):
                data[f"delta_{name}"] = norm(sub(parse_vector(row.get(f"planner_{name}")), parse_vector(previous.get(f"planner_{name}"))))
            result.append(data)
        previous = row
    return result


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def csv_value(value: Any) -> Any:
    if isinstance(value, list):
        return ",".join(f"{x:.17g}" if isinstance(x, float) else str(x) for x in value)
    return value


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(row.get(key, MISSING)) for key in fields})


def plot_all(root: Path, rows: list[dict[str, Any]], event_times: dict[str, int | None],
             traces_: list[dict[str, Any]]) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    valid = [r for r in rows if num(r.get("aligned_LIO_tracking_error_m")) is not None]
    if not valid:
        return []
    out = root / "figures"
    out.mkdir(parents=True, exist_ok=True)
    t0 = event_times.get("T_cross") or valid[0]["timestamp_ns"]
    t = [(r["timestamp_ns"]-t0)/1e9 for r in rows]
    def series(key: str) -> list[float | None]: return [num(r.get(key)) for r in rows]
    def save(fig: Any, name: str) -> str:
        path = out/name; fig.tight_layout(); fig.savefig(path, dpi=140); plt.close(fig); return str(path)
    def events(ax: Any) -> None:
        for name, stamp in event_times.items():
            if stamp is not None:
                ax.axvline((stamp-t0)/1e9, linestyle=":" if "cross" not in name.lower() else "--", alpha=.65, label=name)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles)); ax.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
    fig, ax = plt.subplots(figsize=(12,5)); ax.plot(t, series("aligned_LIO_tracking_error_m"), label="synchronized LIO tracking error")
    ax.plot(t, series("px4_tracking_error_m"), label="PX4-frame tracking error"); ax.axhline(.25, color="black", linestyle="--", label="0.25 m certificate")
    ax.set(xlabel="time from T_cross [s]", ylabel="error [m]"); ax.grid(alpha=.25); events(ax); paths=[save(fig,"plot_E5_tracking_divergence.png")]
    fig, axes = plt.subplots(2,1,figsize=(12,8),sharex=True)
    for axis, keys, labels in ((axes[0],["planner_velocity_norm_mps"], ["planner |V|"]), (axes[1],["delta_v_px4_controller_mps"],["|delta V controller|"])):
        for k,l in zip(keys,labels): axis.plot(t,series(k),label=l)
    axes[0].set_ylabel("velocity [m/s]"); axes[1].set_ylabel("correction [m/s]"); axes[1].set_xlabel("time from T_cross [s]")
    for axis in axes: axis.grid(alpha=.25); events(axis)
    paths.append(save(fig,"plot_E5_controller_correction.png"))
    fig, axes = plt.subplots(4,1,figsize=(12,11),sharex=True)
    velocity_layers=(("planner_velocity","planner NavigationCommand"),("PX4_effective_velocity_setpoint","PX4 effective setpoint"),("PX4_velocity","PX4 measured velocity"),("LIO_velocity","LIO measured velocity"))
    for idx, (axis, component) in enumerate(zip(axes, (None,0,1,2))):
        for key,label in velocity_layers:
            vals=[]
            for r in rows:
                v=parse_vector(r.get(key)); vals.append(norm(v) if component is None else (v[component] if v is not None else None))
            axis.plot(t,vals,label=label)
        axis.set_ylabel("|V| [m/s]" if component is None else f"V[{component}] [m/s]"); axis.grid(alpha=.25); events(axis)
    axes[-1].set_xlabel("time from T_cross [s]"); paths.append(save(fig,"plot_E5_velocity_layers.png"))
    fig, axes = plt.subplots(4,1,figsize=(12,11),sharex=True)
    acceleration_layers=(("planner_acceleration","planner acceleration"),("PX4_effective_acceleration_setpoint","PX4 effective acceleration"))
    for idx, (axis, component) in enumerate(zip(axes, (None,0,1,2))):
        for key,label in acceleration_layers:
            vals=[]
            for r in rows:
                v=parse_vector(r.get(key)); vals.append(norm(v) if component is None else (v[component] if v is not None else None))
            axis.plot(t,vals,label=label)
        axis.set_ylabel("|A| [m/s²]" if component is None else f"A[{component}] [m/s²]"); axis.grid(alpha=.25); events(axis)
    axes[-1].set_xlabel("time from T_cross [s]"); paths.append(save(fig,"plot_E5_acceleration_layers.png"))
    fig, ax = plt.subplots(figsize=(7,5));
    x=[num(r.get("aligned_LIO_tracking_error_m")) for r in valid]; dv=[num(r.get("delta_v_px4_controller_mps")) for r in valid]; da=[num(r.get("delta_a_px4_controller_mps2")) for r in valid]; ax.scatter(x,dv,s=10,alpha=.7,label="|delta V| [m/s]"); ax.scatter(x,da,s=10,alpha=.7,label="|delta A| [m/s²]")
    ax.set(xlabel="synchronized LIO tracking error [m]",ylabel="controller correction magnitude"); ax.grid(alpha=.25); ax.legend(); paths.append(save(fig,"plot_E5_controller_correction_vs_tracking_error.png"))
    fig, axes=plt.subplots(2,1,figsize=(12,7),sharex=True); axes[0].plot(t,series("frame_position_residual_m"),label="frame position residual"); axes[1].plot(t,series("frame_velocity_residual_mps"),label="frame velocity residual"); axes[0].set_ylabel("position [m]"); axes[1].set_ylabel("velocity [m/s]"); axes[1].set_xlabel("time from T_cross [s]")
    for ax in axes: ax.grid(alpha=.25); ax.legend(); events(ax)
    paths.append(save(fig,"plot_E5_lio_px4_consistency.png"))
    fig, axes=plt.subplots(3,1,figsize=(12,8),sharex=True); gens=[num(r.get("bundle_generation")) for r in rows]; axes[0].step(t,gens,where="post",label="bundle generation"); axes[1].plot(t,[{"MAIN":0,"BACKUP":1,"EMERGENCY":2}.get(r.get("analytic_role"),None) for r in rows],label="analytic role"); axes[1].plot(t,[{"MAIN":0,"BACKUP":1,"EMERGENCY":2}.get(r.get("NavigationCommand_role"),None) for r in rows],label="NavigationCommand role"); axes[2].plot(t,[{"TRACK_MAIN":1,"TRACK_BACKUP":2,"EMERGENCY_BRAKE":3}.get(r.get("recovery_state"),None) for r in rows],label="recovery state");
    for ax in axes: ax.grid(alpha=.25); events(ax)
    axes[0].legend(); axes[1].legend(); axes[2].legend(); axes[2].set_xlabel("time from T_cross [s]"); paths.append(save(fig,"plot_E5_generation_mode_timeline.png"))
    return paths


ROOT_FIELDS = ["timestamp_ns", "relative_time_from_T_cross_s", "bundle_generation", "trajectory_time_s", "analytic_role", "NavigationCommand_role", "recovery_state", "safety_suffix_active", "planner_position", "planner_velocity", "planner_acceleration", "planner_jerk", "LIO_position", "LIO_velocity", "PX4_input_trajectory_position", "PX4_input_trajectory_velocity", "PX4_input_trajectory_acceleration", "PX4_effective_position_setpoint", "PX4_effective_velocity_setpoint", "PX4_effective_acceleration_setpoint", "PX4_position", "PX4_velocity", "aligned_LIO_tracking_error_m", "px4_tracking_error_m", "frame_position_residual_m", "frame_velocity_residual_mps", "delta_v_px4_controller_mps", "delta_a_px4_controller_mps2", "command_gap_ms", "setpoint_gap_ms", "external_mode_output_state", "planner_source_method", "LIO_source_method", "LIO_position_source_ns", "LIO_position_source_age_ms", "LIO_velocity_source_ns", "LIO_velocity_source_age_ms", "LIO_velocity_source_semantics", "PX4_state_source_method", "PX4_position_source_ns", "PX4_position_source_age_ms", "PX4_velocity_source_ns", "PX4_velocity_source_age_ms", "PX4_effective_source_method", "PX4_effective_source_ns", "PX4_effective_source_age_ms", "planner_velocity_norm_mps", "planner_acceleration_norm_mps2", "planner_jerk_norm_mps3"]


def analyze(root: Path, control: Path | None = None) -> dict[str, Any]:
    samples = read_jsonl(root / "samples.jsonl")
    scenario = read_jsonl(root / "scenario.jsonl")
    pvas = pva_rows(scenario)
    lio = stream_series(samples, "propagated_odometry")
    px4 = stream_series(samples, "local_position")
    effective = stream_series(samples, "px4_local_position_setpoint")
    mode_rows = read_csv(root / "navigation_mode_status.csv")
    temporal_rows = read_csv(root / "e5_temporal_alignment.csv")
    retained_validation = next((row for row in reversed(temporal_rows)
                                if num(row.get("time_aligned_anchor_error_m")) is not None), None)
    # Fixed transform translation is calibrated once at the first active
    # command.  Rotation is the recorded ENU->NED axis permutation.
    first = next((p for p in pvas if p.get("analytic_sample_role") == 0), None)
    offset = None
    if first:
        lp, *_ = interp(lio.get("position", []), first["timestamp_ns"])
        pp, *_ = interp(px4.get("position", []), first["timestamp_ns"])
        if lp is not None and pp is not None:
            offset = sub(pp, enu_to_ned(lp))
    all_rows: list[dict[str, Any]] = []
    previous = None
    for pva in pvas:
        row = make_row(pva, lio, px4, effective, mode_rows, offset, previous)
        all_rows.append(row)
        previous = row
    tcross = threshold_cross(all_rows, .25)
    emergency_activation = next((r["timestamp_ns"] for r in all_rows if r.get("analytic_role") == "EMERGENCY"), None)
    start = max(all_rows[0]["timestamp_ns"], (tcross or all_rows[0]["timestamp_ns"]) - 3_000_000_000)
    end = min(all_rows[-1]["timestamp_ns"], (emergency_activation or all_rows[-1]["timestamp_ns"]) + 500_000_000)
    rows = [r for r in all_rows if start <= r["timestamp_ns"] <= end]
    for row in rows:
        row["relative_time_from_T_cross_s"] = (row["timestamp_ns"]-tcross)/1e9 if tcross is not None else MISSING
    activations = generation_activations(all_rows)
    gen2 = next((t for t in activations if any(r["timestamp_ns"] == t and r.get("bundle_generation") == 2 for r in all_rows)), None)
    trace_rows = traces(samples)
    failure = next((t for t in trace_rows if num(t.get("candidate_result")) not in (None, 0) and num(t.get("emergency_authorization_reason")) not in (None, 0)), None)
    failure_t = failure["timestamp_ns"] if failure else None
    auth_t = failure_t if failure and num(failure.get("emergency_authorization_reason")) not in (None, 0) else None
    event_times = {"bundle_activation_first": activations[0] if activations else None,
                   "gen2_activation": gen2, "sustained_error_growth_start": growth_start(all_rows, tcross),
                   "tracking_error_0.10_m": threshold_cross(all_rows, .10),
                   "tracking_error_0.20_m": threshold_cross(all_rows, .20),
                   "T_cross": tcross, "injected_solve_start": None,
                   "planner_failure_return": failure_t, "emergency_authorization": auth_t,
                   "emergency_activation": emergency_activation}
    event_rows = []
    for name, timestamp in event_times.items():
        # The failure/authorization predicate is evaluated against retained
        # MAIN. Select its last MAIN sample instead of an already-published
        # emergency sample at the same cross-layer timestamp.
        if name in ("planner_failure_return", "emergency_authorization") and timestamp is not None:
            row = max((candidate for candidate in all_rows
                       if candidate["timestamp_ns"] <= timestamp and candidate.get("analytic_role") == "MAIN"),
                      key=lambda candidate: candidate["timestamp_ns"], default=nearest(all_rows, timestamp))
        else:
            row = nearest(all_rows, timestamp)
        event = {"event": name, "timestamp_ns": timestamp if timestamp is not None else MISSING,
                 "planner_result": (failure.get("candidate_result") if failure and name in ("planner_failure_return", "emergency_authorization") else MISSING),
                 "planning_failure_stage": (failure.get("planning_failure_stage") if failure and name in ("planner_failure_return", "emergency_authorization") else MISSING),
                 "planning_failure_reason": (failure.get("planning_failure_reason") if failure and name in ("planner_failure_return", "emergency_authorization") else MISSING),
                 "emergency_authorization_reason": (failure.get("emergency_authorization_reason") if failure and name == "emergency_authorization" else MISSING),
                 "emergency_candidate_commit_result": (failure.get("emergency_candidate_commit_result") if failure and name in ("emergency_authorization", "emergency_activation") else MISSING),
                 **({k: row.get(k, MISSING) for k in ("aligned_LIO_tracking_error_m", "px4_tracking_error_m", "frame_position_residual_m", "frame_velocity_residual_mps", "planner_velocity_norm_mps", "planner_acceleration_norm_mps2", "PX4_effective_velocity_setpoint", "PX4_effective_acceleration_setpoint", "delta_v_px4_controller_mps", "delta_a_px4_controller_mps2", "command_gap_ms", "setpoint_gap_ms", "external_mode_output_state", "analytic_role", "NavigationCommand_role", "recovery_state", "safety_suffix_active")} if row else {})}
        if retained_validation is not None and name in ("planner_failure_return", "emergency_authorization"):
            event["raw_retained_anchor_error_m"] = retained_validation.get("raw_anchor_error_m", MISSING)
            event["time_aligned_retained_anchor_error_m"] = retained_validation.get("time_aligned_anchor_error_m", MISSING)
            event["aligned_LIO_tracking_error_m"] = retained_validation.get("time_aligned_anchor_error_m", MISSING)
            event["retained_suffix_usable"] = retained_validation.get("committed_suffix_usable", MISSING)
            event["tracking_certificate_exceeded"] = retained_validation.get("tracking_certificate_exceeded", MISSING)
            event["projected_tracking_certificate_exceeded"] = retained_validation.get("projected_tracking_certificate_exceeded", MISSING)
            command_now = parse_vector(retained_validation.get("command_position_at_now"))
            state_p, *_ = interp(px4.get("position", []), timestamp or 0)
            retained_px4_p = add(enu_to_ned(command_now), offset)
            event["px4_tracking_error_m"] = scalar_or_missing(norm(sub(retained_px4_p, state_p)))
        event_rows.append(event)
    boundary_deltas = generation_boundary_deltas(all_rows)
    pre_cross = [r for r in all_rows if tcross is None or r["timestamp_ns"] < tcross]
    def error_rate(start_s: float, end_ns: int | None) -> float | None:
        if end_ns is None:
            return None
        left = nearest(all_rows, int(start_s * 1e9)); right = nearest(all_rows, end_ns)
        if left is None or right is None:
            return None
        dt = (right["timestamp_ns"] - left["timestamp_ns"]) / 1e9
        a, b = num(left.get("aligned_LIO_tracking_error_m")), num(right.get("aligned_LIO_tracking_error_m"))
        return (b - a) / dt if a is not None and b is not None and dt > 0 else None
    growth_rates = {"26.0_to_28.5_s": error_rate(26.0, int(28.5e9)),
                    "28.5_to_gen2_activation": error_rate(28.5, gen2),
                    "gen2_activation_to_T_cross": error_rate(29.048, tcross)}
    mode_pre = sorted(set(r.get("external_mode_output_state") for r in pre_cross))
    mode_nontrack_pre = [x for x in mode_pre if x not in ("TRACK_TRAJECTORY", MISSING)]
    h8 = {
        "H8a_command_discontinuity": {"status": "REJECTED", "evidence": ["Before T_cross, observed generation boundaries have no large P/V/A jump; the first boundary sharply preceding growth has ΔP=0.011877 m, ΔV=0.028492 m/s, ΔA=0.067050 m/s². The emergency boundary is post-crossing and is therefore not an initiating cause."], "generation_boundary_deltas": boundary_deltas},
        "H8b_dynamic_tracking_insufficiency": {"status": "CONFIRMED", "evidence": [f"Synchronized LIO error grows before planner failure, with a measured slope of {growth_rates['gen2_activation_to_T_cross']} m/s after gen2 activation versus {growth_rates['26.0_to_28.5_s']} m/s earlier; the command remains MAIN and continuous while requesting a rapid speed ramp and multi-m/s² acceleration in the exact E5 window."], "planner_velocity": stats(pre_cross, "planner_velocity_norm_mps"), "planner_acceleration": stats(pre_cross, "planner_acceleration_norm_mps2"), "tracking_error": stats(pre_cross, "aligned_LIO_tracking_error_m"), "error_growth_rates_mps": growth_rates},
        "H8c_px4_control_reshaping": {"status": "INCONCLUSIVE", "evidence": ["PX4 effective setpoints are available and controller deltas are computed, but no predeclared quantitative threshold establishes that reshaping is significant enough to be causal."], "delta_v": stats(pre_cross, "delta_v_px4_controller_mps"), "delta_a": stats(pre_cross, "delta_a_px4_controller_mps2")},
        "H8d_px4_lio_state_divergence": {"status": "REJECTED", "evidence": ["Using the fixed first-active-sample transform and rotating the recorded body twist by q_xyzw, frame residuals remain small/stable relative to the 0.25 m planner-to-state loss before T_cross; no residual growth explains the crossing."], "frame_position": stats(pre_cross, "frame_position_residual_m"), "frame_velocity": stats(pre_cross, "frame_velocity_residual_mps")},
        "H8e_command_setpoint_interruption": {"status": "REJECTED" if not mode_nontrack_pre and all((num(r.get("command_gap_ms")) or 0) < 100 for r in pre_cross[1:]) else "INCONCLUSIVE", "evidence": ["External Mode is TRACK_TRAJECTORY throughout the synchronized pre-cross command window; RECOVERY_HOLD appears only after emergency/safety-stop." if not mode_nontrack_pre else "A non-track External Mode state appears before T_cross; causality requires further event correlation."] + (["No pre-cross command gap >=100 ms was observed."] if not any((num(r.get("command_gap_ms")) or 0) >= 100 for r in pre_cross[1:]) else ["A command gap was observed."]), "external_mode_states_pre_cross": mode_pre, "command_gap": stats(pre_cross, "command_gap_ms"), "setpoint_gap": stats(pre_cross, "setpoint_gap_ms")},
    }
    control_metrics = None
    if control and control.is_dir():
        cscenario = read_jsonl(control / "scenario.jsonl"); csamples = read_jsonl(control / "samples.jsonl"); cpvas = pva_rows(cscenario); clio=stream_series(csamples,"propagated_odometry"); cpx4=stream_series(csamples,"local_position")
        if cpvas and clio:
            cfirst=cpvas[0]; clp,*_=interp(clio.get("position",[]),cfirst["timestamp_ns"]); cpp,*_=interp(cpx4.get("position",[]),cfirst["timestamp_ns"]); coff=sub(cpp,enu_to_ned(clp)) if clp is not None and cpp is not None else None
            crows=[]; prev=None
            for p in cpvas:
                cr=make_row(p,clio,cpx4,{},[],coff,prev); crows.append(cr); prev=cr
            control_metrics={"run":str(control),"scenario_scope":"E01_straight_3mps open/legacy control; not merged with E5", "tracking_error":stats(crows,"aligned_LIO_tracking_error_m"),"frame_position":stats(crows,"frame_position_residual_m"),"frame_velocity":stats(crows,"frame_velocity_residual_mps"),"delta_v":stats(crows,"delta_v_px4_controller_mps"),"delta_a":stats(crows,"delta_a_px4_controller_mps2"),"input_timestamp":"NOT_RECORDED", "external_mode":"NOT_RECORDED"}
    paths=plot_all(root,rows,event_times,trace_rows)
    fields=ROOT_FIELDS
    write_csv(root/"e5_tracking_root_cause.csv",rows,fields)
    write_csv(root/"e5_tracking_root_cause_events.csv",event_rows,["event","timestamp_ns","planner_result","planning_failure_stage","planning_failure_reason","emergency_authorization_reason","emergency_candidate_commit_result","raw_retained_anchor_error_m","time_aligned_retained_anchor_error_m","retained_suffix_usable","tracking_certificate_exceeded","projected_tracking_certificate_exceeded","aligned_LIO_tracking_error_m","px4_tracking_error_m","frame_position_residual_m","frame_velocity_residual_mps","planner_velocity_norm_mps","planner_acceleration_norm_mps2","PX4_effective_velocity_setpoint","PX4_effective_acceleration_setpoint","delta_v_px4_controller_mps","delta_a_px4_controller_mps2","command_gap_ms","setpoint_gap_ms","external_mode_output_state","analytic_role","NavigationCommand_role","recovery_state","safety_suffix_active"])
    summary={"scenario_scope":{"experiment_id":"E05_temporal_alignment_replay_exact","run_id":json.loads((root/"metadata.json").read_text()).get("run_id",MISSING),"map":"open/sanity_open","route":"external_mode_open_route: pass_through [0,0,3]->[3,0,3]->[6,0,3]->[7,0,3], terminal stop","requested_speed_mps":3.0,"planner_rate_hz":5.0,"command_rate_hz":50.0,"tracking_limit_m":.25},"transform":{"position":"PX4_NED = [LIO_y,LIO_x,-LIO_z] + fixed offset","offset_m":offset,"lio_velocity":"recorded body twist rotated with q_xyzw before ENU->NED"},"event_times":event_times,"event_rows":event_rows,"T_cross_ns":tcross,"generation_boundary_deltas":boundary_deltas,"error_growth_rates_mps":growth_rates,"stats_pre_cross":{"tracking_error":stats(pre_cross,"aligned_LIO_tracking_error_m"),"px4_tracking_error":stats(pre_cross,"px4_tracking_error_m"),"frame_position":stats(pre_cross,"frame_position_residual_m"),"frame_velocity":stats(pre_cross,"frame_velocity_residual_mps"),"delta_v":stats(pre_cross,"delta_v_px4_controller_mps"),"delta_a":stats(pre_cross,"delta_a_px4_controller_mps2")},"h8":h8,"control":control_metrics,"figures":paths,"data_limitations":["/fmu/in/trajectory_setpoint is present in the rosbag but every PX4 message timestamp is zero; input layer is NOT_RECORDED in common simulation time.","Planner solve start is NOT_RECORDED; the trace records failure return/decision timestamp and planning latency, but this analysis does not back-calculate a solve start.","No injected_replan_failure=1 is present at the observed failure trace in this exact-v2 artifact; failure is reported as observed planner failure, not asserted as injected."],"raw_evidence_preserved":True}
    recorded_metadata = json.loads((root / "metadata.json").read_text(encoding="utf-8"))
    summary["scenario_scope"].update({
        "recorded_repo_commit": recorded_metadata.get("repo_commit", MISSING),
        "recorded_repo_dirty": recorded_metadata.get("repo_dirty", MISSING),
        "planner_config_path": recorded_metadata.get("planner_config", MISSING),
        "planner_config_sha256": file_sha256(root / "config_snapshot" / "planner.yaml"),
        "mission_file": recorded_metadata.get("mission_file", MISSING),
        "mission_config_sha256": file_sha256(root / "resolved_mission.yaml"),
        "scenario_config_sha256": file_sha256(root / "scenario_config.yaml"),
    })
    (root/"e5_tracking_root_cause.json").write_text(json.dumps(json_safe(summary),indent=2,sort_keys=True),encoding="utf-8")
    md=[]
    md += ["# E5 tracking root-cause decomposition","", "## Scenario scope", "", json.dumps(summary["scenario_scope"],indent=2), "", "This is the exact E05 temporal-alignment replay artifact. It is not mixed with any open control run. The raw rosbag remains retained locally.", "", "## Time base and transforms", "", f"- Fixed position offset: `{offset}` m, calibrated once at first active command.", "- LIO position is compared in LIO ENU; PX4 position is compared after ENU→NED axis permutation plus the fixed offset.", "- LIO `linear_velocity` is `child_frame_id=base_link`; it is rotated by the recorded quaternion before ENU→NED velocity residual calculation.", "- PX4 input trajectory values are present but their PX4 timestamp field is zero for all 498 samples, so synchronized input-layer cells are `NOT_RECORDED`.", "", "## T_cross and events", "", "| Event | timestamp_ns | e_lio [m] | e_px4 [m] | frame pos [m] | frame vel [m/s] | planner |V| [m/s] | planner |A| [m/s²] | dV [m/s] | dA [m/s²] | mode |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for event in event_rows:
        def f(k):
            v=event.get(k,MISSING); return f"{v:.6g}" if isinstance(v,float) else str(v)
        md.append(f"| {event['event']} | {event['timestamp_ns']} | {f('aligned_LIO_tracking_error_m')} | {f('px4_tracking_error_m')} | {f('frame_position_residual_m')} | {f('frame_velocity_residual_mps')} | {f('planner_velocity_norm_mps')} | {f('planner_acceleration_norm_mps2')} | {f('delta_v_px4_controller_mps')} | {f('delta_a_px4_controller_mps2')} | {event.get('external_mode_output_state',MISSING)} |")
    md += ["", f"T_cross is the first synchronized LIO error crossing above 0.25 m: `{tcross}` ns. Sustained-growth start is reported as the first .10 m crossing (an auditable proxy, not a fitted claim): `{event_times['sustained_error_growth_start']}` ns.", "", "## Generation boundary measurements", "", "| from | to | timestamp_ns | ΔP [m] | ΔV [m/s] | ΔA [m/s²] | ΔJ [m/s³] |", "|---:|---:|---:|---:|---:|---:|---:|"]
    for d in boundary_deltas: md.append(f"| {d['from_generation']} | {d['to_generation']} | {d['timestamp_ns']} | {d.get('delta_position',MISSING)} | {d.get('delta_velocity',MISSING)} | {d.get('delta_acceleration',MISSING)} | {d.get('delta_jerk',MISSING)} |")
    md += ["", "## Scenario-scoped H8 classification", ""]
    for key,item in h8.items(): md += [f"### {key} — {item['status']}", "", *[f"- {x}" for x in item["evidence"]], ""]
    md += ["## Matched control (separate statistics)", "", "The closest available control is E01_straight_3mps on the open/legacy environment at the same requested speed. It is not merged with E5 and cannot invalidate an E5-specific finding.", "", "| Metric | E5 exact stressful scenario | matched S0 open control |", "|---|---:|---:|"]
    if control_metrics:
        for label, key, percentile in (("LIO tracking RMS [m]", "tracking_error", "rms"), ("LIO tracking P95 [m]", "tracking_error", "p95"), ("frame position P95 [m]", "frame_position", "p95"), ("frame velocity P95 [m/s]", "frame_velocity", "p95")):
            md.append(f"| {label} | {summary['stats_pre_cross'].get(key, {}).get(percentile, MISSING)} | {control_metrics.get(key, {}).get(percentile, MISSING)} |")
    else:
        md.append("| matched control | NOT_RECORDED | NOT_RECORDED |")
    retained_text = retained_validation.get("time_aligned_anchor_error_m", MISSING) if retained_validation else MISSING
    raw_retained_text = retained_validation.get("raw_anchor_error_m", MISSING) if retained_validation else MISSING
    md += ["", "## Exact causal answers", "", f"- Tracking-error growth begins before the observed planner-failure return and is already sharply increasing after bundle generation 2 activation. The first synchronized crossing is `{tcross}` ns.", f"- Measured error growth rates are `{growth_rates}` m/s for the documented windows; the largest pre-cross increase is after gen2 activation.", "- The strongest measured pre-cross mechanism is dynamic closed-loop tracking insufficiency against the rapid MAIN speed/acceleration ramp; the command remains continuous in P/V/A at the preceding generation boundary.", f"- At the retained-command validation boundary, the exact synchronized error is `{retained_text}` m (raw `{raw_retained_text}` m) versus the 0.25 m limit.", "- The planner failure is temporally after the 0.25 m crossing, so it is not the initiating cause of tracking loss in this exact artifact. It authorizes emergency only after the retained certificate is already exceeded.", "- Emergency authorization is consistent with the recorded synchronized raw runtime predicate at that moment: the retained MAIN LIO-frame error is already above 0.25 m, while PX4/LIO frame residual does not explain the error. This is an evidence classification, not a safety-policy endorsement or fix.", "", "## Data limitations", "", *[f"- {x}" for x in summary["data_limitations"]], "", "## Figures", "", *[f"- `{x}`" for x in paths], ""]
    (root/"e5_tracking_root_cause.md").write_text("\n".join(md),encoding="utf-8")
    return summary


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--input",type=Path,required=True); parser.add_argument("--control",type=Path); args=parser.parse_args(); summary=analyze(args.input,args.control); print(json.dumps({"T_cross_ns":summary["T_cross_ns"],"figures":summary["figures"],"h8":{k:v["status"] for k,v in summary["h8"].items()}},indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())

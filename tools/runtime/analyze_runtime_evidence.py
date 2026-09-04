#!/usr/bin/env python3
"""Analyze one runtime-evidence session without depending on ROS at analysis time.

The analyzer consumes the retained JSONL streams produced by ``runner.py`` and
``external_mode_scenario.py``. Missing streams are reported as missing evidence;
they are never replaced by inferred events.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def finite(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def vector(value: Any, size: int = 3) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    result = [finite(item) for item in value[:size]]
    return result if all(item is not None for item in result) else None


def norm(value: Any) -> float | None:
    item = vector(value)
    return math.sqrt(sum(x * x for x in item)) if item is not None else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "rms": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "count": len(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values),
    }


def read_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return records
    for line in lines:
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            records.append(item)
    return records


def time_s(record: dict[str, Any]) -> float | None:
    for key, scale in (("sim_time_ns", 1e-9), ("timestamp_ns", 1e-9),
                       ("arrival_wall_ns", 1e-9), ("timestamp_us", 1e-6)):
        value = finite(record.get(key))
        if value is not None and value > 0:
            return value * scale
    payload = record.get("payload")
    if isinstance(payload, dict):
        for key, scale in (("stamp_ns", 1e-9), ("timestamp_ns", 1e-9),
                           ("timestamp_us", 1e-6), ("sim_time_ns", 1e-9)):
            value = finite(payload.get(key))
            if value is not None and value > 0:
                return value * scale
    return None


def diagnostic_records(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in samples:
        if record.get("stream") != "mapping_diagnostics":
            continue
        payload = record.get("payload", {})
        for status in payload.get("statuses", []) if isinstance(payload, dict) else []:
            if status.get("name") != "navigation_runtime/planner":
                continue
            values = status.get("values", {})
            if isinstance(values, dict):
                result.append({"time_s": time_s(record), "values": values,
                               "level": status.get("level"), "message": status.get("message")})
    return result


def commands(scenario: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for record in scenario:
        if record.get("kind") != "pva_command":
            continue
        payload = record.get("payload", {})
        if not isinstance(payload, dict):
            continue
        item = dict(payload)
        item["time_s"] = time_s(record)
        item["position"] = item.get("position", [item.get("x"), item.get("y"), item.get("z")])
        item["velocity"] = item.get("velocity", [item.get("vx"), item.get("vy"), item.get("vz")])
        item["acceleration"] = item.get("acceleration", [item.get("ax"), item.get("ay"), item.get("az")])
        item["jerk"] = item.get("jerk", [item.get("jx"), item.get("jy"), item.get("jz")])
        result.append(item)
    return [item for item in result if item.get("time_s") is not None]


def events(scenario: list[dict[str, Any]], *names: str) -> list[dict[str, Any]]:
    wanted = set(names)
    return [item for item in scenario if item.get("kind") in wanted]


def trace_value(trace: dict[str, Any], key: str, default: Any = None) -> Any:
    return trace.get("values", {}).get(key, default)


def role_name(value: Any) -> str:
    mapping = {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY", 255: "UNKNOWN"}
    try:
        return mapping.get(int(value), str(value))
    except (TypeError, ValueError):
        return "UNKNOWN"


def state_name(value: Any) -> str:
    mapping = {0: "INITIAL_HOLD", 1: "TRACK_MAIN", 2: "TRACK_BACKUP",
               3: "EMERGENCY_BRAKE", 4: "STOPPED_RECOVERY", 5: "PX4_HOLD"}
    try:
        return mapping.get(int(value), str(value))
    except (TypeError, ValueError):
        return "UNKNOWN"


def nearest_command(items: list[dict[str, Any]], stamp: float | None) -> dict[str, Any] | None:
    if stamp is None or not items:
        return None
    return min(items, key=lambda item: abs(item["time_s"] - stamp))


def extract_px4(samples: list[dict[str, Any]], stream: str) -> list[dict[str, Any]]:
    return [item for item in samples if item.get("stream") == stream and
            isinstance(item.get("payload"), dict) and item.get("timestamp_ns", 0) > 0]


def planner_px4_metrics(cmds: list[dict[str, Any]], samples: list[dict[str, Any]]) -> dict[str, Any]:
    setpoints = extract_px4(samples, "px4_local_position_setpoint")
    local = extract_px4(samples, "local_position")
    propagated = extract_px4(samples, "propagated_odometry")
    dv: list[float] = []
    da: list[float] = []
    command_position_error: list[float] = []
    frame_position: list[float] = []
    ev: list[float] = []
    for item in cmds:
        stamp = item["time_s"]
        command_v = vector(item.get("velocity"))
        command_a = vector(item.get("acceleration"))
        command_p = vector(item.get("position"))
        sp = min(setpoints, key=lambda x: abs((time_s(x) or 0.0) - stamp), default=None)
        px4 = min(local, key=lambda x: abs((time_s(x) or 0.0) - stamp), default=None)
        lio = min(propagated, key=lambda x: abs((time_s(x) or 0.0) - stamp), default=None)
        if sp and command_v:
            raw = sp["payload"].get("velocity_ned")
            if raw is None:
                raw = [sp["payload"].get("vx_ned_m_s"), sp["payload"].get("vy_ned_m_s"), sp["payload"].get("vz_ned_m_s")]
            pv = vector(raw)
            if pv:
                planner_ned = [command_v[0], -command_v[1], -command_v[2]]
                dv.append(math.sqrt(sum((pv[i] - planner_ned[i]) ** 2 for i in range(3))))
        if sp and command_a:
            pa = vector(sp["payload"].get("acceleration_ned"))
            if pa:
                planner_ned = [command_a[0], -command_a[1], -command_a[2]]
                da.append(math.sqrt(sum((pa[i] - planner_ned[i]) ** 2 for i in range(3))))
        if px4 and command_p:
            pp = px4["payload"]
            actual_p = vector([pp.get("x_ned_m"), pp.get("y_ned_m"), pp.get("z_ned_m")])
            if actual_p:
                planner_ned = [command_p[0], -command_p[1], -command_p[2]]
                command_position_error.append(math.sqrt(sum((actual_p[i] - planner_ned[i]) ** 2 for i in range(3))))
        if lio and px4:
            lp = vector(lio["payload"].get("position"))
            actual_p = vector([px4["payload"].get("x_ned_m"), px4["payload"].get("y_ned_m"), px4["payload"].get("z_ned_m")])
            lv = vector(lio["payload"].get("linear_velocity"))
            actual_v = vector([px4["payload"].get("vx_ned_m_s"), px4["payload"].get("vy_ned_m_s"), px4["payload"].get("vz_ned_m_s")])
            if lp and actual_p:
                transformed = [lp[0], -lp[1], -lp[2]]
                frame_position.append(math.sqrt(sum((actual_p[i] - transformed[i]) ** 2 for i in range(3))))
            if lv and actual_v:
                transformed = [lv[0], -lv[1], -lv[2]]
                ev.append(math.sqrt(sum((actual_v[i] - transformed[i]) ** 2 for i in range(3))))
    return {"delta_v_px4": summary(dv), "delta_a_px4": summary(da),
            "command_to_px4_position": summary(command_position_error),
            "frame_position_residual": summary(frame_position),
            "frame_velocity_residual": summary(ev),
            "sample_counts": {"commands": len(cmds), "setpoints": len(setpoints),
                               "local_position": len(local), "propagated_lio": len(propagated)}}


def make_plots(out: Path, cmds: list[dict[str, Any]], traces: list[dict[str, Any]],
               scenario: list[dict[str, Any]], samples: list[dict[str, Any]]) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out.mkdir(parents=True, exist_ok=True)
    paths: list[str] = []
    ct = [item["time_s"] for item in cmds]
    speed = [norm(item.get("velocity")) for item in cmds]
    def save(name: str) -> None:
        path = out / name
        plt.tight_layout()
        plt.savefig(path, dpi=130)
        plt.close()
        paths.append(str(path))
    plt.figure(figsize=(11, 4)); plt.plot(ct, speed, label="|NavigationCommand.V|")
    for event in events(scenario, "waypoint_accepted", "goal_publish", "navigation_mode_status"):
        if time_s(event) is not None: plt.axvline(time_s(event), color="gray", alpha=.2)
    plt.xlabel("time [s]"); plt.ylabel("speed [m/s]"); plt.title("A — trajectory velocity continuity"); plt.legend(); save("plot_A_velocity_continuity.png")
    plt.figure(figsize=(11, 4)); plt.step(ct, [item.get("analytic_sample_role", 255) for item in cmds], where="post", label="analytic role")
    plt.step(ct, [item.get("trajectory_flag", item.get("role", 255)) for item in cmds], where="post", label="NavigationCommand role")
    plt.step(ct, [item.get("execution_recovery_state", 255) for item in cmds], where="post", label="recovery state")
    plt.xlabel("time [s]"); plt.ylabel("enum value"); plt.title("B — command role timeline"); plt.legend(); save("plot_B_command_roles.png")
    accepts = events(scenario, "waypoint_accepted")
    if accepts:
        plt.figure(figsize=(11, 4)); plt.plot(ct, speed, label="command speed")
        for event in accepts:
            if time_s(event) is not None: plt.axvline(time_s(event), color="tab:red", alpha=.7, label="acceptance")
        plt.xlabel("time [s]"); plt.ylabel("speed [m/s]"); plt.title("C — PASS_THROUGH handoff"); plt.legend(); save("plot_C_pass_through_handoff.png")
    setpoints = extract_px4(samples, "px4_local_position_setpoint")
    local = extract_px4(samples, "local_position")
    if setpoints or local:
        plt.figure(figsize=(11, 5)); plt.plot(ct, speed, label="planner command speed")
        if setpoints: plt.plot([time_s(x) for x in setpoints], [norm(x["payload"].get("velocity_ned")) for x in setpoints], label="PX4 setpoint speed")
        if local: plt.plot([time_s(x) for x in local], [norm([x["payload"].get("vx_ned_m_s"), x["payload"].get("vy_ned_m_s"), x["payload"].get("vz_ned_m_s")]) for x in local], label="PX4 actual speed")
        plt.xlabel("time [s]"); plt.ylabel("speed [m/s]"); plt.title("D — planner vs PX4 velocity"); plt.legend(); save("plot_D_planner_px4_velocity.png")
    plt.figure(figsize=(11, 5))
    if setpoints:
        st = [time_s(x) for x in setpoints]
        accel = [norm(x["payload"].get("acceleration_ned")) for x in setpoints]
        valid = [(t, a) for t, a in zip(st, accel) if t is not None and a is not None]
        if valid: plt.plot([x[0] for x in valid], [x[1] for x in valid], label="PX4 acceleration setpoint")
    valid_cmd_acc = [(t, norm(x.get("acceleration"))) for t, x in zip(ct, cmds) if norm(x.get("acceleration")) is not None]
    if valid_cmd_acc: plt.plot([x[0] for x in valid_cmd_acc], [x[1] for x in valid_cmd_acc], label="planner acceleration")
    plt.xlabel("time [s]"); plt.ylabel("acceleration [m/s²]"); plt.title("E — planner vs PX4 acceleration"); plt.legend(); save("plot_E_planner_px4_acceleration.png")
    lio = extract_px4(samples, "propagated_odometry")
    if lio and local:
        residual_p = []
        residual_v = []
        for x in lio:
            match = min(local, key=lambda y: abs((time_s(y) or 0.0) - (time_s(x) or 0.0)), default=None)
            lp = vector(x["payload"].get("position")); lv = vector(x["payload"].get("linear_velocity"))
            if match and lp:
                p = vector([match["payload"].get("x_ned_m"), match["payload"].get("y_ned_m"), match["payload"].get("z_ned_m")])
                if p: residual_p.append((time_s(x), math.sqrt(sum((p[i] - [lp[0], -lp[1], -lp[2]][i]) ** 2 for i in range(3)))))
            if match and lv:
                v = vector([match["payload"].get("vx_ned_m_s"), match["payload"].get("vy_ned_m_s"), match["payload"].get("vz_ned_m_s")])
                if v: residual_v.append((time_s(x), math.sqrt(sum((v[i] - [lv[0], -lv[1], -lv[2]][i]) ** 2 for i in range(3)))))
        plt.figure(figsize=(11, 4))
        if residual_p: plt.plot([x[0] for x in residual_p], [x[1] for x in residual_p], label="position residual")
        if residual_v: plt.plot([x[0] for x in residual_v], [x[1] for x in residual_v], label="velocity residual")
        plt.xlabel("time [s]"); plt.ylabel("residual norm"); plt.title("F — LIO/PX4 frame residual"); plt.legend(); save("plot_F_frame_residual.png")
    if traces:
        plt.figure(figsize=(11, 4)); tt = [x["time_s"] for x in traces if x["time_s"] is not None]; ll = [finite(trace_value(x, "planning_latency_ms")) for x in traces if x["time_s"] is not None]
        plt.plot(tt, ll, marker="."); plt.xlabel("time [s]"); plt.ylabel("solve latency [ms]"); plt.title("G — optimizer/recovery timing"); save("plot_G_optimizer_timing.png")
    return paths


def analyze(input_dir: Path, output: Path) -> dict[str, Any]:
    metadata = read_json(input_dir / "metadata.json", {})
    legacy_report = read_json(input_dir / "report.json", {})
    scenario = read_jsonl(input_dir / "scenario.jsonl")
    samples = read_jsonl(input_dir / "samples.jsonl")
    cmds = commands(scenario)
    traces = diagnostic_records(samples)
    generation_transitions = []
    previous = None
    for item in cmds:
        generation = item.get("trajectory_generation", item.get("bundle_generation"))
        if generation is not None and generation != previous:
            generation_transitions.append({"time_s": item["time_s"], "from": previous, "to": generation, "command": item})
            previous = generation
    commits = [item for item in traces if int(trace_value(item, "commit_observed_this_cycle", 0) or 0) == 1]
    analytic_residuals = {key: [finite(trace_value(item, key)) for item in commits if finite(trace_value(item, key)) is not None]
                          for key in ("splice_position_residual_m", "splice_velocity_residual_mps", "splice_acceleration_residual_mps2", "splice_jerk_residual_mps3", "splice_yaw_residual_rad", "splice_yaw_rate_residual_radps")}
    failures = [item for item in traces if int(trace_value(item, "candidate_result", 0) or 0) != 0 or int(trace_value(item, "injected_replan_failure", 0) or 0) == 1]
    failure_timelines = []
    for failure in failures:
        failure_time = failure.get("time_s")
        before = max((item for item in cmds if item["time_s"] <= (failure_time or 0.0)),
                     key=lambda item: item["time_s"], default=None)
        after = min((item for item in cmds if item["time_s"] > (failure_time or 0.0)),
                    key=lambda item: item["time_s"], default=None)
        backup_time = finite(before.get("time_to_backup_start_s")) if before else None
        premature = bool(after and after.get("safety_suffix_active") and backup_time is not None and backup_time > 0.0)
        later_nominal_retry = any(
            item.get("time_s") is not None and item["time_s"] > (failure_time or 0.0) and
            int(trace_value(item, "candidate_result", 1) or 1) == 0
            for item in traces
        )
        failure_timelines.append({
            "failure_time_s": failure_time,
            "role_before_failure": role_name(before.get("trajectory_flag")) if before else "UNKNOWN",
            "analytic_role_before_failure": role_name(before.get("analytic_sample_role")) if before else "UNKNOWN",
            "safety_suffix_before": bool(before.get("safety_suffix_active")) if before else None,
            "role_after_failure": role_name(after.get("trajectory_flag")) if after else "UNKNOWN",
            "analytic_role_after_failure": role_name(after.get("analytic_sample_role")) if after else "UNKNOWN",
            "safety_suffix_after": bool(after.get("safety_suffix_active")) if after else None,
            "recovery_state_before": state_name(before.get("execution_recovery_state")) if before else "UNKNOWN",
            "recovery_state_after": state_name(after.get("execution_recovery_state")) if after else "UNKNOWN",
            "time_to_backup_start_s_before_failure": backup_time,
            "premature_safety_takeover_before_backup": premature,
            "later_nominal_retry_observed": later_nominal_retry,
        })
    route_events = [item for item in traces if int(trace_value(item, "route_boundary_event_present", 0) or 0) == 1]
    px4 = planner_px4_metrics(cmds, samples)
    figures = make_plots(input_dir / "figures", cmds, traces, scenario, samples)
    bag_files = list((input_dir / "rosbag").glob("*.db3")) if (input_dir / "rosbag").is_dir() else []
    quality = {"scenario_records": len(scenario), "monitor_samples": len(samples), "planner_traces": len(traces),
               "commands": len(cmds), "rosbag_present": bool(bag_files),
               "missing": [name for name, count in (("scenario.jsonl", len(scenario)), ("samples.jsonl", len(samples))) if count == 0]}
    hypothesis_status = {key: "INCONCLUSIVE" for key in ("H1_splice_continuity", "H2_replanning_timing", "H3_pass_through_continuation", "H4_failed_replan_safety_takeover", "H5_corner_overconstraint", "H6_px4_controller_mismatch")}
    if any(item["premature_safety_takeover_before_backup"] for item in failure_timelines):
        hypothesis_status["H4_failed_replan_safety_takeover"] = "CONFIRMED"
    hypothesis_evidence = {key: [] for key in hypothesis_status}
    if hypothesis_status["H4_failed_replan_safety_takeover"] == "CONFIRMED":
        hypothesis_evidence["H4_failed_replan_safety_takeover"].append(
            "Injected failure was followed by a safety-suffix command while the prior command still had positive time_to_backup_start_s."
        )
    return {
        "schema_version": 1, "run": input_dir.name, "metadata": metadata,
        "runtime_verdict": legacy_report.get("verdict", "NOT_TESTED"),
        "runtime_reasons": legacy_report.get("reasons", []),
        "data_quality": quality,
        "metrics": {"generation_transitions": generation_transitions, "commit_count": len(commits),
                     "analytic_splice_residuals": {key: summary(values) for key, values in analytic_residuals.items()},
                     "planner_failure_count": len(failures), "injected_failure_count": sum(int(trace_value(x, "injected_replan_failure", 0) or 0) for x in failures),
                     "failure_timelines": failure_timelines,
                     "route_boundary_event_count": len(route_events), "route_boundary_status": "PRESENT" if route_events else "NO_ROUTE_BOUNDARY_EVENT",
                     "px4": px4, "command_roles": {role: sum(role_name(x.get("analytic_sample_role")) == role for x in cmds) for role in ("MAIN", "BACKUP", "EMERGENCY", "UNKNOWN")},
                     "recovery_states": {state_name(x.get("execution_recovery_state")): sum(state_name(y.get("execution_recovery_state")) == state_name(x.get("execution_recovery_state")) for y in cmds) for x in cmds}},
        "hypotheses": {key: {"status": hypothesis_status[key], "evidence": hypothesis_evidence[key]} for key in hypothesis_status},
        "critical_events": [{"time_s": item.get("time_s"), "type": "planner_failure", "injected": int(trace_value(item, "injected_replan_failure", 0) or 0), "candidate_result": trace_value(item, "candidate_result")} for item in failures],
        "figures": figures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.input, args.output)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Build the single report schema used by all runtime workflows."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
from typing import Any


VERDICTS = {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "OBSERVATION_COMPLETE"}


def _number(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _p(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def _git(cwd: Path, *args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=cwd, text=True, capture_output=True, check=False).stdout.strip()
    except OSError:
        return ""


def provenance(workspace: Path, px4_dir: Path | None = None) -> dict[str, Any]:
    git_status = _git(workspace, "status", "--porcelain")
    px4_commit = _git(px4_dir, "rev-parse", "HEAD") if px4_dir and px4_dir.is_dir() else ""
    px4_status = _git(px4_dir, "status", "--porcelain") if px4_dir and px4_dir.is_dir() else ""
    msgs = workspace / "src/external/px4_msgs"
    return {
        "navigation_commit": _git(workspace, "rev-parse", "HEAD"),
        "navigation_dirty": bool(git_status),
        "px4_commit": px4_commit,
        "px4_dirty": bool(px4_status),
        "px4_msgs_commit": _git(msgs, "rev-parse", "HEAD") if (msgs / ".git").exists() or (msgs / "package.xml").exists() else "",
    }


def _load_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _samples(path: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    if not path.is_file():
        return result
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if item.get("kind") == "sample":
                result.append(item)
    return result


def _diag_values(snapshot: dict[str, Any]) -> dict[str, Any]:
    diagnostics = snapshot.get("diagnostics", {})
    values = diagnostics.get("values", {})
    result = dict(values) if isinstance(values, dict) else {}
    for key in ("state", "navigation_valid", "last_failure_code", "last_failure_reason"):
        if key in diagnostics:
            result[key] = diagnostics[key]
    return result


def _stream(snapshot: dict[str, Any], name: str) -> dict[str, Any]:
    return dict(snapshot.get("streams", {}).get(name, {}))


def _process_failures(session: Path) -> list[str]:
    runtime = _load_json(session / "runtime.json", {})
    failures = list(runtime.get("failures", []))
    cleanup = _load_json(session / "state.json", {}).get("cleanup")
    if cleanup == "FAIL":
        failures.append("cleanup incomplete")
    return [str(item) for item in failures]


def _rate_row(snapshot: dict[str, Any], name: str) -> dict[str, Any]:
    row = _stream(snapshot, name)
    return {
        "sample_count": int(row.get("received", 0)),
        "mean_rate_hz": _number(row.get("mean_rate_hz")),
        "minimum_window_rate_hz": _number(row.get("minimum_window_rate_hz")),
        "p95_interval_ms": row.get("p95_interval_ms"),
        "maximum_gap_ms": _number(row.get("maximum_gap_ms")),
        "stale_event_count": int(row.get("stale_event_count", 0)),
        "stale_event_times_ns": list(row.get("stale_event_times_ns", [])),
        "timestamp_regression_count": int(row.get("timestamp_regression_count", 0)),
        "timestamp_duplicate_count": int(row.get("timestamp_duplicate_count", 0)),
        "nonfinite_message_count": int(row.get("nonfinite_message_count", 0)),
        "invalid_quaternion_count": int(row.get("invalid_quaternion_count", 0)),
        "invalid_covariance_count": int(row.get("invalid_covariance_count", 0)),
    }


def _active_stale_count(row: dict[str, Any], runtime: dict[str, Any]) -> int:
    times = row.get("stale_event_times_ns", [])
    replay_finished = runtime.get("replay_finished_wall_ns")
    if isinstance(times, list) and replay_finished:
        grace_ns = int(_number(runtime.get("replay_tail_grace_s"), 0.5) * 1e9)
        active_until = int(replay_finished) - grace_ns
        return sum(int(value) < active_until for value in times)
    return int(row.get("stale_event_count", 0))


def _metric_summary(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "mean": sum(values) / len(values) if values else None,
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)) if values else None,
        "p50": _p(values, 0.50),
        "p95": _p(values, 0.95),
        "maximum": max(values) if values else None,
    }


def _vector(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _quaternion(value: Any) -> tuple[float, float, float, float] | None:
    if not isinstance(value, list) or len(value) != 4:
        return None
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(sum(item * item for item in result))
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    return tuple(item / norm for item in result)


def _quaternion_inverse(value: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (value[0], -value[1], -value[2], -value[3])


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quaternion_normalize(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(item * item for item in value))
    return tuple(item / norm for item in value)


def _quaternion_rotate(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    pure = (0.0, vector[0], vector[1], vector[2])
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, pure), _quaternion_inverse(quaternion)
    )
    return (rotated[1], rotated[2], rotated[3])


def _quaternion_angle(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    dot = abs(sum(left[index] * right[index] for index in range(4)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _quaternion_heading(
    quaternion: tuple[float, float, float, float],
) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _map_point_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    values: list[float] = []
    for item in _series(samples, "diagnostics"):
        statuses = item.get("payload", {}).get("statuses", [])
        if not isinstance(statuses, list):
            continue
        for status in statuses:
            if not isinstance(status, dict) or status.get("name") != "fast_lio/estimator":
                continue
            value = status.get("values", {}).get("map_point_count")
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number) and number >= 0.0:
                values.append(number)
    return {
        "sample_count": len(values),
        "minimum": min(values) if values else None,
        "maximum": max(values) if values else None,
        "mean": sum(values) / len(values) if values else None,
        "p95": _p(values, 0.95),
        "final": values[-1] if values else None,
    }


def _series(samples: list[dict[str, Any]], stream: str) -> list[dict[str, Any]]:
    return [item for item in samples if item.get("stream") == stream]


def _diagnostic_states(samples: list[dict[str, Any]]) -> list[str]:
    states: list[str] = []
    for item in _series(samples, "diagnostics"):
        payload = item.get("payload", {})
        values = payload.get("values", {})
        state = values.get("state", values.get("status"))
        if state is not None:
            states.append(str(state).upper())
    return states


def _sample_time(item: dict[str, Any]) -> int:
    payload = item.get("payload", {})
    if "timestamp_sample_us" in payload and payload.get("timestamp_sample_us"):
        return int(payload["timestamp_sample_us"]) * 1000
    return int(item.get("timestamp_ns", 0))


def _match(a: list[dict[str, Any]], b: list[dict[str, Any]], tolerance_ns: int) -> list[tuple[dict[str, Any], dict[str, Any], int]]:
    if not a or not b:
        return []
    a0 = _sample_time(a[0])
    b0 = _sample_time(b[0])
    normalized_b = [(item, _sample_time(item) - b0) for item in b]
    result: list[tuple[dict[str, Any], dict[str, Any], int]] = []
    index = 0
    for left in a:
        target = _sample_time(left) - a0
        while index + 1 < len(normalized_b) and normalized_b[index + 1][1] <= target:
            index += 1
        candidates = normalized_b[max(0, index - 1) : min(len(normalized_b), index + 2)]
        if not candidates:
            continue
        right, right_time = min(candidates, key=lambda pair: abs(pair[1] - target))
        delta = target - right_time
        if abs(delta) <= tolerance_ns:
            result.append((left, right, delta))
    return result


def _residuals(samples: list[dict[str, Any]], tolerance_ms: float) -> dict[str, Any]:
    # Compare the exact converted LIO message sent to PX4.  The raw ROS
    # propagated message is local Z-up/FLU while PX4 estimator odometry is
    # reported in NED or FRD, so direct component subtraction is invalid.
    lio = _series(samples, "external_odometry")
    px4 = _series(samples, "px4_odometry")
    matches = _match(lio, px4, int(tolerance_ms * 1e6))
    position: list[float] = []
    velocity: list[float] = []
    yaw: list[float] = []
    attitude: list[float] = []
    deltas = [abs(delta) / 1e6 for _, _, delta in matches]
    alignment: tuple[float, float, float, float] | None = None
    first_lio_position: tuple[float, float, float] | None = None
    first_px4_position: tuple[float, float, float] | None = None
    for left, right, _ in matches:
        lp = _vector(left.get("payload", {}).get("position"))
        rp = _vector(right.get("payload", {}).get("position"))
        lq = _quaternion(left.get("payload", {}).get("q_wxyz"))
        rq = _quaternion(right.get("payload", {}).get("q_wxyz"))
        if lp is None or rp is None or lq is None or rq is None:
            continue
        if alignment is None:
            # Estimate the constant FRD-to-PX4-world heading/attitude offset
            # from the first matched pose.  PX4's local origin is also removed
            # below, so arbitrary startup origins do not look like drift.
            alignment = _quaternion_normalize(_quaternion_multiply(rq, _quaternion_inverse(lq)))
            first_lio_position = lp
            first_px4_position = rp
        assert first_lio_position is not None
        assert first_px4_position is not None
        aligned_position = _quaternion_rotate(
            alignment,
            (
                lp[0] - first_lio_position[0],
                lp[1] - first_lio_position[1],
                lp[2] - first_lio_position[2],
            ),
        )
        position_error = (
            aligned_position[0] - (rp[0] - first_px4_position[0]),
            aligned_position[1] - (rp[1] - first_px4_position[1]),
            aligned_position[2] - (rp[2] - first_px4_position[2]),
        )
        position.append(math.sqrt(sum(value * value for value in position_error)))

        lio_body_velocity = _vector(left.get("payload", {}).get("velocity"))
        px4_velocity = _vector(right.get("payload", {}).get("velocity"))
        if lio_body_velocity is not None and px4_velocity is not None:
            lio_world_velocity = _quaternion_rotate(lq, lio_body_velocity)
            velocity_frame = int(right.get("payload", {}).get("velocity_frame", 0))
            predicted_velocity = (
                lio_body_velocity
                if velocity_frame == 3
                else _quaternion_rotate(alignment, lio_world_velocity)
            )
            velocity_error = tuple(
                predicted_velocity[index] - px4_velocity[index] for index in range(3)
            )
            velocity.append(math.sqrt(sum(value * value for value in velocity_error)))

        aligned_lio_orientation = _quaternion_multiply(alignment, lq)
        orientation_error = _quaternion_normalize(
            _quaternion_multiply(rq, _quaternion_inverse(aligned_lio_orientation))
        )
        attitude.append(_quaternion_angle(rq, aligned_lio_orientation))
        yaw.append(abs(_quaternion_heading(orientation_error)))
    return {
        "source": "lio/external_odometry_input vs px4/estimator_odometry",
        "frame_alignment": "first matched pose; LIO FRD aligned to PX4 pose frame",
        "origin_alignment": "first matched position removed from both streams",
        "maximum_synchronization_tolerance_ms": tolerance_ms,
        "matched_sample_count": len(matches),
        "unmatched_sample_count": max(0, len(lio) - len(matches)),
        "mean_timestamp_delta_ms": sum(deltas) / len(deltas) if deltas else None,
        "p95_timestamp_delta_ms": _p(deltas, 0.95),
        "maximum_timestamp_delta_ms": max(deltas) if deltas else None,
        "position": _metric_summary(position),
        "velocity": _metric_summary(velocity),
        "attitude": _metric_summary(attitude),
        "yaw": _metric_summary(yaw),
        "circular_comparison": True,
        "pre_fusion": "NOT_AVAILABLE",
        "fusion_enabled": "OBSERVED_ONLY",
    }


def _dataset_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path) -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    streams = {name: _rate_row(snapshot, name) for name in ("imu", "lidar", "corrected_odometry", "propagated_odometry")}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    diagnostic_states = _diagnostic_states(samples)
    map_point_count = _map_point_summary(samples)
    reasons: list[str] = []
    minimum_fraction = _number(thresholds.get("minimum_rate_fraction"), 0.90)
    for name, row in streams.items():
        if row["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        expected = _number(config.get("runtime", {}).get("streams", {}).get(name, {}).get("expected_hz"))
        if expected and row["mean_rate_hz"] < expected * minimum_fraction:
            reasons.append(f"{name} rate below contract")
        if row["timestamp_regression_count"] or _active_stale_count(row, runtime) or row["nonfinite_message_count"]:
            reasons.append(f"{name} timestamp/freshness/validity violation")
    diagnostics = _diag_values(snapshot)
    tracking_observed = "TRACKING" in diagnostic_states or str(diagnostics.get("state", "")).upper() == "TRACKING"
    if not tracking_observed:
        reasons.append("LIO did not finish in TRACKING")
    if int(diagnostics.get("current_queue_depth", diagnostics.get("current_input_queue_depth", 0)) or 0) != 0:
        reasons.append("input queue did not drain")
    if int(diagnostics.get("imu_drop_count", 0) or 0) or int(diagnostics.get("lidar_drop_count", 0) or 0):
        reasons.append("runtime drop count is non-zero")
    reasons.extend(failures)
    verdict = "PASS" if not reasons else "FAIL"
    if not any(streams[name]["sample_count"] for name in streams):
        verdict = "BLOCKED"
    return {
        "workflow": "dataset",
        "verdict": verdict,
        "reasons": reasons,
        "dataset": config.get("runtime", {}).get("dataset", ""),
        "rate": config.get("runtime", {}).get("replay_rate", 1.0),
        "streams": streams,
        "lio": {
            "state": diagnostics.get("state", "NOT_AVAILABLE"),
            "tracking_observed": tracking_observed,
            "navigation_valid": diagnostics.get("navigation_valid", False),
            "last_failure_code": diagnostics.get("last_failure_code", "NOT_AVAILABLE"),
            "last_failure_reason": diagnostics.get("last_failure_reason", ""),
            "time_to_tracking_s": diagnostics.get("time_to_tracking_s", "NOT_AVAILABLE"),
            "tracking_percentage": diagnostics.get("tracking_percentage", "NOT_AVAILABLE"),
            "lost_count": diagnostics.get("lost_count", "NOT_AVAILABLE"),
            "correction_accepted": diagnostics.get("correction_success_count", diagnostics.get("correction_accepted_count", "NOT_AVAILABLE")),
            "correction_rejected": diagnostics.get("correction_failure_count", diagnostics.get("correction_rejected_count", "NOT_AVAILABLE")),
            "queue_maximum": diagnostics.get("maximum_queue_depth", "NOT_AVAILABLE"),
            "processing_lag_maximum": diagnostics.get("maximum_processing_lag_ms", diagnostics.get("processing_lag_ns", "NOT_AVAILABLE")),
            "scan_processing_p50_us": diagnostics.get("p50_scan_processing_us", "NOT_AVAILABLE"),
            "scan_processing_p95_us": diagnostics.get("p95_scan_processing_us", "NOT_AVAILABLE"),
            "scan_processing_p99_us": diagnostics.get("p99_scan_processing_us", "NOT_AVAILABLE"),
            "map_point_count": map_point_count,
        },
        "accuracy": "NOT_AVAILABLE",
        "provenance": provenance(workspace),
    }


def _sim_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path, px4_dir: Path | None) -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    names = ("imu", "lidar", "corrected_odometry", "propagated_odometry", "external_odometry", "px4_odometry", "vehicle_status", "local_position", "estimator_status_flags", "aid_ev_pos", "aid_ev_vel", "aid_ev_yaw")
    streams = {name: _rate_row(snapshot, name) for name in names}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    diagnostic_states = _diagnostic_states(samples)
    map_point_count = _map_point_summary(samples)
    reasons: list[str] = []
    for name in ("imu", "lidar", "corrected_odometry", "propagated_odometry", "external_odometry"):
        if streams[name]["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        if streams[name]["timestamp_regression_count"] or _active_stale_count(streams[name], runtime) or streams[name]["nonfinite_message_count"]:
            reasons.append(f"{name} timestamp/freshness/validity violation")
    external_expected = _number(config.get("runtime", {}).get("streams", {}).get("external_odometry", {}).get("expected_hz"))
    if external_expected and streams["external_odometry"]["mean_rate_hz"] < external_expected * _number(thresholds.get("minimum_rate_fraction"), 0.90):
        reasons.append("external odometry rate below contract")
    if streams["estimator_status_flags"]["sample_count"] <= 0 or streams["aid_ev_pos"]["sample_count"] <= 0:
        reasons.append("PX4 estimator aid-source verification is BLOCKED")
    diagnostics = _diag_values(snapshot)
    tracking_observed = "TRACKING" in diagnostic_states or str(diagnostics.get("state", "")).upper() == "TRACKING"
    if not tracking_observed:
        reasons.append("LIO did not finish in TRACKING")
    residuals = _residuals(samples, _number(thresholds.get("maximum_synchronization_tolerance_ms"), 20.0))
    scenario = _load_json(session / "scenario.json", {})
    scenario_reasons = list(scenario.get("failures", []))
    reasons.extend(str(item) for item in scenario_reasons)
    reasons.extend(failures)
    blocked = streams["estimator_status_flags"]["sample_count"] <= 0 or streams["aid_ev_pos"]["sample_count"] <= 0
    verdict = "BLOCKED" if blocked else ("PASS" if not reasons else "FAIL")
    return {
        "workflow": "sim",
        "verdict": verdict,
        "reasons": reasons,
        "streams": streams,
        "px4": {
            "estimator_initialized": streams["estimator_status_flags"]["sample_count"] > 0,
            "local_position_valid": bool(snapshot.get("latest", {}).get("local_position", {}).get("xy_valid")) and bool(snapshot.get("latest", {}).get("local_position", {}).get("z_valid")),
            "local_velocity_valid": bool(snapshot.get("latest", {}).get("local_position", {}).get("v_xy_valid")) and bool(snapshot.get("latest", {}).get("local_position", {}).get("v_z_valid")),
            "external_vision_position_fused": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_pos")),
            "external_vision_velocity_fused": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_vel")),
            "external_vision_yaw_fused": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_yaw")),
            "dead_reckoning_events": int(bool(snapshot.get("latest", {}).get("local_position", {}).get("dead_reckoning"))),
            "estimator_fault_events": int(bool(snapshot.get("latest", {}).get("estimator_status", {}).get("filter_fault_flags"))),
            "fusion_verification": "BLOCKED" if blocked else "OBSERVED",
            "aid_sources": {name: streams[name] for name in ("aid_ev_pos", "aid_ev_vel", "aid_ev_yaw")},
        },
        "lio": {
            "state": diagnostics.get("state", "NOT_AVAILABLE"),
            "tracking_observed": tracking_observed,
            "map_point_count": map_point_count,
        },
        "residuals": residuals,
        "offboard": scenario,
        "provenance": provenance(workspace, px4_dir),
    }


def build(session: Path, workflow: str, config_path: Path, workspace: Path, px4_dir: Path | None = None, observation_complete: bool = False) -> dict[str, Any]:
    import yaml
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    common_path = config_path.parent / "common.yaml"
    if common_path.is_file():
        common = yaml.safe_load(common_path.read_text(encoding="utf-8"))
        if isinstance(common, dict):
            config["runtime"] = dict(common.get("runtime", {}))
            config["runtime"].update(config.get("runtime_overrides", {}))
    runtime_metadata = _load_json(session / "runtime.json", {})
    if isinstance(runtime_metadata, dict):
        if runtime_metadata.get("dataset"):
            config["runtime"]["dataset"] = runtime_metadata["dataset"]
        if runtime_metadata.get("rate") is not None:
            config["runtime"]["replay_rate"] = runtime_metadata["rate"]
    snapshot = _load_json(session / "monitor.json", {})
    if workflow == "dataset":
        report = _dataset_report(session, config, snapshot, workspace)
    else:
        report = _sim_report(session, config, snapshot, workspace, px4_dir)
    if observation_complete:
        report["verdict"] = "OBSERVATION_COMPLETE" if not _process_failures(session) else "FAIL"
        report["reasons"] = _process_failures(session)
    if report["verdict"] not in VERDICTS:
        raise ValueError(f"invalid runtime verdict: {report['verdict']}")
    report["session"] = str(session.resolve())
    report["schema_version"] = 1
    (session / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (session / "REPORT.md").write_text(render(report), encoding="utf-8")
    return report


def _json(value: Any) -> str:
    return json.dumps(value, indent=2, sort_keys=True)


def render(report: dict[str, Any]) -> str:
    lines = [f"# Runtime report: {report.get('workflow', 'unknown')}", "", f"- Verdict: **{report.get('verdict')}**", f"- Session: `{report.get('session', '')}`", ""]
    reasons = report.get("reasons", [])
    lines += ["## Reasons", ""] + ([f"- {reason}" for reason in reasons] if reasons else ["- none"]) + ["", "## Stream metrics", "", "| Stream | Samples | Mean Hz | Min window Hz | p95 interval ms | Max gap ms | Stale | Regressions |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name, row in report.get("streams", {}).items():
        lines.append(f"| {name} | {row.get('sample_count', 0)} | {_number(row.get('mean_rate_hz')):.3f} | {_number(row.get('minimum_window_rate_hz')):.3f} | {row.get('p95_interval_ms', 'n/a')} | {_number(row.get('maximum_gap_ms')):.3f} | {row.get('stale_event_count', 0)} | {row.get('timestamp_regression_count', 0)} |")
    for section in ("lio", "px4", "residuals", "offboard", "provenance"):
        if section in report:
            lines += ["", f"## {section}", "", "```json", _json(report[section]), "```"]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--workflow", choices=("dataset", "sim"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--px4-dir", type=Path)
    parser.add_argument("--observation-complete", action="store_true")
    args = parser.parse_args()
    report = build(args.session.resolve(), args.workflow, args.config.resolve(), args.workspace.resolve(), args.px4_dir, args.observation_complete)
    print(report["verdict"])
    return 0 if report["verdict"] in {"PASS", "OBSERVATION_COMPLETE"} else 1


if __name__ == "__main__":
    raise SystemExit(main())

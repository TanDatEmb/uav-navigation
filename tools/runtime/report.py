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


def _first_tracking_wall_ns(samples: list[dict[str, Any]]) -> int | None:
    tracking_times: list[int] = []
    for item in _series(samples, "diagnostics"):
        payload = item.get("payload", {})
        values = payload.get("values", {})
        state = values.get("state", values.get("status"))
        if str(state).upper() != "TRACKING":
            continue
        try:
            arrival_ns = int(item.get("arrival_wall_ns", 0))
        except (TypeError, ValueError):
            arrival_ns = 0
        if arrival_ns > 0:
            tracking_times.append(arrival_ns)
    return min(tracking_times) if tracking_times else None


def _active_stale_times(
    row: dict[str, Any],
    runtime: dict[str, Any],
    samples: list[dict[str, Any]] | None = None,
) -> list[int] | None:
    raw_times = row.get("stale_event_times_ns", [])
    if not isinstance(raw_times, list):
        return None
    if not raw_times and int(row.get("stale_event_count", 0)):
        # Older or partial monitor snapshots may retain the aggregate but not
        # the individual event times. The caller must then fail closed.
        return None
    times: list[int] = []
    for value in raw_times:
        try:
            times.append(int(value))
        except (TypeError, ValueError):
            return None
    first_tracking_ns = _first_tracking_wall_ns(samples or [])
    # A stream can pause while LIO is still collecting its startup state. That
    # is not a tracking-time freshness violation; if TRACKING is never
    # observed, the report already fails the explicit TRACKING contract.
    active_times = [
        int(value)
        for value in times
        if first_tracking_ns is None or int(value) >= first_tracking_ns
    ]
    replay_finished = runtime.get("replay_finished_wall_ns")
    if replay_finished:
        grace_ns = int(_number(runtime.get("replay_tail_grace_s"), 0.5) * 1e9)
        active_until = int(replay_finished) - grace_ns
        return [value for value in active_times if value < active_until]
    return active_times


def _active_stale_count(
    row: dict[str, Any],
    runtime: dict[str, Any],
    samples: list[dict[str, Any]] | None = None,
) -> int:
    active_times = _active_stale_times(row, runtime, samples)
    return len(active_times) if active_times is not None else int(row.get("stale_event_count", 0))


def _stale_classification(
    name: str,
    row: dict[str, Any],
    config: dict[str, Any],
    runtime: dict[str, Any],
    samples: list[dict[str, Any]],
) -> dict[str, Any]:
    """Separate source-time loss from a delayed monitor callback.

    ``arrival_wall_ns`` is when this observer dispatched a callback, not when
    DDS delivered the sample. A callback gap alone is therefore retained as a
    diagnostic, but is not treated as a source-data outage when consecutive
    message timestamps around it remain within the stream freshness budget.
    Missing bracketing samples or a source-time gap still fail closed.
    """
    active_times = _active_stale_times(row, runtime, samples)
    raw_count = int(row.get("stale_event_count", 0))
    if active_times is None:
        return {
            "active_callback_stall_count": raw_count,
            "source_stale_event_count": raw_count,
            "observer_dispatch_stall_count": 0,
            "maximum_observer_dispatch_source_gap_ms": None,
        }
    result = {
        "active_callback_stall_count": len(active_times),
        "source_stale_event_count": 0,
        "observer_dispatch_stall_count": 0,
        "maximum_observer_dispatch_source_gap_ms": None,
    }
    if not active_times:
        return result

    runtime_config = config.get("runtime", {})
    stream_config = runtime_config.get("streams", {}).get(name, {})
    stale_after_s = _number(
        stream_config.get(
            "stale_after_s", runtime_config.get("thresholds", {}).get("stale_after_s", 1.0)
        )
    )
    stale_after_ns = int(stale_after_s * 1e9)
    records: list[tuple[int, int]] = []
    for item in _series(samples, name):
        try:
            arrival_ns = int(item.get("arrival_wall_ns", 0))
            timestamp_ns = int(item.get("timestamp_ns", 0))
        except (TypeError, ValueError):
            continue
        if arrival_ns > 0:
            records.append((arrival_ns, timestamp_ns))
    records.sort()
    if stale_after_ns <= 0 or not records:
        result["source_stale_event_count"] = len(active_times)
        return result

    maximum_source_gap_ms: float | None = None
    next_index = 0
    for event_ns in sorted(active_times):
        while next_index < len(records) and records[next_index][0] <= event_ns:
            next_index += 1
        before = records[next_index - 1] if next_index else None
        after = records[next_index] if next_index < len(records) else None
        if before is None or after is None:
            result["source_stale_event_count"] += 1
            continue
        source_gap_ns = after[1] - before[1]
        if before[1] <= 0 or after[1] <= 0 or source_gap_ns <= 0 or source_gap_ns > stale_after_ns:
            result["source_stale_event_count"] += 1
            continue
        result["observer_dispatch_stall_count"] += 1
        source_gap_ms = source_gap_ns / 1e6
        maximum_source_gap_ms = (
            source_gap_ms
            if maximum_source_gap_ms is None
            else max(maximum_source_gap_ms, source_gap_ms)
        )
    result["maximum_observer_dispatch_source_gap_ms"] = maximum_source_gap_ms
    return result


def _annotate_stale_classification(
    streams: dict[str, dict[str, Any]],
    config: dict[str, Any],
    runtime: dict[str, Any],
    samples: list[dict[str, Any]],
) -> None:
    for name, row in streams.items():
        row.update(_stale_classification(name, row, config, runtime, samples))


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
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(sum(item * item for item in result))
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    return tuple(item / norm for item in result)


def _quaternion_xyzw(value: Any) -> tuple[float, float, float, float] | None:
    """Read a ROS x,y,z,w quaternion into the report's w,x,y,z order."""
    if not isinstance(value, list) or len(value) != 4:
        return None
    return _quaternion([value[3], value[0], value[1], value[2]])


_C_NED_FROM_ENU = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_C_FRD_FROM_FLU = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def _matrix_multiply(
    left: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    right: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _rotation_matrix_from_quaternion(
    value: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    w, x, y, z = value
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _quaternion_from_rotation_matrix(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[float, float, float, float]:
    # The matrices used here are proper rotations (ENU->NED and FLU->FRD
    # both have determinant +1), so the standard stable branch is sufficient.
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = (
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        result = (
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        result = (
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        result = (
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        )
    return _quaternion(result) or (1.0, 0.0, 0.0, 0.0)


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


def _map_maintenance_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize geometric map maintenance and emergency guard events."""
    maintenance_us: list[float] = []
    guard_trigger_count = 0
    recovery_failure_count = 0
    for item in _series(samples, "diagnostics"):
        statuses = item.get("payload", {}).get("statuses", [])
        if not isinstance(statuses, list):
            continue
        for status in statuses:
            if not isinstance(status, dict) or status.get("name") != "fast_lio/estimator":
                continue
            values = status.get("values", {})
            if not isinstance(values, dict):
                continue
            guard_trigger_count += int(bool(values.get("absolute_guard_triggered", False)))
            recovery_failure_count += int(bool(values.get("absolute_guard_recovery_failed", False)))
            maintenance = _number(values.get("map_maintenance_us"), -1.0)
            if maintenance >= 0.0:
                maintenance_us.append(maintenance)
    return {
        "absolute_guard_trigger_count": guard_trigger_count,
        "absolute_guard_recovery_failure_count": recovery_failure_count,
        "maximum_maintenance_us": max(maintenance_us) if maintenance_us else None,
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
    # These streams share the ROS/PX4 simulation epoch. Never normalize each
    # stream to its own first sample: startup latency is real and subtracting
    # it pairs samples from different physical times.
    timestamped_b = [(item, _sample_time(item)) for item in b]
    result: list[tuple[dict[str, Any], dict[str, Any], int]] = []
    index = 0
    for left in a:
        target = _sample_time(left)
        while index + 1 < len(timestamped_b) and timestamped_b[index + 1][1] <= target:
            index += 1
        candidates = timestamped_b[max(0, index - 1) : min(len(timestamped_b), index + 2)]
        if not candidates:
            continue
        right, right_time = min(candidates, key=lambda pair: abs(pair[1] - target))
        delta = target - right_time
        if abs(delta) <= tolerance_ns:
            result.append((left, right, delta))
    return result


def _residuals(samples: list[dict[str, Any]], tolerance_ms: float) -> dict[str, Any]:
    # Compare the exact converted LIO message sent to PX4. The raw ROS
    # propagated message is checked separately by _frame_contract_residuals.
    # PX4 estimator odometry may be NED or FRD, so direct component subtraction
    # is invalid; the first matched attitude is used only for the estimator
    # residual after absolute timestamp matching.
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

        lio_velocity = _vector(left.get("payload", {}).get("velocity"))
        px4_velocity = _vector(right.get("payload", {}).get("velocity"))
        if lio_velocity is not None and px4_velocity is not None:
            velocity_frame = int(right.get("payload", {}).get("velocity_frame", 0))
            predicted_velocity = (
                _quaternion_rotate(alignment, lio_velocity)
                if velocity_frame == 1
                else _quaternion_rotate(
                    alignment, _quaternion_rotate(lq, lio_velocity)
                )
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
        "timestamp_alignment": "absolute timestamp_sample; no per-stream epoch normalization",
        "first_external_sample_time_ns": _sample_time(lio[0]) if lio else None,
        "first_px4_sample_time_ns": _sample_time(px4[0]) if px4 else None,
        "initial_stream_epoch_offset_ms": (
            (_sample_time(lio[0]) - _sample_time(px4[0])) / 1e6
            if lio and px4
            else None
        ),
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


def _matrix_vector(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))  # type: ignore[return-value]


def _frame_contract_residuals(samples: list[dict[str, Any]], tolerance_ms: float) -> dict[str, Any]:
    """Check every converted vector/quaternion against the one frame contract."""
    lio = _series(samples, "propagated_odometry")
    external = _series(samples, "external_odometry")
    matches = _match(lio, external, int(tolerance_ms * 1e6))
    position: list[float] = []
    velocity: list[float] = []
    angular_velocity: list[float] = []
    attitude: list[float] = []
    frame_contract_violations = 0
    deltas = [abs(delta) / 1e6 for _, _, delta in matches]
    for left, right, _ in matches:
        raw = left.get("payload", {})
        converted = right.get("payload", {})
        raw_position = _vector(raw.get("position"))
        converted_position = _vector(converted.get("position"))
        raw_velocity = _vector(raw.get("linear_velocity"))
        converted_velocity = _vector(converted.get("velocity"))
        raw_angular = _vector(raw.get("angular_velocity"))
        converted_angular = _vector(converted.get("angular_velocity"))
        raw_orientation = _quaternion_xyzw(raw.get("q_xyzw"))
        converted_orientation = _quaternion(converted.get("q_wxyz"))
        if int(converted.get("pose_frame", 0)) != 1 or int(converted.get("velocity_frame", 0)) != 1:
            frame_contract_violations += 1
        if raw_position is not None and converted_position is not None:
            expected = _matrix_vector(_C_NED_FROM_ENU, raw_position)
            position.append(math.sqrt(sum((expected[i] - converted_position[i]) ** 2 for i in range(3))))
        if raw_velocity is not None and converted_velocity is not None and raw_orientation is not None:
            velocity_enu = _quaternion_rotate(raw_orientation, raw_velocity)
            expected = _matrix_vector(_C_NED_FROM_ENU, velocity_enu)
            velocity.append(math.sqrt(sum((expected[i] - converted_velocity[i]) ** 2 for i in range(3))))
        if raw_angular is not None and converted_angular is not None:
            expected = _matrix_vector(_C_FRD_FROM_FLU, raw_angular)
            angular_velocity.append(math.sqrt(sum((expected[i] - converted_angular[i]) ** 2 for i in range(3))))
        if raw_orientation is not None and converted_orientation is not None:
            expected_matrix = _matrix_multiply(
                _matrix_multiply(_C_NED_FROM_ENU, _rotation_matrix_from_quaternion(raw_orientation)),
                _C_FRD_FROM_FLU,
            )
            expected_orientation = _quaternion_from_rotation_matrix(expected_matrix)
            attitude.append(_quaternion_angle(expected_orientation, converted_orientation))
    return {
        "source": "lio/propagated_odometry -> px4/external_odometry",
        "world_transform": "x_px4_ned=y_lio_enu; y_px4_ned=x_lio_enu; z_px4_ned=-z_lio_enu",
        "body_transform": "x_frd=x_flu; y_frd=-y_flu; z_frd=-z_flu",
        "pose_frame": "PX4 POSE_FRAME_NED",
        "velocity_frame": "PX4 VELOCITY_FRAME_NED",
        "frame_contract_violation_count": frame_contract_violations,
        "timestamp_alignment": "absolute ROS header stamp vs PX4 timestamp_sample",
        "matched_sample_count": len(matches),
        "unmatched_sample_count": max(0, len(lio) - len(matches)),
        "mean_timestamp_delta_ms": sum(deltas) / len(deltas) if deltas else None,
        "p95_timestamp_delta_ms": _p(deltas, 0.95),
        "maximum_timestamp_delta_ms": max(deltas) if deltas else None,
        "position": _metric_summary(position),
        "velocity": _metric_summary(velocity),
        "angular_velocity": _metric_summary(angular_velocity),
        "attitude": _metric_summary(attitude),
    }


def _vector_metric_summary(errors: list[tuple[float, float, float]]) -> dict[str, Any]:
    return {
        "norm": _metric_summary([
            math.sqrt(sum(component * component for component in error)) for error in errors
        ]),
        "x": _metric_summary([abs(error[0]) for error in errors]),
        "y": _metric_summary([abs(error[1]) for error in errors]),
        "z": _metric_summary([abs(error[2]) for error in errors]),
    }


def _ground_truth_residuals(samples: list[dict[str, Any]], tolerance_ms: float) -> dict[str, Any]:
    """Compare LIO and PX4 input to independent Gazebo ENU/FLU odometry."""
    ground_truth = _series(samples, "ground_truth_odometry")
    lio = _series(samples, "propagated_odometry")
    external = _series(samples, "external_odometry")
    px4 = _series(samples, "px4_odometry")
    tolerance_ns = int(tolerance_ms * 1e6)
    lio_matches = _match(ground_truth, lio, tolerance_ns)
    external_matches = _match(ground_truth, external, tolerance_ns)
    px4_matches = _match(ground_truth, px4, tolerance_ns)
    lio_position_errors: list[tuple[float, float, float]] = []
    external_position_errors: list[tuple[float, float, float]] = []
    lio_velocity_errors: list[tuple[float, float, float]] = []
    external_velocity_errors: list[tuple[float, float, float]] = []
    lio_angular_errors: list[tuple[float, float, float]] = []
    external_angular_errors: list[tuple[float, float, float]] = []
    px4_position_errors: list[tuple[float, float, float]] = []
    px4_velocity_errors: list[tuple[float, float, float]] = []
    px4_angular_errors: list[tuple[float, float, float]] = []
    lio_attitude: list[float] = []
    external_attitude: list[float] = []
    px4_attitude: list[float] = []
    lio_deltas = [abs(delta) / 1e6 for _, _, delta in lio_matches]
    external_deltas = [abs(delta) / 1e6 for _, _, delta in external_matches]
    px4_deltas = [abs(delta) / 1e6 for _, _, delta in px4_matches]
    first_gt_position: tuple[float, float, float] | None = None
    first_lio_position: tuple[float, float, float] | None = None
    first_gt_position_external: tuple[float, float, float] | None = None
    first_external_position: tuple[float, float, float] | None = None
    first_gt_position_px4: tuple[float, float, float] | None = None
    first_px4_position: tuple[float, float, float] | None = None
    first_lio_attitude: float | None = None
    first_external_attitude: float | None = None
    first_px4_attitude: float | None = None
    gt_child_frames: set[str] = set()

    for gt_item, estimate_item, _ in lio_matches:
        gt = gt_item.get("payload", {})
        estimate = estimate_item.get("payload", {})
        gt_position = _vector(gt.get("position"))
        estimate_position = _vector(estimate.get("position"))
        gt_orientation = _quaternion_xyzw(gt.get("q_xyzw"))
        estimate_orientation = _quaternion_xyzw(estimate.get("q_xyzw"))
        gt_velocity = _vector(gt.get("linear_velocity"))
        estimate_velocity = _vector(estimate.get("linear_velocity"))
        gt_angular = _vector(gt.get("angular_velocity"))
        estimate_angular = _vector(estimate.get("angular_velocity"))
        gt_child_frames.add(str(gt.get("child_frame_id", "")))
        if gt_position is not None and estimate_position is not None:
            if first_gt_position is None:
                first_gt_position = gt_position
                first_lio_position = estimate_position
            assert first_lio_position is not None
            lio_position_errors.append(tuple(
                (estimate_position[index] - first_lio_position[index]) -
                (gt_position[index] - first_gt_position[index])
                for index in range(3)
            ))
        if gt_orientation is not None and estimate_orientation is not None:
            angle = _quaternion_angle(estimate_orientation, gt_orientation)
            lio_attitude.append(angle)
            if first_lio_attitude is None:
                first_lio_attitude = angle
        if gt_velocity is not None and estimate_velocity is not None:
            # nav_msgs/Odometry.twist is expressed in child_frame_id.  The
            # simulator ground truth and LIO both publish a body-frame twist;
            # compare them directly.  Only the PX4 branch below rotates this
            # body-FLU vector into the ENU world before ENU->NED conversion.
            expected_velocity = gt_velocity
            lio_velocity_errors.append(tuple(
                estimate_velocity[index] - expected_velocity[index] for index in range(3)
            ))
        if gt_angular is not None and estimate_angular is not None:
            expected_angular = (
                gt_angular if str(gt.get("child_frame_id", "")) != "base_link" else gt_angular
            )
            lio_angular_errors.append(tuple(
                estimate_angular[index] - expected_angular[index] for index in range(3)
            ))

    for gt_item, estimate_item, _ in external_matches:
        gt = gt_item.get("payload", {})
        estimate = estimate_item.get("payload", {})
        gt_position = _vector(gt.get("position"))
        estimate_position = _vector(estimate.get("position"))
        gt_orientation = _quaternion_xyzw(gt.get("q_xyzw"))
        estimate_orientation = _quaternion(estimate.get("q_wxyz"))
        gt_velocity = _vector(gt.get("linear_velocity"))
        estimate_velocity = _vector(estimate.get("velocity"))
        gt_angular = _vector(gt.get("angular_velocity"))
        estimate_angular = _vector(estimate.get("angular_velocity"))
        gt_child_frames.add(str(gt.get("child_frame_id", "")))
        if gt_position is not None and estimate_position is not None:
            if first_gt_position_external is None:
                first_gt_position_external = gt_position
            if first_external_position is None:
                first_external_position = estimate_position
            assert first_external_position is not None
            assert first_gt_position_external is not None
            expected_delta = _matrix_vector(
                _C_NED_FROM_ENU,
                tuple(gt_position[index] - first_gt_position_external[index] for index in range(3)),
            )
            actual_delta = tuple(
                estimate_position[index] - first_external_position[index] for index in range(3)
            )
            external_position_errors.append(tuple(
                actual_delta[index] - expected_delta[index] for index in range(3)
            ))
        if gt_orientation is not None and estimate_orientation is not None:
            expected_matrix = _matrix_multiply(
                _matrix_multiply(_C_NED_FROM_ENU, _rotation_matrix_from_quaternion(gt_orientation)),
                _C_FRD_FROM_FLU,
            )
            expected_orientation = _quaternion_from_rotation_matrix(expected_matrix)
            angle = _quaternion_angle(estimate_orientation, expected_orientation)
            external_attitude.append(angle)
            if first_external_attitude is None:
                first_external_attitude = angle
        if gt_velocity is not None and estimate_velocity is not None:
            # OdometryPublisher publishes linear velocity in child_frame_id,
            # which is the robot body frame.  Convert body-FLU -> ENU using
            # the ground-truth body-to-world attitude, then ENU -> PX4 NED.
            velocity_enu = (
                _quaternion_rotate(gt_orientation, gt_velocity)
                if gt_orientation is not None
                else gt_velocity
            )
            expected_velocity = _matrix_vector(_C_NED_FROM_ENU, velocity_enu)
            external_velocity_errors.append(tuple(
                estimate_velocity[index] - expected_velocity[index] for index in range(3)
            ))
        if gt_angular is not None and estimate_angular is not None:
            expected_angular = _matrix_vector(_C_FRD_FROM_FLU, gt_angular)
            external_angular_errors.append(tuple(
                estimate_angular[index] - expected_angular[index] for index in range(3)
            ))

    for gt_item, estimate_item, _ in px4_matches:
        gt = gt_item.get("payload", {})
        estimate = estimate_item.get("payload", {})
        gt_position = _vector(gt.get("position"))
        estimate_position = _vector(estimate.get("position"))
        gt_orientation = _quaternion_xyzw(gt.get("q_xyzw"))
        estimate_orientation = _quaternion(estimate.get("q_wxyz"))
        gt_velocity = _vector(gt.get("linear_velocity"))
        estimate_velocity = _vector(estimate.get("velocity"))
        gt_angular = _vector(gt.get("angular_velocity"))
        estimate_angular = _vector(estimate.get("angular_velocity"))
        gt_child_frames.add(str(gt.get("child_frame_id", "")))
        if gt_position is not None and estimate_position is not None:
            if first_gt_position_px4 is None:
                first_gt_position_px4 = gt_position
            if first_px4_position is None:
                first_px4_position = estimate_position
            assert first_gt_position_px4 is not None
            assert first_px4_position is not None
            expected_delta = _matrix_vector(
                _C_NED_FROM_ENU,
                tuple(gt_position[index] - first_gt_position_px4[index] for index in range(3)),
            )
            actual_delta = tuple(
                estimate_position[index] - first_px4_position[index] for index in range(3)
            )
            px4_position_errors.append(tuple(
                actual_delta[index] - expected_delta[index] for index in range(3)
            ))
        if gt_orientation is not None and estimate_orientation is not None:
            expected_matrix = _matrix_multiply(
                _matrix_multiply(_C_NED_FROM_ENU, _rotation_matrix_from_quaternion(gt_orientation)),
                _C_FRD_FROM_FLU,
            )
            expected_orientation = _quaternion_from_rotation_matrix(expected_matrix)
            angle = _quaternion_angle(estimate_orientation, expected_orientation)
            px4_attitude.append(angle)
            if first_px4_attitude is None:
                first_px4_attitude = angle
        if gt_velocity is not None and estimate_velocity is not None:
            velocity_enu = (
                _quaternion_rotate(gt_orientation, gt_velocity)
                if gt_orientation is not None
                else gt_velocity
            )
            expected_velocity = _matrix_vector(_C_NED_FROM_ENU, velocity_enu)
            px4_velocity_errors.append(tuple(
                estimate_velocity[index] - expected_velocity[index] for index in range(3)
            ))
        if gt_angular is not None and estimate_angular is not None:
            expected_angular = _matrix_vector(_C_FRD_FROM_FLU, gt_angular)
            px4_angular_errors.append(tuple(
                estimate_angular[index] - expected_angular[index] for index in range(3)
            ))

    return {
        "source": "Gazebo /sim/ground_truth/odometry (ENU/FLU)",
        "ground_truth_frame_ids": sorted({str(item.get("payload", {}).get("frame_id", "")) for item in ground_truth}),
        "ground_truth_child_frame_ids": sorted(gt_child_frames),
        "world_transform": "x_ned=y_enu; y_ned=x_enu; z_ned=-z_enu",
        "body_transform": "x_frd=x_flu; y_frd=-y_flu; z_frd=-z_flu",
        "timestamp_alignment": "absolute simulation timestamp; no stream epoch normalization",
        "maximum_synchronization_tolerance_ms": tolerance_ms,
        "ground_truth_sample_count": len(ground_truth),
        "lio_matched_sample_count": len(lio_matches),
        "external_matched_sample_count": len(external_matches),
        "lio_timestamp_delta_ms": {
            "mean": sum(lio_deltas) / len(lio_deltas) if lio_deltas else None,
            "p95": _p(lio_deltas, 0.95),
            "maximum": max(lio_deltas) if lio_deltas else None,
        },
        "external_timestamp_delta_ms": {
            "mean": sum(external_deltas) / len(external_deltas) if external_deltas else None,
            "p95": _p(external_deltas, 0.95),
            "maximum": max(external_deltas) if external_deltas else None,
        },
        "px4_timestamp_delta_ms": {
            "mean": sum(px4_deltas) / len(px4_deltas) if px4_deltas else None,
            "p95": _p(px4_deltas, 0.95),
            "maximum": max(px4_deltas) if px4_deltas else None,
        },
        "lio_vs_ground_truth": {
            "origin_alignment": "first matched position removed",
            "position_m": _vector_metric_summary(lio_position_errors),
            "velocity_m_s": _vector_metric_summary(lio_velocity_errors),
            "angular_velocity_rad_s": _vector_metric_summary(lio_angular_errors),
            "attitude_rad": _metric_summary(lio_attitude),
            "initial_attitude_error_rad": first_lio_attitude,
        },
        "external_ned_vs_ground_truth": {
            "origin_alignment": "first matched position removed after ENU->NED conversion",
            "position_m": _vector_metric_summary(external_position_errors),
            "velocity_m_s": _vector_metric_summary(external_velocity_errors),
            "angular_velocity_rad_s": _vector_metric_summary(external_angular_errors),
            "attitude_rad": _metric_summary(external_attitude),
            "initial_attitude_error_rad": first_external_attitude,
            "required_pose_frame": 1,
            "required_velocity_frame": 1,
        },
        "px4_ned_vs_ground_truth": {
            "origin_alignment": "first matched position removed after ENU->NED conversion",
            "position_m": _vector_metric_summary(px4_position_errors),
            "velocity_m_s": _vector_metric_summary(px4_velocity_errors),
            "angular_velocity_rad_s": _vector_metric_summary(px4_angular_errors),
            "attitude_rad": _metric_summary(px4_attitude),
            "initial_attitude_error_rad": first_px4_attitude,
            "matched_sample_count": len(px4_matches),
            "required_pose_frame": 1,
            "required_velocity_frame": 1,
        },
        "px4_matched_sample_count": len(px4_matches),
    }


def _dataset_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path) -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    streams = {name: _rate_row(snapshot, name) for name in ("imu", "lidar", "corrected_odometry", "propagated_odometry")}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    _annotate_stale_classification(streams, config, runtime, samples)
    diagnostic_states = _diagnostic_states(samples)
    map_point_count = _map_point_summary(samples)
    map_maintenance = _map_maintenance_summary(samples)
    reasons: list[str] = []
    minimum_fraction = _number(thresholds.get("minimum_rate_fraction"), 0.90)
    for name, row in streams.items():
        if row["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        expected = _number(config.get("runtime", {}).get("streams", {}).get(name, {}).get("expected_hz"))
        if expected and row["mean_rate_hz"] < expected * minimum_fraction:
            reasons.append(f"{name} rate below contract")
        if row["timestamp_regression_count"] or row["source_stale_event_count"] or row["nonfinite_message_count"]:
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
            "map_maintenance": map_maintenance,
        },
        "accuracy": "NOT_AVAILABLE",
        "provenance": provenance(workspace),
    }


def _sim_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path, px4_dir: Path | None) -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    names = ("imu", "lidar", "corrected_odometry", "propagated_odometry", "ground_truth_odometry", "external_odometry", "px4_odometry", "vehicle_status", "local_position", "estimator_status_flags")
    streams = {name: _rate_row(snapshot, name) for name in names}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    _annotate_stale_classification(streams, config, runtime, samples)
    diagnostic_states = _diagnostic_states(samples)
    map_point_count = _map_point_summary(samples)
    map_maintenance = _map_maintenance_summary(samples)
    reasons: list[str] = []
    for name in ("imu", "lidar", "corrected_odometry", "propagated_odometry", "ground_truth_odometry", "external_odometry", "px4_odometry", "local_position", "estimator_status_flags"):
        if streams[name]["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        if streams[name]["timestamp_regression_count"] or streams[name]["source_stale_event_count"] or streams[name]["nonfinite_message_count"]:
            reasons.append(f"{name} timestamp/freshness/validity violation")
    external_expected = _number(config.get("runtime", {}).get("streams", {}).get("external_odometry", {}).get("expected_hz"))
    if external_expected and streams["external_odometry"]["mean_rate_hz"] < external_expected * _number(thresholds.get("minimum_rate_fraction"), 0.90):
        reasons.append("external odometry rate below contract")
    diagnostics = _diag_values(snapshot)
    tracking_observed = "TRACKING" in diagnostic_states or str(diagnostics.get("state", "")).upper() == "TRACKING"
    if not tracking_observed:
        reasons.append("LIO did not finish in TRACKING")
    residuals = _residuals(samples, _number(thresholds.get("maximum_synchronization_tolerance_ms"), 20.0))
    conversion_contract = _frame_contract_residuals(
        samples, _number(thresholds.get("maximum_synchronization_tolerance_ms"), 20.0)
    )
    ground_truth_residuals = _ground_truth_residuals(
        samples, _number(thresholds.get("maximum_synchronization_tolerance_ms"), 20.0)
    )
    scenario = _load_json(session / "scenario.json", {})
    scenario_reasons = list(scenario.get("failures", []))
    reasons.extend(str(item) for item in scenario_reasons)
    reasons.extend(failures)
    verdict = "PASS" if not reasons else "FAIL"
    return {
        "workflow": "sim",
        "verdict": verdict,
        "reasons": reasons,
        "streams": streams,
        "px4": {
            "estimator_initialized": streams["estimator_status_flags"]["sample_count"] > 0,
            "local_position_valid": bool(snapshot.get("latest", {}).get("local_position", {}).get("xy_valid")) and bool(snapshot.get("latest", {}).get("local_position", {}).get("z_valid")),
            "local_velocity_valid": bool(snapshot.get("latest", {}).get("local_position", {}).get("v_xy_valid")) and bool(snapshot.get("latest", {}).get("local_position", {}).get("v_z_valid")),
            "external_vision_position_control_enabled": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_pos")),
            "external_vision_velocity_control_enabled": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_vel")),
            "external_vision_yaw_control_enabled": bool(snapshot.get("latest", {}).get("estimator_status_flags", {}).get("cs_ev_yaw")),
            "external_vision_status_interpretation": "PX4 control-status observation only; not per-sample fusion proof",
            "dead_reckoning_events": int(bool(snapshot.get("latest", {}).get("local_position", {}).get("dead_reckoning"))),
            "estimator_fault_events": int(bool(snapshot.get("latest", {}).get("estimator_status", {}).get("filter_fault_flags"))),
        },
        "lio": {
            "state": diagnostics.get("state", "NOT_AVAILABLE"),
            "tracking_observed": tracking_observed,
            "map_point_count": map_point_count,
            "map_maintenance": map_maintenance,
        },
        "residuals": residuals,
        "conversion_contract": conversion_contract,
        "ground_truth_residuals": ground_truth_residuals,
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
    lines += ["## Reasons", ""] + ([f"- {reason}" for reason in reasons] if reasons else ["- none"]) + ["", "## Stream metrics", "", "| Stream | Samples | Mean Hz | Min window Hz | p95 interval ms | Max gap ms | Callback stalls | Source stale | Regressions |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, row in report.get("streams", {}).items():
        lines.append(f"| {name} | {row.get('sample_count', 0)} | {_number(row.get('mean_rate_hz')):.3f} | {_number(row.get('minimum_window_rate_hz')):.3f} | {row.get('p95_interval_ms', 'n/a')} | {_number(row.get('maximum_gap_ms')):.3f} | {row.get('active_callback_stall_count', row.get('stale_event_count', 0))} | {row.get('source_stale_event_count', row.get('stale_event_count', 0))} | {row.get('timestamp_regression_count', 0)} |")
    for section in ("lio", "px4", "residuals", "conversion_contract", "ground_truth_residuals", "offboard", "provenance"):
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

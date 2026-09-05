#!/usr/bin/env python3
"""Analyze one direct PX4-local closed-loop characterization run."""

from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import re
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


def _stream_identity(state: dict[str, Any] | None) -> tuple[Any, ...] | None:
    if state is None:
        return None
    reset = state.get("reset_metadata")
    if isinstance(reset, dict):
        reset = tuple(sorted(reset.items()))
    return (state.get("frame_id"), state.get("child_frame_id"), state.get("epoch"),
            state.get("reset_counter"), state.get("reset_id"), reset)


def _has_continuity_metadata(state: dict[str, Any] | None) -> bool:
    return bool(state and (any(state.get(key) is not None for key in ("epoch", "reset_counter", "reset_id"))
                          or isinstance(state.get("reset_metadata"), dict) and bool(state["reset_metadata"])))


def _stream_continuous(state: dict[str, Any] | None, frame_contract: dict[str, Any]) -> bool:
    if not state:
        return False
    if frame_contract.get("continuity") == "ros_sim_session":
        return state.get("source_clock") == "ros_sim_ns" and _number(state.get("source_stamp_ns")) is not None
    return _has_continuity_metadata(state)


def _q(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 4:
        return None
    values = [_number(item) for item in value]
    if any(item is None for item in values):
        return None
    magnitude = math.sqrt(sum(float(item) * float(item) for item in values))
    return [float(item) for item in values] if abs(magnitude - 1.0) <= 1e-3 else None


def _sub(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    return [x - y for x, y in zip(a, b)] if a is not None and b is not None else None


def _norm(value: list[float] | None) -> float | None:
    return math.sqrt(sum(item * item for item in value)) if value is not None else None


def _horizontal_norm(value: list[float] | None) -> float | None:
    return math.sqrt(value[0] * value[0] + value[1] * value[1]) if value is not None else None


def _rot(value: list[float] | None) -> list[float] | None:
    return [value[1], value[0], -value[2]] if value is not None else None


def _quat_rotate(q: list[float] | None, value: list[float] | None) -> list[float] | None:
    if q is None or value is None:
        return None
    x, y, z, w = q
    return [
        (1 - 2 * y * y - 2 * z * z) * value[0] + (2 * x * y - 2 * z * w) * value[1] + (2 * x * z + 2 * y * w) * value[2],
        (2 * x * y + 2 * z * w) * value[0] + (1 - 2 * x * x - 2 * z * z) * value[1] + (2 * y * z - 2 * x * w) * value[2],
        (2 * x * z - 2 * y * w) * value[0] + (2 * y * z + 2 * x * w) * value[1] + (1 - 2 * x * x - 2 * y * y) * value[2],
    ]


def _frame_kind(frame: Any) -> str | None:
    """Map only documented frame names to a canonical convention."""
    name = str(frame or "").strip().lower().lstrip("/")
    return {
        "enu": "ENU", "world_enu": "ENU", "map_enu": "ENU", "odom_enu": "ENU",
        "ned": "NED", "px4_local_ned": "NED", "local_ned": "NED",
        "base_link": "BODY_FLU", "base_link_flu": "BODY_FLU", "body_flu": "BODY_FLU",
    }.get(name)


def _declared_frame_kind(frame: Any, convention: Any) -> str | None:
    if not isinstance(frame, str) or not isinstance(convention, str):
        return None
    kind = convention.strip().upper()
    return kind if kind in {"ENU", "NED", "BODY_FLU"} else None


def _aligned_odom(data: Any, evaluation_ns: int, max_age_ns: int | None = None,
                  frame_contract: dict[str, Any] | None = None) -> tuple[dict[str, Any] | None, str]:
    if not isinstance(data, dict):
        return None, "missing_stream"
    stamp = _number(data.get("source_stamp_ns"))
    if stamp is None or stamp <= 0:
        return None, "missing_source_timestamp"
    if max_age_ns is None:
        return None, "missing_alignment_contract"
    if abs(int(evaluation_ns) - int(stamp)) > max_age_ns:
        return None, "source_timestamp_stale_or_mismatched"
    position = _v(data.get("position")); velocity = _v(data.get("velocity"))
    position_frame = data.get("position_frame_id", data.get("frame_id"))
    velocity_frame = data.get("velocity_frame_id", data.get("child_frame_id"))
    if frame_contract is None:
        position_kind = _frame_kind(position_frame)
        velocity_kind = _frame_kind(velocity_frame)
    else:
        if data.get("source_clock") != "ros_sim_ns":
            return None, "unknown_or_mismatched_source_clock"
        if position_frame != frame_contract.get("position_frame_id") or velocity_frame != frame_contract.get("velocity_frame_id"):
            return None, "frame_contract_mismatch"
        position_kind = _declared_frame_kind(position_frame, frame_contract.get("position_convention"))
        velocity_kind = _declared_frame_kind(velocity_frame, frame_contract.get("velocity_convention"))
    if position is None or position_kind not in {"ENU", "NED"}:
        return None, "missing_or_unknown_position_frame"
    if velocity is None or velocity_kind is None:
        return None, "missing_or_unknown_velocity_frame"
    q = _q(data.get("q_xyzw"))
    if velocity_kind not in {"BODY_FLU", position_kind}:
        return None, "position_velocity_frame_mismatch"
    world_velocity = _quat_rotate(q, velocity) if velocity_kind == "BODY_FLU" else velocity
    if world_velocity is None:
        return None, "missing_or_invalid_orientation_for_body_twist"
    return {
        "position_ned": _rot(position) if position_kind == "ENU" else position,
        "velocity_ned": _rot(world_velocity) if position_kind == "ENU" else world_velocity,
        "source_stamp_ns": int(stamp), "source_age_ms": abs(int(evaluation_ns) - int(stamp)) / 1e6,
        "source_clock": data.get("source_clock"),
        "epoch": data.get("epoch", (data.get("epoch_metadata") or {}).get("localization_epoch")),
        "reset_counter": data.get("reset_counter", (data.get("epoch_metadata") or {}).get("reset_counter")),
        "reset_id": data.get("reset_id", (data.get("epoch_metadata") or {}).get("reset_id")),
        "frame_id": data.get("position_frame_id", data.get("frame_id")),
        "child_frame_id": data.get("velocity_frame_id", data.get("child_frame_id")),
    }, "aligned"


def _normalize_odom_series(samples: list[dict[str, Any]], key: str,
                           frame_contract: dict[str, Any] | None = None) -> tuple[list[tuple[int, dict[str, Any]]], str]:
    points: list[tuple[int, dict[str, Any]]] = []
    previous_stamp: int | None = None
    previous_epoch: int | None = None
    for item in samples:
        raw = item.get(key)
        stamp = _number(raw.get("source_stamp_ns")) if isinstance(raw, dict) else None
        if stamp is not None and previous_stamp is not None and int(stamp) < previous_stamp:
            return points, "source_timestamp_regression"
        if stamp is not None:
            previous_stamp = int(stamp)
        state, reason = _aligned_odom(raw, int(stamp) if stamp is not None else 0, 10**18, frame_contract)
        if state is not None:
            epoch = state.get("epoch")
            if epoch is not None:
                try:
                    value = int(epoch)
                except (TypeError, ValueError):
                    return points, "invalid_epoch_metadata"
                if previous_epoch is not None and value < previous_epoch:
                    return points, "epoch_reset_discontinuity"
                previous_epoch = value
            points.append((int(stamp), state))
    if not points:
        return [], "missing_stream"
    return points, "normalized"


def _interpolate_odom_points(points: list[tuple[int, dict[str, Any]]], evaluation_ns: int, max_gap_ns: int,
                             timestamps: list[int] | None = None, series_reason: str | None = None) -> tuple[dict[str, Any] | None, str]:
    if not points:
        return None, "missing_stream"
    if evaluation_ns < points[0][0] or evaluation_ns > points[-1][0]:
        return None, series_reason or "source_timestamp_out_of_range"
    timestamps = timestamps or [point[0] for point in points]
    lo_index = min(bisect_right(timestamps, evaluation_ns) - 1, len(points) - 1)
    t0, lo = points[lo_index]
    if t0 == evaluation_ns:
        t1, hi = t0, lo
    else:
        t1, hi = points[min(lo_index + 1, len(points) - 1)]
    if t1 - t0 > max_gap_ns or lo["frame_id"] != hi["frame_id"] or lo["child_frame_id"] != hi["child_frame_id"]:
        return None, "source_gap_or_frame_change"
    if lo.get("epoch") is not None and hi.get("epoch") is not None and lo.get("epoch") != hi.get("epoch"):
        return None, "epoch_reset_boundary"
    if lo.get("reset_counter") is not None and hi.get("reset_counter") is not None and lo.get("reset_counter") != hi.get("reset_counter"):
        return None, "reset_counter_boundary"
    if lo.get("reset_id") is not None and hi.get("reset_id") is not None and lo.get("reset_id") != hi.get("reset_id"):
        return None, "reset_id_boundary"
    alpha = 0.0 if t1 == t0 else (evaluation_ns - t0) / (t1 - t0)
    return {
        **lo,
        "position_ned": [lo["position_ned"][i] + alpha * (hi["position_ned"][i] - lo["position_ned"][i]) for i in range(3)],
        "velocity_ned": [lo["velocity_ned"][i] + alpha * (hi["velocity_ned"][i] - lo["velocity_ned"][i]) for i in range(3)],
        "source_stamp_ns": evaluation_ns, "source_age_ms": 0.0,
    }, "interpolated" if t1 != t0 else "aligned"


def _interpolated_odom(samples: list[dict[str, Any]], key: str, evaluation_ns: int, max_gap_ns: int,
                       frame_contract: dict[str, Any] | None = None) -> tuple[dict[str, Any] | None, str]:
    points, reason = _normalize_odom_series(samples, key, frame_contract)
    return _interpolate_odom_points(points, evaluation_ns, max_gap_ns) if points else (None, reason)


def _mapped_px4_stamp(data: Any, contract: dict[str, Any]) -> tuple[int | None, str]:
    """Resolve PX4 time only from an explicit retained clock mapping."""
    mapping = contract.get("px4_to_ros_mapping")
    if not isinstance(mapping, dict) or mapping.get("status") != "VALID":
        return None, "missing_px4_to_ros_clock_mapping"
    clock = data.get("source_clock") if isinstance(data, dict) else None
    if clock != mapping.get("source_clock"):
        return None, "unknown_or_mismatched_px4_source_clock"
    source_stamp = _number(data.get("mapped_source_stamp_ns")) if isinstance(data, dict) else None
    if source_stamp is None and isinstance(data, dict):
        # A recorder's ``source_stamp_ns`` is already in the producer clock.
        # It may be used for PX4 DDS samples only after the retained witness
        # below proves numeric identity with the ROS simulation clock.  A
        # receive stamp is deliberately never considered here.
        source_stamp = _number(data.get("source_stamp_ns"))
    if source_stamp is None:
        return None, "missing_px4_source_timestamp_mapping"
    if mapping.get("source_clock") == "px4_dds_ns":
        if mapping.get("timestamp_relation") != "numeric_identity" or _number(mapping.get("scale_to_ros_ns")) != 1.0 or _number(mapping.get("offset_ns")) != 0.0:
            return None, "missing_px4_numeric_clock_identity_witness"
    scale = _number(mapping.get("scale_to_ros_ns"))
    offset = _number(mapping.get("offset_ns"))
    if scale is None:
        scale = 1.0
    if offset is None:
        offset = 0.0
    mapped = source_stamp * scale + offset
    return int(mapped), "mapped"


def _px4_source_valid(data: Any, evaluation_ns: int, tolerance_ms: float | None,
                      contract: dict[str, Any]) -> tuple[bool, str]:
    """Require an explicit source-clock mapping; receive time is never a substitute."""
    source_stamp, reason = _mapped_px4_stamp(data, contract)
    if source_stamp is None:
        return False, reason
    if tolerance_ms is None or abs(source_stamp - int(evaluation_ns)) > int(tolerance_ms * 1e6):
        return False, "stale_or_unmapped_px4_source_timestamp"
    return True, "aligned"


def _normalize_px4_series(samples: list[dict[str, Any]], key: str,
                          contract: dict[str, Any]) -> tuple[list[tuple[int, dict[str, Any]]], str]:
    points: list[tuple[int, dict[str, Any]]] = []
    previous_stamp: int | None = None
    first_mapping_reason: str | None = None
    for item in samples:
        raw = item.get(key)
        if not isinstance(raw, dict):
            continue
        stamp, reason = _mapped_px4_stamp(raw, contract)
        if stamp is None:
            if first_mapping_reason is None:
                first_mapping_reason = reason
            continue
        if previous_stamp is not None and stamp < previous_stamp:
            return points, "source_timestamp_regression"
        previous_stamp = stamp
        position = _v(raw.get("position_ned"))
        velocity = _v(raw.get("velocity_ned"))
        acceleration = _v(raw.get("acceleration_ned"))
        if key == "px4_state" and (position is None or velocity is None):
            continue
        if key == "px4_effective_setpoint" and (position is None or velocity is None or acceleration is None):
            continue
        points.append((stamp, {"position_ned": position, "velocity_ned": velocity, "acceleration_ned": acceleration,
                               "source_clock": raw.get("source_clock"), "reset_metadata": raw.get("reset_metadata"),
                               "xy_valid": raw.get("xy_valid", MISSING), "z_valid": raw.get("z_valid", MISSING),
                               "v_xy_valid": raw.get("v_xy_valid", MISSING), "v_z_valid": raw.get("v_z_valid", MISSING)}))
    if not points:
        return [], first_mapping_reason or "missing_px4_source_time_mapping"
    return points, "normalized"


def _interpolate_px4_points(points: list[tuple[int, dict[str, Any]]], evaluation_ns: int,
                            max_gap_ns: int, timestamps: list[int] | None = None,
                            series_reason: str | None = None) -> tuple[dict[str, Any] | None, str]:
    if not points:
        return None, "missing_px4_source_time_mapping"
    if evaluation_ns < points[0][0] or evaluation_ns > points[-1][0]:
        return None, series_reason or "stale_or_unmapped_px4_source_timestamp"
    timestamps = timestamps or [point[0] for point in points]
    lo_index = min(bisect_right(timestamps, evaluation_ns) - 1, len(points) - 1)
    t0, lo = points[lo_index]
    t1, hi = (t0, lo) if t0 == evaluation_ns else points[min(lo_index + 1, len(points) - 1)]
    if t1 - t0 > max_gap_ns:
        return None, "source_gap_or_frame_change"
    if lo.get("reset_metadata") != hi.get("reset_metadata"):
        return None, "reset_counter_boundary"
    alpha = 0.0 if t1 == t0 else (evaluation_ns - t0) / (t1 - t0)
    result: dict[str, Any] = {"source_stamp_ns": evaluation_ns, "mapped_source_stamp_ns": evaluation_ns,
                              "source_clock": lo.get("source_clock"), "reset_metadata": lo.get("reset_metadata"),
                              "source_age_ms": 0.0}
    for field in ("position_ned", "velocity_ned", "acceleration_ned"):
        value = lo.get(field)
        other = hi.get(field)
        result[field] = ([value[i] + alpha * (other[i] - value[i]) for i in range(3)]
                         if value is not None and other is not None else None)
    for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"):
        result[field] = lo.get(field, MISSING)
    return result, "interpolated" if t1 != t0 else "aligned"


def _interpolated_px4(samples: list[dict[str, Any]], key: str, evaluation_ns: int,
                      max_gap_ns: int, contract: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
    """Compatibility helper: normalize once for a direct caller, then interpolate."""
    points, reason = _normalize_px4_series(samples, key, contract)
    return _interpolate_px4_points(points, evaluation_ns, max_gap_ns) if points else (None, reason)


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


def _scope_value(summary: dict[str, Any], metadata: dict[str, Any], keys: tuple[str, ...]) -> str | None:
    candidates = [summary, summary.get("scope") if isinstance(summary.get("scope"), dict) else {},
                  metadata, metadata.get("environment") if isinstance(metadata.get("environment"), dict) else {}]
    for container in candidates:
        for key in keys:
            value = container.get(key) if isinstance(container, dict) else None
            if isinstance(value, str) and value.strip():
                return value.strip()
    return None


def _read_clock_witness(run_dir: Path, summary: dict[str, Any]) -> dict[str, Any]:
    """Read a retained SITL clock witness; never infer it from launcher text."""
    log_path = run_dir / "logs" / "px4_gazebo.log"
    metadata_path = run_dir / "metadata.json"
    if not metadata_path.is_file():
        metadata_path = run_dir / "runtime.json"
    result: dict[str, Any] = {"status": MISSING, "reason": "missing_retained_sitl_clock_artifacts",
                              "log_path": str(log_path), "metadata_path": str(metadata_path)}
    if not log_path.is_file() or not metadata_path.is_file():
        return result
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        lines = log_path.read_text(encoding="utf-8").splitlines()
    except (OSError, ValueError, json.JSONDecodeError):
        result["reason"] = "malformed_retained_sitl_clock_artifacts"
        return result
    build = metadata.get("build_provenance") if isinstance(metadata, dict) else None
    if not isinstance(build, dict) or build.get("status") != "VALID":
        result["reason"] = "missing_or_invalid_build_provenance"
        return result
    actual: list[tuple[int, float]] = []
    parameter_pattern = re.compile(r"^\s*x\s+(?:\+\s+)?UXRCE_DDS_SYNCT\s+\[[^]]+\]\s*:\s*(-?[0-9]+(?:\.[0-9]+)?)\s*$")
    for number, line in enumerate(lines, 1):
        match = parameter_pattern.match(line)
        if match:
            actual.append((number, float(match.group(1))))
    if not actual:
        result["reason"] = "missing_actual_uxrce_dds_synct_parameter"
        return result
    if len({value for _, value in actual}) != 1:
        result["reason"] = "conflicting_actual_uxrce_dds_synct_parameter"
        return result
    if actual[0][1] != 0.0:
        result["reason"] = "uxrce_dds_synct_not_zero"
        return result
    world_lines: list[tuple[int, str]] = []
    model_lines: list[tuple[int, str]] = []
    bridge_lines: list[tuple[int, str, str]] = []
    for number, line in enumerate(lines, 1):
        match = re.match(r"^\s*World\s*:\s*(\S+)\s*$", line)
        if match:
            world_lines.append((number, match.group(1)))
        match = re.match(r"^\s*Existing model\s*:\s*(\S+)\s*$", line)
        if match:
            model_lines.append((number, match.group(1)))
        match = re.search(r"\[gz_bridge\]\s+world:\s*([^,]+),\s*model:\s*(\S+)", line)
        if match:
            bridge_lines.append((number, match.group(1).strip(), match.group(2).strip()))
    if not world_lines or not model_lines or not bridge_lines:
        result["reason"] = "missing_sitl_world_model_witness"
        return result
    worlds = {value for _, value in world_lines} | {value for _, value, _ in bridge_lines}
    models = {value for _, value in model_lines} | {value for _, _, value in bridge_lines}
    expected_world = _scope_value(summary, metadata, ("world", "world_name", "map_scene", "map_profile"))
    expected_model = _scope_value(summary, metadata, ("model", "model_name", "px4_model"))
    if len(worlds) != 1 or len(models) != 1 or (expected_world is not None and expected_world not in worlds) or (expected_model is not None and expected_model not in models):
        result["reason"] = "conflicting_or_wrong_sitl_scope"
        return result
    world, model = next(iter(worlds)), next(iter(models))
    result.update({"status": "VALID", "reason": "actual_sitl_clock_witness", "world": world, "model": model,
                   "parameter": "UXRCE_DDS_SYNCT", "parameter_value": 0.0,
                   "parameter_lines": [number for number, _ in actual],
                   "world_lines": [number for number, _ in world_lines],
                   "model_lines": [number for number, _ in model_lines],
                   "bridge_lines": [number for number, _, _ in bridge_lines],
                   "build_status": build.get("status"),
                   "source_clock": "px4_dds_ns", "target_clock": "ros_sim_ns"})
    return result


def _read_samples(run_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    read_errors: list[str] = []
    try:
        summary = json.loads((run_dir / "scenario.json").read_text(encoding="utf-8"))
        if not isinstance(summary, dict):
            raise ValueError("scenario summary is not an object")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        summary = {"schema_version": 2, "evidence_contract": {}, "read_error": str(exc)}
        read_errors.append("malformed_or_missing_scenario_summary")
    rows: list[dict[str, Any]] = []
    try:
        stream = (run_dir / "scenario.jsonl").open(encoding="utf-8")
    except OSError:
        read_errors.append("missing_or_unreadable_scenario_trace")
        stream = None
    if stream is not None:
        with stream:
            for line in stream:
                try:
                    item = json.loads(line)
                except (TypeError, ValueError, json.JSONDecodeError):
                    read_errors.append("malformed_trace_row")
                    continue
                if not isinstance(item, dict) or item.get("kind") != "sample":
                    continue
                payload = item.get("payload")
                if not isinstance(payload, dict):
                    read_errors.append("malformed_sample_payload")
                    continue
                try:
                    timestamp_ns = int(item.get("timestamp_ns", 0))
                except (TypeError, ValueError):
                    read_errors.append("malformed_sample_timestamp")
                    continue
                rows.append(dict(payload, timestamp_ns=timestamp_ns))
    clock_witness = _read_clock_witness(run_dir, summary)
    source_witness = dict(summary.get("source_clock_witness") or {})
    source_witness["px4_to_ros_mapping"] = clock_witness if clock_witness.get("status") == "VALID" else MISSING
    summary["source_clock_witness"] = source_witness
    evidence_contract = dict(summary.get("evidence_contract") or {})
    evidence_contract["px4_to_ros_mapping"] = {
        "status": "VALID", "source_clock": "px4_dds_ns", "target_clock": "ros_sim_ns",
        "timestamp_relation": "numeric_identity", "scale_to_ros_ns": 1.0, "offset_ns": 0.0,
        "provenance": {key: clock_witness[key] for key in ("log_path", "metadata_path", "parameter_lines", "world_lines", "model_lines", "bridge_lines") if key in clock_witness},
    } if clock_witness.get("status") == "VALID" else MISSING
    summary["evidence_contract"] = evidence_contract
    summary["clock_witness"] = clock_witness
    summary["read_errors"] = sorted(set(read_errors))
    return summary, rows


def _analyze_rows(summary: dict[str, Any], samples: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    initial_px4 = _v(summary.get("initial_px4_local_ned"))
    contract = summary.get("evidence_contract") or {}
    tolerance_ms = _number(contract.get("maximum_synchronization_tolerance_ms"))
    derived: list[dict[str, Any]] = []
    previous_segment = ""
    previous_epochs: dict[str, Any] = {"ground_truth": None, "lio": None}
    previous_resets: dict[str, Any] = {"ground_truth": None, "lio": None, "px4": None}
    previous_px4_source_stamp: int | None = None
    lio_to_gt_offset: list[float] | None = None
    gt_to_px4_offset: list[float] | None = None
    lio_gt_calibration_ids: dict[str, tuple[Any, ...] | None] = {"ground_truth": None, "lio": None}
    px4_gt_calibration_ids: dict[str, tuple[Any, ...] | None] = {"ground_truth": None, "px4": None}
    frame_contracts = contract.get("frame_conventions") if isinstance(contract.get("frame_conventions"), dict) else {}
    gt_frame_contract = frame_contracts.get("ground_truth") if isinstance(frame_contracts.get("ground_truth"), dict) else {}
    lio_frame_contract = frame_contracts.get("lio") if isinstance(frame_contracts.get("lio"), dict) else {}
    gt_points, gt_series_reason = _normalize_odom_series(samples, "ground_truth", gt_frame_contract) if tolerance_ms is not None else ([], "missing_alignment_contract")
    lio_points, lio_series_reason = _normalize_odom_series(samples, "lio", lio_frame_contract) if tolerance_ms is not None else ([], "missing_alignment_contract")
    px4_points, px4_series_reason = _normalize_px4_series(samples, "px4_state", contract) if tolerance_ms is not None else ([], "missing_alignment_contract")
    px4sp_points, px4sp_series_reason = _normalize_px4_series(samples, "px4_effective_setpoint", contract) if tolerance_ms is not None else ([], "missing_alignment_contract")
    gt_timestamps = [point[0] for point in gt_points]
    lio_timestamps = [point[0] for point in lio_points]
    px4_timestamps = [point[0] for point in px4_points]
    px4sp_timestamps = [point[0] for point in px4sp_points]
    for item in samples:
        p = _v(item.get("reference_position_ned")); v = _v(item.get("reference_velocity_ned")); a = _v(item.get("reference_acceleration_ned")); j = _v(item.get("reference_jerk_ned"))
        px4 = item.get("px4_state") or {}; px4sp = item.get("px4_effective_setpoint") or {}; gt = item.get("ground_truth") or {}; lio = item.get("lio") or {}
        t = int(item.get("timestamp_ns", 0))
        max_gap_ns = int(tolerance_ms * 1e6) if tolerance_ms is not None and tolerance_ms >= 0.0 else 0
        gt_state, gt_reason = _interpolate_odom_points(gt_points, t, max_gap_ns, gt_timestamps, gt_series_reason if gt_series_reason != "normalized" else None) if gt_points else (None, gt_series_reason)
        lio_state, lio_reason = _interpolate_odom_points(lio_points, t, max_gap_ns, lio_timestamps, lio_series_reason if lio_series_reason != "normalized" else None) if lio_points else (None, lio_series_reason)
        contract = summary.get("evidence_contract") or {}
        px4_eval, px4_eval_reason = _interpolate_px4_points(px4_points, t, max_gap_ns, px4_timestamps, px4_series_reason if px4_series_reason != "normalized" else None) if px4_points else (None, px4_series_reason)
        px4sp_eval, px4sp_eval_reason = _interpolate_px4_points(px4sp_points, t, max_gap_ns, px4sp_timestamps, px4sp_series_reason if px4sp_series_reason != "normalized" else None) if px4sp_points else (None, px4sp_series_reason)
        px4_source_valid = px4_eval is not None
        px4_source_reason = "aligned" if px4_source_valid else px4_eval_reason
        px4_effective_source_valid = px4sp_eval is not None
        px4_effective_source_reason = "aligned" if px4_effective_source_valid else px4sp_eval_reason
        px4 = px4_eval or px4
        px4sp = px4sp_eval or px4sp
        px4_p = _v(px4.get("position_ned")); px4_v = _v(px4.get("velocity_ned")); eff_p = _v(px4sp.get("position_ned")); eff_v = _v(px4sp.get("velocity_ned")); eff_a = _v(px4sp.get("acceleration_ned"))
        px4_stamp = _number(px4.get("mapped_source_stamp_ns"))
        if px4_source_valid and px4_stamp is not None:
            if previous_px4_source_stamp is not None and int(px4_stamp) < previous_px4_source_stamp:
                px4_source_valid = False
                px4_source_reason = "source_timestamp_regression"
            previous_px4_source_stamp = int(px4_stamp)
        synchronized_witness = bool(gt_state and lio_state)
        gt_continuous = _stream_continuous(gt_state, gt_frame_contract)
        lio_continuous = _stream_continuous(lio_state, lio_frame_contract)
        if synchronized_witness and gt_continuous and lio_continuous and lio_to_gt_offset is None:
            lio_to_gt_offset = _sub(gt_state["position_ned"], lio_state["position_ned"])
            lio_gt_calibration_ids["ground_truth"] = _stream_identity(gt_state)
            lio_gt_calibration_ids["lio"] = _stream_identity(lio_state)
        def shifted(state: dict[str, Any] | None, offset: list[float] | None) -> list[float] | None:
            return [state["position_ned"][i] + offset[i] for i in range(3)] if state is not None and offset is not None else None
        gt_identity = _stream_identity(gt_state)
        gt_calibration_allowed = gt_state is not None and (px4_gt_calibration_ids["ground_truth"] is None or gt_identity == px4_gt_calibration_ids["ground_truth"])
        if px4_source_valid and gt_calibration_allowed and gt_continuous and _has_continuity_metadata(px4) and px4_p is not None and gt_to_px4_offset is None:
            gt_to_px4_offset = _sub(px4_p, gt_state["position_ned"])
            px4_gt_calibration_ids["ground_truth"] = gt_identity
            px4_gt_calibration_ids["px4"] = _stream_identity(px4)
        gt_frame_p = gt_state["position_ned"] if gt_state else None
        gt_p = shifted(gt_state, gt_to_px4_offset)
        lio_p = shifted(lio_state, lio_to_gt_offset)
        lio_px4_offset = ([lio_to_gt_offset[i] + gt_to_px4_offset[i] for i in range(3)]
                          if lio_to_gt_offset is not None and gt_to_px4_offset is not None else None)
        lio_px4_p = shifted(lio_state, lio_px4_offset)
        gt_v = gt_state["velocity_ned"] if gt_state else None; lio_v = lio_state["velocity_ned"] if lio_state else None
        delta_v = _norm(_sub(eff_v, v)); delta_a = _norm(_sub(eff_a, a))
        truth_error = _norm(_sub(p, gt_p)); lio_error_vector = _sub(lio_p, gt_frame_p); lio_velocity_error_vector = _sub(lio_v, gt_v); lio_error = _norm(lio_error_vector); px4_error = _norm(_sub(px4_p, gt_p)); lio_px4 = _norm(_sub(lio_px4_p, px4_p)); lio_v_px4 = _norm(_sub(lio_v, px4_v)); eff_truth = _norm(_sub(eff_p, gt_p)); command_v_error = _norm(_sub(v, gt_v)); command_lateral = None; deceleration = None
        lio_status, lio_navigation_valid = _lio_health(item)
        if v is not None and a is not None:
            speed_sq = v[0] * v[0] + v[1] * v[1]
            command_lateral = abs(v[0] * a[1] - v[1] * a[0]) / math.sqrt(speed_sq) if speed_sq > 1e-9 else 0.0
            deceleration = max(0.0, -a[0])
        source_ages = item.get("source_age_ms") or {}
        epoch_valid = bool(gt_state and lio_state)
        reset_valid = bool(gt_state and lio_state)
        for stream_name, state in (("ground_truth", gt_state), ("lio", lio_state)):
            current_epoch = state.get("epoch") if state else None
            previous_epoch = previous_epochs[stream_name]
            if current_epoch is not None and previous_epoch is not None:
                try:
                    if int(current_epoch) < int(previous_epoch):
                        epoch_valid = False
                except (TypeError, ValueError):
                    epoch_valid = False
            if current_epoch is not None:
                previous_epochs[stream_name] = current_epoch
            current_reset = state.get("reset_counter") if state else None
            if current_reset is None and state:
                current_reset = state.get("reset_id")
            previous_reset = previous_resets[stream_name]
            if current_reset is not None and previous_reset is not None and current_reset != previous_reset:
                reset_valid = False
            if current_reset is not None:
                previous_resets[stream_name] = current_reset
        px4_reset = px4.get("reset_metadata") if isinstance(px4.get("reset_metadata"), dict) else None
        if px4_reset:
            px4_reset = tuple(sorted(px4_reset.items()))
            if previous_resets["px4"] is not None and px4_reset != previous_resets["px4"]:
                reset_valid = False
            previous_resets["px4"] = px4_reset
        gt_lio_identity_valid = bool(gt_state and gt_continuous and lio_gt_calibration_ids["ground_truth"] is not None and _stream_identity(gt_state) == lio_gt_calibration_ids["ground_truth"])
        lio_identity_valid = bool(lio_state and lio_continuous and lio_gt_calibration_ids["lio"] is not None and _stream_identity(lio_state) == lio_gt_calibration_ids["lio"])
        gt_px4_identity_valid = bool(gt_state and gt_continuous and px4_gt_calibration_ids["ground_truth"] is not None and _stream_identity(gt_state) == px4_gt_calibration_ids["ground_truth"])
        px4_identity_valid = bool(px4_source_valid and _has_continuity_metadata(px4) and px4_gt_calibration_ids["px4"] is not None and _stream_identity(px4) == px4_gt_calibration_ids["px4"])
        gt_identity_valid = gt_lio_identity_valid or gt_px4_identity_valid
        alignment_valid = bool(gt_state and lio_state and synchronized_witness and epoch_valid and reset_valid and gt_identity_valid and lio_identity_valid and p is not None and v is not None and lio_to_gt_offset is not None)
        gt_pair_valid = bool(gt_state and p is not None and v is not None and gt_to_px4_offset is not None and gt_px4_identity_valid)
        lio_gt_pair_valid = bool(gt_state and lio_state and lio_to_gt_offset is not None and gt_lio_identity_valid and lio_identity_valid)
        px4_gt_pair_valid = bool(gt_p is not None and px4_p is not None and px4_source_valid and gt_px4_identity_valid and px4_identity_valid)
        px4_lio_pair_valid = bool(lio_px4_p is not None and px4_p is not None and lio_gt_pair_valid and px4_source_valid and px4_identity_valid)
        px4_position_pair_valid = bool(px4_gt_pair_valid and px4.get("xy_valid") is True and px4.get("z_valid") is True)
        px4_velocity_pair_valid = bool(px4_gt_pair_valid and px4.get("v_xy_valid") is True and px4.get("v_z_valid") is True)
        # Controller attribution is metric-specific.  Acceleration correction
        # uses the velocity-valid PX4 state as its relevant state witness; it
        # must not accidentally inherit position validity requirements.
        controller_position_pair_valid = bool(px4_position_pair_valid and px4_effective_source_valid and eff_p is not None)
        controller_velocity_pair_valid = bool(px4_velocity_pair_valid and px4_effective_source_valid and eff_v is not None)
        controller_acceleration_pair_valid = bool(px4_velocity_pair_valid and px4_effective_source_valid and eff_a is not None and a is not None)
        controller_pair_valid = bool(controller_position_pair_valid or controller_velocity_pair_valid or controller_acceleration_pair_valid)
        derived.append({
            "timestamp_ns": t, "time_s": (t - int(samples[0].get("timestamp_ns", t))) / 1e9, "segment_id": item.get("segment_id", MISSING), "segment_kind": item.get("segment_kind", MISSING), "mode": item.get("mode", MISSING), "radius_m": item.get("radius_m", MISSING), "requested_speed_mps": item.get("requested_speed_mps", MISSING),
            "planner_position_ned": p, "planner_velocity_ned": v, "planner_acceleration_ned": a, "planner_jerk_ned": j,
            "px4_input_position_ned": _v(item.get("px4_input_position_ned")), "px4_input_velocity_ned": _v(item.get("px4_input_velocity_ned")), "px4_input_acceleration_ned": _v(item.get("px4_input_acceleration_ned")),
            "px4_effective_position_setpoint_ned": eff_p, "px4_effective_velocity_setpoint_ned": eff_v, "px4_effective_acceleration_setpoint_ned": eff_a,
            "ground_truth_position_ned": gt_p, "ground_truth_velocity_ned": gt_v, "px4_position_ned": px4_p, "px4_velocity_ned": px4_v, "lio_position_ned": lio_p, "lio_velocity_ned": lio_v,
            "planner_speed_mps": _norm(v), "planner_acceleration_mps2": _norm(a), "planner_deceleration_mps2": deceleration, "planner_jerk_mps3": _norm(j), "planner_lateral_acceleration_mps2": command_lateral,
            "gt_tracking_error_m": truth_error if gt_pair_valid else None, "command_velocity_error_mps": command_v_error if gt_pair_valid else None, "lio_position_error_gt_m": lio_error if lio_gt_pair_valid else None, "lio_position_error_gt_horizontal_m": _horizontal_norm(lio_error_vector) if lio_gt_pair_valid else None, "lio_velocity_error_gt_mps": _norm(lio_velocity_error_vector) if lio_gt_pair_valid else None, "lio_velocity_error_gt_horizontal_mps": _horizontal_norm(lio_velocity_error_vector) if lio_gt_pair_valid else None, "px4_position_error_gt_m": px4_error if px4_position_pair_valid else None, "px4_velocity_error_gt_mps": _norm(_sub(px4_v, gt_v)) if px4_velocity_pair_valid else None, "px4_lio_position_residual_m": lio_px4 if px4_position_pair_valid and px4_lio_pair_valid else None, "px4_lio_velocity_residual_mps": lio_v_px4 if px4_velocity_pair_valid and px4_lio_pair_valid else None, "px4_effective_setpoint_error_gt_m": eff_truth if controller_position_pair_valid else None, "px4_effective_tracking_error_m": _norm(_sub(px4_p, eff_p)) if controller_position_pair_valid else None, "delta_v_px4_controller_mps": delta_v if controller_velocity_pair_valid else None, "delta_a_px4_controller_mps2": delta_a if controller_acceleration_pair_valid else None,
            "source_age_px4_ms": px4.get("source_age_ms", source_ages.get("px4", MISSING)), "source_age_ground_truth_ms": gt_state["source_age_ms"] if gt_state else MISSING, "source_age_lio_ms": lio_state["source_age_ms"] if lio_state else MISSING, "source_age_px4_effective_sp_ms": px4sp.get("source_age_ms", source_ages.get("px4_effective_sp", MISSING)),
            "ground_truth_frame_id": gt_state.get("frame_id", MISSING) if gt_state else MISSING, "ground_truth_child_frame_id": gt_state.get("child_frame_id", MISSING) if gt_state else MISSING, "lio_frame_id": lio_state.get("frame_id", MISSING) if lio_state else MISSING, "lio_child_frame_id": lio_state.get("child_frame_id", MISSING) if lio_state else MISSING, "ground_truth_epoch": gt_state.get("epoch", MISSING) if gt_state else MISSING, "lio_epoch": lio_state.get("epoch", MISSING) if lio_state else MISSING, "gt_pair_valid": gt_pair_valid, "lio_gt_pair_valid": lio_gt_pair_valid, "px4_gt_pair_valid": px4_gt_pair_valid, "px4_position_pair_valid": px4_position_pair_valid, "px4_velocity_pair_valid": px4_velocity_pair_valid, "px4_lio_pair_valid": px4_lio_pair_valid, "controller_pair_valid": controller_pair_valid, "controller_position_pair_valid": controller_position_pair_valid, "controller_velocity_pair_valid": controller_velocity_pair_valid, "controller_acceleration_pair_valid": controller_acceleration_pair_valid, "px4_source_valid": px4_source_valid, "px4_source_reason": px4_source_reason, "px4_effective_source_valid": px4_effective_source_valid, "px4_effective_source_reason": px4_effective_source_reason, "reset_valid": reset_valid, "ground_truth_identity_valid": gt_identity_valid, "lio_identity_valid": lio_identity_valid, "px4_identity_valid": px4_identity_valid, "alignment_valid": alignment_valid, "alignment_reason": "aligned" if alignment_valid else (gt_reason if gt_state is None else lio_reason if lio_state is None else "epoch_reset_discontinuity" if not epoch_valid else "reset_boundary" if not reset_valid else "calibration_identity_changed" if not (gt_identity_valid and lio_identity_valid and px4_identity_valid) else "missing_lio_gt_pair_witness"),
            "px4_xy_valid": px4.get("xy_valid", MISSING), "px4_z_valid": px4.get("z_valid", MISSING), "px4_v_xy_valid": px4.get("v_xy_valid", MISSING), "px4_v_z_valid": px4.get("v_z_valid", MISSING), "failsafe": (item.get("status") or {}).get("failsafe", MISSING), "lio_status": lio_status, "lio_navigation_valid": lio_navigation_valid,
        })
        derived[-1]["segment_boundary"] = bool(previous_segment and previous_segment != item.get("segment_id"))
        previous_segment = str(item.get("segment_id", ""))
    return derived, {
        "initial_px4_local_ned": initial_px4,
        "lio_to_gt_offset_ned": lio_to_gt_offset,
        "gt_to_px4_offset_ned": gt_to_px4_offset,
        "calibration_witness": bool(lio_to_gt_offset is not None or gt_to_px4_offset is not None),
        "lio_gt_calibration_identities": lio_gt_calibration_ids,
        "px4_gt_calibration_identities": px4_gt_calibration_ids,
        "px4_mapping_witness": any(row.get("px4_source_valid") is True for row in derived),
        "maximum_synchronization_tolerance_ms": tolerance_ms if tolerance_ms is not None else MISSING,
    }


def _metric(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    return _stats([float(row[key]) for row in rows if isinstance(row.get(key), (int, float)) and math.isfinite(float(row[key]))])


def _segment_evidence_reason(segment: dict[str, Any], rows: list[dict[str, Any]]) -> str | None:
    """Explain incomplete required evidence without collapsing metric coverage."""
    if not rows or segment.get("segment_kind") not in {"longitudinal", "arc"}:
        return None
    missing: list[str] = []
    if not all(row.get("gt_tracking_error_m") is not None for row in rows):
        missing.append("GT/PX4 calibration pair witness")
    if not all(row.get("lio_gt_pair_valid") is True for row in rows):
        missing.append("LIO/GT pair witness")
    if not all(row.get("lio_navigation_valid") is True for row in rows):
        missing.append("LIO diagnostic health")
    if not all(row.get("px4_source_valid") is True for row in rows):
        missing.append("PX4 source clock/time")
    if not all(row.get("px4_position_pair_valid") is True for row in rows):
        missing.append("PX4 position validity")
    if not all(row.get("px4_velocity_pair_valid") is True for row in rows):
        missing.append("PX4 velocity validity")
    if not all(row.get("px4_effective_source_valid") is True for row in rows):
        missing.append("PX4 effective setpoint source")
    if not missing and segment.get("required_evidence_coverage", 0.0) < 1.0:
        missing.append("complete per-sample source coverage")
    return f"segment {segment.get('segment_id', MISSING)}: missing " + ", ".join(missing) if missing else None


def _evidence_status_reasons(rows: list[dict[str, Any]], segments: list[dict[str, Any]]) -> list[str]:
    reasons = {str(row.get("alignment_reason")) for row in rows if row.get("alignment_valid") is not True}
    for segment in segments:
        segment_rows = [row for row in rows if str(row.get("segment_id")) == str(segment.get("segment_id"))]
        reason = _segment_evidence_reason(segment, segment_rows)
        if reason:
            reasons.add(reason)
    return sorted(reasons)


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
        truth_rows = [row for row in segment if isinstance(row.get("gt_tracking_error_m"), (int, float)) and math.isfinite(float(row["gt_tracking_error_m"]))]
        truth = _metric(truth_rows, "gt_tracking_error_m")
        lio_observed = any(row.get("lio_navigation_valid") != MISSING for row in segment)
        required_rows = [row for row in segment if isinstance(row.get("lio_navigation_valid"), bool) and row.get("lio_navigation_valid") is True and row.get("gt_tracking_error_m") is not None and row.get("lio_gt_pair_valid") is True and row.get("px4_source_valid") is True and row.get("px4_position_pair_valid") is True and row.get("px4_velocity_pair_valid") is True and row.get("px4_effective_source_valid") is True]
        coverage_complete = bool(required_rows) and len(required_rows) == len(segment)
        safety_bad = any(row.get("failsafe") is True or row.get("px4_xy_valid") is False or row.get("lio_navigation_valid") is False for row in segment)
        healthy = coverage_complete and lio_observed and not safety_bad
        if safety_bad:
            quality = "UNUSABLE"
        elif not coverage_complete or not lio_observed:
            quality = "INSUFFICIENT_EVIDENCE"
        elif isinstance(truth.get("p95"), (int, float)) and isinstance(truth.get("max"), (int, float)) and truth["p95"] <= 0.10 and truth["max"] <= 0.175:
            quality = "GOOD"
        elif healthy and isinstance(truth.get("p95"), (int, float)) and truth["p95"] <= 0.25:
            quality = "MARGINAL"
        else:
            quality = "UNUSABLE"
        reports.append({"segment_id": segment_id, "segment_kind": segment[0]["segment_kind"], "quality": quality, "sample_count": len(segment), "valid_sample_count": len(truth_rows), "required_valid_sample_count": len(required_rows), "alignment_coverage": len(truth_rows) / len(segment) if segment else 0.0, "required_evidence_coverage": len(required_rows) / len(segment) if segment else 0.0, "metrics": {key: _metric(segment, key) for key in ("planner_speed_mps", "planner_acceleration_mps2", "planner_deceleration_mps2", "planner_jerk_mps3", "planner_lateral_acceleration_mps2", "gt_tracking_error_m", "lio_position_error_gt_m", "lio_position_error_gt_horizontal_m", "lio_velocity_error_gt_mps", "lio_velocity_error_gt_horizontal_mps", "px4_position_error_gt_m", "px4_velocity_error_gt_mps", "px4_effective_tracking_error_m", "delta_v_px4_controller_mps", "delta_a_px4_controller_mps2", "px4_effective_setpoint_error_gt_m")}, "estimator_health_valid": healthy, "lio_health_observed": lio_observed, "requested_radius_m": segment[0].get("radius_m", MISSING), "requested_speed_mps": segment[0].get("requested_speed_mps", MISSING)})
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
        output_dir.mkdir(parents=True, exist_ok=True)
        result = {"run_id": run_dir.name, "profile": str(summary.get("profile", "unknown")), "sample_count": 0, "evidence_status": "INSUFFICIENT_EVIDENCE", "evidence_status_reasons": ["no characterization samples"], "initial_frames": initial, "summary": summary, "segments": [], "envelope": {"status": "NOT_IDENTIFIED", "criterion": {"evidence_required": True}}, "figures": [], "trace": MISSING}
        (output_dir / "analysis.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return result
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
    motion_segments = [item for item in segments if item["segment_kind"] in {"longitudinal", "arc"}]
    evidence_status = "VALID" if motion_segments and all(item.get("required_evidence_coverage", 0.0) >= 1.0 for item in motion_segments) else "INSUFFICIENT_EVIDENCE"
    envelope: dict[str, Any] = {"status": "IDENTIFIED" if good and evidence_status == "VALID" else "NOT_IDENTIFIED", "criterion": {"gt_tracking_p95_m": 0.10, "gt_tracking_max_m": 0.175, "no_recovery": True, "estimator_health_valid": True, "evidence_status": evidence_status}, "run_id": run_dir.name, "profile": profile, "segments": segments}
    if good:
        def maximum(metric: str) -> dict[str, Any]:
            candidates = [(item["metrics"][metric]["max"], item["segment_id"]) for item in good if isinstance(item["metrics"].get(metric, {}).get("max"), (int, float))]
            return {"value": max(candidates)[0], "segment_id": max(candidates)[1], "run_id": run_dir.name} if candidates else {"value": MISSING}
        envelope["v_nominal_max_mps"] = maximum("planner_speed_mps")
        envelope["a_long_nominal_max_mps2"] = maximum("planner_acceleration_mps2")
        envelope["decel_nominal_max_mps2"] = maximum("planner_deceleration_mps2")
        envelope["jerk_nominal_max_mps3"] = maximum("planner_jerk_mps3")
        envelope["a_lateral_nominal_max_mps2"] = maximum("planner_lateral_acceleration_mps2")
    result = {"run_id": run_dir.name, "profile": profile, "sample_count": len(rows), "evidence_status": evidence_status, "evidence_status_reasons": _evidence_status_reasons(rows, segments), "initial_frames": initial, "summary": summary, "segments": segments, "envelope": envelope, "figures": figures, "trace": str(csv_path)}
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

#!/usr/bin/env python3
"""Offline, scenario-scoped decomposition of retained tracking evidence.

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

try:
    import yaml
except ImportError:  # pragma: no cover - the runtime image normally supplies PyYAML
    yaml = None


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


def integer(value: Any) -> int | None:
    number = finite(value)
    return int(number) if number is not None else None


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


def frame_kind(value: Any, declared_convention: Any = None) -> str | None:
    """Return a convention only when the retained producer declares it.

    Frame identifiers are names, not axis conventions.  In particular,
    ``map``, ``odom`` and ``lio_odom`` are intentionally not interpreted here.
    """
    if not isinstance(declared_convention, str):
        return None
    convention = declared_convention.strip().upper()
    return convention if convention in {"ENU", "NED", "BODY_FLU"} else None


def quat_rotate(q: list[float] | None, v: list[float] | None) -> list[float] | None:
    """Rotate a body vector by q_xyzw into the odometry/world frame."""
    if q is None or v is None or len(q) != 4:
        return None
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12 or abs(n - 1.0) > 1e-3:
        return None
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


def jsonl_integrity(path: Path) -> list[str]:
    """Report malformed retained rows separately from the usable subset."""
    errors: list[str] = []
    if not path.is_file():
        return [f"missing artifact: {path.name}"]
    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            errors.append(f"{path.name}: malformed JSON at line {line_number}")
            continue
        if not isinstance(value, dict):
            errors.append(f"{path.name}: non-object row at line {line_number}")
    return errors


def read_csv(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def retained_tracking_threshold_info(
    root: Path,
    scenario_records: list[dict[str, Any]] | None = None,
    sample_records: list[dict[str, Any]] | None = None,
) -> tuple[float | None, list[str], list[str]]:
    """Resolve an explicitly retained limit and preserve its provenance.

    A threshold is run-wide only when every retained occurrence agrees.  In
    particular, this does not promote a single decision-trace value over a
    conflicting command value.
    """
    candidates: list[tuple[str, Any]] = []
    for filename in ("metadata.json", "report.json", "benchmark_metrics.json", "runtime.json"):
        path = root / filename
        if not path.is_file():
            continue
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(value, dict):
            for key in ("retained_tracking_limit_m", "tracking_certificate_threshold_m"):
                if key in value:
                    candidates.append((f"{filename}:{key}", value[key]))
            acceptance = value.get("acceptance")
            if isinstance(acceptance, dict):
                for key in ("retained_tracking_limit_m", "tracking_certificate_threshold_m"):
                    if key in acceptance:
                        candidates.append((f"{filename}:acceptance.{key}", acceptance[key]))
    for index, record in enumerate(scenario_records or [], 1):
        payload = record.get("payload") if isinstance(record, dict) else None
        value = payload.get("retained_tracking_limit_m") if isinstance(payload, dict) else None
        if value is not None:
            candidates.append((f"scenario.jsonl:{index}:retained_tracking_limit_m", value))
    for index, record in enumerate(sample_records or [], 1):
        payload = record.get("payload") if isinstance(record, dict) else None
        values: list[tuple[str, Any]] = []
        if isinstance(payload, dict):
            values.append(("payload", payload.get("retained_tracking_limit_m")))
            statuses = payload.get("statuses")
            if isinstance(statuses, list):
                for status_index, status in enumerate(statuses):
                    if isinstance(status, dict) and isinstance(status.get("values"), dict):
                        values.append((f"status[{status_index}].values", status["values"].get("retained_tracking_limit_m")))
        for label, value in values:
            if value is not None:
                candidates.append((f"samples.jsonl:{index}:{label}:retained_tracking_limit_m", value))
    valid: list[tuple[str, float]] = []
    errors: list[str] = []
    for provenance, raw in candidates:
        value = finite(raw)
        if value is None or value <= 0:
            errors.append(f"invalid retained tracking threshold: {provenance}")
            continue
        valid.append((provenance, value))
    distinct = {value for _, value in valid}
    if len(distinct) > 1:
        details = ", ".join(f"{provenance}={value:g}" for provenance, value in valid)
        errors.append(f"conflicting retained tracking thresholds: {details}")
        return None, [provenance for provenance, _ in valid], errors
    if not valid:
        return None, [], errors
    return valid[0][1], [provenance for provenance, _ in valid], errors


def retained_tracking_threshold(root: Path) -> float | None:
    """Compatibility wrapper for callers that only need the value."""
    return retained_tracking_threshold_info(root)[0]


def artifact_scope(root: Path) -> tuple[dict[str, Any], list[str]]:
    """Read scope/provenance from the retained artifact; never synthesize it."""
    errors: list[str] = []
    metadata_path = root / "metadata.json"
    metadata: dict[str, Any] = {}
    if metadata_path.is_file():
        try:
            loaded = json.loads(metadata_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                metadata = loaded
            else:
                errors.append("metadata.json is not an object")
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f"invalid metadata: {error}")
    else:
        errors.append("missing artifact: metadata.json")
    config: dict[str, Any] = {}
    config_path = root / "scenario_config.yaml"
    if yaml is not None and config_path.is_file():
        try:
            config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            errors.append(f"invalid scenario config: {error}")
    elif yaml is not None:
        errors.append("missing artifact: scenario_config.yaml")
    scenario = config.get("scenario", {}) if isinstance(config, dict) else {}
    if not isinstance(scenario, dict):
        scenario = {}
    experiment_id = metadata.get("experiment_id")
    run_id = metadata.get("run_id")
    environment = metadata.get("environment") if isinstance(metadata.get("environment"), dict) else {}
    mission_planning = metadata.get("mission_planning") if isinstance(metadata.get("mission_planning"), dict) else {}
    map_profile = scenario.get("map_profile", environment.get("map_profile"))
    map_scene = scenario.get("map_scene")
    route = scenario.get("route_name", scenario.get("route"))
    mission_file = metadata.get("mission_file")
    # A metadata mission_file may point at the live checkout.  It is retained
    # only as provenance; never read it to fill a missing scope field.
    retained_mission = root / "resolved_mission.yaml"
    if not route and retained_mission.is_file() and yaml is not None:
        mission_path = retained_mission
        try:
            mission_data = yaml.safe_load(mission_path.read_text(encoding="utf-8")) or {}
            mission_section = mission_data.get("mission") if isinstance(mission_data, dict) else {}
            route = mission_section.get("id") if isinstance(mission_section, dict) else None
        except (OSError, yaml.YAMLError):
            errors.append("invalid retained mission file")
    elif not route and mission_file and yaml is not None:
        # Keep the reason explicit when only an external path was recorded.
        if Path(str(mission_file)).is_absolute():
            errors.append("route missing from retained artifact; external mission path not read")
        else:
            try:
                mission_data = yaml.safe_load((root / str(mission_file)).read_text(encoding="utf-8")) or {}
                mission_section = mission_data.get("mission") if isinstance(mission_data, dict) else {}
                route = mission_section.get("id") if isinstance(mission_section, dict) else None
            except (OSError, yaml.YAMLError):
                errors.append("invalid retained mission file")
    speed = metadata.get("requested_cruise_speed_mps", mission_planning.get("requested_cruise_speed_mps", scenario.get("expected_max_velocity_mps")))
    pre_window_s = scenario.get("analysis_pre_window_s")
    post_window_s = scenario.get("analysis_post_window_s")
    runtime_config = config.get("runtime", {}) if isinstance(config, dict) else {}
    thresholds = runtime_config.get("thresholds", {}) if isinstance(runtime_config, dict) else {}
    tolerance_ms = scalar_or_missing(thresholds.get("maximum_synchronization_tolerance_ms"))
    frame_conventions = metadata.get("frame_conventions", scenario.get("frame_conventions"))
    if not isinstance(frame_conventions, dict):
        frame_conventions = {}
    common_clock = metadata.get("common_source_clock", scenario.get("common_source_clock"))
    px4_mapping = metadata.get("px4_to_ros_mapping", scenario.get("px4_to_ros_mapping"))
    generation_boundary_continuity = metadata.get(
        "generation_boundary_continuity", scenario.get("generation_boundary_continuity"))
    command_interruption_contract = metadata.get(
        "command_interruption_contract", scenario.get("command_interruption_contract"))
    scope = {"experiment_id": experiment_id or MISSING, "run_id": run_id or MISSING, "map_profile": map_profile or MISSING, "map_scene": map_scene or MISSING, "route": route or MISSING, "requested_speed_mps": scalar_or_missing(speed), "maximum_synchronization_tolerance_ms": tolerance_ms, "analysis_pre_window_s": scalar_or_missing(pre_window_s), "analysis_post_window_s": scalar_or_missing(post_window_s), "common_source_clock": common_clock or MISSING, "frame_conventions": frame_conventions, "px4_to_ros_mapping": px4_mapping if isinstance(px4_mapping, dict) else MISSING, "generation_boundary_continuity": generation_boundary_continuity if isinstance(generation_boundary_continuity, dict) else MISSING, "command_interruption_contract": command_interruption_contract if isinstance(command_interruption_contract, dict) else MISSING, "metadata_path": str(metadata_path), "scenario_config_path": str(config_path)}
    for key in ("experiment_id", "run_id", "map_profile", "map_scene", "route", "requested_speed_mps", "maximum_synchronization_tolerance_ms", "common_source_clock"):
        if scope[key] == MISSING:
            errors.append(f"missing scope field: {key}")
    lio_contract = frame_conventions.get("propagated_odometry")
    if not isinstance(lio_contract, dict):
        errors.append("missing scope field: frame_conventions.propagated_odometry")
    else:
        for key in ("frame_id", "child_frame_id", "position_convention", "velocity_convention"):
            if not isinstance(lio_contract.get(key), str) or not lio_contract.get(key).strip():
                errors.append(f"missing scope field: frame_conventions.propagated_odometry.{key}")
    if not isinstance(px4_mapping, dict) or px4_mapping.get("status") != "VALID":
        errors.append("missing scope field: px4_to_ros_mapping")
    if run_id not in (None, MISSING) and root.name not in str(run_id) and root.name != "":
        errors.append("artifact directory and recorded run_id disagree")
    return scope, errors


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
           max_gap_ns: int | None = None) -> tuple[Any, int | None, float | None, str]:
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
    if max_gap_ns is None or t1 - t0 > max_gap_ns:
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
    previous_stamp: int | None = None
    for record in records:
        if record.get("stream") != name:
            continue
        payload = record.get("payload", {})
        if not isinstance(payload, dict):
            continue
        stamp = integer(payload.get("stamp_ns", record.get("timestamp_ns", 0))) or 0
        if name in ("local_position", "px4_local_position_setpoint"):
            timestamp_us = integer(payload.get("timestamp_us"))
            stamp = timestamp_us * 1000 if timestamp_us is not None else 0
        if stamp <= 0:
            continue
        if previous_stamp is not None and stamp < previous_stamp:
            result.setdefault("_errors", []).append("source_timestamp_regression")
        previous_stamp = stamp
        if name == "propagated_odometry":
            position_raw = vec(payload.get("position")); velocity_raw = vec(payload.get("linear_velocity"))
            position_kind = frame_kind(payload.get("frame_id"), payload.get("frame_convention", payload.get("position_convention")))
            velocity_kind = frame_kind(payload.get("child_frame_id"), payload.get("child_frame_convention", payload.get("velocity_convention")))
            position = enu_to_ned(position_raw) if position_kind == "ENU" else position_raw if position_kind == "NED" else None
            world_velocity = quat_rotate(vec4(payload.get("q_xyzw")), velocity_raw) if velocity_kind == "BODY_FLU" else velocity_raw if velocity_kind in {"ENU", "NED"} and velocity_kind == position_kind else None
            velocity = enu_to_ned(world_velocity) if position_kind == "ENU" else world_velocity if position_kind == "NED" else None
            result.setdefault("position", []).append((stamp, position))
            result.setdefault("velocity", []).append((stamp, velocity))
            result.setdefault("quaternion", []).append((stamp, vec4(payload.get("q_xyzw"))))
            result.setdefault("position_frame", []).append((stamp, payload.get("frame_id")))
            result.setdefault("velocity_frame", []).append((stamp, payload.get("child_frame_id")))
            result.setdefault("position_convention", []).append((stamp, payload.get("frame_convention", payload.get("position_convention"))))
            result.setdefault("velocity_convention", []).append((stamp, payload.get("child_frame_convention", payload.get("velocity_convention"))))
            result.setdefault("clock", []).append((stamp, payload.get("source_clock")))
            epoch = payload.get("localization_epoch", payload.get("epoch"))
            reset_counter = payload.get("reset_counter")
            result.setdefault("epoch", []).append((stamp, epoch))
            result.setdefault("reset_counter", []).append((stamp, reset_counter))
            result.setdefault("continuity", []).append((stamp, (epoch, reset_counter)))
        elif name == "local_position":
            clock = payload.get("source_clock")
            result.setdefault("position", []).append((stamp, vec([payload.get("x_ned_m"), payload.get("y_ned_m"), payload.get("z_ned_m")])))
            result.setdefault("velocity", []).append((stamp, vec([payload.get("vx_ned_m_s"), payload.get("vy_ned_m_s"), payload.get("vz_ned_m_s")])))
            result.setdefault("clock", []).append((stamp, clock))
            result.setdefault("position_frame", []).append((stamp, payload.get("frame_id")))
            result.setdefault("velocity_frame", []).append((stamp, payload.get("velocity_frame")))
            result.setdefault("position_convention", []).append((stamp, payload.get("position_convention")))
            result.setdefault("velocity_convention", []).append((stamp, payload.get("velocity_convention")))
            result.setdefault("epoch", []).append((stamp, payload.get("epoch", payload.get("reset_counter"))))
            result.setdefault("reset_counter", []).append((stamp, payload.get("reset_counter")))
            result.setdefault("continuity", []).append((stamp, (payload.get("epoch"), payload.get("reset_counter"))))
        else:
            clock = payload.get("source_clock")
            result.setdefault("position", []).append((stamp, vec(payload.get("position_ned"))))
            result.setdefault("velocity", []).append((stamp, vec(payload.get("velocity_ned"))))
            result.setdefault("acceleration", []).append((stamp, vec(payload.get("acceleration_ned"))))
            result.setdefault("position_frame", []).append((stamp, payload.get("frame_id")))
            result.setdefault("velocity_frame", []).append((stamp, payload.get("velocity_frame")))
            result.setdefault("position_convention", []).append((stamp, payload.get("position_convention")))
            result.setdefault("velocity_convention", []).append((stamp, payload.get("velocity_convention")))
            result.setdefault("clock", []).append((stamp, clock))
    for key, values in result.items():
        if not key.startswith("_"):
            values.sort(key=lambda item: item[0])
    return result


def apply_clock_mapping(series: dict[str, list[tuple[int, Any]]], mapping: Any,
                        target_clock: str | None) -> tuple[dict[str, list[tuple[int, Any]]], str]:
    """Apply only a retained, explicit producer-clock mapping."""
    if not isinstance(mapping, dict) or mapping.get("status") != "VALID":
        return series, "missing_clock_mapping"
    source_clock = mapping.get("source_clock")
    if not isinstance(source_clock, str) or not source_clock or mapping.get("target_clock") != target_clock:
        return series, "invalid_clock_mapping_contract"
    scale = finite(mapping.get("scale_to_ros_ns"))
    offset = finite(mapping.get("offset_ns"))
    if scale is None or offset is None or scale <= 0:
        return series, "invalid_clock_mapping_contract"
    if mapping.get("timestamp_relation") not in {"numeric_identity", "explicit_affine"}:
        return series, "invalid_clock_mapping_contract"
    clocks = dict(series.get("clock", []))
    if not clocks or any(clock != source_clock for clock in clocks.values()):
        return series, "source_clock_mapping_mismatch"
    mapped: dict[str, list[tuple[int, Any]]] = {}
    for key, values in series.items():
        if key.startswith("_"):
            mapped[key] = values
            continue
        mapped[key] = [(int(round(stamp * scale + offset)), value) for stamp, value in values]
    mapped["clock"] = [(stamp, target_clock) for stamp, _ in mapped.get("clock", [])]
    return mapped, "mapped"


def metadata_at(series: list[tuple[int, Any]], timestamp_ns: int,
                max_gap_ns: int | None = None) -> Any:
    """Return metadata only when its enclosing samples agree."""
    if not series:
        return None
    if timestamp_ns < series[0][0] or timestamp_ns > series[-1][0]:
        return None
    lo_index = 0
    while lo_index + 1 < len(series) and series[lo_index + 1][0] <= timestamp_ns:
        lo_index += 1
    lo = series[lo_index]
    if lo[0] == timestamp_ns:
        return lo[1]
    hi = series[min(lo_index + 1, len(series) - 1)]
    if max_gap_ns is None or hi[0] - lo[0] > max_gap_ns or lo[1] != hi[1]:
        return None
    return lo[1]


def epoch_continuous(series: list[tuple[int, Any]]) -> bool:
    """Check one stream's epoch without comparing estimator namespaces."""
    previous: int | None = None
    observed = False
    for _, raw in series:
        value = finite(raw)
        if value is None:
            continue
        observed = True
        if previous is not None and int(value) != previous:
            return False
        previous = int(value)
    return observed


def continuity_continuous(series: list[tuple[int, Any]]) -> bool:
    """Reject interpolation across any observed epoch/reset identity change."""
    previous: Any = None
    observed = False
    for _, value in series:
        if value is None or (isinstance(value, tuple) and all(item is None for item in value)):
            continue
        if observed and value != previous:
            return False
        observed = True
        previous = value
    return observed


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
        payload = record.get("payload")
        if not isinstance(payload, dict) or not isinstance(payload.get("statuses"), list):
            continue
        for status in payload["statuses"]:
            if not isinstance(status, dict):
                continue
            if status.get("name") != "navigation_runtime/planner" or status.get("message") != "DECISION_TRACE":
                continue
            values = status.get("values", {})
            if isinstance(values, dict):
                timestamp_ns = integer(record.get("timestamp_ns"))
                if timestamp_ns is not None and timestamp_ns > 0:
                    result.append({"timestamp_ns": timestamp_ns, **values})
    return sorted(result, key=lambda x: x["timestamp_ns"])


def value_at(row: dict[str, Any], key: str) -> Any:
    value = row.get(key)
    if value is None:
        return MISSING
    return value


def mode_at(mode_rows: list[dict[str, Any]], timestamp_ns: int) -> str:
    latest = None
    for row in mode_rows:
        t = integer(row.get("timestamp_ns"))
        if t is None:
            continue
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
             previous: dict[str, Any] | None, max_gap_ns: int | None = None,
             frame_contract: dict[str, Any] | None = None,
             common_clock: str | None = None) -> dict[str, Any]:
    t = pva["timestamp_ns"]
    lp, ls, la, lm = interp(lio.get("position", []), t, max_gap_ns)
    lv, lvs, lva, lvm = interp(lio.get("velocity", []), t, max_gap_ns)
    pp, ps, pa, pm = interp(px4.get("position", []), t, max_gap_ns)
    pv, pvs, pvaa, pvm = interp(px4.get("velocity", []), t, max_gap_ns)
    ep, es, epa, em = interp(effective.get("position", []), t, max_gap_ns)
    ev, evs, eva, evm = interp(effective.get("velocity", []), t, max_gap_ns)
    ea, eas, eaa, eam = interp(effective.get("acceleration", []), t, max_gap_ns)
    lio_frame = metadata_at(lio.get("position_frame", []), t, max_gap_ns)
    lio_velocity_frame = metadata_at(lio.get("velocity_frame", []), t, max_gap_ns)
    lio_position_convention = metadata_at(lio.get("position_convention", []), t, max_gap_ns)
    lio_velocity_convention = metadata_at(lio.get("velocity_convention", []), t, max_gap_ns)
    lio_epoch = metadata_at(lio.get("epoch", []), t, max_gap_ns)
    lio_continuity = metadata_at(lio.get("continuity", []), t, max_gap_ns)
    lio_kind = frame_kind(lio_frame, lio_position_convention)
    lio_velocity_kind = frame_kind(lio_velocity_frame, lio_velocity_convention)
    if isinstance(frame_contract, dict):
        if lio_frame != frame_contract.get("frame_id") or lio_velocity_frame != frame_contract.get("child_frame_id"):
            lio_kind = lio_velocity_kind = None
        elif lio_position_convention != frame_contract.get("position_convention") or lio_velocity_convention != frame_contract.get("velocity_convention"):
            lio_kind = lio_velocity_kind = None
    px4_clock = metadata_at(px4.get("clock", []), t, max_gap_ns)
    effective_clock = metadata_at(effective.get("clock", []), t, max_gap_ns)
    px4_state_valid = common_clock is not None and px4_clock == common_clock and pp is not None and pv is not None
    controller_setpoint_valid = common_clock is not None and effective_clock == common_clock and ep is not None and ev is not None and ea is not None
    if not px4_state_valid:
        pp = pv = None
    if not controller_setpoint_valid:
        ep = ev = ea = None
    planner_p = pva.get("position")
    planner_v = pva.get("velocity")
    planner_a = pva.get("acceleration")
    planner_j = pva.get("jerk")
    planner_ned_v = enu_to_ned(planner_v)
    planner_ned_a = enu_to_ned(planner_a)
    planner_lio_ned_p = enu_to_ned(planner_p)
    planner_ned_p = add(planner_lio_ned_p, offset)
    analytic_role = integer(pva.get("analytic_sample_role"))
    navigation_role = integer(pva.get("trajectory_flag"))
    recovery_state = integer(pva.get("execution_recovery_state"))
    frame_p = norm(sub(pp, add(lp, offset))) if pp is not None and lp is not None and offset is not None else None
    frame_v = norm(sub(pv, lv)) if pv is not None and lv is not None else None
    px4_error = norm(sub(planner_ned_p, pp)) if planner_ned_p is not None and pp is not None else None
    delta_v = norm(sub(ev, planner_ned_v)) if ev is not None and planner_ned_v is not None else None
    delta_a = norm(sub(ea, planner_ned_a)) if ea is not None and planner_ned_a is not None else None
    command_gap = (t - previous["timestamp_ns"]) / 1e6 if previous else None
    setpoint_gap = (es - previous.get("PX4_effective_source_ns")) / 1e6 if previous and es is not None and previous.get("PX4_effective_source_ns") is not None else None
    continuity_ok = previous is None or previous.get("LIO_continuity") == lio_continuity
    lio_evidence_valid = bool(lp is not None and lv is not None and lio_kind in {"ENU", "NED"} and lio_velocity_kind in {"BODY_FLU", "ENU", "NED"} and lio_continuity is not None and continuity_ok and common_clock is not None and metadata_at(lio.get("clock", []), t, max_gap_ns) == common_clock)
    evidence_valid = lio_evidence_valid
    required_evidence_valid = bool(lio_evidence_valid and px4_state_valid and controller_setpoint_valid)
    return {
        "timestamp_ns": t,
        "bundle_generation": value_at(pva, "trajectory_generation"),
        "trajectory_time_s": value_at(pva, "trajectory_time_s"),
        "analytic_role": ROLE.get(analytic_role, MISSING) if analytic_role is not None else MISSING,
        "NavigationCommand_role": ROLE.get(navigation_role, MISSING) if navigation_role is not None else MISSING,
        "recovery_state": RECOVERY.get(recovery_state, MISSING) if recovery_state is not None else MISSING,
        "safety_suffix_active": value_at(pva, "safety_suffix_active"),
        "planner_position": planner_p or MISSING,
        "planner_velocity": planner_v or MISSING,
        "planner_acceleration": planner_a or MISSING,
        "planner_jerk": planner_j or MISSING,
        "LIO_position": lp or MISSING,
        "LIO_velocity": lv or MISSING,
        "LIO_frame_id": lio_frame or MISSING,
        "LIO_child_frame_id": lio_velocity_frame or MISSING,
        "LIO_position_convention": lio_position_convention or MISSING,
        "LIO_velocity_convention": lio_velocity_convention or MISSING,
        "LIO_epoch": lio_epoch if lio_epoch is not None else MISSING,
        "LIO_continuity": lio_continuity if lio_continuity is not None else MISSING,
        "evidence_valid": evidence_valid,
        "required_evidence_valid": required_evidence_valid,
        "LIO_evidence_valid": lio_evidence_valid,
        "PX4_state_evidence_valid": px4_state_valid,
        "PX4_controller_setpoint_evidence_valid": controller_setpoint_valid,
        "retained_tracking_limit_m": scalar_or_missing(pva.get("retained_tracking_limit_m")),
        "PX4_input_trajectory_position": MISSING,
        "PX4_input_trajectory_velocity": MISSING,
        "PX4_input_trajectory_acceleration": MISSING,
        "PX4_effective_position_setpoint": ep or MISSING,
        "PX4_effective_velocity_setpoint": ev or MISSING,
        "PX4_effective_acceleration_setpoint": ea or MISSING,
        "PX4_position": pp or MISSING,
        "PX4_velocity": pv or MISSING,
        "PX4_timestamp_domain": px4_clock or MISSING,
        "PX4_effective_timestamp_domain": effective_clock or MISSING,
        "aligned_LIO_tracking_error_m": scalar_or_missing(norm(sub(planner_lio_ned_p, lp))) if lio_evidence_valid else MISSING,
        "px4_tracking_error_m": scalar_or_missing(px4_error) if px4_state_valid else MISSING,
        "frame_position_residual_m": scalar_or_missing(frame_p) if lio_evidence_valid and px4_state_valid else MISSING,
        "frame_velocity_residual_mps": scalar_or_missing(frame_v) if lio_evidence_valid and px4_state_valid else MISSING,
        "delta_v_px4_controller_mps": scalar_or_missing(delta_v) if controller_setpoint_valid else MISSING,
        "delta_a_px4_controller_mps2": scalar_or_missing(delta_a) if controller_setpoint_valid else MISSING,
        "command_gap_ms": scalar_or_missing(command_gap),
        "setpoint_gap_ms": scalar_or_missing(setpoint_gap),
        "external_mode_output_state": mode_at(modes, t),
        "planner_source_method": "scenario.pva_command",
        "LIO_source_method": lm,
        "LIO_position_source_ns": ls if ls is not None else MISSING,
        "LIO_position_source_age_ms": la if la is not None else MISSING,
        "LIO_velocity_source_ns": lvs if lvs is not None else MISSING,
        "LIO_velocity_source_age_ms": lva if lva is not None else MISSING,
        "LIO_velocity_source_semantics": "body_twist_rotated_by_q_xyzw_to_header_frame_then_explicit_header_frame_to_PX4_NED",
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
        # The limit must be retained on this command row itself.  A value
        # found only on a later failure trace cannot be promoted backwards.
        row_threshold = num(row.get("retained_tracking_limit_m"))
        if row_threshold is None or row_threshold != threshold:
            return None
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
             traces_: list[dict[str, Any]], tracking_threshold: float | None = None) -> list[str]:
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
    ax.plot(t, series("px4_tracking_error_m"), label="PX4-frame tracking error")
    if tracking_threshold is not None:
        ax.axhline(tracking_threshold, color="black", linestyle="--", label=f"retained tracking threshold ({tracking_threshold:g} m)")
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


ROOT_FIELDS = ["timestamp_ns", "relative_time_from_T_cross_s", "bundle_generation", "trajectory_time_s", "analytic_role", "NavigationCommand_role", "recovery_state", "safety_suffix_active", "evidence_valid", "LIO_evidence_valid", "required_evidence_valid", "PX4_state_evidence_valid", "PX4_controller_setpoint_evidence_valid", "retained_tracking_limit_m", "LIO_frame_id", "LIO_child_frame_id", "LIO_epoch", "planner_position", "planner_velocity", "planner_acceleration", "planner_jerk", "LIO_position", "LIO_velocity", "PX4_input_trajectory_position", "PX4_input_trajectory_velocity", "PX4_input_trajectory_acceleration", "PX4_effective_position_setpoint", "PX4_effective_velocity_setpoint", "PX4_effective_acceleration_setpoint", "PX4_position", "PX4_velocity", "PX4_timestamp_domain", "PX4_effective_timestamp_domain", "aligned_LIO_tracking_error_m", "px4_tracking_error_m", "frame_position_residual_m", "frame_velocity_residual_mps", "delta_v_px4_controller_mps", "delta_a_px4_controller_mps2", "command_gap_ms", "setpoint_gap_ms", "external_mode_output_state", "planner_source_method", "LIO_source_method", "LIO_position_source_ns", "LIO_position_source_age_ms", "LIO_velocity_source_ns", "LIO_velocity_source_age_ms", "LIO_velocity_source_semantics", "PX4_state_source_method", "PX4_position_source_ns", "PX4_position_source_age_ms", "PX4_velocity_source_ns", "PX4_velocity_source_age_ms", "PX4_effective_source_method", "PX4_effective_source_ns", "PX4_effective_source_age_ms", "planner_velocity_norm_mps", "planner_acceleration_norm_mps2", "planner_jerk_norm_mps3"]


def analyze(root: Path, control: Path | None = None) -> dict[str, Any]:
    samples = read_jsonl(root / "samples.jsonl")
    scenario = read_jsonl(root / "scenario.jsonl")
    scope, scope_errors = artifact_scope(root)
    scope_errors.extend(jsonl_integrity(root / "samples.jsonl"))
    scope_errors.extend(jsonl_integrity(root / "scenario.jsonl"))
    tolerance_ms = num(scope.get("maximum_synchronization_tolerance_ms"))
    max_gap_ns = int(tolerance_ms * 1e6) if tolerance_ms is not None and tolerance_ms >= 0.0 else None
    common_clock = scope.get("common_source_clock") if scope.get("common_source_clock") != MISSING else None
    frame_contract = scope.get("frame_conventions", {}).get("propagated_odometry") if isinstance(scope.get("frame_conventions"), dict) else None
    px4_mapping = scope.get("px4_to_ros_mapping")
    pvas = pva_rows(scenario)
    lio = stream_series(samples, "propagated_odometry")
    px4 = stream_series(samples, "local_position")
    effective = stream_series(samples, "px4_local_position_setpoint")
    px4, px4_mapping_status = apply_clock_mapping(px4, px4_mapping, common_clock)
    effective, effective_mapping_status = apply_clock_mapping(effective, px4_mapping, common_clock)
    for stream_name, stream in (("propagated_odometry", lio), ("local_position", px4), ("px4_local_position_setpoint", effective)):
        for stream_error in stream.get("_errors", []):
            scope_errors.append(f"{stream_name}: {stream_error}")
    if px4.get("position") and px4_mapping_status != "mapped":
        scope_errors.append(f"PX4 source clock mapping: {px4_mapping_status}")
    if effective.get("position") and effective_mapping_status != "mapped":
        scope_errors.append(f"PX4 effective source clock mapping: {effective_mapping_status}")
    mode_rows = read_csv(root / "navigation_mode_status.csv")
    tracking_threshold, tracking_threshold_provenance, threshold_errors = retained_tracking_threshold_info(
        root, scenario, samples)
    scope_errors.extend(threshold_errors)
    # Fixed transform translation is calibrated once at the first active
    # command.  Rotation is the recorded ENU->NED axis permutation.
    first = next((p for p in pvas if p.get("position") is not None and p.get("velocity") is not None), None)
    offset = None
    if first:
        lp, *_ = interp(lio.get("position", []), first["timestamp_ns"], max_gap_ns)
        pp, *_ = interp(px4.get("position", []), first["timestamp_ns"], max_gap_ns)
        px4_clock = metadata_at(px4.get("clock", []), first["timestamp_ns"], max_gap_ns)
        lio_clock = metadata_at(lio.get("clock", []), first["timestamp_ns"], max_gap_ns)
        lio_continuity = metadata_at(lio.get("continuity", []), first["timestamp_ns"], max_gap_ns)
        lio_frame = metadata_at(lio.get("position_frame", []), first["timestamp_ns"], max_gap_ns)
        lio_child_frame = metadata_at(lio.get("velocity_frame", []), first["timestamp_ns"], max_gap_ns)
        lio_position_convention = metadata_at(lio.get("position_convention", []), first["timestamp_ns"], max_gap_ns)
        lio_velocity_convention = metadata_at(lio.get("velocity_convention", []), first["timestamp_ns"], max_gap_ns)
        witness_frame_valid = isinstance(frame_contract, dict) and lio_frame == frame_contract.get("frame_id") and lio_child_frame == frame_contract.get("child_frame_id") and lio_position_convention == frame_contract.get("position_convention") and lio_velocity_convention == frame_contract.get("velocity_convention")
        if lp is not None and pp is not None and px4_clock == common_clock and lio_clock == common_clock and lio_continuity is not None and witness_frame_valid:
            offset = sub(pp, lp)
    all_rows: list[dict[str, Any]] = []
    previous = None
    for pva in pvas:
        row = make_row(pva, lio, px4, effective, mode_rows, offset, previous, max_gap_ns, frame_contract, common_clock)
        all_rows.append(row)
        previous = row
    if not pvas:
        scope_errors.append("row coverage: no retained pva_command samples")
    invalid_rows = len(all_rows) - sum(row.get("required_evidence_valid") is True for row in all_rows)
    if invalid_rows:
        scope_errors.append(f"row coverage: {invalid_rows}/{len(all_rows)} rows lack complete LIO/frame/clock evidence")
    for stream_name, stream in (("LIO", lio), ("PX4", px4), ("PX4 effective", effective)):
        if not stream.get("position"):
            scope_errors.append(f"row coverage: {stream_name} position samples missing or nonfinite")
    lio_continuity_ok = continuity_continuous(lio.get("continuity", []))
    if not lio_continuity_ok:
        scope_errors.append("row continuity: LIO epoch/reset identity is discontinuous")
        for row in all_rows:
            row["evidence_valid"] = False
            row["LIO_evidence_valid"] = False
            row["required_evidence_valid"] = False
            row["aligned_LIO_tracking_error_m"] = MISSING
            row["frame_position_residual_m"] = MISSING
            row["frame_velocity_residual_mps"] = MISSING
            row["alignment_reason"] = "missing_or_discontinuous_lio_epoch"
    tcross = threshold_cross(all_rows, tracking_threshold) if all_rows and tracking_threshold is not None else None
    emergency_activation = next((r["timestamp_ns"] for r in all_rows if r.get("analytic_role") == "EMERGENCY"), None)
    pre_window_ns = int(scope["analysis_pre_window_s"] * 1e9) if isinstance(scope.get("analysis_pre_window_s"), (int, float)) and scope["analysis_pre_window_s"] >= 0 else None
    post_window_ns = int(scope["analysis_post_window_s"] * 1e9) if isinstance(scope.get("analysis_post_window_s"), (int, float)) and scope["analysis_post_window_s"] >= 0 else None
    start = max(all_rows[0]["timestamp_ns"], (tcross or all_rows[0]["timestamp_ns"]) - pre_window_ns) if all_rows and pre_window_ns is not None else (all_rows[0]["timestamp_ns"] if all_rows else 0)
    end = min(all_rows[-1]["timestamp_ns"], (emergency_activation or all_rows[-1]["timestamp_ns"]) + post_window_ns) if all_rows and post_window_ns is not None else (all_rows[-1]["timestamp_ns"] if all_rows else 0)
    rows = [r for r in all_rows if start <= r["timestamp_ns"] <= end]
    for row in rows:
        row["relative_time_from_T_cross_s"] = (row["timestamp_ns"]-tcross)/1e9 if tcross is not None else MISSING
    activations = generation_activations(all_rows)
    first_replan_activation = activations[1] if len(activations) > 1 else None
    trace_rows = traces(samples)
    failure = next((t for t in trace_rows if num(t.get("candidate_result")) not in (None, 0) and num(t.get("emergency_authorization_reason")) not in (None, 0)), None)
    failure_t = failure["timestamp_ns"] if failure else None
    auth_t = failure_t if failure and num(failure.get("emergency_authorization_reason")) not in (None, 0) else None
    event_times = {"bundle_activation_first": activations[0] if activations else None,
                   "first_replan_activation": first_replan_activation,
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
    def rate_between(start_ns: int | None, end_ns: int | None) -> float | None:
        if start_ns is None or end_ns is None or end_ns <= start_ns:
            return None
        left, right = nearest(all_rows, start_ns), nearest(all_rows, end_ns)
        if left is None or right is None:
            return None
        a, b = num(left.get("aligned_LIO_tracking_error_m")), num(right.get("aligned_LIO_tracking_error_m"))
        return (b - a) / ((right["timestamp_ns"] - left["timestamp_ns"]) / 1e9) if a is not None and b is not None and right["timestamp_ns"] > left["timestamp_ns"] else None
    first_t = all_rows[0]["timestamp_ns"] if all_rows else None
    growth_rates = {"run_start_to_first_replan_mps": rate_between(first_t, first_replan_activation),
                    "first_replan_to_tracking_cross_mps": rate_between(first_replan_activation, tcross),
                    "run_start_to_tracking_cross_mps": rate_between(first_t, tcross)}
    mode_pre = sorted(set(r.get("external_mode_output_state") for r in pre_cross))
    mode_nontrack_pre = [x for x in mode_pre if x not in ("TRACK_TRAJECTORY", MISSING)]
    first_boundary = boundary_deltas[0] if boundary_deltas else {}
    relevant_boundaries = [boundary for boundary in boundary_deltas
                           if tcross is None or boundary["timestamp_ns"] < tcross]
    generation_boundary_observed = bool(relevant_boundaries and all(
        all(num(boundary.get(f"delta_{name}")) is not None
            for name in ("position", "velocity", "acceleration", "jerk"))
        for boundary in relevant_boundaries))
    boundary_contract = scope.get("generation_boundary_continuity")
    boundary_limits = {
        "position": finite(boundary_contract.get("max_delta_position_m")) if isinstance(boundary_contract, dict) else None,
        "velocity": finite(boundary_contract.get("max_delta_velocity_mps")) if isinstance(boundary_contract, dict) else None,
        "acceleration": finite(boundary_contract.get("max_delta_acceleration_mps2")) if isinstance(boundary_contract, dict) else None,
        "jerk": finite(boundary_contract.get("max_delta_jerk_mps3")) if isinstance(boundary_contract, dict) else None,
    }
    boundary_contract_complete = isinstance(boundary_contract, dict) and all(
        value is not None and value >= 0 for value in boundary_limits.values())
    boundary_within_contract = bool(generation_boundary_observed and boundary_contract_complete and all(
        abs(num(boundary[f"delta_{name}"])) <= boundary_limits[name]
        for boundary in relevant_boundaries for name in boundary_limits))
    tracking_observed = stats(pre_cross, "aligned_LIO_tracking_error_m")["count"] > 0
    controller_observed = stats(pre_cross, "delta_v_px4_controller_mps")["count"] > 0 or stats(pre_cross, "delta_a_px4_controller_mps2")["count"] > 0
    frame_observed = stats(pre_cross, "frame_position_residual_m")["count"] > 0 and stats(pre_cross, "frame_velocity_residual_mps")["count"] > 0
    mode_observed = bool(pre_cross and all(r.get("external_mode_output_state") != MISSING for r in pre_cross))
    command_gap_observed = bool(pre_cross and all(num(r.get("command_gap_ms")) is not None for r in pre_cross[1:]))
    interruption_contract = scope.get("command_interruption_contract")
    max_command_gap_ms = finite(interruption_contract.get("max_command_gap_ms")) if isinstance(interruption_contract, dict) else None
    max_setpoint_gap_ms = finite(interruption_contract.get("max_setpoint_gap_ms")) if isinstance(interruption_contract, dict) else None
    interruption_contract_complete = (max_command_gap_ms is not None and max_command_gap_ms >= 0 and
                                      max_setpoint_gap_ms is not None and max_setpoint_gap_ms >= 0)
    effective_setpoint_coverage = bool(pre_cross and all(
        r.get("PX4_controller_setpoint_evidence_valid") is True for r in pre_cross))
    gap_within_contract = bool(command_gap_observed and interruption_contract_complete and
                               stats(pre_cross, "command_gap_ms")["max"] is not None and
                               stats(pre_cross, "command_gap_ms")["max"] <= max_command_gap_ms and
                               stats(pre_cross, "setpoint_gap_ms")["max"] is not None and
                               stats(pre_cross, "setpoint_gap_ms")["max"] <= max_setpoint_gap_ms)
    h8a_status = ("REJECTED" if generation_boundary_observed and boundary_contract_complete and boundary_within_contract
                  else "INCONCLUSIVE" if generation_boundary_observed else "INSUFFICIENT_EVIDENCE")
    h8e_status = ("REJECTED" if mode_observed and command_gap_observed and not mode_nontrack_pre and
                  effective_setpoint_coverage and gap_within_contract else
                  "INCONCLUSIVE" if mode_observed and command_gap_observed else
                  "INSUFFICIENT_EVIDENCE")
    h8 = {
        "H8a_command_discontinuity": {"status": h8a_status, "evidence": [f"Generation-boundary deltas are retained, but rejection requires an explicit continuity contract and every relevant boundary within its limits; earliest boundary ΔP={first_boundary.get('delta_position', MISSING)}, ΔV={first_boundary.get('delta_velocity', MISSING)}, ΔA={first_boundary.get('delta_acceleration', MISSING)}."], "generation_boundary_deltas": boundary_deltas, "continuity_contract": boundary_contract if isinstance(boundary_contract, dict) else MISSING, "contract_limits": boundary_limits},
        "H8b_dynamic_tracking_insufficiency": {"status": "INCONCLUSIVE" if tracking_observed else "INSUFFICIENT_EVIDENCE", "evidence": [f"Synchronized LIO error growth rates are measured as {growth_rates}; this correlation is observational and does not prove a causal mechanism."], "planner_velocity": stats(pre_cross, "planner_velocity_norm_mps"), "planner_acceleration": stats(pre_cross, "planner_acceleration_norm_mps2"), "tracking_error": stats(pre_cross, "aligned_LIO_tracking_error_m"), "error_growth_rates_mps": growth_rates},
        "H8c_px4_control_reshaping": {"status": "INCONCLUSIVE" if controller_observed else "INSUFFICIENT_EVIDENCE", "evidence": ["PX4 effective setpoints and controller deltas are available; their association with tracking is observational and does not establish causality."], "delta_v": stats(pre_cross, "delta_v_px4_controller_mps"), "delta_a": stats(pre_cross, "delta_a_px4_controller_mps2")},
        "H8d_px4_lio_state_divergence": {"status": "INCONCLUSIVE" if frame_observed else "INSUFFICIENT_EVIDENCE", "evidence": ["Frame residuals are computed only after explicit frame normalization and body-twist quaternion rotation; this observation does not establish causality."], "frame_position": stats(pre_cross, "frame_position_residual_m"), "frame_velocity": stats(pre_cross, "frame_velocity_residual_mps")},
        "H8e_command_setpoint_interruption": {"status": h8e_status, "evidence": ["External Mode is TRACK_TRAJECTORY throughout the synchronized pre-cross command window and the retained interruption contract bounds both command and effective-setpoint gaps." if h8e_status == "REJECTED" else "Finite mode/gap observations alone do not establish an interruption; a retained gap contract and complete effective-setpoint interval are required."], "external_mode_states_pre_cross": mode_pre, "command_gap": stats(pre_cross, "command_gap_ms"), "setpoint_gap": stats(pre_cross, "setpoint_gap_ms"), "interruption_contract": interruption_contract if isinstance(interruption_contract, dict) else MISSING, "effective_setpoint_coverage": effective_setpoint_coverage},
    }
    evidence_rows = [row for row in all_rows if row.get("required_evidence_valid") is True]
    evidence_status = "VALID" if all_rows and len(evidence_rows) == len(all_rows) and not scope_errors else "INSUFFICIENT_EVIDENCE"
    control_metrics = None
    if control and control.is_dir():
        control_scope, control_scope_errors = artifact_scope(control)
        ccommon_clock = control_scope.get("common_source_clock") if control_scope.get("common_source_clock") != MISSING else None
        ctolerance_ms = num(control_scope.get("maximum_synchronization_tolerance_ms"))
        cmax_gap_ns = int(ctolerance_ms * 1e6) if ctolerance_ms is not None and ctolerance_ms >= 0.0 else None
        if cmax_gap_ns is None:
            control_scope_errors.append("control alignment: own synchronization tolerance is missing or invalid")
        cframe_contract = control_scope.get("frame_conventions", {}).get("propagated_odometry") if isinstance(control_scope.get("frame_conventions"), dict) else None
        cscenario = read_jsonl(control / "scenario.jsonl"); csamples = read_jsonl(control / "samples.jsonl"); cpvas = pva_rows(cscenario); clio=stream_series(csamples,"propagated_odometry"); cpx4=stream_series(csamples,"local_position")
        control_scope_errors.extend(jsonl_integrity(control / "samples.jsonl"))
        control_scope_errors.extend(jsonl_integrity(control / "scenario.jsonl"))
        cpx4, cpx4_mapping_status = apply_clock_mapping(cpx4, control_scope.get("px4_to_ros_mapping"), ccommon_clock)
        if cpx4.get("position") and cpx4_mapping_status != "mapped":
            control_scope_errors.append(f"control PX4 source clock mapping: {cpx4_mapping_status}")
        lio_continuity_ok = continuity_continuous(clio.get("continuity", []))
        if not lio_continuity_ok:
            control_scope_errors.append("control row continuity: LIO epoch/reset identity is missing or discontinuous")
        if not cpvas:
            control_scope_errors.append("control row coverage: no retained pva_command samples")
        if not clio.get("position"):
            control_scope_errors.append("control row coverage: LIO position samples missing or nonfinite")
        if not cpx4.get("position"):
            control_scope_errors.append("control row coverage: PX4 position samples missing or nonfinite")
        control_metrics = {"run":str(control), "scenario_scope":control_scope,
                           "scope_status":"INSUFFICIENT_EVIDENCE",
                           "scope_errors":control_scope_errors,
                           "tracking_error":MISSING, "frame_position":MISSING,
                           "frame_velocity":MISSING, "delta_v":MISSING,
                           "delta_a":MISSING, "input_timestamp":MISSING,
                           "external_mode":MISSING}
        control_contract_valid = not control_scope_errors and cmax_gap_ns is not None and lio_continuity_ok
        if control_contract_valid and cpvas and clio and cpx4:
            cfirst=cpvas[0]
            ct = cfirst["timestamp_ns"]
            clp,*_=interp(clio.get("position",[]),ct,cmax_gap_ns)
            clv,*_=interp(clio.get("velocity",[]),ct,cmax_gap_ns)
            cpp,*_=interp(cpx4.get("position",[]),ct,cmax_gap_ns)
            cpv,*_=interp(cpx4.get("velocity",[]),ct,cmax_gap_ns)
            clio_clock = metadata_at(clio.get("clock",[]),ct,cmax_gap_ns)
            cpx4_clock = metadata_at(cpx4.get("clock",[]),ct,cmax_gap_ns)
            clio_continuity = metadata_at(clio.get("continuity",[]),ct,cmax_gap_ns)
            clio_frame = metadata_at(clio.get("position_frame",[]),ct,cmax_gap_ns)
            clio_child_frame = metadata_at(clio.get("velocity_frame",[]),ct,cmax_gap_ns)
            clio_position_convention = metadata_at(clio.get("position_convention",[]),ct,cmax_gap_ns)
            clio_velocity_convention = metadata_at(clio.get("velocity_convention",[]),ct,cmax_gap_ns)
            witness_frame_valid = (isinstance(cframe_contract, dict) and
                                   clio_frame == cframe_contract.get("frame_id") and
                                   clio_child_frame == cframe_contract.get("child_frame_id") and
                                   clio_position_convention == cframe_contract.get("position_convention") and
                                   clio_velocity_convention == cframe_contract.get("velocity_convention"))
            witness_valid = (clp is not None and clv is not None and cpp is not None and cpv is not None and
                             clio_clock == ccommon_clock and cpx4_clock == ccommon_clock and
                             clio_continuity is not None and witness_frame_valid)
            coff=sub(cpp,clp) if witness_valid else None
            if not witness_valid:
                control_scope_errors.append("control calibration: same-time frame/clock/continuity witness is incomplete")
                control_contract_valid = False
        if control_contract_valid and cpvas and clio and cpx4:
            crows=[]; prev=None
            for p in cpvas:
                cr=make_row(p,clio,cpx4,{},[],coff,prev,cmax_gap_ns,cframe_contract,ccommon_clock); crows.append(cr); prev=cr
            lio_rows_complete = all(row.get("LIO_evidence_valid") is True for row in crows)
            px4_rows_complete = all(row.get("PX4_state_evidence_valid") is True for row in crows)
            if not lio_rows_complete:
                control_scope_errors.append("control row coverage: one or more rows lack LIO evidence")
            if not px4_rows_complete:
                control_scope_errors.append("control row coverage: one or more rows lack PX4 state evidence")
            if lio_rows_complete and px4_rows_complete:
                control_metrics.update({"scope_status":"VALID", "tracking_error":stats(crows,"aligned_LIO_tracking_error_m"),"frame_position":stats(crows,"frame_position_residual_m"),"frame_velocity":stats(crows,"frame_velocity_residual_mps"),"input_timestamp":MISSING, "external_mode":MISSING})
            else:
                control_metrics["scope_status"] = "INSUFFICIENT_EVIDENCE"
    paths=plot_all(root,rows,event_times,trace_rows,tracking_threshold)
    fields=ROOT_FIELDS
    write_csv(root/"e5_tracking_root_cause.csv",rows,fields)
    write_csv(root/"e5_tracking_root_cause_events.csv",event_rows,["event","timestamp_ns","planner_result","planning_failure_stage","planning_failure_reason","emergency_authorization_reason","emergency_candidate_commit_result","raw_retained_anchor_error_m","time_aligned_retained_anchor_error_m","retained_suffix_usable","tracking_certificate_exceeded","projected_tracking_certificate_exceeded","aligned_LIO_tracking_error_m","px4_tracking_error_m","frame_position_residual_m","frame_velocity_residual_mps","planner_velocity_norm_mps","planner_acceleration_norm_mps2","PX4_effective_velocity_setpoint","PX4_effective_acceleration_setpoint","delta_v_px4_controller_mps","delta_a_px4_controller_mps2","command_gap_ms","setpoint_gap_ms","external_mode_output_state","analytic_role","NavigationCommand_role","recovery_state","safety_suffix_active"])
    summary={"evidence_status": evidence_status, "evidence_status_reasons": scope_errors, "scenario_scope":scope, "transform":{"position":"explicit source frame normalized to PX4 local NED; fixed translation calibrated once at synchronized witness","offset_m":offset,"lio_velocity":"body twist rotated by recorded q_xyzw before header-frame conversion to PX4 NED"},"alignment_contract":{"maximum_synchronization_tolerance_ms":scope.get("maximum_synchronization_tolerance_ms", MISSING),"source":"retained scenario runtime thresholds; common.yaml is provenance only","method":"absolute common simulation timestamps with interpolation only across finite same-frame samples"},"retained_tracking_threshold_m":tracking_threshold if tracking_threshold is not None else MISSING,"retained_tracking_threshold_provenance":tracking_threshold_provenance,"event_times":event_times,"event_rows":event_rows,"T_cross_ns":tcross if tracking_threshold is not None else MISSING,"generation_boundary_deltas":boundary_deltas,"error_growth_rates_mps":growth_rates,"stats_pre_cross":{"tracking_error":stats(pre_cross,"aligned_LIO_tracking_error_m"),"px4_tracking_error":stats(pre_cross,"px4_tracking_error_m"),"frame_position":stats(pre_cross,"frame_position_residual_m"),"frame_velocity":stats(pre_cross,"frame_velocity_residual_mps"),"delta_v":stats(pre_cross,"delta_v_px4_controller_mps"),"delta_a":stats(pre_cross,"delta_a_px4_controller_mps2")},"h8":h8,"control":control_metrics,"figures":paths,"data_limitations":["PX4 timestamp_sample and ROS/simulation timestamps are retained with their domains; no cross-domain comparison is made without an explicit mapping.","Planner solve start is NOT_RECORDED; the trace records failure return/decision timestamp and planning latency, but this analysis does not back-calculate a solve start.","Tracking crossing events are NOT_RECORDED when no retained instantaneous certificate threshold is present.","Causal statuses are INSUFFICIENT_EVIDENCE when scope, frame, epoch, finite-value, or source-time coverage is incomplete."],"raw_evidence_preserved":True}
    recorded_metadata: dict[str, Any] = {}
    metadata_path = root / "metadata.json"
    if metadata_path.is_file():
        try:
            loaded_metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            if isinstance(loaded_metadata, dict):
                recorded_metadata = loaded_metadata
        except (OSError, json.JSONDecodeError):
            # artifact_scope() already retained the structured error; this
            # second read must not turn a malformed artifact into a traceback.
            pass
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
    md += ["# Tracking root-cause decomposition","", "## Scenario scope", "", f"Evidence status: **{evidence_status}**", "", json.dumps(summary["scenario_scope"],indent=2), "", "Scope and provenance are read from retained artifact files. This report is diagnostic and does not rewrite source evidence.", "", "## Time base and transforms", "", f"- Fixed translation offset: `{offset}` m, calibrated once at the first valid witness.", "- Position and velocity are normalized only from explicit recorded frame IDs; body twist is rotated by its recorded quaternion before header-frame conversion to PX4 NED.", f"- Samples are interpolated only within the captured `{scope.get('maximum_synchronization_tolerance_ms', MISSING)}` ms common-simulation-time gap and a continuous frame/epoch witness.", "- PX4 input timestamp domains remain separate; no producer timestamp is compared directly to the common clock without a retained mapping.", "", "## T_cross and events", "", "| Event | timestamp_ns | e_lio [m] | e_px4 [m] | frame pos [m] | frame vel [m/s] | planner |V| [m/s] | planner |A| [m/s²] | dV [m/s] | dA [m/s²] | mode |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for event in event_rows:
        def f(k):
            v=event.get(k,MISSING); return f"{v:.6g}" if isinstance(v,float) else str(v)
        md.append(f"| {event['event']} | {event['timestamp_ns']} | {f('aligned_LIO_tracking_error_m')} | {f('px4_tracking_error_m')} | {f('frame_position_residual_m')} | {f('frame_velocity_residual_mps')} | {f('planner_velocity_norm_mps')} | {f('planner_acceleration_norm_mps2')} | {f('delta_v_px4_controller_mps')} | {f('delta_a_px4_controller_mps2')} | {event.get('external_mode_output_state',MISSING)} |")
    threshold_text = f"the retained tracking threshold ({tracking_threshold:g} m)" if tracking_threshold is not None else "a retained tracking threshold (NOT_RECORDED)"
    md += ["", f"T_cross is the first synchronized LIO error crossing above {threshold_text}: `{event_times.get('T_cross', MISSING)}` ns. Growth rates are reported separately from event crossing.", "", "## Generation boundary measurements", "", "| from | to | timestamp_ns | ΔP [m] | ΔV [m/s] | ΔA [m/s²] | ΔJ [m/s³] |", "|---:|---:|---:|---:|---:|---:|---:|"]
    for d in boundary_deltas: md.append(f"| {d['from_generation']} | {d['to_generation']} | {d['timestamp_ns']} | {d.get('delta_position',MISSING)} | {d.get('delta_velocity',MISSING)} | {d.get('delta_acceleration',MISSING)} | {d.get('delta_jerk',MISSING)} |")
    md += ["", "## Scenario-scoped H8 classification", ""]
    for key,item in h8.items(): md += [f"### {key} — {item['status']}", "", *[f"- {x}" for x in item["evidence"]], ""]
    md += ["## Matched control (separate statistics)", "", "The optional control scope and statistics are retained separately. They are not pooled with this artifact and cannot establish causality for it.", "", "| Metric | selected artifact | optional control |", "|---|---:|---:|"]
    if control_metrics:
        for label, key, percentile in (("LIO tracking RMS [m]", "tracking_error", "rms"), ("LIO tracking P95 [m]", "tracking_error", "p95"), ("frame position P95 [m]", "frame_position", "p95"), ("frame velocity P95 [m/s]", "frame_velocity", "p95")):
            selected_metric = summary["stats_pre_cross"].get(key, {})
            control_metric = control_metrics.get(key, {})
            selected_value = selected_metric.get(percentile, MISSING) if isinstance(selected_metric, dict) else MISSING
            control_value = control_metric.get(percentile, MISSING) if isinstance(control_metric, dict) else MISSING
            md.append(f"| {label} | {selected_value} | {control_value} |")
    else:
        md.append("| matched control | NOT_RECORDED | NOT_RECORDED |")
    md += ["", "## Observations", "", f"- The first synchronized tracking crossing is `{event_times.get('T_cross', MISSING)}` ns; measured growth rates are `{growth_rates}` m/s.", "- These observations do not by themselves establish causal ordering or authorize a product change; causal statuses remain scenario-scoped and evidence-gated.", "", "## Data limitations", "", *[f"- {x}" for x in summary["data_limitations"]], "", "## Figures", "", *[f"- `{x}`" for x in paths], ""]
    (root/"e5_tracking_root_cause.md").write_text("\n".join(md),encoding="utf-8")
    return summary


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--input",type=Path,required=True); parser.add_argument("--control",type=Path); args=parser.parse_args(); summary=analyze(args.input,args.control); print(json.dumps({"T_cross_ns":summary["T_cross_ns"],"figures":summary["figures"],"h8":{k:v["status"] for k,v in summary["h8"].items()}},indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())

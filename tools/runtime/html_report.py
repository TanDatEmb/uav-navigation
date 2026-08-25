#!/usr/bin/env python3
"""Internal metric extraction for the single runtime report tool.

The metric extraction stays here for compatibility with the runtime report
contract and tests.  It is not a standalone command; ``report.py`` owns the
public report workflow.
"""

from __future__ import annotations

import html
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET
from typing import Any

from planner_trace import collect_planner_trace_records, planner_trace_summary


def _load(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def _summary(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": statistics.fmean(values) if values else None,
        "rmse": math.sqrt(statistics.fmean([value * value for value in values])) if values else None,
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "maximum": max(values) if values else None,
    }


def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _acceptance_summary(
    session: Path,
    scenario: dict[str, Any],
    waypoint_count: int,
    cross_track_p95: float | None,
) -> dict[str, Any]:
    """Expose mission acceptance evidence without treating goal publication as acceptance."""
    report = _load(session / "report.json", {})
    report_acceptance = report.get("acceptance", {}) if isinstance(report, dict) else {}
    if not isinstance(report_acceptance, dict):
        report_acceptance = {}

    events = scenario.get("waypoint_acceptance_events")
    events_available = isinstance(events, list)
    event_indices: list[int] = []
    if events_available:
        for event in events:
            value = event.get("accepted_waypoint_index") if isinstance(event, dict) else event
            if isinstance(event, dict) and event.get("waypoint_accepted") is False:
                continue
            try:
                event_indices.append(int(value))
            except (TypeError, ValueError):
                continue

    reported_indices = report_acceptance.get("waypoint_acceptance_indices")
    if isinstance(reported_indices, list):
        accepted_indices = list(reported_indices)
    elif events_available:
        accepted_indices = event_indices
    else:
        accepted_indices = None

    allow_initial_skip = False
    try:
        import yaml
        scenario_config = yaml.safe_load((session / "scenario_config.yaml").read_text(encoding="utf-8"))
        scenario_parameters = scenario_config.get("scenario", {}) if isinstance(scenario_config, dict) else {}
        allow_initial_skip = bool(
            scenario_parameters.get("allow_initial_pass_through_skip", False)
        ) if isinstance(scenario_parameters, dict) else False
    except (ImportError, OSError, ValueError):
        pass

    expected_indices = list(range(max(0, waypoint_count)))
    expected_acceptance = expected_indices[1:] if allow_initial_skip and expected_indices else expected_indices
    acceptance_complete = report_acceptance.get("waypoint_acceptance_complete")
    if not isinstance(acceptance_complete, bool):
        acceptance_complete = accepted_indices == expected_acceptance if accepted_indices is not None else False

    threshold = _finite_number(report_acceptance.get("max_cross_track_p95_m"))
    if threshold is None:
        threshold = 0.5
    reasons = report_acceptance.get("reasons")
    if not isinstance(reasons, list):
        reasons = []
        if not bool(scenario.get("mission_complete_observed", False)):
            reasons.append("mission completion event was not observed")
        if accepted_indices is None:
            reasons.append("waypoint acceptance evidence is unavailable")
        elif not acceptance_complete:
            reasons.append(
                f"waypoint acceptance coverage incomplete: expected {expected_acceptance}, got {accepted_indices}"
            )
        if cross_track_p95 is None:
            reasons.append("tracking cross-track p95 is unavailable")
        elif cross_track_p95 > threshold:
            reasons.append(
                f"tracking cross-track p95 exceeded {threshold:.3f} m "
                f"(observed {cross_track_p95:.3f} m)"
            )

    return {
        "mission_complete_observed": scenario.get("mission_complete_observed"),
        "goal_indices": scenario.get("goal_indices", []),
        "waypoint_acceptance_events": events if events_available else None,
        "waypoint_acceptance_indices": accepted_indices,
        "waypoint_acceptance_complete": acceptance_complete,
        "expected_waypoint_indices": expected_indices,
        "initial_pass_through_skip_allowed": allow_initial_skip,
        "cross_track_p95_m": cross_track_p95,
        "max_cross_track_p95_m": threshold,
        "reasons": [str(reason) for reason in reasons],
    }


def _point(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def _segment_distance_2d(
    point: tuple[float, float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    projection = max(0.0, min(1.0, projection))
    return math.hypot(point[0] - (start[0] + projection * dx), point[1] - (start[1] + projection * dy))


def _mission_waypoints(session: Path, descriptor: dict[str, Any]) -> list[dict[str, Any]]:
    try:
        import yaml
        config = yaml.safe_load((session / "scenario_config.yaml").read_text(encoding="utf-8"))
    except (OSError, ValueError, ImportError):
        config = {}
    scenario = config.get("scenario", {}) if isinstance(config, dict) else {}
    mission_file = scenario.get("mission_file")
    if mission_file:
        mission_path = Path(str(mission_file))
        if not mission_path.is_absolute():
            mission_path = (session.parent.parent.parent / mission_path).resolve()
        try:
            import yaml
            value = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
            waypoints = value.get("mission", {}).get("waypoints", []) if isinstance(value, dict) else []
            if isinstance(waypoints, list):
                return [item for item in waypoints if isinstance(item, dict) and _point(item.get("position"))]
        except (OSError, ValueError, ImportError):
            pass
    return []


def _obstacles(session: Path, descriptor: dict[str, Any]) -> list[dict[str, Any]]:
    names = set(str(item) for item in descriptor.get("collision_truth", []))
    worlds = sorted(session.glob("resolved_*.sdf"))
    world = worlds[0] if worlds else session / "resolved_map.sdf"
    if not world.is_file():
        return []
    result: list[dict[str, Any]] = []
    try:
        root = ET.parse(world).getroot()
        for model in root.findall(".//model"):
            name = str(model.get("name", ""))
            if names and name not in names:
                continue
            pose = (model.findtext("pose") or "0 0 0").split()
            center = _point(pose)
            geometry = model.find("./link/collision/geometry")
            if center is None or geometry is None:
                continue
            cylinder = geometry.find("cylinder")
            box = geometry.find("box/size")
            if cylinder is not None:
                result.append({
                    "name": name, "type": "cylinder", "center": center,
                    "radius_m": float(cylinder.findtext("radius", "0")),
                })
            elif box is not None and box.text:
                size = [float(item) for item in box.text.split()]
                if len(size) == 3:
                    result.append({
                        "name": name, "type": "box", "center": center,
                        "half_extents": [item / 2.0 for item in size],
                    })
    except (ET.ParseError, OSError, ValueError):
        return []
    return result


def _samples(session: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    ground_truth: list[dict[str, Any]] = []
    planning: list[dict[str, Any]] = []
    path = session / "samples.jsonl"
    if path.is_file():
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                except ValueError:
                    continue
                if item.get("kind") != "sample" or not item.get("accepted_by_monitor", True):
                    continue
                if item.get("stream") == "ground_truth_odometry":
                    payload = item.get("payload", {})
                    position = _point(payload.get("position"))
                    if position is not None:
                        ground_truth.append({
                            "t": float(payload.get("stamp_ns", item.get("timestamp_ns", 0))) / 1e9,
                            "position": position,
                            "velocity": _point(payload.get("linear_velocity")),
                        })
                elif item.get("stream") in {"planning_diagnostics", "mapping_diagnostics", "diagnostics"}:
                    payload = item.get("payload", {})
                    for status in payload.get("statuses", []):
                        if isinstance(status, dict) and status.get("name") in {
                            "navigation_planning/planner",
                            "super_navigation/super_planner",
                        }:
                            values = status.get("values", {})
                            if isinstance(values, dict):
                                planning.append({
                                    "t": float(payload.get("stamp_ns", item.get("timestamp_ns", 0))) / 1e9,
                                    **values,
                                })
    ground_truth.sort(key=lambda item: item["t"])
    planning.sort(key=lambda item: item["t"])
    return ground_truth, planning


def _trajectory_records(session: Path) -> list[dict[str, Any]]:
    scenario = _load(session / "scenario.json", {})
    # scenario.json keeps the latest history but intentionally does not retain
    # the publication timestamp of each overlapping rolling trajectory.  Read
    # the event log first so smoothness can be evaluated on the execution
    # timeline.  Concatenating the profiles end-to-end is invalid: a 3-second
    # plan is commonly replaced after ~0.16 s and the profiles overlap.
    records: list[dict[str, Any]] = []
    event_log = session / "scenario.jsonl"
    try:
        with event_log.open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if event.get("kind") != "trajectory":
                    continue
                payload = event.get("payload")
                if not isinstance(payload, dict) or not payload.get("position_points"):
                    continue
                record = dict(payload)
                timestamp_ns = _finite_number(event.get("sim_time_ns"))
                if timestamp_ns is not None:
                    record["_publish_time_s"] = timestamp_ns / 1e9
                records.append(record)
    except OSError:
        pass
    if records:
        return records
    if isinstance(scenario, dict):
        history = scenario.get("trajectory_history", [])
        records = history if isinstance(history, list) and history else scenario.get(
            "trajectory_records", []
        )
    return [item for item in records if isinstance(item, dict) and item.get("position_points")]


def _sample_time_seconds(item: dict[str, Any], payload: dict[str, Any] | None = None) -> float | None:
    """Return the recorder timestamp in seconds, preferring message time."""
    payload = payload if isinstance(payload, dict) else {}
    for value in (payload.get("stamp_ns"), item.get("timestamp_ns"), item.get("sim_time_ns")):
        number = _finite_number(value)
        if number is not None and number > 0.0:
            return number / 1e9
    return None


def _sampled_dicts(values: list[dict[str, Any]], limit: int = 900) -> list[dict[str, Any]]:
    if len(values) <= limit:
        return values
    step = (len(values) - 1) / max(1, limit - 1)
    return [values[round(index * step)] for index in range(limit)]


def _axis_stats(records: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = [_finite_number(record.get(key)) for record in records]
    values = [value for value in values if value is not None]
    return _summary(values)


def _enu_from_ned(value: Any) -> list[float] | None:
    """Convert PX4 NED position/velocity to the report's ENU display frame."""
    point = _point(value)
    if point is None:
        return None
    north, east, down = point
    return [east, north, -down]


def _runtime_observability(session: Path, limit: int = 900) -> dict[str, Any]:
    """Extract system-level telemetry that the acceptance gates do not score.

    This is intentionally a view of explicit recorder fields. Missing streams,
    missing axes, and missing planner roles remain unavailable; no values are
    reconstructed from IDs or aggregate counters.
    """
    stream_records: dict[str, list[dict[str, Any]]] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    pva: list[dict[str, Any]] = []
    setpoints: list[dict[str, Any]] = []
    goals: list[dict[str, Any]] = []
    waypoint_events: list[dict[str, Any]] = []
    mode_events: list[dict[str, Any]] = []
    vehicle_events: list[dict[str, Any]] = []
    current_goal: dict[str, Any] = {}

    samples_path = session / "samples.jsonl"
    if samples_path.is_file():
        with samples_path.open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                except ValueError:
                    continue
                if item.get("kind") != "sample" or not item.get("accepted_by_monitor", True):
                    continue
                stream_name = str(item.get("stream") or "")
                payload = item.get("payload", {})
                if not stream_name or not isinstance(payload, dict):
                    continue
                timestamp = _sample_time_seconds(item, payload)
                if timestamp is None:
                    continue
                position = _point(payload.get("position"))
                velocity = _point(payload.get("linear_velocity"))
                if stream_name == "local_position":
                    position = _enu_from_ned([
                        payload.get("x_ned_m"), payload.get("y_ned_m"), payload.get("z_ned_m")
                    ])
                    velocity = _enu_from_ned([
                        payload.get("vx_ned_m_s"), payload.get("vy_ned_m_s"), payload.get("vz_ned_m_s")
                    ])
                record: dict[str, Any] = {"t": timestamp}
                if position is not None:
                    record["position"] = list(position)
                if velocity is not None:
                    record["velocity"] = list(velocity)
                if stream_name in {"ground_truth_odometry", "propagated_odometry", "corrected_odometry", "external_odometry", "px4_odometry", "local_position"}:
                    if position is not None or velocity is not None:
                        stream_records.setdefault(stream_name, []).append(record)

                if stream_name in {"mapping_diagnostics", "diagnostics", "planning_diagnostics"}:
                    for status in payload.get("statuses", []):
                        if not isinstance(status, dict):
                            continue
                        name = str(status.get("name") or "unknown")
                        values = status.get("values", {})
                        if not isinstance(values, dict):
                            continue
                        entry = diagnostics.setdefault(name, {"count": 0, "latest": {}, "fields": {}})
                        entry["count"] += 1
                        entry["latest"] = dict(values)
                        for key, value in values.items():
                            number = _finite_number(value)
                            if number is not None:
                                entry["fields"].setdefault(key, []).append(number)

    scenario_path = session / "scenario.jsonl"
    if scenario_path.is_file():
        with scenario_path.open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                kind = str(event.get("kind") or "")
                payload = event.get("payload", {})
                if not isinstance(payload, dict):
                    continue
                timestamp = _finite_number(event.get("sim_time_ns"))
                timestamp = timestamp / 1e9 if timestamp is not None else None
                if kind == "goal":
                    current_goal = dict(payload)
                    goals.append({"t": timestamp, **payload})
                elif kind in {"waypoint_accepted", "navigation_mode_status"}:
                    waypoint_events.append({"t": timestamp, "kind": kind, **payload})
                elif kind == "event":
                    mode_events.append({"t": timestamp, **payload})
                elif kind == "vehicle_status":
                    vehicle_events.append({"t": timestamp, **payload})
                elif kind == "setpoint":
                    setpoints.append({"t": timestamp, **payload})
                elif kind == "pva_command":
                    point = _point(payload.get("position"))
                    velocity = _point(payload.get("velocity"))
                    acceleration = _point(payload.get("acceleration"))
                    if timestamp is None or point is None:
                        continue
                    pva.append({
                        "t": timestamp,
                        "position": list(point),
                        "velocity": list(velocity) if velocity is not None else None,
                        "acceleration": list(acceleration) if acceleration is not None else None,
                        "trajectory_id": payload.get("trajectory_id"),
                        "trajectory_flag": payload.get("trajectory_flag"),
                        "trajectory_status": payload.get("trajectory_status"),
                        "trajectory_generation": payload.get("trajectory_generation"),
                        "trajectory_time_s": payload.get("trajectory_time_s"),
                        "waypoint_index": current_goal.get("waypoint_index"),
                        "mission_id": current_goal.get("mission_id"),
                    })

    stream_summary: dict[str, Any] = {}
    for name, records in stream_records.items():
        records.sort(key=lambda item: item["t"])
        times = [item["t"] for item in records]
        gaps = [right - left for left, right in zip(times, times[1:]) if right >= left]
        stream_summary[name] = {
            "count": len(records),
            "first_t": times[0] if times else None,
            "last_t": times[-1] if times else None,
            "duration_s": (times[-1] - times[0]) if len(times) >= 2 else None,
            "mean_rate_hz": (len(records) - 1) / (times[-1] - times[0]) if len(times) >= 2 and times[-1] > times[0] else None,
            "max_gap_ms": max(gaps, default=None) * 1000.0,
            "p95_gap_ms": _percentile([gap * 1000.0 for gap in gaps], 0.95),
            "position_stats": {axis: _axis_stats([{"v": record["position"][index]} for record in records if "position" in record], "v") for index, axis in enumerate(("x", "y", "z"))},
            "velocity_stats": {axis: _axis_stats([{"v": record["velocity"][index]} for record in records if "velocity" in record], "v") for index, axis in enumerate(("vx", "vy", "vz"))},
            "position_series": _sampled_dicts([
                {"t": record["t"], "x": record["position"][0], "y": record["position"][1], "z": record["position"][2]}
                for record in records if "position" in record
            ], limit),
            "velocity_series": _sampled_dicts([
                {"t": record["t"], "vx": record["velocity"][0], "vy": record["velocity"][1], "vz": record["velocity"][2]}
                for record in records if "velocity" in record
            ], limit),
        }

    timing: list[dict[str, Any]] = []
    health: list[dict[str, Any]] = []
    for name, entry in diagnostics.items():
        component = "LIO" if name.startswith("fast_lio/") else "Planner / ROG-Map" if name.startswith("super_navigation/") else name
        for field, values in sorted(entry["fields"].items()):
            unit = None
            converted = list(values)
            if field.endswith("_us"):
                unit = "us"
            elif field.endswith("_ms"):
                unit = "ms"
            elif field.endswith("_ns") and field in {"processing_lag_ns"}:
                unit = "us"
                converted = [value / 1000.0 for value in values]
            if unit is not None:
                timing.append({
                    "component": component,
                    "source": name,
                    "metric": field,
                    "unit": unit,
                    "count": len(converted),
                    "nonzero_count": sum(abs(value) > 1e-12 for value in converted),
                    "stats": _summary(converted),
                })
        health_fields = {
            "state", "status", "navigation_valid", "transport_ok", "cycle_count",
            "received_observation_count", "accepted_observation_count", "dropped_cloud_count",
            "mapping_failed_count", "processing_exception_count", "command_available",
            "planner_failure_latched", "solve_deadline_exceeded", "imu_drop_count", "lidar_drop_count",
            "measurement_callback_count", "observability_rejection_count", "translation_observability_ratio",
            "publication_count", "publication_skip_count", "queue_depth", "queue_maximum",
            "planning_latency_ms", "planning_horizon_distance_m", "known_free_horizon_m",
            "horizon_progress_m", "horizon_forward_projection_m", "replan_code", "solve_stage",
            "solve_stage_name", "candidate_result", "status_name", "last_failure_code",
            "last_failure_reason",
        }
        for field in sorted(health_fields.intersection(entry["latest"])):
            health.append({
                "component": component,
                "source": name,
                "metric": field,
                "value": entry["latest"].get(field),
                "count": entry["count"],
            })

    trajectory_paths: list[dict[str, Any]] = []
    if pva:
        # A trajectory generation is the committed-bundle identity. Never
        # join samples from different generations into one visual path: doing
        # so would manufacture continuity across a replan/emergency handover.
        # Legacy messages without generation fall back to message identity.
        grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
        for index, item in enumerate(pva):
            generation = _finite_number(item.get("trajectory_generation"))
            message_id = _finite_number(item.get("trajectory_id"))
            identity = (
                "generation", int(generation)
            ) if generation is not None and generation > 0.0 else (
                "message", int(message_id)
            ) if message_id is not None and message_id > 0.0 else (
                "sample", index
            )
            key = (
                item.get("mission_id"), item.get("waypoint_index"),
                item.get("trajectory_flag"), identity,
            )
            grouped.setdefault(key, []).append(item)
        for (mission_id, waypoint_index, flag, identity), records in grouped.items():
            records.sort(key=lambda item: item["t"])
            trajectory_paths.append({
                "mission_id": mission_id,
                "waypoint_index": waypoint_index,
                "trajectory_flag": flag,
                "trajectory_identity": identity[0],
                "trajectory_identity_value": identity[1],
                "trajectory_generation": records[-1].get("trajectory_generation"),
                "trajectory_status": records[-1].get("trajectory_status"),
                "t_start": records[0]["t"],
                "t_end": records[-1]["t"],
                "points": [item["position"] for item in records],
                "velocity_points": [item["velocity"] for item in records if item.get("velocity") is not None],
                "count": len(records),
            })

    return {
        "streams": stream_summary,
        "diagnostics": diagnostics,
        "timing": timing,
        "health": health,
        "pva": _sampled_dicts(pva, limit),
        "setpoints": _sampled_dicts(setpoints, limit),
        "goals": goals,
        "waypoint_events": waypoint_events,
        "mode_events": mode_events,
        "vehicle_events": vehicle_events,
        "trajectory_paths": trajectory_paths,
    }


def _planning_continuity(planning: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize rolling-horizon behavior from one diagnostic sample per tick."""
    terminal_hold_samples = sum(
        str(item.get("replan_reason", "")) == "reuse_terminal_hold" for item in planning
    )
    # Terminal STOP refreshes are deliberately repeated to keep PX4's hold
    # setpoint alive; they are not local-subgoal endpoint reuse and must not
    # dominate rolling-horizon continuity metrics.
    rolling_planning = [
        item for item in planning
        if str(item.get("replan_reason", "")) != "reuse_terminal_hold"
    ]
    endpoints: list[tuple[float, float, float]] = []
    progress: list[float] = []
    forward_projection: list[float] = []
    roles: dict[str, int] = {}
    safety_kinds: dict[str, int] = {}
    endpoint_change_count = 0
    endpoint_repeat_count = 0
    maximum_repeat_run = 0
    current_repeat_run = 0
    tangent_reversal_count = 0
    previous_endpoint: tuple[float, float, float] | None = None
    previous_tangent: tuple[float, float, float] | None = None
    endpoint_tolerance_m = 0.25

    for item in rolling_planning:
        endpoint = _point((
            item.get("effective_goal_x"), item.get("effective_goal_y"),
            item.get("effective_goal_z"),
        ))
        if endpoint is not None:
            endpoints.append(endpoint)
            if previous_endpoint is None:
                current_repeat_run = 1
            elif _distance(endpoint, previous_endpoint) <= endpoint_tolerance_m:
                endpoint_repeat_count += 1
                current_repeat_run += 1
            else:
                endpoint_change_count += 1
                current_repeat_run = 1
            maximum_repeat_run = max(maximum_repeat_run, current_repeat_run)
            previous_endpoint = endpoint
        value = _finite_number(item.get("horizon_progress_m"))
        if value is not None:
            progress.append(value)
        value = _finite_number(item.get("horizon_forward_projection_m"))
        if value is not None:
            forward_projection.append(value)
        role = str(item.get("plan_role", "unknown"))
        roles[role] = roles.get(role, 0) + 1
        safety_kind = str(item.get("safety_plan_kind", "unknown"))
        safety_kinds[safety_kind] = safety_kinds.get(safety_kind, 0) + 1
        tangent = _point((
            item.get("horizon_tangent_x"), item.get("horizon_tangent_y"),
            item.get("horizon_tangent_z"),
        ))
        if tangent is not None and previous_tangent is not None:
            dot = sum(tangent[index] * previous_tangent[index] for index in range(3))
            if dot < -0.25:
                tangent_reversal_count += 1
        if tangent is not None:
            previous_tangent = tangent

    measured_safety_kinds = [
        str(item.get("safety_plan_kind"))
        for item in rolling_planning
        if item.get("safety_plan_kind") not in (None, "", "unknown")
    ]
    return {
        "sample_count": len(planning),
        "rolling_sample_count": len(rolling_planning),
        "terminal_hold_sample_count": terminal_hold_samples,
        "endpoint_sample_count": len(endpoints),
        "unique_endpoint_count": len({
            tuple(round(value / endpoint_tolerance_m) for value in endpoint)
            for endpoint in endpoints
        }),
        "endpoint_change_count": endpoint_change_count,
        "endpoint_repeat_count": endpoint_repeat_count,
        "maximum_endpoint_repeat_run": maximum_repeat_run,
        "backward_projection_count": sum(value < -0.05 for value in forward_projection),
        "tangent_reversal_count": tangent_reversal_count,
        "progress_m": _summary(progress),
        "forward_projection_m": _summary(forward_projection),
        "plan_role_counts": roles,
        "safety_plan_kind_counts": safety_kinds,
        "safety_stop_ratio": (
            sum(key.lower() in {"braking_stop", "2"} for key in measured_safety_kinds)
            / len(measured_safety_kinds)
            if measured_safety_kinds else None
        ),
    }


def _trajectory_smoothness(
    records: list[dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, list[tuple[float, float]]]]:
    """Measure continuity of overlapping trajectory artifacts on execution time.

    Ground-truth smoothness is useful but can hide a planner handover jump when
    PX4 filters it.  Each rolling trajectory is published before the previous
    one expires, so profiles must not be concatenated.  Handover metrics compare
    the old trajectory evaluated at the next publication timestamp with the new
    trajectory's first point; this is the signal PX4 can actually see.
    """
    plan_speeds: list[float] = []
    plan_accelerations: list[float] = []
    boundary_velocity_jumps: list[float] = []
    boundary_position_jumps: list[float] = []
    boundary_heading_steps_deg: list[float] = []
    planned_start_speed_series: list[tuple[float, float]] = []
    velocity_jump_series: list[tuple[float, float]] = []
    heading_step_series: list[tuple[float, float]] = []
    previous_record: dict[str, Any] | None = None
    previous_publish_time_s: float | None = None
    time_anchored = bool(records) and all(
        _finite_number(record.get("_publish_time_s")) is not None for record in records
    )
    handover_expired_count = 0

    def interpolate(
        points: list[tuple[float, float, float]], offset_s: float, duration_s: float
    ) -> tuple[float, float, float] | None:
        if not points:
            return None
        if len(points) == 1 or duration_s <= 1e-9:
            return points[0]
        normalized = max(0.0, min(1.0, offset_s / duration_s)) * (len(points) - 1)
        index = min(len(points) - 2, max(0, int(math.floor(normalized))))
        fraction = normalized - index
        return tuple(
            points[index][axis] * (1.0 - fraction) +
            points[index + 1][axis] * fraction
            for axis in range(3)
        )

    for record in records:
        positions = [_point(item) for item in record.get("position_points", [])]
        velocities = [_point(item) for item in record.get("velocity_points", [])]
        positions = [item for item in positions if item is not None]
        velocities = [item for item in velocities if item is not None]
        if not positions:
            continue
        duration = _finite_number(record.get("duration_s")) or 0.0
        point_count = max(len(velocities), len(positions), 1)
        dt = duration / max(1, point_count - 1)

        publish_time_s = _finite_number(record.get("_publish_time_s"))
        first_position = positions[0]
        first_velocity = velocities[0] if velocities else None
        if publish_time_s is not None and first_velocity is not None:
            planned_start_speed_series.append((publish_time_s, _distance(first_velocity, (0.0, 0.0, 0.0))))

        if (
            previous_record is not None and
            publish_time_s is not None and
            previous_publish_time_s is not None
        ):
            elapsed_s = publish_time_s - previous_publish_time_s
            previous_duration = _finite_number(previous_record.get("duration_s")) or 0.0
            previous_positions = [
                item for item in (_point(value) for value in previous_record.get("position_points", []))
                if item is not None
            ]
            previous_velocities = [
                item for item in (_point(value) for value in previous_record.get("velocity_points", []))
                if item is not None
            ]
            if elapsed_s >= previous_duration - 1e-9:
                # An expired trajectory is a watchdog/replacement event, not a
                # smooth handover sample. Keep it visible as a separate count.
                handover_expired_count += 1
            elif elapsed_s >= -1e-9 and previous_positions:
                old_position = interpolate(previous_positions, elapsed_s, previous_duration)
                if old_position is not None:
                    boundary_position_jumps.append(_distance(first_position, old_position))
                if previous_velocities and first_velocity is not None:
                    old_velocity = interpolate(previous_velocities, elapsed_s, previous_duration)
                    if old_velocity is not None:
                        velocity_jump = _distance(first_velocity, old_velocity)
                        boundary_velocity_jumps.append(velocity_jump)
                        velocity_jump_series.append((publish_time_s, velocity_jump))
                        previous_norm = _distance(old_velocity, (0.0, 0.0, 0.0))
                        current_norm = _distance(first_velocity, (0.0, 0.0, 0.0))
                        if previous_norm > 0.05 and current_norm > 0.05:
                            dot = sum(old_velocity[i] * first_velocity[i] for i in range(3))
                            angle = math.degrees(math.acos(max(
                                -1.0, min(1.0, dot / (previous_norm * current_norm)))))
                            boundary_heading_steps_deg.append(angle)
                            heading_step_series.append((publish_time_s, angle))

        for index, velocity in enumerate(velocities):
            speed = math.sqrt(sum(value * value for value in velocity))
            if not math.isfinite(speed):
                continue
            plan_speeds.append(speed)
            if index > 0 and dt > 1e-9:
                previous_velocity = velocities[index - 1]
                acceleration = _distance(velocity, previous_velocity) / dt
                if math.isfinite(acceleration):
                    plan_accelerations.append(acceleration)

        previous_record = record
        previous_publish_time_s = publish_time_s

    metrics = {
        "trajectory_record_count": len(records),
        "trajectory_sample_count": len(plan_speeds),
        "trajectory_duration_s": (
            max((_finite_number(record.get("duration_s")) or 0.0 for record in records), default=0.0)
            if time_anchored else sum(_finite_number(record.get("duration_s")) or 0.0 for record in records)
        ),
        "timeline_time_anchored": time_anchored,
        "handover_sample_count": len(boundary_velocity_jumps),
        "handover_expired_count": handover_expired_count,
        "plan_speed_mps": _summary(plan_speeds),
        "planned_start_speed_mps": _summary([value for _, value in planned_start_speed_series]),
        "plan_velocity_acceleration_mps2": _summary(plan_accelerations),
        "boundary_velocity_jump_mps": _summary(boundary_velocity_jumps),
        "boundary_position_jump_m": _summary(boundary_position_jumps),
        "boundary_heading_step_deg": _summary(boundary_heading_steps_deg),
    }
    return metrics, {
        "speed": planned_start_speed_series,
        "velocity_jump": velocity_jump_series,
        "heading_step": heading_step_series,
    }


def _analyze(session: Path) -> dict[str, Any]:
    descriptor = _load(session / "map_descriptor.json", {})
    scenario = _load(session / "scenario.json", {})
    waypoints_raw = _mission_waypoints(session, descriptor)
    waypoints = [_point(item.get("position")) for item in waypoints_raw]
    waypoints = [item for item in waypoints if item is not None]
    ground_truth, planning = _samples(session)
    obstacles = _obstacles(session, descriptor)
    trajectory_records = _trajectory_records(session)
    observability = _runtime_observability(session)
    smoothness, plan_series = _trajectory_smoothness(trajectory_records)
    planning_continuity = _planning_continuity(planning)
    planner_trace_records = collect_planner_trace_records(scenario)
    planner_trace = {
        **planner_trace_summary(planner_trace_records),
        "records": planner_trace_records,
    }

    tracking_errors: list[float] = []
    speeds: list[float] = []
    acceleration: list[float] = []
    heading_rates: list[float] = []
    previous_t: float | None = None
    previous_speed: float | None = None
    previous_heading: float | None = None
    for sample_index, item in enumerate(ground_truth):
        point = item["position"]
        if len(waypoints) >= 2:
            tracking_errors.append(min(
                _segment_distance_2d(point, waypoints[index], waypoints[index + 1])
                for index in range(len(waypoints) - 1)
            ))
        velocity = item.get("velocity")
        speed = math.sqrt(sum(value * value for value in velocity)) if velocity else None
        if speed is not None and math.isfinite(speed):
            speeds.append(speed)
            if previous_t is not None and previous_speed is not None and item["t"] > previous_t:
                acceleration.append(abs(speed - previous_speed) / (item["t"] - previous_t))
            previous_speed = speed
        if previous_t is not None and item["t"] > previous_t and sample_index > 0:
            previous = ground_truth[sample_index - 1]["position"]
            dx = point[0] - previous[0]
            dy = point[1] - previous[1]
            if math.hypot(dx, dy) > 0.01:
                heading = math.atan2(dy, dx)
                if previous_heading is not None:
                    delta = (heading - previous_heading + math.pi) % (2.0 * math.pi) - math.pi
                    heading_rates.append(abs(delta) / (item["t"] - previous_t))
                previous_heading = heading
        previous_t = item["t"]

    segment_lengths = [_distance(waypoints[index], waypoints[index + 1]) for index in range(max(0, len(waypoints) - 1))]
    turn_angles: list[float] = []
    for index in range(1, len(waypoints) - 1):
        incoming = (waypoints[index][0] - waypoints[index - 1][0], waypoints[index][1] - waypoints[index - 1][1])
        outgoing = (waypoints[index + 1][0] - waypoints[index][0], waypoints[index + 1][1] - waypoints[index][1])
        denominator = math.hypot(*incoming) * math.hypot(*outgoing)
        if denominator > 1e-9:
            turn_angles.append(math.degrees(math.acos(max(-1.0, min(1.0, (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / denominator)))))

    first_plan = planning[0] if planning else {}
    horizon_values = [number for item in planning if (number := _finite_number(item.get("known_free_horizon_m"))) is not None]
    planning_horizon_values = [number for item in planning if (number := _finite_number(item.get("planning_horizon_distance_m"))) is not None]
    path_lengths = [number for item in planning if (number := _finite_number(item.get("geometric_path_length_m"))) is not None]
    total_us = [number for item in planning if (number := _finite_number(item.get("planning_total_us"))) is not None]
    min_clearance = [number for item in planning if (number := _finite_number(item.get("minimum_clearance_m"))) is not None]
    long_leg = max(segment_lengths, default=0.0)
    known_horizon = max(horizon_values, default=None)
    metrics = {
        "mission": {
            "waypoint_count": len(waypoints),
            "segment_lengths_m": segment_lengths,
            "longest_leg_m": long_leg,
            "long_leg_exceeds_known_free_horizon": bool(known_horizon and long_leg > known_horizon),
            "turn_angles_deg": turn_angles,
            "orthogonal_turn_count": sum(1 for angle in turn_angles if angle >= 80.0),
            "duration_sim_s": scenario.get("duration_s"),
            "wall_elapsed_s": scenario.get("wall_elapsed_s"),
        },
        "tracking": {
            "cross_track_error_m": _summary(tracking_errors),
            "speed_mps": _summary(speeds),
            "speed_acceleration_mps2": _summary(acceleration),
            "heading_rate_rad_s": _summary(heading_rates),
        },
        "smoothness": smoothness,
        "planning": {
            "diagnostic_sample_count": len(planning),
            "first_plan_total_us": first_plan.get("planning_total_us"),
            "first_plan_path_length_m": first_plan.get("geometric_path_length_m"),
            "first_plan_duration_s": first_plan.get("duration_s"),
            "known_free_horizon_m": _summary(horizon_values),
            "planning_horizon_distance_m": _summary(planning_horizon_values),
            "geometric_path_length_m": _summary(path_lengths),
            "planning_total_us": _summary(total_us),
            "minimum_clearance_m": _summary(min_clearance),
            "full_replan_count": max((_finite_number(item.get("full_replan_count")) or 0.0 for item in planning), default=0.0),
            "local_subgoal_selected_count": max((_finite_number(item.get("local_subgoal_selected_count")) or 0.0 for item in planning), default=0.0),
            "continuity": planning_continuity,
            "rolling_bundle_trace": planner_trace,
        },
        "safety": {
            "outcome": scenario.get("outcome"),
            "collision_count": scenario.get("collision_count"),
            "minimum_collision_clearance_m": scenario.get("minimum_collision_clearance_m"),
            "minimum_collision_obstacle_name": scenario.get("minimum_collision_obstacle_name"),
            "actual_min_clearance_m": scenario.get("actual_min_clearance_m"),
            "trajectory_failure_count": scenario.get("trajectory_failure_count"),
            "safety_transition_count": scenario.get("safety_transition_count"),
        },
        "control": {
            "setpoint_speed_mps": scenario.get("speed_metrics", {}).get("setpoint_mps", {}),
            "measured_speed_mps": scenario.get("speed_metrics", {}).get("measured_mps", {}),
            "trajectory_role_counts": scenario.get("trajectory_role_counts", {}),
        },
        "localization": scenario.get("localization_watchdog", {}),
        "obstacles": obstacles,
        "route_obstacles": list(descriptor.get("route_obstacles", [])),
        "route_segment_waypoints": list(descriptor.get("route_segment_waypoints", [])),
    }
    metrics["acceptance"] = _acceptance_summary(
        session,
        scenario,
        len(waypoints),
        metrics["tracking"]["cross_track_error_m"].get("p95"),
    )
    return {
        "metrics": metrics,
        "waypoints": waypoints,
        "ground_truth": ground_truth,
        "planning": planning,
        "trajectory_records": trajectory_records,
        "plan_series": plan_series,
        "observability": observability,
        "descriptor": descriptor,
    }

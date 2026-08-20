#!/usr/bin/env python3
"""Generate a self-contained HTML benchmark report for an SITL session.

The report deliberately uses only inline SVG and JSON so it remains useful on
an isolated test machine.  It consumes the recorder's scenario.jsonl and
samples.jsonl files; no ROS graph or running simulator is required.
"""

from __future__ import annotations

import argparse
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
                elif item.get("stream") == "planning_diagnostics":
                    payload = item.get("payload", {})
                    for status in payload.get("statuses", []):
                        if isinstance(status, dict) and status.get("name") == "navigation_planning/planner":
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
            sum(count for key, count in safety_kinds.items()
                if key.lower() in {"braking_stop", "2"}) / len(planning)
            if rolling_planning else 0.0
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
        "descriptor": descriptor,
    }


def _number(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return html.escape(str(value))
    return f"{number:.{digits}f}"


def _polyline(points: list[tuple[float, float, float]], transform: Any, color: str, width: int = 3, dash: str = "") -> str:
    if len(points) < 2:
        return ""
    coordinates = " ".join(f"{transform(point)[0]:.1f},{transform(point)[1]:.1f}" for point in points)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>'


def _map_svg(data: dict[str, Any]) -> str:
    actual = [item["position"] for item in data["ground_truth"]]
    all_points = actual + data["waypoints"]
    for obstacle in data["metrics"]["obstacles"]:
        all_points.append(obstacle["center"])
    if not all_points:
        return "<p>No trajectory samples recorded.</p>"
    min_x = min(point[0] for point in all_points) - 3.0
    max_x = max(point[0] for point in all_points) + 3.0
    min_y = min(point[1] for point in all_points) - 3.0
    max_y = max(point[1] for point in all_points) + 3.0
    width, height = 940.0, 460.0
    scale = min(width / max(1.0, max_x - min_x), height / max(1.0, max_y - min_y))
    def transform(point: tuple[float, float, float]) -> tuple[float, float]:
        return (60.0 + (point[0] - min_x) * scale, height - 30.0 - (point[1] - min_y) * scale)
    parts = [f'<svg viewBox="0 0 {width:.0f} {height:.0f}" role="img" aria-label="2D flight path">', '<rect width="100%" height="100%" fill="#101722"/>']
    for obstacle in data["metrics"]["obstacles"]:
        center = transform(obstacle["center"])
        is_route_obstacle = obstacle["name"] in set(data["metrics"].get("route_obstacles", []))
        obstacle_fill = "#e05252" if is_route_obstacle else "#64748b"
        if obstacle["type"] == "cylinder":
            radius = float(obstacle.get("radius_m", 0.0)) * scale
            parts.append(f'<circle cx="{center[0]:.1f}" cy="{center[1]:.1f}" r="{radius:.1f}" fill="{obstacle_fill}" fill-opacity="0.75" stroke="#ffb4b4"/>')
        else:
            half = obstacle.get("half_extents", [0.0, 0.0, 0.0])
            left = transform((obstacle["center"][0] - half[0], obstacle["center"][1] + half[1], 0.0))
            right = transform((obstacle["center"][0] + half[0], obstacle["center"][1] - half[1], 0.0))
            parts.append(f'<rect x="{left[0]:.1f}" y="{left[1]:.1f}" width="{right[0]-left[0]:.1f}" height="{right[1]-left[1]:.1f}" fill="{obstacle_fill}" fill-opacity="0.75"/>')
        parts.append(f'<text x="{center[0]+6:.1f}" y="{center[1]-6:.1f}" fill="#ffd8d8" font-size="11">{html.escape(str(obstacle["name"]))}</text>')
    for index, waypoint in enumerate(data["waypoints"]):
        x, y = transform(waypoint)
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#ffd166" stroke="#fff3c4"/><text x="{x+7:.1f}" y="{y-7:.1f}" fill="#fff3c4" font-size="11">WP{index}</text>')
    parts.append(_polyline(actual, transform, "#4cc9f0", 3))
    records = data["trajectory_records"]
    # Keep the raw history complete in scenario.json/benchmark_metrics while
    # sampling the SVG overlay so a long SITL run remains inspectable in a
    # browser instead of becoming a megabyte-scale path graphic.
    record_step = max(1, len(records) // 64)
    for index, record in enumerate(records):
        if index % record_step != 0 and index != len(records) - 1:
            continue
        points = [_point(item) for item in record.get("position_points", [])]
        points = [item for item in points if item is not None]
        parts.append(_polyline(points, transform, "#9be564", 1, "5 4"))
    parts.append('<text x="18" y="24" fill="#dbeafe" font-size="14">blue: ground truth · green dashed: committed local plans · red: route columns · slate: texture/features · yellow: waypoints</text></svg>')
    return "".join(parts)


def _chart_svg(series: list[tuple[str, list[tuple[float, float]], str]], y_label: str, width: int = 940, height: int = 250) -> str:
    points = [point for _, values, _ in series for point in values]
    if not points:
        return "<p>No chart samples recorded.</p>"
    min_t, max_t = min(point[0] for point in points), max(point[0] for point in points)
    max_y = max(1.0, max(point[1] for point in points))
    def transform(point: tuple[float, float]) -> tuple[float, float]:
        x = 55.0 + (point[0] - min_t) / max(1e-9, max_t - min_t) * (width - 80.0)
        y = height - 30.0 - point[1] / max_y * (height - 60.0)
        return x, y
    parts = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(y_label)} chart"><rect width="100%" height="100%" fill="#101722"/><line x1="55" y1="20" x2="55" y2="{height-30}" stroke="#94a3b8"/><line x1="55" y1="{height-30}" x2="{width-25}" y2="{height-30}" stroke="#94a3b8"/><text x="8" y="28" fill="#dbeafe" font-size="12">{html.escape(y_label)}</text>']
    for label, values, color in series:
        if values:
            coordinates = " ".join(f"{transform(point)[0]:.1f},{transform(point)[1]:.1f}" for point in values)
            parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round"/><text x="{width-180 + len(parts)*2}" y="22" fill="{color}" font-size="11">{html.escape(label)}</text>')
    parts.append("</svg>")
    return "".join(parts)


def generate(session: Path) -> Path:
    data = _analyze(session)
    metrics_path = session / "benchmark_metrics.json"
    metrics_path.write_text(json.dumps(data["metrics"], indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ground_truth = data["ground_truth"]
    speed_series: list[tuple[float, float]] = []
    error_series: list[tuple[float, float]] = []
    for item in ground_truth:
        velocity = item.get("velocity")
        if velocity:
            speed_series.append((item["t"], math.sqrt(sum(value * value for value in velocity))))
        if data["waypoints"]:
            error_series.append((item["t"], min(_segment_distance_2d(item["position"], data["waypoints"][index], data["waypoints"][index + 1]) for index in range(len(data["waypoints"]) - 1))))
    planning_path = [(item["t"], number) for item in data["planning"] if (number := _finite_number(item.get("geometric_path_length_m"))) is not None]
    horizon_path = [(item["t"], number) for item in data["planning"] if (number := _finite_number(item.get("known_free_horizon_m"))) is not None]
    planning_horizon_path = [(item["t"], number) for item in data["planning"] if (number := _finite_number(item.get("planning_horizon_distance_m"))) is not None]
    horizon_progress_series = [(item["t"], number) for item in data["planning"] if (number := _finite_number(item.get("horizon_progress_m"))) is not None]
    forward_projection_series = [(item["t"], number) for item in data["planning"] if (number := _finite_number(item.get("horizon_forward_projection_m"))) is not None]
    plan_speed_series = data["plan_series"]["speed"]
    plan_velocity_jump_series = data["plan_series"]["velocity_jump"]
    plan_heading_step_series = data["plan_series"]["heading_step"]
    metrics = data["metrics"]
    acceptance = metrics.get("acceptance", {})
    def cell(value: Any, digits: int = 3) -> str:
        return html.escape(_number(value, digits))
    def json_cell(value: Any) -> str:
        if value is None:
            return "n/a"
        return html.escape(json.dumps(value, sort_keys=True))
    acceptance_events = acceptance.get("waypoint_acceptance_events")
    acceptance_events_detail = (
        f"<details><summary>{len(acceptance_events)} event(s)</summary>"
        f"<pre>{html.escape(json.dumps(acceptance_events, indent=2, sort_keys=True))}</pre></details>"
        if isinstance(acceptance_events, list)
        else "unavailable"
    )
    acceptance_reasons = acceptance.get("reasons", [])
    acceptance_reasons_text = (
        "none" if not acceptance_reasons else html.escape("; ".join(str(item) for item in acceptance_reasons))
    )
    table = "".join([
        f"<tr><th>Outcome</th><td>{html.escape(str(metrics['safety'].get('outcome')))}</td></tr>",
        f"<tr><th>Mission completion observed</th><td>{html.escape(str(acceptance.get('mission_complete_observed')))}</td></tr>",
        f"<tr><th>Waypoint acceptance indices</th><td>{json_cell(acceptance.get('waypoint_acceptance_indices'))} / expected {json_cell(acceptance.get('expected_waypoint_indices'))}</td></tr>",
        f"<tr><th>Waypoint acceptance events</th><td>{acceptance_events_detail}</td></tr>",
        f"<tr><th>Goal indices (diagnostic only)</th><td>{json_cell(acceptance.get('goal_indices'))}</td></tr>",
        f"<tr><th>Waypoint acceptance complete</th><td>{html.escape(str(acceptance.get('waypoint_acceptance_complete')))}</td></tr>",
        f"<tr><th>Cross-track p95 / acceptance limit</th><td>{cell(acceptance.get('cross_track_p95_m'))} m / {cell(acceptance.get('max_cross_track_p95_m'))} m</td></tr>",
        f"<tr><th>Acceptance gate reasons</th><td>{acceptance_reasons_text}</td></tr>",
        f"<tr><th>Mission sim / wall time</th><td>{cell(metrics['mission'].get('duration_sim_s'))} s / {cell(metrics['mission'].get('wall_elapsed_s'))} s</td></tr>",
        f"<tr><th>Longest leg / known horizon</th><td>{cell(metrics['mission'].get('longest_leg_m'))} m / {cell(metrics['planning']['known_free_horizon_m'].get('maximum'))} m</td></tr>",
        f"<tr><th>Rolling planning horizon p50 / max</th><td>{cell(metrics['planning']['planning_horizon_distance_m'].get('p50'))} m / {cell(metrics['planning']['planning_horizon_distance_m'].get('maximum'))} m</td></tr>",
        f"<tr><th>Cross-track RMSE / p95</th><td>{cell(metrics['tracking']['cross_track_error_m'].get('rmse'))} m / {cell(metrics['tracking']['cross_track_error_m'].get('p95'))} m</td></tr>",
        f"<tr><th>Speed mean / p95</th><td>{cell(metrics['tracking']['speed_mps'].get('mean'))} / {cell(metrics['tracking']['speed_mps'].get('p95'))} m/s</td></tr>",
        f"<tr><th>Plan profile speed mean / p95</th><td>{cell(metrics['smoothness']['plan_speed_mps'].get('mean'))} / {cell(metrics['smoothness']['plan_speed_mps'].get('p95'))} m/s</td></tr>",
        f"<tr><th>Time-aligned handover velocity jump p95</th><td>{cell(metrics['smoothness']['boundary_velocity_jump_mps'].get('p95'))} m/s</td></tr>",
        f"<tr><th>Time-aligned handover position jump p95</th><td>{cell(metrics['smoothness']['boundary_position_jump_m'].get('p95'))} m</td></tr>",
        f"<tr><th>Time-aligned handover heading step p95</th><td>{cell(metrics['smoothness']['boundary_heading_step_deg'].get('p95'))}°</td></tr>",
        f"<tr><th>Handover samples / expired plans</th><td>{cell(metrics['smoothness'].get('handover_sample_count'), 0)} / {cell(metrics['smoothness'].get('handover_expired_count'), 0)}</td></tr>",
        f"<tr><th>PX4 setpoint max / LIO residual p95</th><td>{cell(metrics['control']['setpoint_speed_mps'].get('maximum'))} m/s / {cell(metrics['localization'].get('p95_position_residual_m'))} m</td></tr>",
        f"<tr><th>Planning total p95</th><td>{cell(metrics['planning']['planning_total_us'].get('p95'))} µs</td></tr>",
        f"<tr><th>Replans / local subgoals</th><td>{cell(metrics['planning'].get('full_replan_count'), 0)} / {cell(metrics['planning'].get('local_subgoal_selected_count'), 0)}</td></tr>",
        f"<tr><th>Rolling endpoints / changes / repeats</th><td>{cell(metrics['planning']['continuity'].get('unique_endpoint_count'), 0)} / {cell(metrics['planning']['continuity'].get('endpoint_change_count'), 0)} / {cell(metrics['planning']['continuity'].get('endpoint_repeat_count'), 0)}</td></tr>",
        f"<tr><th>Max endpoint repeat / backward projections</th><td>{cell(metrics['planning']['continuity'].get('maximum_endpoint_repeat_run'), 0)} / {cell(metrics['planning']['continuity'].get('backward_projection_count'), 0)}</td></tr>",
        f"<tr><th>Safety-stop ratio / tangent reversals</th><td>{cell(100.0 * metrics['planning']['continuity'].get('safety_stop_ratio', 0.0), 1)}% / {cell(metrics['planning']['continuity'].get('tangent_reversal_count'), 0)}</td></tr>",
        f"<tr><th>Rolling samples / terminal hold refreshes</th><td>{cell(metrics['planning']['continuity'].get('rolling_sample_count'), 0)} / {cell(metrics['planning']['continuity'].get('terminal_hold_sample_count'), 0)}</td></tr>",
        f"<tr><th>Collision / min clearance</th><td>{cell(metrics['safety'].get('collision_count'), 0)} / {cell(metrics['safety'].get('minimum_collision_clearance_m'))} m ({html.escape(str(metrics['safety'].get('minimum_collision_obstacle_name') or 'n/a'))})</td></tr>",
    ])
    trace = metrics["planning"].get("rolling_bundle_trace", {})
    trace_records = trace.get("records", []) if isinstance(trace, dict) else []
    trace_rows = "".join(
        "<tr>"
        f"<td>{html.escape(str(record.get('planning_cycle_id')))}</td>"
        f"<td>{html.escape(str(record.get('bundle_id')))}</td>"
        f"<td>{html.escape(str(record.get('route_id')))}</td>"
        f"<td>{cell(record.get('horizon_start_arc_m'))} → {cell(record.get('horizon_end_arc_m', record.get('horizon_arc_m')))}</td>"
        f"<td>{html.escape(str(record.get('selected_branch')))}</td>"
        f"<td>{cell(record.get('splice_position_residual_m'))}</td>"
        f"<td>{html.escape(str(record.get('failure_code') or ''))}</td>"
        "</tr>"
        for record in trace_records
    )
    if not trace_rows:
        trace_rows = '<tr><td colspan="7">No explicit rolling-bundle trace was published; this session is partial.</td></tr>'
    trace_section = (
        "<section><h2>Rolling bundle trace</h2>"
        f"<p>records: {html.escape(str(trace.get('record_count', 0)))} · "
        f"complete: {html.escape(str(trace.get('complete_record_count', 0)))} · "
        f"partial: {html.escape(str(trace.get('partial_record_count', 0)))}</p>"
        '<table><tr><th>cycle</th><th>bundle</th><th>route</th><th>arc horizon (m)</th>'
        '<th>branch</th><th>splice p (m)</th><th>failure</th></tr>'
        f"{trace_rows}</table>"
        f"<details><summary>Raw rolling trace JSON</summary><pre>{html.escape(json.dumps(trace_records, indent=2, sort_keys=True))}</pre></details></section>"
    )
    html_text = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>UAV navigation benchmark</title><style>body{{font-family:system-ui,sans-serif;max-width:1000px;margin:2rem auto;padding:0 1rem;background:#0b1220;color:#e5e7eb}}section{{background:#111b2e;border:1px solid #263653;border-radius:10px;padding:1rem;margin:1rem 0}}h1,h2{{color:#dbeafe}}table{{border-collapse:collapse;width:100%}}th,td{{border-bottom:1px solid #263653;text-align:left;padding:.45rem}}th{{width:38%;color:#a5b4fc}}svg{{width:100%;height:auto;border-radius:6px}}.ok{{color:#86efac}}.warn{{color:#fbbf24}}</style></head><body><h1>UAV mission benchmark</h1><p>Session: <code>{html.escape(str(session))}</code></p><section><h2>Acceptance metrics</h2><table>{table}</table></section>{trace_section}<section><h2>2D flight path</h2>{_map_svg(data)}</section><section><h2>Speed</h2>{_chart_svg([('measured speed', speed_series, '#4cc9f0')], 'm/s')}</section><section><h2>Planner smoothness</h2><p>Green is the planned start speed at each trajectory publication. Orange is the time-aligned velocity difference between the previous active plan and the newly published plan. Rolling profiles overlap in time and are not concatenated.</p>{_chart_svg([('planned start speed', plan_speed_series, '#9be564'), ('time-aligned handover jump', plan_velocity_jump_series, '#f97316')], 'm/s')} {_chart_svg([('time-aligned heading step', plan_heading_step_series, '#c084fc')], 'degrees')}</section><section><h2>Cross-track error</h2>{_chart_svg([('cross-track error', error_series, '#fbbf24')], 'm')}</section><section><h2>Planner horizon and path</h2>{_chart_svg([('geometric path length', planning_path, '#9be564'), ('known-free horizon', horizon_path, '#f472b6'), ('planning horizon', planning_horizon_path, '#facc15'), ('horizon progress', horizon_progress_series, '#38bdf8'), ('forward projection', forward_projection_series, '#fb7185')], 'm')}</section><section><h2>Raw metrics</h2><pre>{html.escape(json.dumps(metrics, indent=2, sort_keys=True))}</pre></section></body></html>"""
    output = session / "REPORT.html"
    output.write_text(html_text, encoding="utf-8")
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    output = generate(args.session.resolve())
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

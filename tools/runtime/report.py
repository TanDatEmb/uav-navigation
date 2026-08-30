#!/usr/bin/env python3
"""Build the single report artifact used by all runtime workflows.

This is the only public report tool.  The HTML analysis and renderer modules
are implementation details invoked from this entrypoint.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import html
import json
import hashlib
import math
from pathlib import Path
import subprocess
from typing import Any

import yaml

from planner_trace import (
    collect_planner_trace_records,
    planner_timing_is_current,
    planner_trace_summary,
)


VERDICTS = {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "OBSERVATION_COMPLETE"}


def _number(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _dedupe_reasons(reasons: list[str]) -> list[str]:
    """Preserve first-seen report reasons without repeating the same finding."""
    return list(dict.fromkeys(str(reason) for reason in reasons))


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


def _captured_provenance_valid(captured: Any) -> bool:
    if not isinstance(captured, dict) or captured.get("status") != "VALID":
        return False
    manifest = captured.get("manifest")
    source = manifest.get("source") if isinstance(manifest, dict) else None
    artifacts = manifest.get("artifacts") if isinstance(manifest, dict) else None
    if not (
        isinstance(manifest, dict)
        and manifest.get("schema_version") == 1
        and manifest.get("authoritative") is True
        and manifest.get("build_mode") == "release"
        and isinstance(source, dict)
        and isinstance(source.get("sha256"), str)
        and isinstance(source.get("git_head"), str)
        and isinstance(artifacts, list) and artifacts
    ):
        return False
    expected_manifest_sha = hashlib.sha256(
        (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    ).hexdigest()
    if captured.get("manifest_sha256") != expected_manifest_sha:
        return False
    for artifact in artifacts:
        if not (
            isinstance(artifact, dict)
            and isinstance(artifact.get("path"), str) and artifact["path"]
            and isinstance(artifact.get("resolved_path"), str) and artifact["resolved_path"]
            and isinstance(artifact.get("size_bytes"), int)
            and not isinstance(artifact.get("size_bytes"), bool)
            and artifact["size_bytes"] >= 0
            and isinstance(artifact.get("sha256"), str)
            and len(artifact["sha256"]) == 64
        ):
            return False
        if any(
            character not in "0123456789abcdef" for character in artifact["sha256"].lower()
        ):
            return False
        resolved_path = Path(artifact["resolved_path"])
        if not resolved_path.is_absolute():
            return False
        try:
            resolved_path = resolved_path.resolve(strict=True)
            stat = resolved_path.stat()
            if not resolved_path.is_file() or stat.st_size != artifact["size_bytes"]:
                return False
            digest = hashlib.sha256()
            with resolved_path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            if digest.hexdigest() != artifact["sha256"].lower():
                return False
        except (OSError, ValueError):
            return False
    return True


def _session_provenance(session: Path, workspace: Path, px4_dir: Path | None = None) -> dict[str, Any]:
    runtime = _load_json(session / "runtime.json", {})
    captured = runtime.get("build_provenance") if isinstance(runtime, dict) else None
    if _captured_provenance_valid(captured):
        return dict(captured)
    return {
        "status": "INVALID",
        "reason": "session has no validated authoritative build provenance",
        "legacy_git_observation": provenance(workspace, px4_dir),
    }


def _provenance_reasons(runtime: dict[str, Any]) -> list[str]:
    captured = runtime.get("build_provenance") if isinstance(runtime, dict) else None
    if not _captured_provenance_valid(captured):
        return ["runtime did not capture a validated authoritative Release build manifest"]
    return []


def _gazebo_native_diagnostics(session: Path, runtime: dict[str, Any]) -> dict[str, Any]:
    summary = _load_json(session / "gazebo_native_summary.json", {})
    observer = runtime.get("gazebo_native_observer", {})
    if not isinstance(summary, dict):
        summary = {}
    if not isinstance(observer, dict):
        observer = {}
    if not summary and not observer:
        return {}
    result = dict(summary)
    result["observer"] = observer
    result["verdict_owner"] = "diagnostic_only"
    return result


def _load_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _load_yaml_dict(path: Path) -> dict[str, Any]:
    """Load a YAML mapping without making report generation fail closed on a bad artifact."""
    try:
        import yaml

        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (ImportError, OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _point3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        point = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


def _segment_distance_2d(
    point: tuple[float, float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if not all(math.isfinite(value) for value in (dx, dy)):
        return math.inf
    scale = max(abs(dx), abs(dy))
    if scale == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ux = dx / scale
    uy = dy / scale
    px = (point[0] - start[0]) / scale
    py = (point[1] - start[1]) / scale
    if not all(math.isfinite(value) for value in (px, py)):
        return math.inf
    length_sq = ux * ux + uy * uy
    projection = (px * ux + py * uy) / length_sq
    projection = max(0.0, min(1.0, projection))
    residual = scale * math.hypot(px - projection * ux, py - projection * uy)
    return residual if math.isfinite(residual) else math.inf


def _acceptance_threshold(config: dict[str, Any], scenario_config: dict[str, Any]) -> float:
    """Resolve the report-only cross-track quality threshold.

    The session scenario is allowed to override the top-level report config so
    a benchmark can carry its acceptance contract in its own artifact. The
    default is deliberately conservative: a report cannot silently pass a
    mission with metre-scale tracking error.
    """
    candidates = (
        config.get("acceptance"),
        scenario_config.get("acceptance"),
        config.get("runtime", {}).get("acceptance"),
        config.get("runtime", {}).get("thresholds"),
        scenario_config.get("scenario", {}).get("acceptance"),
    )
    for candidate in candidates:
        if not isinstance(candidate, dict) or "max_cross_track_p95_m" not in candidate:
            continue
        value = _number(candidate.get("max_cross_track_p95_m"), math.nan)
        if math.isfinite(value) and value >= 0.0:
            return value
    return 0.5


def _mission_waypoints_for_acceptance(
    session: Path,
    scenario_config: dict[str, Any],
    workspace: Path,
) -> list[tuple[float, float, float]]:
    scenario = scenario_config.get("scenario", {})
    if not isinstance(scenario, dict):
        return []
    mission_file = scenario.get("mission_file")
    if not mission_file:
        return []
    mission_path = Path(str(mission_file))
    if not mission_path.is_absolute():
        mission_path = (workspace / mission_path).resolve()
    mission = _load_yaml_dict(mission_path)
    raw_waypoints = mission.get("mission", {}).get("waypoints", [])
    if not isinstance(raw_waypoints, list):
        return []
    return [
        point
        for item in raw_waypoints
        if isinstance(item, dict)
        for point in [_point3(item.get("position"))]
        if point is not None
    ]


def _mission_cross_track_p95(
    session: Path,
    waypoints: list[tuple[float, float, float]],
    start_after_sim_time_ns: int | None = None,
) -> tuple[float | None, int]:
    if len(waypoints) < 2:
        return None, 0
    distances: list[float] = []
    for item in _samples(session / "samples.jsonl"):
        if item.get("stream") != "ground_truth_odometry":
            continue
        if start_after_sim_time_ns is not None:
            raw_stamp = item.get(
                "sim_time_ns", item.get("source_stamp_ns", item.get("timestamp_ns"))
            )
            try:
                if int(raw_stamp) < start_after_sim_time_ns:
                    continue
            except (TypeError, ValueError):
                continue
        position = _point3(item.get("payload", {}).get("position"))
        if position is None:
            continue
        distances.append(min(
            _segment_distance_2d(position, waypoints[index], waypoints[index + 1])
            for index in range(len(waypoints) - 1)
        ))
    return _p(distances, 0.95), len(distances)


def _mission_acceptance(
    session: Path,
    config: dict[str, Any],
    scenario: dict[str, Any],
    workspace: Path,
) -> dict[str, Any]:
    """Validate mission completion independently from the scenario process.

    ``scenario.json`` is an observation produced by the runner and may contain
    an optimistic terminal outcome. This gate is intentionally report-owned so
    a missing completion event, incomplete waypoint coverage, or bad tracking
    cannot become a quality PASS merely because the process exited cleanly.
    Fail-closed scenarios are excluded because not completing the mission is
    their expected result.
    """
    session_config = _load_yaml_dict(session / "scenario_config.yaml")
    scenario_parameters = session_config.get("scenario", {})
    if not isinstance(scenario_parameters, dict):
        scenario_parameters = {}
    if not scenario_parameters:
        fallback = config.get("scenario", {})
        scenario_parameters = fallback if isinstance(fallback, dict) else {}

    expected_outcome = str(
        scenario.get("expected_outcome", scenario_parameters.get("expected_outcome", "complete"))
    )
    execution = str(scenario_parameters.get("execution", ""))
    waypoint_count = int(_number(
        scenario_parameters.get("mission_waypoint_count", scenario.get("mission_waypoint_count", 0)),
        0.0,
    ))
    enabled = execution == "mission" or waypoint_count > 0 or "mission_complete_observed" in scenario
    result: dict[str, Any] = {
        "enabled": enabled,
        "expected_outcome": expected_outcome,
        "mission_waypoint_count": waypoint_count,
        "mission_complete_observed": scenario.get("mission_complete_observed"),
        # Goal publications are useful planner diagnostics only. They are not
        # evidence that the vehicle entered a waypoint acceptance radius.
        "goal_indices": list(scenario.get("goal_indices", [])),
        "waypoint_acceptance_indices": [],
        "waypoint_acceptance_complete": None,
        "cross_track_error_p95_m": None,
        "cross_track_sample_count": 0,
        "max_cross_track_p95_m": _acceptance_threshold(config, session_config),
        "reasons": [],
    }
    if not enabled or expected_outcome == "fail_closed":
        return result

    reasons = result["reasons"]
    if str(scenario.get("outcome", "")) == "COMPLETE" and not bool(
        scenario.get("mission_complete_observed", False)
    ):
        reasons.append("mission COMPLETE without mission_complete_observed")
    elif not bool(scenario.get("mission_complete_observed", False)):
        reasons.append("mission completion event was not observed")

    raw_goal_indices = scenario.get("goal_indices", [])
    goal_indices: list[int] = []
    if isinstance(raw_goal_indices, list):
        for value in raw_goal_indices:
            try:
                goal_indices.append(int(value))
            except (TypeError, ValueError):
                continue
    expected_indices = list(range(max(0, waypoint_count)))
    allow_initial_skip = bool(scenario_parameters.get("allow_initial_pass_through_skip", False))
    result["goal_indices"] = goal_indices
    if waypoint_count <= 0:
        reasons.append("mission_waypoint_count is not configured")

    raw_acceptance_events = scenario.get("waypoint_acceptance_events")
    accepted_indices: list[int] = []
    if isinstance(raw_acceptance_events, list):
        for event in raw_acceptance_events:
            if isinstance(event, dict):
                if event.get("waypoint_accepted") is False:
                    continue
                value = event.get("accepted_waypoint_index")
            else:
                # Keep the parser tolerant of a compact artifact representation
                # while retaining acceptance events as the sole authority.
                value = event
            try:
                accepted_indices.append(int(value))
            except (TypeError, ValueError):
                continue
    result["waypoint_acceptance_indices"] = accepted_indices
    valid_acceptance_indices = accepted_indices == expected_indices or (
        allow_initial_skip and bool(expected_indices) and accepted_indices == expected_indices[1:]
    )
    result["waypoint_acceptance_complete"] = (
        valid_acceptance_indices if waypoint_count > 0 else False
    )
    if not isinstance(raw_acceptance_events, list):
        reasons.append("waypoint acceptance evidence is unavailable")
    elif waypoint_count <= 0:
        pass
    elif not valid_acceptance_indices:
        reasons.append(
            "waypoint acceptance coverage incomplete: "
            f"expected {expected_indices}, got {accepted_indices}"
        )

    waypoints = _mission_waypoints_for_acceptance(session, session_config, workspace)
    # A pass-through mission may deliberately begin at a waypoint that is not
    # the takeoff pose.  Ground-truth samples before that waypoint is accepted
    # describe the initial repositioning, not tracking of the mission polyline
    # and can be several metres away by construction.  Start the metric at the
    # first acceptance event while retaining the normal gate and fail-closed
    # behaviour when no such event exists.
    first_acceptance_sim_time_ns: int | None = None
    first_expected_acceptance_index = (
        expected_indices[1]
        if allow_initial_skip and expected_indices
        else expected_indices[0] if expected_indices else None
    )
    for event in _scenario_events(session / "scenario.jsonl"):
        if event.get("kind") != "waypoint_accepted":
            continue
        payload = event.get("payload", {})
        if not isinstance(payload, dict) or payload.get("waypoint_accepted") is not True:
            continue
        try:
            accepted_index = int(payload.get("accepted_waypoint_index"))
            sim_time_ns = int(event.get("sim_time_ns"))
        except (TypeError, ValueError):
            continue
        if (first_expected_acceptance_index is not None and
                accepted_index == first_expected_acceptance_index and sim_time_ns >= 0):
            first_acceptance_sim_time_ns = sim_time_ns
            break
    cross_track_p95, sample_count = _mission_cross_track_p95(
        session, waypoints, first_acceptance_sim_time_ns
    )
    result["cross_track_error_p95_m"] = cross_track_p95
    result["cross_track_sample_count"] = sample_count
    if cross_track_p95 is None:
        reasons.append("tracking cross-track p95 is unavailable")
    elif cross_track_p95 > result["max_cross_track_p95_m"]:
        reasons.append(
            "tracking cross-track p95 exceeded "
            f"{result['max_cross_track_p95_m']:.3f} m "
            f"(observed {cross_track_p95:.3f} m)"
        )
    if str(scenario.get("outcome", "")) != "COMPLETE":
        reasons.append(f"mission did not reach COMPLETE outcome: {scenario.get('outcome', 'UNKNOWN')}")
    return result


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
            if not isinstance(item, dict):
                continue
            if item.get("kind") == "sample":
                result.append(item)
    return result


_PERCEPTION_STREAMS = {
    "imu", "lidar", "corrected_odometry", "propagated_odometry",
    "diagnostics", "mapping_diagnostics", "tf", "tf_static",
}
_EXECUTION_STREAMS = {
    "ground_truth_odometry", "external_odometry", "px4_odometry",
    "vehicle_status", "local_position", "vehicle_attitude",
    "estimator_status", "estimator_status_flags", "estimator_innovations",
}


def _qualification_timeline_domain(record: dict[str, Any]) -> str:
    """Assign one evidence owner without inferring missing runtime state."""
    stream = str(record.get("stream", "")).lower()
    kind = str(record.get("kind", "")).lower()
    if stream in _PERCEPTION_STREAMS or any(
        token in kind for token in ("lidar", "mapping", "odometry", "estimator", "health")
    ):
        return "perception"
    if stream in _EXECUTION_STREAMS or any(
        token in kind for token in
        ("setpoint", "vehicle", "px4", "mode_status", "mission", "waypoint", "handover")
    ):
        return "execution"
    return "planning"


def _qualification_event_time_ns(record: dict[str, Any]) -> int | None:
    for key in ("sim_time_ns", "timestamp_ns", "arrival_wall_ns"):
        value = record.get(key)
        if isinstance(value, int) and value >= 0:
            return value
    payload = record.get("payload")
    if isinstance(payload, dict):
        for key in ("stamp_ns", "source_stamp_ns", "receive_stamp_ns"):
            value = payload.get(key)
            if isinstance(value, int) and value >= 0:
                return value
    return None


def _write_qualification_timelines(session: Path) -> dict[str, Any]:
    """Write three lossless, owner-separated JSONL evidence timelines."""
    records: list[dict[str, Any]] = []
    records.extend(_samples(session / "samples.jsonl"))
    records.extend(_scenario_events(session / "scenario.jsonl"))
    buckets: dict[str, list[dict[str, Any]]] = {
        "perception": [], "planning": [], "execution": []
    }
    for sequence, record in enumerate(records):
        domain = _qualification_timeline_domain(record)
        buckets[domain].append({
            "schema_version": 1,
            "sequence": sequence,
            "time_ns": _qualification_event_time_ns(record),
            "record": record,
        })
    result: dict[str, Any] = {"schema_version": 1, "complete": True}
    for domain, items in buckets.items():
        name = f"{domain}_timeline.jsonl"
        body = "".join(
            json.dumps(item, sort_keys=True, allow_nan=False) + "\n" for item in items
        )
        (session / name).write_text(body, encoding="utf-8")
        result[domain] = {"path": name, "event_count": len(items)}
    return result


def _scenario_events(path: Path) -> list[dict[str, Any]]:
    """Load compact scenario events without making malformed rows fatal."""
    result: list[dict[str, Any]] = []
    if not path.is_file():
        return result
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except (TypeError, ValueError):
                continue
            if isinstance(item, dict):
                result.append(item)
    return result


def _planner_trace_report(session: Path, samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Return explicit rolling-bundle telemetry, preserving partial sessions."""
    scenario = _load_json(session / "scenario.json", {})
    records = collect_planner_trace_records(scenario, samples)
    return {**planner_trace_summary(records), "records": records}


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
    result = {
        "sample_count": int(row.get("received", 0)),
        "mean_rate_hz": _number(row.get("mean_rate_hz")),
        "minimum_window_rate_hz": _number(row.get("minimum_window_rate_hz")),
        "p95_interval_ms": row.get("p95_interval_ms"),
        "maximum_gap_ms": _number(row.get("maximum_gap_ms")),
        "stale_event_count": int(row.get("stale_event_count", 0)),
        "stale_event_times_ns": list(row.get("stale_event_times_ns", [])),
        "timestamp_regression_count": int(row.get("timestamp_regression_count", 0)),
        "timestamp_epoch_discard_count": int(row.get("timestamp_epoch_discard_count", 0)),
        "invalid_source_timestamp_count": int(row.get("invalid_source_timestamp_count", 0)),
        "timestamp_duplicate_count": int(row.get("timestamp_duplicate_count", 0)),
        "nonfinite_message_count": int(row.get("nonfinite_message_count", 0)),
        "sampled_nonfinite_point_count": int(row.get("sampled_nonfinite_point_count", 0)),
        "invalid_quaternion_count": int(row.get("invalid_quaternion_count", 0)),
        "invalid_covariance_count": int(row.get("invalid_covariance_count", 0)),
    }
    for key in (
        "arrival_gap_event_count", "arrival_gap_event_record_count",
        "arrival_gap_event_overflow_count", "arrival_gap_event_times_ns",
        "arrival_gap_events", "maximum_arrival_gap_ms",
    ):
        if key in row:
            result[key] = row[key]
    return result


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
    observation_finished = runtime.get("observation_finished_wall_ns")
    if observation_finished:
        # The observation boundary is captured before shutdown starts. Count
        # freshness failures through that exact boundary; subtracting a grace
        # period creates a blind window while the system is still observed.
        active_times = [
            value for value in active_times
            if value < int(observation_finished)
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


def _active_arrival_gap_summary(
    name: str,
    config: dict[str, Any],
    runtime: dict[str, Any],
    samples: list[dict[str, Any]],
    row: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Measure wall-arrival gaps without depending on the monitor timer.

    A queued callback can run before the monitor's stale timer and otherwise
    erase an outage.  Deriving gaps from consecutive recorded callbacks makes
    the evidence deterministic.  Startup and post-observation gaps use the
    same active-window policy as the existing stale-event contract.
    """
    runtime_config = config.get("runtime", {})
    stream_config = runtime_config.get("streams", {}).get(name, {})
    stale_after_s = _number(
        stream_config.get(
            "stale_after_s", runtime_config.get("thresholds", {}).get("stale_after_s", 1.0)
        )
    )
    if not math.isfinite(stale_after_s) or stale_after_s <= 0.0:
        return {"count": 0, "maximum_gap_ms": None}
    threshold_ns = int(stale_after_s * 1e9)
    # Current schema: callback-owned direct gap events survive even when the
    # high-rate clock stream is intentionally absent from samples.jsonl.
    v2_keys = {
        "arrival_gap_event_count", "arrival_gap_event_record_count",
        "arrival_gap_event_overflow_count", "arrival_gap_event_times_ns",
        "arrival_gap_events", "maximum_arrival_gap_ms",
    }
    if isinstance(row, dict) and any(key in row for key in v2_keys):
        if not v2_keys.issubset(row) or not isinstance(row.get("arrival_gap_event_times_ns"), list):
            return {"count": 1, "maximum_gap_ms": None, "evidence_valid": False}
        direct_times = [
            int(value) for value in row["arrival_gap_event_times_ns"]
            if isinstance(value, int) and not isinstance(value, bool) and value > 0
        ]
        direct_records = row.get("arrival_gap_events")
        if (
            int(row.get("arrival_gap_event_count", -1)) != len(direct_times)
            or int(row.get("arrival_gap_event_record_count", -1)) != len(direct_times)
            or int(row.get("arrival_gap_event_overflow_count", -1)) != 0
            or not isinstance(direct_records, list)
            or len(direct_records) != len(direct_times)
        ):
            return {"count": 1, "maximum_gap_ms": None, "evidence_valid": False}
        gaps_by_event: dict[int, float] = {}
        intervals: list[tuple[int, int]] = []
        try:
            for expected_time, record in zip(direct_times, direct_records):
                if not isinstance(record, dict):
                    raise ValueError
                event_time = int(record["event_wall_ns"])
                before = int(record["previous_arrival_wall_ns"])
                after = int(record["arrival_wall_ns"])
                gap_ms = float(record["gap_ms"])
                if (
                    event_time != expected_time
                    or before <= 0
                    or after <= before
                    or event_time != before + threshold_ns
                    or not math.isfinite(gap_ms)
                    or abs(gap_ms - (after - before) / 1e6) > 1e-6
                ):
                    raise ValueError
                gaps_by_event[event_time] = gap_ms
                intervals.append((before, after))
        except (KeyError, TypeError, ValueError, OverflowError):
            return {"count": 1, "maximum_gap_ms": None, "evidence_valid": False}
        first_tracking_ns = _first_tracking_wall_ns(samples)
        active_until_ns: int | None = None
        if runtime.get("observation_finished_wall_ns"):
            active_until_ns = int(runtime["observation_finished_wall_ns"])
        elif runtime.get("replay_finished_wall_ns"):
            grace_ns = int(_number(runtime.get("replay_tail_grace_s"), 0.5) * 1e9)
            active_until_ns = int(runtime["replay_finished_wall_ns"]) - grace_ns
        active_direct: list[int] = []
        active_intervals: list[tuple[int, int]] = []
        active_gap_values: list[float] = []
        for before, after in intervals:
            violation_begin = before + threshold_ns
            active_begin = max(violation_begin, first_tracking_ns or violation_begin)
            active_end = min(after, active_until_ns or after)
            if active_end > active_begin:
                active_direct.append(active_begin)
                active_intervals.append((active_begin, active_end))
                active_gap_values.append((after - before) / 1e6)
        timer_times = row.get("stale_event_times_ns", [])
        active_timer = _active_stale_times(
            {"stale_event_count": len(timer_times), "stale_event_times_ns": timer_times},
            runtime,
            samples,
        ) if isinstance(timer_times, list) else []
        if active_timer is None:
            active_timer = timer_times
        # A stale timer and the later direct gap record can describe the same
        # outage. Keep the callback-owned record and add only terminal timer
        # events for which no returning callback interval exists.
        terminal_timer = [
            event for event in (active_timer or [])
            if not any(before <= event <= after for before, after in active_intervals)
        ]
        events = sorted(set(active_direct) | set(terminal_timer))
        return {
            "count": len(events),
            "maximum_gap_ms": max(active_gap_values, default=None),
            "evidence_valid": True,
        }

    # Legacy artifact fallback: derive gaps from raw callback samples.
    arrivals = sorted(
        int(item.get("arrival_wall_ns", 0))
        for item in _series(samples, name)
        if int(item.get("arrival_wall_ns", 0)) > 0
    )
    events: list[int] = []
    gaps_by_event: dict[int, float] = {}
    for before, after in zip(arrivals, arrivals[1:]):
        gap_ns = after - before
        if gap_ns > threshold_ns:
            event_ns = before + threshold_ns
            events.append(event_ns)
            gaps_by_event[event_ns] = gap_ns / 1e6
    active_events = _active_stale_times(
        {"stale_event_count": len(events), "stale_event_times_ns": events},
        runtime,
        samples,
    )
    if active_events is None:
        active_events = events
    active_gaps = [gaps_by_event[event] for event in active_events]
    return {
        "count": len(active_gaps),
        "maximum_gap_ms": max(active_gaps, default=None),
        "evidence_valid": True,
    }


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
        arrival_gaps = _active_arrival_gap_summary(
            name, config, runtime, samples, row
        )
        row["active_wall_arrival_gap_count"] = arrival_gaps["count"]
        row["maximum_active_wall_arrival_gap_ms"] = arrival_gaps["maximum_gap_ms"]
        row["arrival_gap_evidence_valid"] = arrival_gaps.get("evidence_valid", True)


def _sim_stream_stale_violation(name: str, row: dict[str, Any]) -> int:
    """Return the authoritative sim freshness count for one stream."""
    if name == "simulation_clock":
        if row.get("arrival_gap_evidence_valid") is False:
            return 1
        return int(row.get("active_wall_arrival_gap_count", 0) or 0)
    return int(row.get("source_stale_event_count", 0) or 0)


def _metric_summary(values: list[float]) -> dict[str, Any]:
    if not all(math.isfinite(value) for value in values):
        return {
            "count": len(values), "mean": None, "rmse": None,
            "p50": None, "p95": None, "maximum": None,
        }
    scale = max((abs(value) for value in values), default=0.0)
    if scale == 0.0:
        rmse = 0.0 if values else None
    else:
        scaled_sum = math.fsum((value / scale) ** 2 for value in values)
        rmse = scale * math.sqrt(scaled_sum / len(values)) if values else None
        if not math.isfinite(rmse):
            rmse = None
    return {
        "count": len(values),
        "mean": math.fsum(values) / len(values) if values else None,
        "rmse": rmse,
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
    return [
        item
        for item in samples
        if item.get("stream") == stream
        and item.get("accepted_by_monitor", True)
    ]


def _diagnostic_states(samples: list[dict[str, Any]]) -> list[str]:
    states: list[str] = []
    for item in _series(samples, "diagnostics"):
        payload = item.get("payload", {})
        values = payload.get("values", {})
        state = values.get("state", values.get("status"))
        if state is not None:
            states.append(str(state).upper())
    return states


def _observability_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    ratios: list[float] = []
    accepted_ratios: list[float] = []
    invalid_samples = 0
    rejection_counts: list[int] = []
    invalid_times: list[int] = []
    tracking_started = False
    for item in _series(samples, "diagnostics"):
        values = item.get("payload", {}).get("values", {})
        state = str(values.get("state", values.get("status", ""))).upper()
        observability_valid = values.get("translation_observability_valid") is True or str(
            values.get("translation_observability_valid", "")
        ).lower() == "true"
        # Initial IMU/map warm-up legitimately reports an invalid ratio before
        # FAST-LIO enters TRACKING. Coverage is a mission-health metric, so do
        # not charge those pre-flight samples against the airborne window.
        if not tracking_started:
            if state == "TRACKING" or observability_valid:
                tracking_started = True
            else:
                continue
        ratio = _number(values.get("translation_observability_ratio"), math.nan)
        if math.isfinite(ratio):
            ratios.append(ratio)
        if values.get("translation_observability_valid") is False or str(
            values.get("translation_observability_valid", "")
        ).lower() == "false":
            invalid_samples += 1
            invalid_times.append(_sample_time(item))
        accepted_value = values.get("accepted_residual_ratio")
        if accepted_value is None:
            accepted_count = _number(values.get("accepted_residual_count"), math.nan)
            residual_count = _number(values.get("residual_count", values.get("input_point_count")), math.nan)
            accepted_value = accepted_count / residual_count if residual_count > 0.0 else math.nan
        accepted = _number(accepted_value, math.nan)
        if math.isfinite(accepted):
            accepted_ratios.append(accepted)
        rejection_counts.append(int(_number(values.get("observability_rejection_count"), 0.0)))
    bursts: list[float] = []
    if invalid_times:
        start = previous = invalid_times[0]
        for timestamp in invalid_times[1:]:
            if timestamp - previous > 400_000_000:
                bursts.append((previous - start) / 1e9)
                start = timestamp
            previous = timestamp
        bursts.append((previous - start) / 1e9)
    return {
        "minimum_ratio": min(ratios) if ratios else None,
        "p05_ratio": _p(ratios, 0.05),
        "maximum_ratio": max(ratios) if ratios else None,
        "invalid_sample_count": invalid_samples,
        "rejection_count": max(rejection_counts, default=0),
        "accepted_residual_ratio": _p(accepted_ratios, 0.50),
        "rejection_burst_count": len(bursts),
        "maximum_rejection_burst_s": max(bursts, default=0.0),
        "tracking_coverage": (1.0 - invalid_samples / len(ratios)) if ratios else None,
    }


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
    timestamped_a = [(item, _sample_time(item)) for item in a]
    timestamped_b = [(item, _sample_time(item)) for item in b]
    if any(left[1] > right[1] for left, right in zip(timestamped_a, timestamped_a[1:])):
        return []
    if any(left[1] > right[1] for left, right in zip(timestamped_b, timestamped_b[1:])):
        return []
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
        if first_lio_position is None or first_px4_position is None:
            continue
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


def _planner_reference_residuals(
    records: list[dict[str, Any]],
    samples: list[dict[str, Any]],
    tolerance_ms: float,
) -> dict[str, Any]:
    """Compare committed candidate/execution states with PX4 in ENU.

    This is diagnostic-only.  It deliberately does not classify or reject a
    mission: the planner state is in the LIO planning frame while PX4 reports
    NED.  Execution state is matched at ``execution_stamp_ns``; a candidate is
    matched at its own ``candidate_start_wall_time_s``.  Each comparison has
    its own origin and PX4 reset segment, so a permanent initial offset or a
    reset cannot be mistaken for planner drift.
    """
    px4 = _series(samples, "px4_odometry")
    timestamped_px4: list[tuple[int, dict[str, Any]]] = []
    invalid_pose_frame_count = 0
    for item in px4:
        timestamp = _sample_time(item)
        payload = item.get("payload", {})
        position = _vector(payload.get("position"))
        try:
            pose_frame = int(payload.get("pose_frame"))
        except (TypeError, ValueError):
            pose_frame = -1
        if pose_frame != 1:
            invalid_pose_frame_count += 1
            continue
        if timestamp <= 0 or position is None:
            continue
        timestamped_px4.append((timestamp, item))
    timestamped_px4.sort(key=lambda pair: pair[0])
    px4_times = [pair[0] for pair in timestamped_px4]

    eligible = [
        record for record in records
        if isinstance(record, dict)
        and record.get("commit_observed_this_cycle") is True
        and isinstance(record.get("execution_stamp_ns"), int)
        and _vector(record.get("candidate_start_position")) is not None
        and _vector(record.get("planning_state_position")) is not None
    ]
    tolerance_ns = int(max(0.0, tolerance_ms) * 1e6)

    def match_at(target_ns: int) -> tuple[tuple[float, float, float], float, Any] | None:
        if target_ns <= 0 or not px4_times:
            return None
        index = bisect_left(px4_times, target_ns)
        candidates = timestamped_px4[max(0, index - 1):min(len(timestamped_px4), index + 1)]
        if not candidates:
            return None
        px4_time, px4_item = min(candidates, key=lambda pair: abs(pair[0] - target_ns))
        delta_ns = target_ns - px4_time
        if abs(delta_ns) > tolerance_ns:
            return None
        px4_position_ned = _vector(px4_item.get("payload", {}).get("position"))
        if px4_position_ned is None:
            return None
        return (
            _matrix_vector(_C_NED_FROM_ENU, px4_position_ned),
            delta_ns / 1e6,
            px4_item.get("payload", {}).get("reset_counter"),
        )

    execution_matches: list[tuple[dict[str, Any], tuple[float, float, float], float, Any]] = []
    candidate_matches: list[tuple[dict[str, Any], tuple[float, float, float], float, Any]] = []
    for record in eligible:
        execution_match = match_at(int(record["execution_stamp_ns"]))
        if execution_match is not None:
            execution_matches.append((record, *execution_match))
        candidate_wall_time = record.get("candidate_start_wall_time_s")
        try:
            candidate_time_ns = int(float(candidate_wall_time) * 1e9)
        except (TypeError, ValueError, OverflowError):
            candidate_time_ns = 0
        if candidate_time_ns > 0:
            candidate_match = match_at(candidate_time_ns)
            if candidate_match is not None:
                candidate_matches.append((record, *candidate_match))

    if not execution_matches and not candidate_matches:
        return {
            "available": False,
            "source": "planner commit trace vs px4/estimator_odometry",
            "timestamp_alignment": "execution_stamp_ns and candidate_start_wall_time_s vs PX4 timestamp_sample",
            "maximum_synchronization_tolerance_ms": tolerance_ms,
            "eligible_commit_count": len(eligible),
            "execution_matched_commit_count": 0,
            "candidate_matched_commit_count": 0,
            "invalid_pose_frame_count": invalid_pose_frame_count,
        }

    def relative_errors(
        matches: list[tuple[dict[str, Any], tuple[float, float, float], float, Any]],
        state_key: str,
    ) -> tuple[list[tuple[float, float, float]], list[float], int]:
        errors: list[tuple[float, float, float]] = []
        deltas: list[float] = []
        reset_segments = 0
        origin_state: tuple[float, float, float] | None = None
        origin_px4: tuple[float, float, float] | None = None
        origin_reset: Any = object()
        for record, px4_position, timestamp_delta, reset_counter in matches:
            state = _vector(record.get(state_key))
            if state is None:
                continue
            if origin_state is None or reset_counter != origin_reset:
                if origin_state is not None:
                    reset_segments += 1
                origin_state = state
                origin_px4 = px4_position
                origin_reset = reset_counter
            if origin_px4 is None:
                continue
            state_delta = tuple(state[index] - origin_state[index] for index in range(3))
            px4_delta = tuple(px4_position[index] - origin_px4[index] for index in range(3))
            errors.append(tuple(state_delta[index] - px4_delta[index] for index in range(3)))
            deltas.append(abs(timestamp_delta))
        return errors, deltas, reset_segments

    execution_errors, execution_timestamp_deltas, execution_reset_segments = relative_errors(
        execution_matches, "planning_state_position"
    )
    candidate_errors, candidate_timestamp_deltas, candidate_reset_segments = relative_errors(
        candidate_matches, "candidate_start_position"
    )
    candidate_execution_errors: list[tuple[float, float, float]] = []
    candidate_execution_time_deltas: list[float] = []
    for record in eligible:
        candidate = _vector(record.get("candidate_start_position"))
        execution = _vector(record.get("planning_state_position"))
        if candidate is None or execution is None:
            continue
        candidate_execution_errors.append(tuple(candidate[index] - execution[index] for index in range(3)))
        try:
            candidate_execution_time_deltas.append(
                abs(float(record.get("candidate_start_wall_time_s")) * 1e3 -
                    float(record["execution_stamp_ns"]) / 1e6)
            )
        except (TypeError, ValueError, OverflowError):
            continue
    first_bundle_id = (
        execution_matches[0][0].get("bundle_id") if execution_matches
        else candidate_matches[0][0].get("bundle_id")
    )
    last_match = execution_matches[-1] if execution_matches else candidate_matches[-1]
    return {
        "available": True,
        "source": "planner commit trace vs px4/estimator_odometry",
        "timestamp_alignment": "execution_stamp_ns for execution; candidate_start_wall_time_s for candidate; absolute PX4 timestamp_sample",
        "origin_alignment": "separate first-state origins per comparison and PX4 reset segment",
        "world_transform": "x_enu=y_ned; y_enu=x_ned; z_enu=-z_ned",
        "maximum_synchronization_tolerance_ms": tolerance_ms,
        "eligible_commit_count": len(eligible),
        "execution_matched_commit_count": len(execution_matches),
        "candidate_matched_commit_count": len(candidate_matches),
        "execution_unmatched_commit_count": len(eligible) - len(execution_matches),
        "candidate_unmatched_commit_count": len(eligible) - len(candidate_matches),
        "execution_mean_timestamp_delta_ms": sum(execution_timestamp_deltas) / len(execution_timestamp_deltas) if execution_timestamp_deltas else None,
        "execution_maximum_timestamp_delta_ms": max(execution_timestamp_deltas) if execution_timestamp_deltas else None,
        "candidate_mean_timestamp_delta_ms": sum(candidate_timestamp_deltas) / len(candidate_timestamp_deltas) if candidate_timestamp_deltas else None,
        "candidate_maximum_timestamp_delta_ms": max(candidate_timestamp_deltas) if candidate_timestamp_deltas else None,
        "candidate_execution_time_delta_ms": _metric_summary(candidate_execution_time_deltas),
        "execution_reset_segment_count": execution_reset_segments,
        "candidate_reset_segment_count": candidate_reset_segments,
        "invalid_pose_frame_count": invalid_pose_frame_count,
        "first_bundle_id": first_bundle_id,
        "last_bundle_id": last_match[0].get("bundle_id"),
        "candidate_vs_px4": _vector_metric_summary(candidate_errors),
        "execution_vs_px4": _vector_metric_summary(execution_errors),
        "candidate_vs_execution": _vector_metric_summary(candidate_execution_errors),
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


def _relative_position_error(
    matches: list[tuple[dict[str, Any], dict[str, Any], int]],
    *,
    horizon_s: float | None = None,
    distance_m: float | None = None,
) -> dict[str, Any]:
    """Compute RPE without changing the absolute-origin ATE contract."""
    samples: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]] = []
    for left, right, _ in matches:
        gt = _vector(left.get("payload", {}).get("position"))
        estimate = _vector(right.get("payload", {}).get("position"))
        if gt is not None and estimate is not None:
            samples.append((_sample_time(left), gt, estimate))
    errors: list[tuple[float, float, float]] = []
    timestamps = [sample[0] for sample in samples]
    # The old implementation scanned every future sample for every start
    # sample.  That is quadratic and made a long SITL report take minutes (or
    # appear hung) even though the RPE lookup is fundamentally a monotonic
    # time/path-index query.  Use binary search for fixed horizons and a
    # cumulative travelled-distance index for the 5 m horizon.  The short
    # local scan preserves the original first-Euclidean-crossing definition
    # for normal flight; if a vehicle dithers for a very long time, the
    # path-length target is a bounded, conservative fallback.
    cumulative_distance = [0.0]
    for index in range(1, len(samples)):
        previous = samples[index - 1][1]
        current = samples[index][1]
        cumulative_distance.append(
            cumulative_distance[-1] + math.sqrt(sum(
                (current[axis] - previous[axis]) ** 2 for axis in range(3)
            ))
        )
    for index, (timestamp, gt_start, estimate_start) in enumerate(samples):
        target: int | None = None
        if horizon_s is not None and math.isfinite(horizon_s) and horizon_s > 0.0:
            target = bisect_left(timestamps, timestamp + int(horizon_s * 1e9), index + 1)
            if target >= len(samples):
                target = None
        elif distance_m is not None and math.isfinite(distance_m) and distance_m > 0.0:
            path_target = bisect_left(
                cumulative_distance, cumulative_distance[index] + distance_m, index + 1
            )
            if path_target < len(samples):
                # Recover the exact first Euclidean crossing when it is close
                # to the path-length crossing (the usual smooth-flight case).
                scan_end = min(path_target, index + 512)
                for future_index in range(index + 1, scan_end + 1):
                    gt_future = samples[future_index][1]
                    delta = tuple(gt_future[k] - gt_start[k] for k in range(3))
                    if math.sqrt(sum(value * value for value in delta)) >= distance_m:
                        target = future_index
                        break
                if target is None:
                    target = path_target
        if target is None or target <= index:
            continue
        _, gt_end, estimate_end = samples[target]
        estimate_delta = tuple(estimate_end[k] - estimate_start[k] for k in range(3))
        gt_delta = tuple(gt_end[k] - gt_start[k] for k in range(3))
        errors.append(tuple(estimate_delta[k] - gt_delta[k] for k in range(3)))
    return _vector_metric_summary(errors)


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
            if first_lio_position is None:
                continue
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
            if first_external_position is None or first_gt_position_external is None:
                continue
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
            if first_gt_position_px4 is None or first_px4_position is None:
                continue
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
            "ate": _vector_metric_summary(lio_position_errors),
            "velocity_m_s": _vector_metric_summary(lio_velocity_errors),
            "angular_velocity_rad_s": _vector_metric_summary(lio_angular_errors),
            "attitude_rad": _metric_summary(lio_attitude),
            "initial_attitude_error_rad": first_lio_attitude,
            "rpe_1s": _relative_position_error(lio_matches, horizon_s=1.0),
            "rpe_5m": _relative_position_error(lio_matches, distance_m=5.0),
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


def _timing_distribution(values: list[float]) -> dict[str, Any]:
    return {
        "sample_count": len(values),
        "mean": sum(values) / len(values) if values else None,
        "p50": _p(values, 0.50),
        "p95": _p(values, 0.95),
        "p99": _p(values, 0.99),
        "max": max(values) if values else None,
    }


def _diagnostic_timing_summary(
    samples: list[dict[str, Any]], status_name: str | tuple[str, ...], fields: tuple[str, ...],
    stream_names: tuple[str, ...] = ("diagnostics", "planning_diagnostics"),
) -> dict[str, dict[str, Any]]:
    status_names = {status_name} if isinstance(status_name, str) else set(status_name)
    values_by_field = {field: [] for field in fields}
    for stream_name in stream_names:
        for item in _series(samples, stream_name):
            statuses = item.get("payload", {}).get("statuses", [])
            if not isinstance(statuses, list):
                continue
            for status in statuses:
                if not isinstance(status, dict) or status.get("name") not in status_names:
                    continue
                status_values = status.get("values", {})
                if not isinstance(status_values, dict):
                    continue
                for field in fields:
                    if not planner_timing_is_current(status_values, field):
                        continue
                    # Snapshot export is emitted only on publication after the
                    # mapping cadence optimization. Do not turn deferred
                    # samples into zero-latency export observations. Legacy
                    # artifacts without the marker retain their old behavior.
                    if (
                        field == "world_snapshot_export_us"
                        and "world_snapshot_published" in status_values
                        and int(_number(status_values.get("world_snapshot_published"), 0.0)) != 1
                    ):
                        continue
                    value = _number(status_values.get(field), -1.0)
                    if value >= 0.0 and math.isfinite(value):
                        values_by_field[field].append(value)
    return {field: _timing_distribution(values) for field, values in values_by_field.items()}


def _planning_timing_summary(samples: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return _diagnostic_timing_summary(
        samples,
        ("navigation_planning/planner", "navigation_runtime/planner"),
        (
            "planning_path_search_us",
            "planning_corridor_us",
            "planning_trajectory_optimization_us",
            "planning_total_us",
            "exp_frontend_us",
            "exp_opt_us",
            "backup_frontend_us",
            "backup_opt_us",
            "mapping_input_lock_wait_us",
            "planning_scheduling_gap_us",
        ),
        stream_names=("planning_diagnostics", "mapping_diagnostics", "diagnostics"),
    )


def _planning_execution_summary(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Return the latest planner counters without treating them as timings."""
    latest_values = snapshot.get("latest", {})
    latest = (
        latest_values.get("planning_diagnostics") or
        latest_values.get("mapping_diagnostics") or
        latest_values.get("diagnostics") or
        {}
    )
    statuses = latest.get("statuses", []) if isinstance(latest, dict) else []
    keys = (
        "plan_count",
        "plan_skip_count",
        "success_count",
        "failure_count",
        "safety_fallback_count",
        "safety_route_selected_count",
        "safety_stop_selected_count",
        "nominal_plan_count",
        "nominal_selected_count",
        "dual_verification_failure_count",
        "verification_failure_count",
        "local_subgoal_selected_count",
        "local_subgoal_failure_count",
        "trajectory_revalidation_count",
        "trajectory_revalidation_failure_count",
        "trajectory_reuse_count",
        "full_replan_count",
    )
    optional_count_fields = ("raw_path_node_count", "simplified_path_node_count", "shortcut_count")
    numeric_fields = (
        "maximum_jerk_mps3", "integrated_squared_jerk", "c2_continuity_residual",
        "geometric_path_length_m", "trajectory_length_m", "duration_s",
        "trajectory_duration_s", "kinematic_lower_bound_s", "minimum_clearance_m",
        "maximum_velocity_mps", "maximum_acceleration_mps2",
        "maximum_deceleration_mps2", "adaptive_velocity_cap_mps",
        "known_free_horizon_m", "splice_position_residual_m",
        "splice_velocity_residual_mps", "splice_acceleration_residual_mps2",
    )
    for status in statuses:
        if not isinstance(status, dict) or status.get("name") not in {
            "navigation_planning/planner", "navigation_runtime/planner"
        }:
            continue
        values = status.get("values", {})
        if not isinstance(values, dict):
            continue
        if status.get("name") == "navigation_runtime/planner":
            return {
                "available": True,
                "status_name": status.get("name"),
                **{
                    key: int(_number(values[key], 0.0))
                    for key in (
                        "received_observation_count",
                        "accepted_observation_count",
                        "cycle_count",
                        "trajectory_publish_count",
                        "dropped_cloud_count",
                        "processing_exception_count",
                    )
                    if key in values
                },
            }
        result: dict[str, Any] = {key: int(_number(values.get(key), 0.0)) for key in keys}
        if any(field in values for field in optional_count_fields):
            result.update({key: int(_number(values.get(key), 0.0)) for key in optional_count_fields})
        if "replan_reason" in values:
            result["replan_reason"] = str(values.get("replan_reason"))
        if any(field in values for field in numeric_fields):
            result.update({key: _number(values.get(key), 0.0) for key in numeric_fields})
            length = result.get("geometric_path_length_m", 0.0)
            trajectory_length = result.get("trajectory_length_m", 0.0)
            result["path_efficiency"] = (
                length / trajectory_length if trajectory_length > 1e-9 else None
            )
        return result
    return {"available": False}


def _navigation_mapping_summary(
    snapshot: dict[str, Any], samples: list[dict[str, Any]] | None = None
) -> dict[str, Any]:
    """Summarize the product-owned mapping diagnostics surface."""
    stream = snapshot.get("streams", {}).get("mapping_diagnostics", {})
    latest = snapshot.get("latest", {}).get("mapping_diagnostics", {})
    statuses = latest.get("statuses", []) if isinstance(latest, dict) else []
    latest_by_owner: dict[str, dict[str, Any]] = {}
    # A lifecycle snapshot is coherent only when all of its counters come
    # from the same diagnostic event.  Planner decision traces reuse the
    # planner status name without carrying lifecycle fields, so keep lifecycle
    # ordering separate from the latest telemetry status.
    latest_lifecycle_by_owner: dict[str, tuple[int, dict[str, Any]]] = {}
    lifecycle_event: tuple[int, dict[str, Any]] | None = None
    lifecycle_fields = {
        "received_observation_count",
        "observation_rejected_before_inbox_count",
        "accepted_observation_count",
        "observation_accepted_to_inbox_count",
        "dropped_cloud_count",
        "observation_replaced_pending_count",
        "observation_replaced_waiting_count",
        "observation_replaced_ready_count",
        "observation_discarded_pending_count",
        "observation_discarded_waiting_count",
        "observation_discarded_ready_count",
        "observation_discarded_shutdown_ready_count",
        "observation_discarded_nonmonotonic_count",
        "observation_ready_submitted_count",
        "observation_waiting_count",
        "observation_ready_count",
        "mapping_started_count",
        "mapping_published_count",
        "mapping_failed_count",
        "mapping_pending_count",
        "mapping_in_flight_count",
        "observation_accounting_valid",
        "observation_accounting_violation_count",
    }
    event_order = 0
    # Keep the original monitor order across both diagnostic streams.  Do not
    # group by stream first: that would make an older world event appear newer
    # merely because ``mapping_diagnostics`` is iterated before ``diagnostics``.
    for item in samples or []:
        if item.get("stream") not in {"mapping_diagnostics", "diagnostics"}:
            continue
        if not item.get("accepted_by_monitor", True):
            continue
        event_order += 1
        try:
            event_key = int(item.get("arrival_wall_ns", 0))
        except (TypeError, ValueError):
            event_key = 0
        # Preserve input order when legacy samples have no wall clock.
        event_key = event_key if event_key > 0 else event_order
        for status in item.get("payload", {}).get("statuses", []):
            if not isinstance(status, dict):
                continue
            status_name = str(status.get("name", ""))
            owner: str | None = None
            if status_name.endswith("/world_model"):
                owner = "world_model"
            elif status_name == "navigation_runtime/planner":
                owner = "planner"
            if owner is None:
                continue
            latest_by_owner[owner] = status
            candidate = status.get("values", {})
            if isinstance(candidate, dict) and any(
                key in candidate for key in lifecycle_fields
            ):
                latest_lifecycle_by_owner[owner] = (event_key, status)
                if lifecycle_event is None or event_key >= lifecycle_event[0]:
                    lifecycle_event = (event_key, status)
    # The monitor's final snapshot contains only the last diagnostic message,
    # so retain it as a fallback for workflows without a sample stream.
    for status in statuses:
        if not isinstance(status, dict):
            continue
        status_name = str(status.get("name", ""))
        if status_name.endswith("/world_model"):
            latest_by_owner.setdefault("world_model", status)
            candidate = status.get("values", {})
            if isinstance(candidate, dict) and any(
                key in candidate for key in lifecycle_fields
            ):
                latest_lifecycle_by_owner.setdefault("world_model", (0, status))
        elif status_name == "navigation_runtime/planner":
            latest_by_owner.setdefault("planner", status)
            candidate = status.get("values", {})
            if isinstance(candidate, dict) and any(
                key in candidate for key in lifecycle_fields
            ):
                latest_lifecycle_by_owner.setdefault("planner", (0, status))

    if lifecycle_event is None and latest_lifecycle_by_owner:
        lifecycle_event = max(latest_lifecycle_by_owner.values(), key=lambda item: item[0])
    values: dict[str, Any] = {}
    level = "NOT_AVAILABLE"
    message = "NOT_AVAILABLE"
    # Start with the newest owner-specific diagnostics for world identity and
    # telemetry, then overlay the *single newest lifecycle event*.  The latter
    # is essential: independently merging planner ingress counters with an
    # older world-model terminal event manufactures a conservation failure.
    for owner in ("planner", "world_model"):
        status = latest_by_owner.get(owner)
        if not status:
            continue
        candidate = status.get("values", {})
        if isinstance(candidate, dict):
            values.update(candidate)
        level = status.get("level", "NOT_AVAILABLE")
        message = status.get("message", "NOT_AVAILABLE")
    if lifecycle_event is not None:
        candidate = lifecycle_event[1].get("values", {})
        if isinstance(candidate, dict):
            values.update({key: candidate[key] for key in lifecycle_fields if key in candidate})
    integer_fields = (
        "received_observation_count",
        "accepted_observation_count",
        "generation",
        "revision",
        "generation_reset_count",
        "world_generation",
        "world_revision",
        "observation_stamp_ns",
        "last_update_attempt_stamp_ns",
        "world_snapshot_bytes",
        "world_snapshot_owned_bytes",
        "world_snapshot_shared_metadata_bytes",
        "world_snapshot_live_count",
        "world_snapshot_peak_live_count",
        "world_snapshot_live_owned_bytes",
        "world_snapshot_peak_live_owned_bytes",
        "world_snapshot_published",
        "world_snapshot_published_count",
        "world_snapshot_deferred_count",
        "world_snapshot_full_export_count",
        "world_snapshot_patch_export_count",
        "world_snapshot_export_mode",
        "world_snapshot_full_export_reason",
        "world_snapshot_export_base_cells",
        "world_snapshot_export_inflated_cells",
        "world_snapshot_patch_depth",
        "old_generation_drop_count",
        "invalid_stamp_count",
        "invalid_frame_count",
        "invalid_pose_count",
        "invalid_cloud_count",
        "dropped_cloud_count",
        "stale_input_count",
        "stale_mapping_input_count",
        "future_mapping_input_count",
        "stale_execution_state_count",
        "future_execution_state_count",
        "invalid_corrected_pose_count",
        "corrected_pair_mismatch_count",
        "invalid_execution_state_count",
        "observation_rejected_before_inbox_count",
        "observation_accepted_to_inbox_count",
        "observation_replaced_pending_count",
        "observation_replaced_waiting_count",
        "observation_replaced_ready_count",
        "observation_discarded_waiting_count",
        "observation_discarded_pending_count",
        "observation_discarded_ready_count",
        "observation_discarded_shutdown_ready_count",
        "observation_discarded_nonmonotonic_count",
        "observation_ready_submitted_count",
        "observation_waiting_count",
        "observation_ready_count",
        "worker_discarded_stale_count",
        "worker_discarded_future_count",
        "worker_discarded_invalid_count",
        "mapping_started_count",
        "mapping_published_count",
        "mapping_failed_count",
        "mapping_pending_count",
        "mapping_in_flight_count",
        "mapping_outcome_updated_count",
        "mapping_outcome_accumulated_count",
        "mapping_outcome_slide_only_count",
        "mapping_outcome_empty_cloud_count",
        "mapping_outcome_callback_owned_count",
        "mapping_outcome_below_ground_count",
        "mapping_outcome_above_ceiling_count",
        "command_revalidation_fast_path_count",
        "command_revalidation_full_count",
        "observation_accounting_valid",
        "observation_accounting_violation_count",
        "nonfinite_point_count",
        "post_filter_nonfinite_point_count",
        "transform_nonfinite_point_count",
        "processing_exception_count",
        "visualization_publish_count",
        "visualization_subscriber_count",
        "visualization_exception_count",
        "visualization_occupied_point_count",
        "visualization_inflated_occupied_point_count",
        "visualization_unknown_point_count",
        "visualization_frontier_point_count",
        "last_received_observation_sequence",
        "last_received_observation_stream_id",
        "observation_sequence_stream_switch_count",
        "observation_sequence_missing_count",
        "observation_sequence_duplicate_count",
        "observation_sequence_regression_count",
        "observation_sequence_max_consecutive_missing",
    )
    result: dict[str, Any] = {
        "topic": "/navigation/diagnostics",
        "sample_count": stream.get("received", 0),
        "mean_rate_hz": stream.get("mean_rate_hz", 0.0),
        "status_level": level,
        "status_message": message,
    }
    for field in integer_fields:
        if field in values and values[field] is not None:
            result[field] = int(values[field] or 0)
    if "mapping_update_outcome" in values:
        result["mapping_update_outcome"] = str(values["mapping_update_outcome"])
    result["timing_distributions"] = _diagnostic_timing_summary(
        samples or [],
        ("navigation_mapping/world_model", "navigation_runtime/planner"),
        (
            "ros_pointcloud_decode_us",
            "observation_pair_wait_us",
            "mapping_filter_us",
            "transform_to_odom_us",
            "mapping_raycast_us",
            "mapping_probability_update_us",
            "mapping_inflation_us",
            "mapping_slide_us",
            "mapping_total_update_us",
            "world_snapshot_export_us",
            "mapping_callback_total_us",
        ),
        stream_names=("mapping_diagnostics", "diagnostics"),
    )
    result["output_topics"] = ["/navigation/navigation_command", "/navigation/diagnostics"]
    return result


def _mapping_integrity_reasons(mapping: dict[str, Any]) -> list[str]:
    labels = {
        "dropped_cloud_count": "mapping replaced an unconsumed cloud",
        "stale_mapping_input_count": "mapping rejected stale input",
        "future_mapping_input_count": "mapping rejected future-dated input",
        "corrected_pair_mismatch_count": "mapping rejected a corrected-pose pair",
        "invalid_corrected_pose_count": "mapping rejected invalid corrected pose",
        "stale_execution_state_count": "planner rejected stale execution state",
        "future_execution_state_count": "planner rejected future-dated execution state",
        "invalid_execution_state_count": "planner execution state was invalid",
        "processing_exception_count": "mapping processing raised an exception",
    }
    reasons: list[str] = []
    for field, label in labels.items():
        count = int(_number(mapping.get(field), 0.0))
        if count > 0:
            reasons.append(f"{label}: {count}")
    if "observation_accounting_valid" in mapping and int(
        _number(mapping.get("observation_accounting_valid"), 0.0)
    ) != 1:
        reasons.append("mapping observation accounting invariant failed")
    violation_count = int(
        _number(mapping.get("observation_accounting_violation_count"), 0.0)
    )
    if violation_count > 0:
        reasons.append(f"mapping observation accounting violation: {violation_count}")
    for field, label in (
        ("mapping_pending_count", "mapping observation remained pending"),
        ("mapping_in_flight_count", "mapping observation remained in flight"),
        ("mapping_failed_count", "mapping observation failed after pairing"),
    ):
        count = int(_number(mapping.get(field), 0.0))
        if count > 0:
            reasons.append(f"{label}: {count}")
    received = int(_number(mapping.get("received_observation_count"), 0.0))
    accepted = int(_number(mapping.get("accepted_observation_count"), 0.0))
    rejected_before_inbox = mapping.get("observation_rejected_before_inbox_count")
    if received > 0:
        if rejected_before_inbox is None:
            reasons.append(
                "mapping observation input lifecycle evidence incomplete; "
                "input conservation not evaluated"
            )
        elif received != int(_number(rejected_before_inbox, 0.0)) + accepted:
            reasons.append(
                "mapping observation input conservation equation failed"
            )
    if "mapping_published_count" in mapping:
        terminal_fields = (
            "accepted_observation_count",
            "observation_replaced_pending_count",
            "observation_discarded_pending_count",
            "mapping_published_count",
            "mapping_failed_count",
            "mapping_pending_count",
            "mapping_in_flight_count",
        )
        missing_terminal_fields = [
            field for field in terminal_fields if field not in mapping
        ]
        if missing_terminal_fields:
            reasons.append(
                "mapping observation lifecycle evidence incomplete; "
                "terminal conservation not evaluated"
            )
        else:
            published = int(_number(mapping.get("mapping_published_count"), 0.0))
            terminal_pending = int(
                _number(mapping.get("observation_discarded_pending_count"), 0.0)
            )
            replaced = int(_number(mapping.get("observation_replaced_pending_count"), 0.0))
            failed = int(_number(mapping.get("mapping_failed_count"), 0.0))
            pending = int(_number(mapping.get("mapping_pending_count"), 0.0))
            in_flight = int(_number(mapping.get("mapping_in_flight_count"), 0.0))
            if accepted != published + terminal_pending + replaced + failed + pending + in_flight:
                reasons.append(
                    "mapping accepted-observation conservation equation failed"
                )
    outcome_fields = (
        "mapping_outcome_updated_count",
        "mapping_outcome_accumulated_count",
        "mapping_outcome_slide_only_count",
        "mapping_outcome_empty_cloud_count",
        "mapping_outcome_callback_owned_count",
        "mapping_outcome_below_ground_count",
        "mapping_outcome_above_ceiling_count",
    )
    present_outcome_fields = [field for field in outcome_fields if field in mapping]
    if not present_outcome_fields and "mapping_published_count" in mapping:
        reasons.append(
            "mapping update-outcome evidence is unavailable; runtime binary may be stale"
        )
    elif present_outcome_fields:
        missing = [field for field in outcome_fields if field not in mapping]
        if missing:
            reasons.append(
                "mapping update-outcome evidence is incomplete: " + ", ".join(missing)
            )
        else:
            outcomes = {
                field: int(_number(mapping.get(field), 0.0)) for field in outcome_fields
            }
            published = int(_number(mapping.get("mapping_published_count"), 0.0))
            if sum(outcomes.values()) != published:
                reasons.append("mapping update-outcome conservation equation failed")
            advanced = (
                outcomes["mapping_outcome_updated_count"] +
                outcomes["mapping_outcome_slide_only_count"]
            )
            revision = int(_number(mapping.get("world_revision"), 0.0))
            published_snapshots = mapping.get("world_snapshot_published_count")
            if published_snapshots is None:
                if revision != advanced:
                    reasons.append(
                        "mapping world revision does not equal advancing update outcomes"
                    )
            else:
                published_snapshots = int(_number(published_snapshots, 0.0))
                if published_snapshots > advanced:
                    reasons.append(
                        "mapping published snapshot count exceeds advancing update outcomes"
                    )
                if revision > advanced:
                    reasons.append(
                        "mapping published world revision exceeds advancing update outcomes"
                    )
                if published_snapshots > 0 and revision <= 0:
                    reasons.append(
                        "mapping published snapshot count has no published world revision"
                    )
            if published > 0:
                observation_stamp = int(_number(
                    mapping.get("observation_stamp_ns"), 0.0
                ))
                if observation_stamp <= 0:
                    reasons.append(
                        "mapping published-world observation stamp is unavailable"
                    )
                attempt_stamp = int(_number(
                    mapping.get("last_update_attempt_stamp_ns"), 0.0
                ))
                if attempt_stamp > 0 and observation_stamp > attempt_stamp:
                    reasons.append(
                        "mapping published-world observation stamp is newer than the last update attempt"
                    )
            for field, label in (
                ("mapping_outcome_accumulated_count", "mapping observations remained accumulated without publication"),
                ("mapping_outcome_slide_only_count", "mapping observations only slid the local window without applying the cloud"),
                ("mapping_outcome_empty_cloud_count", "mapping received empty clouds"),
                ("mapping_outcome_callback_owned_count", "mapping rejected external update ownership"),
                ("mapping_outcome_below_ground_count", "mapping rejected odometry below virtual ground"),
                ("mapping_outcome_above_ceiling_count", "mapping rejected odometry above virtual ceiling"),
            ):
                if outcomes[field] > 0:
                    reasons.append(f"{label}: {outcomes[field]}")
    if "observation_replaced_pending_count" in mapping:
        replaced_waiting = int(_number(
            mapping.get(
                "observation_replaced_waiting_count",
                mapping.get("observation_replaced_pending_count", 0),
            ),
            0.0,
        ))
        replaced_ready = int(
            _number(mapping.get("observation_replaced_ready_count"), 0.0)
        )
        legacy_dropped = int(_number(mapping.get("dropped_cloud_count"), 0.0))
        if replaced_waiting + replaced_ready != legacy_dropped:
            reasons.append(
                "mapping replacement compatibility counter mismatch: "
                f"lifecycle {replaced_waiting + replaced_ready}, legacy {legacy_dropped}"
            )
    return reasons


def _dataset_source_count_reasons(
    streams: dict[str, dict[str, Any]], runtime: dict[str, Any]
) -> list[str]:
    reasons: list[str] = []
    expected_stream_counts = runtime.get("dataset_context", {}).get(
        "expected_stream_counts"
    )
    if not isinstance(expected_stream_counts, dict):
        reasons.append("dataset expected source counts are missing")
        expected_stream_counts = {}
    for name in ("imu", "lidar"):
        expected = expected_stream_counts.get(name)
        observed = streams[name]["sample_count"]
        valid_expected = isinstance(expected, int) and not isinstance(expected, bool) and expected > 0
        streams[name]["expected_source_count"] = expected if valid_expected else None
        streams[name]["observed_source_count"] = observed
        streams[name]["source_count_complete"] = bool(valid_expected and observed == expected)
        if not valid_expected:
            reasons.append(f"{name} expected source count is invalid or missing")
        elif observed != expected:
            reasons.append(
                f"{name} source count mismatch: observed {observed}, expected {expected}"
            )
    return reasons


def _dataset_playback_summary(runtime: dict[str, Any], requested_rate: float) -> dict[str, Any]:
    started = runtime.get("replay_started_wall_ns")
    finished = runtime.get("replay_finished_wall_ns")
    source_duration = runtime.get("dataset_context", {}).get("source_duration_ns")
    valid = all(
        isinstance(value, int) and not isinstance(value, bool) and value > 0
        for value in (started, finished, source_duration)
    ) and finished > started
    wall_duration_s = (finished - started) / 1e9 if valid else None
    achieved_rate = (source_duration / 1e9) / wall_duration_s if valid else None
    return {
        "requested_rate": requested_rate,
        "source_duration_s": source_duration / 1e9 if valid else None,
        "wall_duration_s": wall_duration_s,
        "achieved_rate": achieved_rate,
        "requested_rate_fraction": achieved_rate / requested_rate
        if valid and requested_rate > 0 else None,
        "available": valid,
    }


def _dataset_shadow_planning_reasons(
    runtime: dict[str, Any], evidence: dict[str, Any], planning: dict[str, Any]
) -> list[str]:
    contract = runtime.get("dataset_shadow_planning", {})
    if not isinstance(contract, dict) or not bool(contract.get("enabled", False)):
        return []
    reasons: list[str] = []
    if not isinstance(evidence, dict):
        return ["dataset shadow planning evidence is missing or malformed"]
    if evidence.get("status") != "PASS":
        failure = evidence.get("failure")
        reason = "dataset shadow planning did not complete"
        if isinstance(failure, str) and failure.strip():
            reason += f": {failure.strip()}"
        reasons.append(reason)
    if evidence.get("goal_published") is not True:
        reasons.append("dataset shadow planning goal was not published")
    try:
        ready_count = int(evidence.get("ready_command_count", 0))
        generations = [int(value) for value in evidence.get("unique_ready_generations", [])]
        emergency_count = int(evidence.get("emergency_command_count", 0))
    except (TypeError, ValueError):
        return ["dataset shadow planning evidence is malformed"]
    if ready_count <= 0 or not generations or min(generations) <= 0:
        reasons.append("dataset shadow planning produced no committed READY command")
    if emergency_count != 0:
        reasons.append("dataset shadow planning emitted an emergency command")
    planning_total = planning.get("planning_total_us", {})
    planning_timing_count = (
        int(_number(planning_total.get("sample_count"), 0.0))
        if isinstance(planning_total, dict)
        else 0
    )
    trace = planning.get("rolling_bundle_trace", {})
    if planning_timing_count <= 0 and isinstance(trace, dict):
        records = trace.get("records", [])
        if isinstance(records, list):
            planning_timing_count = sum(
                1
                for record in records
                if isinstance(record, dict)
                and isinstance(record.get("planning_latency_ms"), (int, float))
                and not isinstance(record.get("planning_latency_ms"), bool)
                and math.isfinite(float(record["planning_latency_ms"]))
                and float(record["planning_latency_ms"]) >= 0.0
            )
    if planning_timing_count <= 0:
        reasons.append("dataset shadow planning has no planner timing samples")
    if not isinstance(trace, dict) or int(trace.get("record_count", 0)) <= 0:
        reasons.append("dataset shadow planning has no runtime planner trace")
    return reasons


def _dataset_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path) -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    streams = {name: _rate_row(snapshot, name) for name in ("imu", "lidar", "corrected_odometry", "propagated_odometry")}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    _annotate_stale_classification(streams, config, runtime, samples)
    diagnostic_states = _diagnostic_states(samples)
    observability = _observability_summary(samples)
    map_point_count = _map_point_summary(samples)
    map_maintenance = _map_maintenance_summary(samples)
    navigation_mapping = _navigation_mapping_summary(snapshot, samples)
    planning = _planning_timing_summary(samples)
    planning["execution"] = _planning_execution_summary(snapshot)
    planning["rolling_bundle_trace"] = _planner_trace_report(session, samples)
    shadow_planning = _load_json(session / "dataset_shadow_planning.json", {})
    planning["shadow_goal"] = shadow_planning
    reasons: list[str] = []
    reasons.extend(_provenance_reasons(runtime))
    reasons.extend(_dataset_source_count_reasons(streams, runtime))
    minimum_fraction = _number(thresholds.get("minimum_rate_fraction"), 0.90)
    for name, row in streams.items():
        if row["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        expected = _number(config.get("runtime", {}).get("streams", {}).get(name, {}).get("expected_hz"))
        if expected and row["mean_rate_hz"] < expected * minimum_fraction:
            reasons.append(f"{name} rate below contract")
        if (row["timestamp_regression_count"] or
                row["invalid_source_timestamp_count"] or
                row["source_stale_event_count"] or
                row["nonfinite_message_count"]):
            reasons.append(f"{name} timestamp/freshness/validity violation")
    diagnostics = _diag_values(snapshot)
    tracking_observed = "TRACKING" in diagnostic_states or str(diagnostics.get("state", "")).upper() == "TRACKING"
    if not tracking_observed:
        reasons.append("LIO did not finish in TRACKING")
    if int(diagnostics.get("current_queue_depth", diagnostics.get("current_input_queue_depth", 0)) or 0) != 0:
        reasons.append("input queue did not drain")
    if int(diagnostics.get("imu_drop_count", 0) or 0) or int(diagnostics.get("lidar_drop_count", 0) or 0):
        reasons.append("runtime drop count is non-zero")
    reasons.extend(_mapping_integrity_reasons(navigation_mapping))
    reasons.extend(_dataset_shadow_planning_reasons(runtime, shadow_planning, planning))
    reasons.extend(failures)
    reasons = _dedupe_reasons(reasons)
    verdict = "PASS" if not reasons else "FAIL"
    if not any(streams[name]["sample_count"] for name in streams):
        verdict = "BLOCKED"
    requested_rate = config.get("runtime", {}).get("replay_rate", 1.0)
    return {
        "workflow": "dataset",
        "verdict": verdict,
        "reasons": reasons,
        "dataset": config.get("runtime", {}).get("dataset", ""),
        "rate": requested_rate,
        "playback": _dataset_playback_summary(runtime, float(requested_rate)),
        "streams": streams,
        "lio": {
            "state": diagnostics.get("state", "NOT_AVAILABLE"),
            "tracking_observed": tracking_observed,
            "navigation_valid": diagnostics.get("navigation_valid", False),
            "translation_observability_valid": diagnostics.get("translation_observability_valid", False),
            "translation_observability_ratio": diagnostics.get("translation_observability_ratio", "NOT_AVAILABLE"),
            "translation_observability_min_eigenvalue": diagnostics.get("translation_observability_min_eigenvalue", "NOT_AVAILABLE"),
            "translation_observability_max_eigenvalue": diagnostics.get("translation_observability_max_eigenvalue", "NOT_AVAILABLE"),
            "observability_rejection_count": diagnostics.get("observability_rejection_count", "NOT_AVAILABLE"),
            "observability_summary": observability,
            "tracking_coverage": observability.get("tracking_coverage"),
            "observability_p05": observability.get("p05_ratio"),
            "observability_min": observability.get("minimum_ratio"),
            "rotational_information_spectrum": {
                "status": "diagnostic_only",
                "product_health_gate": "translation_observability",
            },
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
        "navigation_mapping": navigation_mapping,
        "planning": planning,
        "experimental_bypasses": {},
        "accuracy": "NOT_AVAILABLE",
        "provenance": _session_provenance(session, workspace),
    }


def _sim_report(session: Path, config: dict[str, Any], snapshot: dict[str, Any], workspace: Path, px4_dir: Path | None, workflow: str = "sim") -> dict[str, Any]:
    thresholds = config.get("runtime", {}).get("thresholds", {})
    names = ("simulation_clock", "imu", "lidar", "corrected_odometry", "propagated_odometry", "ground_truth_odometry", "external_odometry", "px4_odometry", "vehicle_status", "local_position", "estimator_status_flags")
    streams = {name: _rate_row(snapshot, name) for name in names}
    runtime = _load_json(session / "runtime.json", {})
    failures = _process_failures(session)
    samples = _samples(session / "samples.jsonl")
    _annotate_stale_classification(streams, config, runtime, samples)
    diagnostic_states = _diagnostic_states(samples)
    observability = _observability_summary(samples)
    map_point_count = _map_point_summary(samples)
    map_maintenance = _map_maintenance_summary(samples)
    navigation_mapping = _navigation_mapping_summary(snapshot, samples)
    planning = _planning_timing_summary(samples)
    planning["execution"] = _planning_execution_summary(snapshot)
    planning["rolling_bundle_trace"] = _planner_trace_report(session, samples)
    planner_reference_residuals = _planner_reference_residuals(
        planning["rolling_bundle_trace"].get("records", []),
        samples,
        _number(thresholds.get("maximum_synchronization_tolerance_ms"), 20.0),
    )
    scenario = _load_json(session / "scenario.json", {})
    terminal_outcome = str(scenario.get("outcome", ""))
    terminal_handover = terminal_outcome in {
        "COMPLETE", "ABORTED_OPERATOR", "PAUSED_SAFETY_STOP", "FAILED_COMPONENT"
    }
    expected_fail_closed = str(scenario.get("expected_outcome", "complete")) == "fail_closed"
    structured_safety_stop = (
        terminal_outcome == "PAUSED_SAFETY_STOP" and
        (bool(scenario.get("mode_failure_observed", False)) or
         bool(scenario.get("safety_stop_observed", False)))
    )
    fail_closed_handover = expected_fail_closed and structured_safety_stop
    reasons: list[str] = []
    reasons.extend(_provenance_reasons(runtime))
    for name in ("simulation_clock", "imu", "lidar", "corrected_odometry", "propagated_odometry", "ground_truth_odometry", "external_odometry", "px4_odometry", "local_position", "estimator_status_flags"):
        if streams[name]["sample_count"] <= 0:
            reasons.append(f"{name} has no samples")
        # /clock is itself the source-time authority, so a long wall-arrival
        # gap cannot be downgraded merely because the queued clock values catch
        # up with small source deltas afterward.  A monitor-wide stall is also
        # insufficient evidence for flight acceptance and therefore remains
        # fail-closed here.
        stale_violation = _sim_stream_stale_violation(name, streams[name])
        # A fail-closed External Mode hands control to the supervisor before
        # landing. The PX4 external-odometry bridge may stop publishing during
        # that handover; it is outside the active navigation interval and must
        # not turn an otherwise valid safety rejection into a stream verdict.
        if name in {"external_odometry", "propagated_odometry"} and (
            fail_closed_handover or terminal_handover
        ):
            stale_violation = 0
        if (streams[name]["timestamp_regression_count"] or
                streams[name]["invalid_source_timestamp_count"] or
                stale_violation or streams[name]["nonfinite_message_count"]):
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
    reasons.extend(_mapping_integrity_reasons(navigation_mapping))
    scenario_reasons = list(scenario.get("failures", []))
    reasons.extend(str(item) for item in scenario_reasons)
    reasons.extend(failures)
    acceptance = _mission_acceptance(session, config, scenario, workspace)
    reasons.extend(str(item) for item in acceptance["reasons"])
    reasons = _dedupe_reasons(reasons)
    verdict = "PASS" if not reasons else "FAIL"
    if workflow == "external-mode":
        outcome = str(scenario.get("outcome", ""))
        # These are deliberate non-success terminal states, not report
        # defects.  Stream/process observations may still contain warnings
        # from the handover interval, so classify by the structured outcome
        # before applying the generic reason list.
        if outcome == "PAUSED_SAFETY_STOP":
            if not expected_fail_closed:
                verdict = "BLOCKED"
            elif not structured_safety_stop:
                reasons.append("structured safety-stop evidence is missing")
                verdict = "FAIL"
            else:
                handover_reason = "mission did not reach COMPLETE outcome: PAUSED_SAFETY_STOP"
                reasons = [reason for reason in reasons if reason != handover_reason]
                verdict = "PASS" if not reasons else "FAIL"
        elif outcome == "ABORTED_OPERATOR":
            verdict = "BLOCKED"
        elif outcome == "FAILED_COMPONENT":
            verdict = "FAIL"
            reasons.append("External Mode component failure")
    return {
        "workflow": workflow,
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
            "navigation_valid": diagnostics.get("navigation_valid", False),
            "translation_observability_valid": diagnostics.get("translation_observability_valid", False),
            "translation_observability_ratio": diagnostics.get("translation_observability_ratio", "NOT_AVAILABLE"),
            "translation_observability_min_eigenvalue": diagnostics.get("translation_observability_min_eigenvalue", "NOT_AVAILABLE"),
            "translation_observability_max_eigenvalue": diagnostics.get("translation_observability_max_eigenvalue", "NOT_AVAILABLE"),
            "observability_rejection_count": diagnostics.get("observability_rejection_count", "NOT_AVAILABLE"),
            "observability_summary": observability,
            "tracking_coverage": observability.get("tracking_coverage"),
            "observability_p05": observability.get("p05_ratio"),
            "observability_min": observability.get("minimum_ratio"),
            "rotational_information_spectrum": {
                "status": "diagnostic_only",
                "product_health_gate": "translation_observability",
            },
            "map_point_count": map_point_count,
            "map_maintenance": map_maintenance,
        },
        "navigation_mapping": navigation_mapping,
        "planning": planning,
        "residuals": residuals,
        "conversion_contract": conversion_contract,
        "ground_truth_residuals": ground_truth_residuals,
        "acceptance": acceptance,
        "experimental_bypasses": {},
        "gazebo_native_diagnostics": _gazebo_native_diagnostics(session, runtime),
        "tracking": {
            "reference_vs_lio": "NOT_AVAILABLE",
            "reference_vs_ground_truth": "NOT_AVAILABLE",
            "lio_vs_ground_truth": ground_truth_residuals.get("lio_vs_ground_truth", {}),
            "planner_reference_vs_px4": planner_reference_residuals,
            "coverage": observability.get("tracking_coverage"),
        },
        "offboard": scenario,
        "external_mode": scenario if workflow == "external-mode" else {},
        "provenance": _session_provenance(session, workspace, px4_dir),
    }


def _build_complete_report(session: Path, workflow: str, config_path: Path, workspace: Path, px4_dir: Path | None = None, observation_complete: bool = False) -> dict[str, Any]:
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
        report = _sim_report(session, config, snapshot, workspace, px4_dir, workflow)
    descriptor = _load_json(session / "map_descriptor.json", {})
    if descriptor:
        report["map"] = descriptor
    if observation_complete:
        # Observation completion is a lifecycle fact, not an acceptance
        # verdict. Preserve the evaluated stream/mission reasons so an
        # interactive stop cannot erase a genuine FAIL.
        report["observation_complete"] = True
        report["observation_status"] = (
            "FAIL" if _process_failures(session) else "OBSERVATION_COMPLETE"
        )
    if report["verdict"] not in VERDICTS:
        raise ValueError(f"invalid runtime verdict: {report['verdict']}")
    report["session"] = str(session.resolve())
    report["schema_version"] = 1
    report["qualification_timelines"] = _write_qualification_timelines(session)
    (session / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    # Keep the machine-readable report as the contract, and add a standalone
    # visualization artifact for SITL mission analysis. Every completed
    # session gets REPORT.html, including sessions with incomplete telemetry;
    # a minimal fallback keeps the artifact contract intact if a diagnostic
    # chart encounters malformed data.
    try:
        from flight_review_report import render as render_html

        html_path = render_html(session.resolve(), session.resolve() / "REPORT.html")
        if not Path(html_path).is_file():
            raise RuntimeError("HTML renderer did not create REPORT.html")
    except Exception as error:  # pragma: no cover - defensive artifact path
        (session / "REPORT_HTML_ERROR.txt").write_text(str(error) + "\n", encoding="utf-8")
        fallback = (
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<title>UAV navigation report</title></head><body>"
            f"<h1>UAV navigation report</h1><p>Verdict: {html.escape(str(report.get('verdict')))}</p>"
            f"<p>HTML diagnostics failed: <code>{html.escape(str(error))}</code></p>"
            f"<p>Session: <code>{html.escape(str(session.resolve()))}</code></p>"
            "</body></html>"
        )
        (session / "REPORT.html").write_text(fallback, encoding="utf-8")
    return report


def build(session: Path, workflow: str, config_path: Path, workspace: Path, px4_dir: Path | None = None, observation_complete: bool = False) -> dict[str, Any]:
    """Build a complete report, or a durable FAIL report if analysis crashes."""
    try:
        return _build_complete_report(
            session,
            workflow,
            config_path,
            workspace,
            px4_dir,
            observation_complete,
        )
    except Exception as error:
        session = session.resolve()
        session.mkdir(parents=True, exist_ok=True)
        error_text = f"{type(error).__name__}: {error}"
        timelines = _write_qualification_timelines(session)
        fallback = {
            "workflow": workflow,
            "verdict": "FAIL",
            "reasons": [f"report generation failed: {error_text}"],
            "streams": {},
            "session": str(session),
            "schema_version": 1,
            "qualification_timelines": timelines,
        }
        (session / "REPORT_BUILD_ERROR.txt").write_text(error_text + "\n", encoding="utf-8")
        (session / "report.json").write_text(
            json.dumps(fallback, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (session / "REPORT.html").write_text(
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<title>UAV navigation report</title></head><body>"
            "<h1>UAV navigation report</h1><p>Verdict: FAIL</p>"
            f"<p>Report generation failed: <code>{html.escape(error_text)}</code></p>"
            f"<p>Session: <code>{html.escape(str(session))}</code></p>"
            "</body></html>",
            encoding="utf-8",
        )
        return fallback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--workflow", choices=("dataset", "sim", "external-mode"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--px4-dir", type=Path)
    parser.add_argument("--observation-complete", action="store_true")
    args = parser.parse_args()
    report = build(args.session.resolve(), args.workflow, args.config.resolve(), args.workspace.resolve(), args.px4_dir, args.observation_complete)
    print(report["verdict"])
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Optional planner/bundle trace adapter for runtime reports.

This module accepts only explicit runtime fields. It never derives planning
cycle IDs from request or trajectory IDs, and therefore reports missing
telemetry as partial rather than inventing a trace.
"""

from __future__ import annotations

import math
import json
from typing import Any, Iterable


_ALIASES: dict[str, tuple[str, ...]] = {
    "planning_cycle_id": ("planning_cycle_id",),
    "bundle_id": ("bundle_id", "trajectory_bundle_id"),
    "route_id": ("route_id", "trajectory_route_id"),
    "horizon_start_arc_m": ("horizon_start_arc_m",),
    "horizon_end_arc_m": ("horizon_end_arc_m",),
    "horizon_arc_m": ("horizon_arc_m", "horizon_arc_length_m", "horizon_end_arc_m"),
    "horizon_endpoint": ("horizon_endpoint",),
    "planning_state_position": ("planning_state_position",),
    "planning_state_velocity": ("planning_state_velocity",),
    "planning_horizon_distance_m": ("planning_horizon_distance_m",),
    "horizon_forward_projection_m": ("horizon_forward_projection_m",),
    "horizon_progress_m": ("horizon_progress_m",),
    "known_free_horizon_m": ("known_free_horizon_m",),
    "horizon_ray_occupied": ("horizon_ray_occupied",),
    "selected_branch": ("selected_branch", "selected_candidate"),
    "safety_activation": ("safety_activation", "safety_activated"),
    "solver_latency_us": (
        "solver_latency_us",
        "planning_trajectory_optimization_us",
        "optimizer_latency_us",
    ),
    "planning_latency_ms": ("planning_latency_ms",),
    "exp_frontend_us": ("exp_frontend_us",),
    "exp_opt_us": ("exp_opt_us",),
    "backup_frontend_us": ("backup_frontend_us",),
    "backup_opt_us": ("backup_opt_us",),
    "optimizer_latency_ms": ("optimizer_latency_ms",),
    "exp_diagnostics_valid": ("exp_diagnostics_valid",),
    "exp_lbfgs_attempt_count": ("exp_lbfgs_attempt_count",),
    "exp_retry_count": ("exp_retry_count",),
    "exp_retry_violation_mask": ("exp_retry_violation_mask",),
    "exp_retry_stop_reason": ("exp_retry_stop_reason",),
    "exp_lbfgs_first_return_code": ("exp_lbfgs_first_return_code",),
    "exp_lbfgs_last_return_code": ("exp_lbfgs_last_return_code",),
    "exp_lbfgs_cancelled": ("exp_lbfgs_cancelled",),
    "exp_initial_normalized_dynamic_violation": (
        "exp_initial_normalized_dynamic_violation",
    ),
    "exp_best_normalized_dynamic_violation": (
        "exp_best_normalized_dynamic_violation",
    ),
    "exp_final_normalized_dynamic_violation": (
        "exp_final_normalized_dynamic_violation",
    ),
    "exp_initial_duration_s": ("exp_initial_duration_s",),
    "exp_final_duration_s": ("exp_final_duration_s",),
    "guide_path_length_m": ("guide_path_length_m",),
    "guide_duration_s": ("guide_duration_s",),
    "exp_retry_budget_remaining_us": ("exp_retry_budget_remaining_us",),
    "exp_nonfinite_evaluation_count": ("exp_nonfinite_evaluation_count",),
    "exp_first_nonfinite_stage": ("exp_first_nonfinite_stage",),
    "exp_first_nonfinite_attempt": ("exp_first_nonfinite_attempt",),
    "exp_first_nonfinite_iteration": ("exp_first_nonfinite_iteration",),
    "exp_first_nonfinite_min_duration_s": ("exp_first_nonfinite_min_duration_s",),
    "exp_first_nonfinite_max_duration_s": ("exp_first_nonfinite_max_duration_s",),
    "exp_first_nonfinite_cost": ("exp_first_nonfinite_cost",),
    "exp_first_nonfinite_gradient_norm": ("exp_first_nonfinite_gradient_norm",),
    "status": ("status",),
    "failure_code": ("failure_code",),
    "maximum_velocity_mps": ("maximum_velocity_mps",),
    "maximum_acceleration_mps2": ("maximum_acceleration_mps2",),
    "maximum_jerk_mps3": ("maximum_jerk_mps3",),
    "splice_position_residual_m": ("splice_position_residual_m",),
    "splice_velocity_residual_mps": ("splice_velocity_residual_mps",),
    "splice_acceleration_residual_mps2": ("splice_acceleration_residual_mps2",),
    "splice_jerk_residual_mps3": ("splice_jerk_residual_mps3",),
    "splice_yaw_residual_rad": ("splice_yaw_residual_rad",),
    "splice_yaw_rate_residual_radps": ("splice_yaw_rate_residual_radps",),
    "route_candidate_count": ("route_candidate_count",),
    "corridor_region_count": ("corridor_region_count",),
    "world_generation": ("world_generation",),
    "world_revision": ("world_revision",),
    "pinned_world_generation": ("pinned_world_generation",),
    "pinned_world_revision": ("pinned_world_revision",),
    "pinned_world_stamp_ns": ("pinned_world_stamp_ns",),
    "solve_generation": ("solve_generation",),
    "commit_observed_this_cycle": ("commit_observed_this_cycle",),
    "execution_stamp_ns": ("execution_stamp_ns",),
    "state_age_at_solve_ms": ("state_age_at_solve_ms",),
    "state_age_at_trace_ms": ("state_age_at_trace_ms",),
    "candidate_start_position": ("candidate_start_position",),
    "candidate_start_velocity": ("candidate_start_velocity",),
    "candidate_start_acceleration": ("candidate_start_acceleration",),
    "candidate_start_jerk": ("candidate_start_jerk",),
    "candidate_start_wall_time_s": ("candidate_start_wall_time_s",),
    "commit_previous_generation": ("commit_previous_generation",),
    "splice_previous_valid": ("splice_previous_valid",),
    "splice_previous_sample_tt_s": ("splice_previous_sample_tt_s",),
    "candidate_result": ("candidate_result",),
    "replan_code": ("replan_code",),
    "solve_stage": ("solve_stage",),
    "solve_stage_name": ("solve_stage_name",),
    "solve_deadline_exceeded": ("solve_deadline_exceeded",),
    "command_available": ("command_available",),
    "planner_failure_latched": ("planner_failure_latched",),
}


def _first(raw: dict[str, Any], names: Iterable[str]) -> Any:
    for name in names:
        if name in raw:
            return raw[name]
    return None


def _int(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value >= 0 else None


def _signed_int(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _float(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "on", "active", "activated", "1"}:
            return True
        if normalized in {"false", "no", "off", "inactive", "deactivated", "0"}:
            return False
    return None


def _branch(value: Any) -> str | int | None:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return None
    value = str(value).strip()
    return value or None


def _point(value: Any) -> list[float] | None:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (json.JSONDecodeError, TypeError):
            return None
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("z")]
    elif all(hasattr(value, name) for name in ("x", "y", "z")):
        value = [value.x, value.y, value.z]
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    result = [_float(item) for item in value[:3]]
    return result if all(item is not None for item in result) else None


def normalize_planner_trace_record(
    raw: dict[str, Any], *, source: str, timestamp_s: float | None = None
) -> dict[str, Any] | None:
    """Normalize one explicit trace record without filling absent fields."""
    if not isinstance(raw, dict):
        return None
    values = {name: _first(raw, aliases) for name, aliases in _ALIASES.items()}
    if not any(value is not None for value in values.values()):
        return None
    record: dict[str, Any] = {
        "planning_cycle_id": _int(values["planning_cycle_id"]),
        "bundle_id": _int(values["bundle_id"]),
        "route_id": _int(values["route_id"]),
        "horizon_start_arc_m": _float(values["horizon_start_arc_m"]),
        "horizon_end_arc_m": _float(values["horizon_end_arc_m"]),
        "horizon_arc_m": _float(values["horizon_arc_m"]),
        "horizon_endpoint": _point(values["horizon_endpoint"]),
        "planning_state_position": _point(values["planning_state_position"]),
        "planning_state_velocity": _point(values["planning_state_velocity"]),
        "planning_horizon_distance_m": _float(values["planning_horizon_distance_m"]),
        "horizon_forward_projection_m": _float(values["horizon_forward_projection_m"]),
        "horizon_progress_m": _float(values["horizon_progress_m"]),
        "known_free_horizon_m": _float(values["known_free_horizon_m"]),
        "horizon_ray_occupied": _bool(values["horizon_ray_occupied"]),
        "selected_branch": _branch(values["selected_branch"]),
        "safety_activation": _bool(values["safety_activation"]),
        "solver_latency_us": _float(values["solver_latency_us"]),
        "planning_latency_ms": _float(values["planning_latency_ms"]),
        "exp_frontend_us": _float(values["exp_frontend_us"]),
        "exp_opt_us": _float(values["exp_opt_us"]),
        "backup_frontend_us": _float(values["backup_frontend_us"]),
        "backup_opt_us": _float(values["backup_opt_us"]),
        "optimizer_latency_ms": _float(values["optimizer_latency_ms"]),
        "exp_diagnostics_valid": _bool(values["exp_diagnostics_valid"]),
        "exp_lbfgs_attempt_count": _int(values["exp_lbfgs_attempt_count"]),
        "exp_retry_count": _int(values["exp_retry_count"]),
        "exp_retry_violation_mask": _int(values["exp_retry_violation_mask"]),
        "exp_retry_stop_reason": _int(values["exp_retry_stop_reason"]),
        "exp_lbfgs_first_return_code": _signed_int(
            values["exp_lbfgs_first_return_code"]
        ),
        "exp_lbfgs_last_return_code": _signed_int(
            values["exp_lbfgs_last_return_code"]
        ),
        "exp_lbfgs_cancelled": _bool(values["exp_lbfgs_cancelled"]),
        "exp_initial_normalized_dynamic_violation": _float(
            values["exp_initial_normalized_dynamic_violation"]
        ),
        "exp_best_normalized_dynamic_violation": _float(
            values["exp_best_normalized_dynamic_violation"]
        ),
        "exp_final_normalized_dynamic_violation": _float(
            values["exp_final_normalized_dynamic_violation"]
        ),
        "exp_initial_duration_s": _float(values["exp_initial_duration_s"]),
        "exp_final_duration_s": _float(values["exp_final_duration_s"]),
        "guide_path_length_m": _float(values["guide_path_length_m"]),
        "guide_duration_s": _float(values["guide_duration_s"]),
        "exp_retry_budget_remaining_us": _float(
            values["exp_retry_budget_remaining_us"]
        ),
        "exp_nonfinite_evaluation_count": _int(
            values["exp_nonfinite_evaluation_count"]
        ),
        "exp_first_nonfinite_stage": _int(values["exp_first_nonfinite_stage"]),
        "exp_first_nonfinite_attempt": _int(values["exp_first_nonfinite_attempt"]),
        "exp_first_nonfinite_iteration": _int(
            values["exp_first_nonfinite_iteration"]
        ),
        "exp_first_nonfinite_min_duration_s": _float(
            values["exp_first_nonfinite_min_duration_s"]
        ),
        "exp_first_nonfinite_max_duration_s": _float(
            values["exp_first_nonfinite_max_duration_s"]
        ),
        "exp_first_nonfinite_cost": _float(values["exp_first_nonfinite_cost"]),
        "exp_first_nonfinite_gradient_norm": _float(
            values["exp_first_nonfinite_gradient_norm"]
        ),
        "status": _int(values["status"]),
        "failure_code": str(values["failure_code"]) if values["failure_code"] is not None else None,
        "maximum_velocity_mps": _float(values["maximum_velocity_mps"]),
        "maximum_acceleration_mps2": _float(values["maximum_acceleration_mps2"]),
        "maximum_jerk_mps3": _float(values["maximum_jerk_mps3"]),
        "splice_position_residual_m": _float(values["splice_position_residual_m"]),
        "splice_velocity_residual_mps": _float(values["splice_velocity_residual_mps"]),
        "splice_acceleration_residual_mps2": _float(values["splice_acceleration_residual_mps2"]),
        "splice_jerk_residual_mps3": _float(values["splice_jerk_residual_mps3"]),
        "splice_yaw_residual_rad": _float(values["splice_yaw_residual_rad"]),
        "splice_yaw_rate_residual_radps": _float(values["splice_yaw_rate_residual_radps"]),
        "route_candidate_count": _int(values["route_candidate_count"]),
        "corridor_region_count": _int(values["corridor_region_count"]),
        "world_generation": _int(values["world_generation"]),
        "world_revision": _int(values["world_revision"]),
        "pinned_world_generation": _int(values["pinned_world_generation"]),
        "pinned_world_revision": _int(values["pinned_world_revision"]),
        "pinned_world_stamp_ns": _int(values["pinned_world_stamp_ns"]),
        "commit_observed_this_cycle": _bool(values["commit_observed_this_cycle"]),
        "execution_stamp_ns": _int(values["execution_stamp_ns"]),
        "state_age_at_solve_ms": _float(values["state_age_at_solve_ms"]),
        "state_age_at_trace_ms": _float(values["state_age_at_trace_ms"]),
        "candidate_start_position": _point(values["candidate_start_position"]),
        "candidate_start_velocity": _point(values["candidate_start_velocity"]),
        "candidate_start_acceleration": _point(values["candidate_start_acceleration"]),
        "candidate_start_jerk": _point(values["candidate_start_jerk"]),
        "candidate_start_wall_time_s": _float(values["candidate_start_wall_time_s"]),
        "commit_previous_generation": _int(values["commit_previous_generation"]),
        "splice_previous_valid": _bool(values["splice_previous_valid"]),
        "splice_previous_sample_tt_s": _float(values["splice_previous_sample_tt_s"]),
        "solve_generation": _int(values["solve_generation"]),
        "candidate_result": str(values["candidate_result"])
        if values["candidate_result"] is not None else None,
        "replan_code": str(values["replan_code"])
        if values["replan_code"] is not None else None,
        "solve_stage": _int(values["solve_stage"]),
        "solve_stage_name": str(values["solve_stage_name"])
        if values["solve_stage_name"] is not None else None,
        "solve_deadline_exceeded": _bool(values["solve_deadline_exceeded"]),
        "command_available": _bool(values["command_available"]),
        "planner_failure_latched": _bool(values["planner_failure_latched"]),
        "source": source,
    }
    if timestamp_s is not None:
        record["timestamp_s"] = _float(timestamp_s)
    record["record_key"] = [record["planning_cycle_id"], record["bundle_id"]]
    record["missing_key_fields"] = [
        name for name in ("planning_cycle_id", "bundle_id") if record[name] is None
    ]
    record["complete"] = not record["missing_key_fields"]
    return record


def _payload_records(samples: Iterable[dict[str, Any]]) -> Iterable[tuple[dict[str, Any], str, float | None]]:
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        timestamp_s = sample.get("t")
        if timestamp_s is None and sample.get("timestamp_ns") is not None:
            timestamp_s = _float(sample["timestamp_ns"])
            timestamp_s = timestamp_s / 1e9 if timestamp_s is not None else None
        payload = sample.get("payload")
        statuses = payload.get("statuses", []) if isinstance(payload, dict) else []
        found = False
        if isinstance(statuses, list):
            for status in statuses:
                if not isinstance(status, dict) or status.get("name") not in {
                    "navigation_planning/planner", "navigation_runtime/planner"
                }:
                    continue
                values = status.get("values")
                if isinstance(values, dict):
                    found = True
                    yield values, "planning_diagnostics", _float(timestamp_s)
        if not found:
            values = sample.get("values")
            if isinstance(values, dict):
                yield values, "planning_diagnostics", _float(timestamp_s)
            elif any(alias in sample for aliases in _ALIASES.values() for alias in aliases):
                yield sample, "planning_diagnostics", _float(timestamp_s)


def collect_planner_trace_records(
    scenario: dict[str, Any] | None = None,
    samples: Iterable[dict[str, Any]] = (),
) -> list[dict[str, Any]]:
    """Collect records keyed by the explicit ``(cycle_id, bundle_id)`` pair."""
    scenario = scenario if isinstance(scenario, dict) else {}
    records: dict[tuple[int | None, int | None], dict[str, Any]] = {}
    explicit = scenario.get("planner_trace_records", [])
    if isinstance(explicit, list):
        for index, raw in enumerate(explicit):
            record = normalize_planner_trace_record(
                raw, source=f"scenario.planner_trace_records[{index}]"
            )
            if record is not None and tuple(record["record_key"]) != (None, None):
                records[tuple(record["record_key"])] = record
    for raw, source, timestamp_s in _payload_records(samples):
        record = normalize_planner_trace_record(raw, source=source, timestamp_s=timestamp_s)
        if record is None or tuple(record["record_key"]) == (None, None):
            continue
        key = tuple(record["record_key"])
        previous = records.get(key)
        if previous is None:
            records[key] = record
        elif previous.get("source", "").startswith("scenario."):
            for name in _ALIASES:
                if previous.get(name) is None and record.get(name) is not None:
                    previous[name] = record[name]
            if previous.get("timestamp_s") is None and record.get("timestamp_s") is not None:
                previous["timestamp_s"] = record["timestamp_s"]
    return sorted(records.values(), key=lambda item: (
        item.get("timestamp_s") is None,
        item.get("timestamp_s") if item.get("timestamp_s") is not None else 0.0,
        item["record_key"],
    ))


def planner_trace_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "record_count": len(records),
        "complete_record_count": sum(bool(record.get("complete")) for record in records),
        "partial_record_count": sum(not bool(record.get("complete")) for record in records),
        "fields_are_runtime_supplied": True,
        "missing_data_policy": "omitted, never inferred from request/trajectory IDs or aggregate counters",
    }

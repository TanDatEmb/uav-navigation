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
    "planning_total_us": ("planning_total_us",),
    "route_yaw_source": ("route_yaw_source",),
    "route_yaw_target_rad": ("route_yaw_target_rad",),
    "route_yaw_lookahead_m": ("route_yaw_lookahead_m",),
    "route_yaw_progress_arc_m": ("route_yaw_progress_arc_m",),
    "yaw_rate_limit_rad_s": ("yaw_rate_limit_rad_s",),
    "yaw_acceleration_limit_rad_s2": ("yaw_acceleration_limit_rad_s2",),
    "candidate_maximum_yaw_rate_rad_s": ("candidate_maximum_yaw_rate_rad_s",),
    "candidate_maximum_yaw_acceleration_rad_s2": (
        "candidate_maximum_yaw_acceleration_rad_s2",
    ),
    "exp_frontend_us": ("exp_frontend_us",),
    "exp_opt_us": ("exp_opt_us",),
    "backup_frontend_us": ("backup_frontend_us",),
    "backup_opt_us": ("backup_opt_us",),
    "backup_certificate_attempted": ("backup_certificate_attempted",),
    "backup_switch_candidate_count": ("backup_switch_candidate_count",),
    "backup_feasible_seed_count": ("backup_feasible_seed_count",),
    "backup_visibility_hull_pass_count": ("backup_visibility_hull_pass_count",),
    "backup_aligned_sfc_built_count": ("backup_aligned_sfc_built_count",),
    "backup_aligned_hull_pass_count": ("backup_aligned_hull_pass_count",),
    "backup_known_free_check_count": ("backup_known_free_check_count",),
    "backup_known_free_pass_count": ("backup_known_free_pass_count",),
    "backup_certificate_selected": ("backup_certificate_selected",),
    "backup_last_reject_stage": ("backup_last_reject_stage",),
    "backup_last_known_free_failure_code": ("backup_last_known_free_failure_code",),
    "backup_last_known_free_cell_state": ("backup_last_known_free_cell_state",),
    "backup_last_known_free_blocked_role": ("backup_last_known_free_blocked_role",),
    "backup_last_known_free_first_blocked_time_s": (
        "backup_last_known_free_first_blocked_time_s",
    ),
    "backup_last_known_free_blocked_position": (
        "backup_last_known_free_blocked_position",
    ),
    "backup_last_seed_switch_time_s": ("backup_last_seed_switch_time_s",),
    "backup_last_seed_duration_s": ("backup_last_seed_duration_s",),
    "backup_last_seed_initial_velocity_mps": (
        "backup_last_seed_initial_velocity_mps",
    ),
    "backup_last_seed_max_velocity_mps": ("backup_last_seed_max_velocity_mps",),
    "backup_last_seed_max_acceleration_mps2": (
        "backup_last_seed_max_acceleration_mps2",
    ),
    "backup_last_seed_max_jerk_mps3": ("backup_last_seed_max_jerk_mps3",),
    "backup_last_seed_endpoint": ("backup_last_seed_endpoint",),
    "optimizer_latency_ms": ("optimizer_latency_ms",),
    "exp_diagnostics_valid": ("exp_diagnostics_valid",),
    "exp_used_certified_seed": ("exp_used_certified_seed",),
    "exp_certified_seed_failure_stage": (
        "exp_certified_seed_failure_stage",
    ),
    "exp_corridor_seed_build_failure_stage": (
        "exp_corridor_seed_build_failure_stage",
    ),
    "exp_corridor_seed_retry_attempt_count": (
        "exp_corridor_seed_retry_attempt_count",
    ),
    "exp_corridor_seed_retry_build_valid_count": (
        "exp_corridor_seed_retry_build_valid_count",
    ),
    "exp_corridor_seed_retry_last_certificate_stage": (
        "exp_corridor_seed_retry_last_certificate_stage",
    ),
    "exp_corridor_seed_selected_mode": ("exp_corridor_seed_selected_mode",),
    "exp_corridor_seed_selected_max_duration_scale": (
        "exp_corridor_seed_selected_max_duration_scale",
    ),
    "exp_lbfgs_attempt_count": ("exp_lbfgs_attempt_count",),
    "exp_lbfgs_evaluation_count": ("exp_lbfgs_evaluation_count",),
    "exp_lbfgs_first_attempt_evaluation_count": (
        "exp_lbfgs_first_attempt_evaluation_count",
    ),
    "exp_lbfgs_last_attempt_evaluation_count": (
        "exp_lbfgs_last_attempt_evaluation_count",
    ),
    "exp_retry_count": ("exp_retry_count",),
    "exp_retry_violation_mask": ("exp_retry_violation_mask",),
    "exp_retry_stop_reason": ("exp_retry_stop_reason",),
    "exp_lbfgs_first_return_code": ("exp_lbfgs_first_return_code",),
    "exp_lbfgs_last_return_code": ("exp_lbfgs_last_return_code",),
    "exp_lbfgs_cancelled": ("exp_lbfgs_cancelled",),
    "exp_hard_deadline_observed": ("exp_hard_deadline_observed",),
    "exp_initial_normalized_dynamic_violation": (
        "exp_initial_normalized_dynamic_violation",
    ),
    "exp_best_normalized_dynamic_violation": (
        "exp_best_normalized_dynamic_violation",
    ),
    "exp_final_normalized_dynamic_violation": (
        "exp_final_normalized_dynamic_violation",
    ),
    "exp_certified_seed_maximum_velocity_mps": (
        "exp_certified_seed_maximum_velocity_mps",
    ),
    "exp_certified_seed_maximum_acceleration_mps2": (
        "exp_certified_seed_maximum_acceleration_mps2",
    ),
    "exp_certified_seed_maximum_jerk_mps3": (
        "exp_certified_seed_maximum_jerk_mps3",
    ),
    "exp_initial_duration_s": ("exp_initial_duration_s",),
    "exp_initial_minimum_piece_duration_s": (
        "exp_initial_minimum_piece_duration_s",
    ),
    "exp_initial_maximum_piece_duration_s": (
        "exp_initial_maximum_piece_duration_s",
    ),
    "exp_final_duration_s": ("exp_final_duration_s",),
    "exp_retry_duration_lower_bound_min_s": (
        "exp_retry_duration_lower_bound_min_s",
    ),
    "exp_retry_duration_lower_bound_max_s": (
        "exp_retry_duration_lower_bound_max_s",
    ),
    "exp_retry_free_duration_seed_min_s": (
        "exp_retry_free_duration_seed_min_s",
    ),
    "exp_retry_free_duration_seed_max_s": (
        "exp_retry_free_duration_seed_max_s",
    ),
    "guide_path_length_m": ("guide_path_length_m",),
    "guide_duration_s": ("guide_duration_s",),
    "required_lookahead_m": ("required_lookahead_m",),
    "certified_lookahead_m": ("certified_lookahead_m",),
    "lookahead_complete": ("lookahead_complete",),
    "exp_retry_budget_remaining_us": ("exp_retry_budget_remaining_us",),
    "exp_refinement_budget_at_entry_us": (
        "exp_refinement_budget_at_entry_us",
    ),
    "exp_nonfinite_evaluation_count": ("exp_nonfinite_evaluation_count",),
    "exp_first_nonfinite_stage": ("exp_first_nonfinite_stage",),
    "exp_first_nonfinite_value_mask": ("exp_first_nonfinite_value_mask",),
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
    # These are emitted by the structured planner outcome contract.  Keep
    # them explicit; legacy solve_stage is not a substitute for either field.
    "planning_outcome": ("planning_outcome",),
    "planning_failure_stage": ("planning_failure_stage",),
    "planning_failure_reason": ("planning_failure_reason",),
    "planner_backend_outcome": ("planner_backend_outcome",),
    "planner_backend_failure_stage": ("planner_backend_failure_stage",),
    "planner_backend_failure_reason": ("planner_backend_failure_reason",),
    "planning_failure_stage_name": ("planning_failure_stage_name",),
    "planning_failure_reason_name": ("planning_failure_reason_name",),
    "planner_backend_failure_stage_name": ("planner_backend_failure_stage_name",),
    "planner_backend_failure_reason_name": ("planner_backend_failure_reason_name",),
    "commit_decision": ("commit_decision",),
    "solve_stage": ("solve_stage",),
    "solve_stage_name": ("solve_stage_name",),
    "solve_deadline_exceeded": ("solve_deadline_exceeded",),
    "command_available": ("command_available",),
    "planner_failure_latched": ("planner_failure_latched",),
    "backend_outcome_scope": ("backend_outcome_scope",),
    "worker_transaction_identity_scope": ("worker_transaction_identity_scope",),
    "runtime_admission_disposition": ("runtime_admission_disposition",),
    "execution_disposition": ("execution_disposition",),
    "first_causal_failure_scope": ("first_causal_failure_scope",),
    "first_causal_failure_stage": ("first_causal_failure_stage",),
    "first_causal_failure_reason": ("first_causal_failure_reason",),
    "first_causal_failure_class": ("first_causal_failure_class",),
    "first_causal_failure_event_steady_ns": (
        "first_causal_failure_event_steady_ns",
    ),
    "first_causal_failure_event_order": ("first_causal_failure_event_order",),
    "backend_failure_event_class": ("backend_failure_event_class",),
    "runtime_admission_failure_event_class": (
        "runtime_admission_failure_event_class",
    ),
    "watchdog_event_class": ("watchdog_event_class",),
    "planner_trace_schema_version": ("planner_trace_schema_version",),
    "candidate_generation": ("candidate_generation",),
    "candidate_activation_stamp_ns": ("candidate_activation_stamp_ns",),
    "planner_backend_failure_event_steady_ns": (
        "planner_backend_failure_event_steady_ns",
    ),
    "watchdog_event_steady_ns": ("watchdog_event_steady_ns",),
    "runtime_admission_failure_event_steady_ns": (
        "runtime_admission_failure_event_steady_ns",
    ),
}

_PLANNER_TIMELINE_FIELDS = (
    "planner_request_received_steady_ns",
    "planner_solve_started_steady_ns",
    "planner_solve_finished_steady_ns",
    "planner_hard_deadline_steady_ns",
    "planner_remaining_hard_budget_us_at_finish",
)
for _stage_index in range(1, 6):
    _PLANNER_TIMELINE_FIELDS += (
        f"planner_stage_{_stage_index}_name",
        f"planner_stage_{_stage_index}_observed",
        f"planner_stage_{_stage_index}_begin_steady_ns",
        f"planner_stage_{_stage_index}_end_steady_ns",
        f"planner_stage_{_stage_index}_remaining_hard_budget_us",
        f"planner_stage_{_stage_index}_result_code",
    )
for _field in _PLANNER_TIMELINE_FIELDS:
    _ALIASES[_field] = (_field,)

_RUNTIME_TRANSACTION_FIELDS = (
    "runtime_request_created_steady_ns",
    "runtime_request_enqueued_steady_ns",
    "worker_dequeued_steady_ns",
    "backend_received_steady_ns",
    "backend_finished_steady_ns",
    "runtime_result_received_steady_ns",
    "runtime_currentness_checked_steady_ns",
    "runtime_admission_started_steady_ns",
    "runtime_admission_finished_steady_ns",
    "runtime_admission_attempted",
    "runtime_admission_succeeded",
    "runtime_successor_staged",
    "latest_execution_activation_generation",
    "latest_execution_activation_started_steady_ns",
    "latest_execution_activation_finished_steady_ns",
    "latest_execution_activation_result",
    "latest_command_sampled_generation",
    "latest_command_sampled_steady_ns",
    "latest_command_sampled_result",
)
for _field in _RUNTIME_TRANSACTION_FIELDS:
    _ALIASES[_field] = (_field,)

_BACKUP_CERTIFICATE_REJECT_STAGE_NAMES = {
    0: "none",
    1: "command_boundary",
    2: "seed",
    3: "visibility_hull",
    4: "aligned_sfc",
    5: "aligned_hull",
    6: "known_free",
    7: "refinement_known_free",
    8: "yaw",
    9: "dynamic",
    10: "deadline",
}

_CAUSAL_EVENT_CLASSES = frozenset({
    "CAUSAL",
    "CONTAINMENT",
    "DOWNSTREAM_CONSEQUENCE",
    "INFORMATIONAL",
})


def reduce_first_causal_failure(
    events: Iterable[dict[str, Any]],
) -> dict[str, Any]:
    """Select the earliest causal event with deterministic tie-breaking.

    Events without a valid steady timestamp are not eligible to rewrite the
    causal result. This keeps a late result receipt from outranking an earlier
    watchdog or backend event merely because it was observed later.
    """
    candidates: list[tuple[int, int, str, str, str, str]] = []
    invalid_timestamp_count = 0
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            continue
        event_class = str(event.get("class", "")).upper()
        if event_class not in _CAUSAL_EVENT_CLASSES or event_class != "CAUSAL":
            continue
        timestamp_ns = _signed_int(event.get("steady_ns"))
        if timestamp_ns is None or timestamp_ns <= 0:
            invalid_timestamp_count += 1
            continue
        sequence = _int(event.get("sequence"))
        candidates.append(
            (
                timestamp_ns,
                sequence if sequence is not None else index,
                str(event.get("scope", "UNKNOWN")),
                str(event.get("stage", "unknown")),
                str(event.get("reason", "unknown")),
                str(event.get("source", "unknown")),
            )
        )
    if not candidates:
        return {
            "present": False,
            "scope": "NONE",
            "stage": "none",
            "reason": "none",
            "steady_ns": None,
            "sequence": None,
            "source": None,
            "invalid_timestamp_count": invalid_timestamp_count,
        }
    selected = min(candidates, key=lambda item: (item[0], item[1], item[2:]))
    return {
        "present": True,
        "scope": selected[2],
        "stage": selected[3],
        "reason": selected[4],
        "steady_ns": selected[0],
        "sequence": selected[1],
        "source": selected[5],
        "invalid_timestamp_count": invalid_timestamp_count,
    }


# These fields describe an optimizer execution, not a planner state snapshot.
# A later cycle may legitimately reuse the last committed diagnostic values
# while reporting ``exp_diagnostics_valid=false``.  Reports must not count
# those carried values as new work.
_EXECUTION_TIMING_FIELDS = frozenset({
    "exp_frontend_us",
    "exp_opt_us",
    "backup_frontend_us",
    "backup_opt_us",
    "exp_refinement_budget_at_entry_us",
})


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


def _backup_reject_stage_name(value: Any) -> str | None:
    """Map the producer enum without turning missing data into ``none``."""
    numeric = _int(value)
    if numeric is None:
        return None
    return _BACKUP_CERTIFICATE_REJECT_STAGE_NAMES.get(
        numeric, "unknown_enum_value"
    )


def planner_timing_is_current(values: dict[str, Any], field: str) -> bool:
    """Return whether a planner execution timing belongs to this cycle.

    Older producers did not expose ``exp_diagnostics_valid``; those records
    remain usable.  Once the producer exposes the field, an absent, malformed,
    or false validity flag is conservative for execution-specific timings.
    Non-execution fields are unaffected.
    """
    if field not in _EXECUTION_TIMING_FIELDS:
        return True
    if "exp_diagnostics_valid" not in values:
        return True
    return _bool(values.get("exp_diagnostics_valid")) is True


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
        "planning_total_us": _float(values["planning_total_us"]),
        "route_yaw_source": _int(values["route_yaw_source"]),
        "route_yaw_target_rad": _float(values["route_yaw_target_rad"]),
        "route_yaw_lookahead_m": _float(values["route_yaw_lookahead_m"]),
        "route_yaw_progress_arc_m": _float(values["route_yaw_progress_arc_m"]),
        "yaw_rate_limit_rad_s": _float(values["yaw_rate_limit_rad_s"]),
        "yaw_acceleration_limit_rad_s2": _float(
            values["yaw_acceleration_limit_rad_s2"]
        ),
        "candidate_maximum_yaw_rate_rad_s": _float(
            values["candidate_maximum_yaw_rate_rad_s"]
        ),
        "candidate_maximum_yaw_acceleration_rad_s2": _float(
            values["candidate_maximum_yaw_acceleration_rad_s2"]
        ),
        "exp_frontend_us": _float(values["exp_frontend_us"]),
        "exp_opt_us": _float(values["exp_opt_us"]),
        "backup_frontend_us": _float(values["backup_frontend_us"]),
        "backup_opt_us": _float(values["backup_opt_us"]),
        "backup_certificate_attempted": _bool(values["backup_certificate_attempted"]),
        "backup_switch_candidate_count": _int(values["backup_switch_candidate_count"]),
        "backup_feasible_seed_count": _int(values["backup_feasible_seed_count"]),
        "backup_visibility_hull_pass_count": _int(
            values["backup_visibility_hull_pass_count"]
        ),
        "backup_aligned_sfc_built_count": _int(
            values["backup_aligned_sfc_built_count"]
        ),
        "backup_aligned_hull_pass_count": _int(
            values["backup_aligned_hull_pass_count"]
        ),
        "backup_known_free_check_count": _int(
            values["backup_known_free_check_count"]
        ),
        "backup_known_free_pass_count": _int(
            values["backup_known_free_pass_count"]
        ),
        "backup_certificate_selected": _bool(values["backup_certificate_selected"]),
        "backup_last_reject_stage": _int(values["backup_last_reject_stage"]),
        "backup_last_reject_stage_name": _backup_reject_stage_name(
            values["backup_last_reject_stage"]
        ),
        "backup_last_known_free_failure_code": _int(
            values["backup_last_known_free_failure_code"]
        ),
        "backup_last_known_free_cell_state": _int(
            values["backup_last_known_free_cell_state"]
        ),
        "backup_last_known_free_blocked_role": _int(
            values["backup_last_known_free_blocked_role"]
        ),
        "backup_last_known_free_first_blocked_time_s": _float(
            values["backup_last_known_free_first_blocked_time_s"]
        ),
        "backup_last_known_free_blocked_position": _point(
            values["backup_last_known_free_blocked_position"]
        ),
        "backup_last_seed_switch_time_s": _float(
            values["backup_last_seed_switch_time_s"]
        ),
        "backup_last_seed_duration_s": _float(values["backup_last_seed_duration_s"]),
        "backup_last_seed_initial_velocity_mps": _float(
            values["backup_last_seed_initial_velocity_mps"]
        ),
        "backup_last_seed_max_velocity_mps": _float(
            values["backup_last_seed_max_velocity_mps"]
        ),
        "backup_last_seed_max_acceleration_mps2": _float(
            values["backup_last_seed_max_acceleration_mps2"]
        ),
        "backup_last_seed_max_jerk_mps3": _float(
            values["backup_last_seed_max_jerk_mps3"]
        ),
        "backup_last_seed_endpoint": _point(values["backup_last_seed_endpoint"]),
        "optimizer_latency_ms": _float(values["optimizer_latency_ms"]),
        "exp_diagnostics_valid": _bool(values["exp_diagnostics_valid"]),
        "exp_used_certified_seed": _bool(values["exp_used_certified_seed"]),
        "exp_certified_seed_failure_stage": _int(
            values["exp_certified_seed_failure_stage"]
        ),
        "exp_corridor_seed_build_failure_stage": _int(
            values["exp_corridor_seed_build_failure_stage"]
        ),
        "exp_corridor_seed_retry_attempt_count": _int(
            values["exp_corridor_seed_retry_attempt_count"]
        ),
        "exp_corridor_seed_retry_build_valid_count": _int(
            values["exp_corridor_seed_retry_build_valid_count"]
        ),
        "exp_corridor_seed_retry_last_certificate_stage": _int(
            values["exp_corridor_seed_retry_last_certificate_stage"]
        ),
        "exp_corridor_seed_selected_mode": _int(
            values["exp_corridor_seed_selected_mode"]
        ),
        "exp_corridor_seed_selected_max_duration_scale": _float(
            values["exp_corridor_seed_selected_max_duration_scale"]
        ),
        "exp_lbfgs_attempt_count": _int(values["exp_lbfgs_attempt_count"]),
        "exp_lbfgs_evaluation_count": _int(values["exp_lbfgs_evaluation_count"]),
        "exp_lbfgs_first_attempt_evaluation_count": _int(
            values["exp_lbfgs_first_attempt_evaluation_count"]
        ),
        "exp_lbfgs_last_attempt_evaluation_count": _int(
            values["exp_lbfgs_last_attempt_evaluation_count"]
        ),
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
        "exp_hard_deadline_observed": _bool(values["exp_hard_deadline_observed"]),
        "exp_initial_normalized_dynamic_violation": _float(
            values["exp_initial_normalized_dynamic_violation"]
        ),
        "exp_best_normalized_dynamic_violation": _float(
            values["exp_best_normalized_dynamic_violation"]
        ),
        "exp_final_normalized_dynamic_violation": _float(
            values["exp_final_normalized_dynamic_violation"]
        ),
        "exp_certified_seed_maximum_velocity_mps": _float(
            values["exp_certified_seed_maximum_velocity_mps"]
        ),
        "exp_certified_seed_maximum_acceleration_mps2": _float(
            values["exp_certified_seed_maximum_acceleration_mps2"]
        ),
        "exp_certified_seed_maximum_jerk_mps3": _float(
            values["exp_certified_seed_maximum_jerk_mps3"]
        ),
        "exp_initial_duration_s": _float(values["exp_initial_duration_s"]),
        "exp_initial_minimum_piece_duration_s": _float(
            values["exp_initial_minimum_piece_duration_s"]
        ),
        "exp_initial_maximum_piece_duration_s": _float(
            values["exp_initial_maximum_piece_duration_s"]
        ),
        "exp_final_duration_s": _float(values["exp_final_duration_s"]),
        "exp_retry_duration_lower_bound_min_s": _float(
            values["exp_retry_duration_lower_bound_min_s"]
        ),
        "exp_retry_duration_lower_bound_max_s": _float(
            values["exp_retry_duration_lower_bound_max_s"]
        ),
        "exp_retry_free_duration_seed_min_s": _float(
            values["exp_retry_free_duration_seed_min_s"]
        ),
        "exp_retry_free_duration_seed_max_s": _float(
            values["exp_retry_free_duration_seed_max_s"]
        ),
        "guide_path_length_m": _float(values["guide_path_length_m"]),
        "guide_duration_s": _float(values["guide_duration_s"]),
        "required_lookahead_m": _float(values["required_lookahead_m"]),
        "certified_lookahead_m": _float(values["certified_lookahead_m"]),
        "lookahead_complete": _bool(values["lookahead_complete"]),
        "exp_retry_budget_remaining_us": _float(
            values["exp_retry_budget_remaining_us"]
        ),
        "exp_refinement_budget_at_entry_us": _float(
            values["exp_refinement_budget_at_entry_us"]
        ),
        "exp_nonfinite_evaluation_count": _int(
            values["exp_nonfinite_evaluation_count"]
        ),
        "exp_first_nonfinite_stage": _int(values["exp_first_nonfinite_stage"]),
        "exp_first_nonfinite_value_mask": _int(
            values["exp_first_nonfinite_value_mask"]
        ),
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
        "planning_outcome": _int(values["planning_outcome"]),
        "planning_failure_stage": _int(values["planning_failure_stage"]),
        "planning_failure_reason": _int(values["planning_failure_reason"]),
        "planner_backend_outcome": _int(values["planner_backend_outcome"]),
        "planner_backend_failure_stage": _int(
            values["planner_backend_failure_stage"]
        ),
        "planner_backend_failure_reason": _int(
            values["planner_backend_failure_reason"]
        ),
        "planning_failure_stage_name": str(values["planning_failure_stage_name"])
        if values["planning_failure_stage_name"] is not None else None,
        "planning_failure_reason_name": str(values["planning_failure_reason_name"])
        if values["planning_failure_reason_name"] is not None else None,
        "planner_backend_failure_stage_name": str(
            values["planner_backend_failure_stage_name"]
        ) if values["planner_backend_failure_stage_name"] is not None else None,
        "planner_backend_failure_reason_name": str(
            values["planner_backend_failure_reason_name"]
        ) if values["planner_backend_failure_reason_name"] is not None else None,
        "backend_outcome_scope": str(values["backend_outcome_scope"])
        if values["backend_outcome_scope"] is not None else None,
        "worker_transaction_identity_scope": str(
            values["worker_transaction_identity_scope"]
        ) if values["worker_transaction_identity_scope"] is not None else None,
        "runtime_admission_disposition": str(values["runtime_admission_disposition"])
        if values["runtime_admission_disposition"] is not None else None,
        "execution_disposition": str(values["execution_disposition"])
        if values["execution_disposition"] is not None else None,
        "first_causal_failure_scope": str(values["first_causal_failure_scope"])
        if values["first_causal_failure_scope"] is not None else None,
        "first_causal_failure_stage": str(values["first_causal_failure_stage"])
        if values["first_causal_failure_stage"] is not None else None,
        "first_causal_failure_reason": str(values["first_causal_failure_reason"])
        if values["first_causal_failure_reason"] is not None else None,
        "first_causal_failure_class": str(values["first_causal_failure_class"])
        if values["first_causal_failure_class"] is not None else None,
        "backend_failure_event_class": str(values["backend_failure_event_class"])
        if values["backend_failure_event_class"] is not None else None,
        "runtime_admission_failure_event_class": str(
            values["runtime_admission_failure_event_class"]
        ) if values["runtime_admission_failure_event_class"] is not None else None,
        "watchdog_event_class": str(values["watchdog_event_class"])
        if values["watchdog_event_class"] is not None else None,
        "planner_trace_schema_version": _int(values["planner_trace_schema_version"]),
        "candidate_generation": _int(values["candidate_generation"]),
        "candidate_activation_stamp_ns": _signed_int(
            values["candidate_activation_stamp_ns"]
        ),
        "planner_backend_failure_event_steady_ns": _signed_int(
            values["planner_backend_failure_event_steady_ns"]
        ),
        "watchdog_event_steady_ns": _signed_int(values["watchdog_event_steady_ns"]),
        "runtime_admission_failure_event_steady_ns": _signed_int(
            values["runtime_admission_failure_event_steady_ns"]
        ),
        "first_causal_failure_event_steady_ns": _signed_int(
            values["first_causal_failure_event_steady_ns"]
        ),
        "first_causal_failure_event_order": _int(
            values["first_causal_failure_event_order"]
        ),
        "commit_decision": _int(values["commit_decision"]),
        "solve_stage": _int(values["solve_stage"]),
        "solve_stage_name": str(values["solve_stage_name"])
        if values["solve_stage_name"] is not None else None,
        "solve_deadline_exceeded": _bool(values["solve_deadline_exceeded"]),
        "command_available": _bool(values["command_available"]),
        "planner_failure_latched": _bool(values["planner_failure_latched"]),
        "source": source,
    }
    for _field in _PLANNER_TIMELINE_FIELDS:
        if _field.endswith("_name"):
            record[_field] = (
                str(values[_field]) if values[_field] is not None else None
            )
        elif _field.endswith("_observed"):
            record[_field] = _bool(values[_field])
        else:
            record[_field] = _signed_int(values[_field])
    if timestamp_s is not None:
        record["timestamp_s"] = _float(timestamp_s)
    for _field in _RUNTIME_TRANSACTION_FIELDS:
        if _field.endswith("_steady_ns"):
            record[_field] = _signed_int(values[_field])
        elif _field.endswith("_attempted") or _field.endswith("_succeeded") or \
                _field.endswith("_staged"):
            record[_field] = _bool(values[_field])
        else:
            record[_field] = _int(values[_field])
    for _field in (
        "worker_transaction_identity_scope",
        "backend_outcome_scope",
        "runtime_admission_disposition",
        "execution_disposition",
        "first_causal_failure_scope",
        "first_causal_failure_stage",
        "first_causal_failure_reason",
    ):
        record[_field] = (
            str(values[_field]) if values[_field] is not None else None
        )
    causal_events: list[dict[str, Any]] = []
    backend_failure_stage = record.get("planner_backend_failure_stage")
    if isinstance(backend_failure_stage, int) and backend_failure_stage > 0:
        backend_event_ns = record.get("planner_backend_failure_event_steady_ns")
        if not isinstance(backend_event_ns, int) or backend_event_ns <= 0:
            backend_event_ns = record.get("planner_solve_finished_steady_ns")
        causal_events.append({
            "class": record.get("backend_failure_event_class") or "CAUSAL",
            "scope": "BACKEND",
            "stage": record.get("planner_backend_failure_stage_name") or "unknown",
            "reason": record.get("planner_backend_failure_reason_name") or "unknown",
            "steady_ns": backend_event_ns,
            "sequence": 0,
            "source": "backend_failure",
        })
    if record.get("runtime_admission_disposition") == "REJECTED":
        admission_event_ns = record.get(
            "runtime_admission_failure_event_steady_ns"
        )
        if not isinstance(admission_event_ns, int) or admission_event_ns <= 0:
            admission_event_ns = record.get("runtime_admission_finished_steady_ns")
        causal_events.append({
            "class": record.get("runtime_admission_failure_event_class") or "CAUSAL",
            "scope": "RUNTIME_ADMISSION",
            "stage": "execution_boundary",
            "reason": "execution_boundary_rejected",
            "steady_ns": admission_event_ns,
            "sequence": 1,
            "source": "runtime_admission",
        })
    watchdog_event_ns = record.get("watchdog_event_steady_ns")
    if isinstance(watchdog_event_ns, int) and watchdog_event_ns > 0:
        causal_events.append({
            "class": record.get("watchdog_event_class") or "CAUSAL",
            "scope": "WATCHDOG",
            "stage": "watchdog",
            "reason": "timeout",
            "steady_ns": watchdog_event_ns,
            "sequence": 2,
            "source": "watchdog",
        })
    reduced_causal_failure = reduce_first_causal_failure(causal_events)
    record["causal_events"] = causal_events
    record["causal_reducer_status"] = (
        "COMPLETE" if reduced_causal_failure["present"] else
        "PARTIAL_TIMESTAMP" if reduced_causal_failure["invalid_timestamp_count"] else
        "NO_CAUSAL_EVENT"
    )
    if reduced_causal_failure["present"]:
        record["first_causal_failure_class"] = "CAUSAL"
        record["first_causal_failure_scope"] = reduced_causal_failure["scope"]
        record["first_causal_failure_stage"] = reduced_causal_failure["stage"]
        record["first_causal_failure_reason"] = reduced_causal_failure["reason"]
        record["first_causal_failure_event_steady_ns"] = reduced_causal_failure[
            "steady_ns"
        ]
        record["first_causal_failure_event_order"] = reduced_causal_failure[
            "sequence"
        ]
    record["record_key"] = [record["planning_cycle_id"], record["bundle_id"]]
    record["missing_key_fields"] = [
        name for name in ("planning_cycle_id", "bundle_id") if record[name] is None
    ]
    record["complete"] = not record["missing_key_fields"]
    required_transaction_fields = (
        "backend_outcome_scope",
        "runtime_admission_disposition",
        "execution_disposition",
        "planner_trace_schema_version",
    )
    record["transaction_missing_fields"] = [
        field for field in required_transaction_fields if record.get(field) is None
    ]
    record["transaction_complete"] = bool(record["complete"]) and not record[
        "transaction_missing_fields"
    ]
    record["transaction_completeness"] = (
        "COMPLETE" if record["transaction_complete"] else "PARTIAL_TRACE"
    )
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
        # Legacy traces can contain a partially populated record key. Sort its
        # presentation deterministically without comparing None to integers;
        # identity validity is checked by the reducer, not hidden here.
        tuple("" if value is None else str(value) for value in item["record_key"]),
    ))


def planner_trace_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    failure_stage_counts: dict[str, int] = {}
    corridor_build_failure_stage_counts: dict[str, int] = {}
    corridor_retry_certificate_stage_counts: dict[str, int] = {}
    corridor_selected_mode_counts: dict[str, int] = {}
    runtime_admission_disposition_counts: dict[str, int] = {}
    execution_disposition_counts: dict[str, int] = {}
    first_causal_failure_scope_counts: dict[str, int] = {}
    for record in records:
        for field, counts in (
            (
                "runtime_admission_disposition",
                runtime_admission_disposition_counts,
            ),
            ("execution_disposition", execution_disposition_counts),
            ("first_causal_failure_scope", first_causal_failure_scope_counts),
        ):
            value = record.get(field)
            if isinstance(value, str) and value:
                counts[value] = counts.get(value, 0) + 1
        stage = record.get("exp_certified_seed_failure_stage")
        if isinstance(stage, int) and not isinstance(stage, bool):
            key = str(stage)
            failure_stage_counts[key] = failure_stage_counts.get(key, 0) + 1
        for field, counts in (
            (
                "exp_corridor_seed_build_failure_stage",
                corridor_build_failure_stage_counts,
            ),
            ("exp_corridor_seed_selected_mode", corridor_selected_mode_counts),
        ):
            value = record.get(field)
            if isinstance(value, int) and not isinstance(value, bool):
                key = str(value)
                counts[key] = counts.get(key, 0) + 1
        retry_build_valid_count = record.get(
            "exp_corridor_seed_retry_build_valid_count"
        )
        retry_certificate_stage = record.get(
            "exp_corridor_seed_retry_last_certificate_stage"
        )
        # Stage zero is the valid-certificate value, but it is also the
        # producer's default when no retry trajectory reached certification.
        # Count the stage only when at least one retry build was valid; never
        # turn a default-initialized field into evidence of a certificate.
        if (
            isinstance(retry_build_valid_count, int)
            and not isinstance(retry_build_valid_count, bool)
            and retry_build_valid_count > 0
            and isinstance(retry_certificate_stage, int)
            and not isinstance(retry_certificate_stage, bool)
        ):
            key = str(retry_certificate_stage)
            corridor_retry_certificate_stage_counts[key] = (
                corridor_retry_certificate_stage_counts.get(key, 0) + 1
            )
    return {
        "record_count": len(records),
        "complete_record_count": sum(bool(record.get("complete")) for record in records),
        "partial_record_count": sum(not bool(record.get("complete")) for record in records),
        "transaction_complete_count": sum(
            record.get("transaction_complete") is True for record in records
        ),
        "transaction_partial_count": sum(
            record.get("transaction_complete") is not True for record in records
        ),
        "exp_certified_seed_used_count": sum(
            record.get("exp_used_certified_seed") is True for record in records
        ),
        "exp_certified_seed_failure_stage_counts": failure_stage_counts,
        "exp_corridor_seed_build_failure_stage_counts": (
            corridor_build_failure_stage_counts
        ),
        "exp_corridor_seed_retry_attempt_total": sum(
            value
            for record in records
            if isinstance(
                value := record.get("exp_corridor_seed_retry_attempt_count"), int
            )
            and not isinstance(value, bool)
        ),
        "exp_corridor_seed_retry_build_valid_total": sum(
            value
            for record in records
            if isinstance(
                value := record.get("exp_corridor_seed_retry_build_valid_count"),
                int,
            )
            and not isinstance(value, bool)
        ),
        "exp_corridor_seed_retry_last_certificate_stage_counts": (
            corridor_retry_certificate_stage_counts
        ),
        "exp_corridor_seed_selected_mode_counts": corridor_selected_mode_counts,
        "runtime_admission_disposition_counts": runtime_admission_disposition_counts,
        "execution_disposition_counts": execution_disposition_counts,
        "first_causal_failure_scope_counts": first_causal_failure_scope_counts,
        "fields_are_runtime_supplied": True,
        "missing_data_policy": "omitted, never inferred from request/trajectory IDs or aggregate counters",
    }

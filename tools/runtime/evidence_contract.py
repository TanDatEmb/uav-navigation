"""Versioned metric-to-evidence contract for offline navigation reports."""

from __future__ import annotations

from typing import Any
import math


EVIDENCE_CONTRACT_SCHEMA_VERSION = 1


METRIC_CONTRACTS: tuple[dict[str, Any], ...] = (
    # Command lineage belongs to the PVA/reference source.  Independent
    # ground-truth rows must not inherit request or bundle IDs by inference.
    {
        "metric": "tracking.position_error_m",
        "unit": "m",
        "time_basis": "source_stamp_ns",
        "required_sources": (
            {"source": "pva", "fields": ("source_stamp_ns", "position", "frame_id", "time_basis", "source_clock"),
             "identity_fields": ("session_id", "request_id", "bundle_generation", "sample_id")},
            {"source": "ground_truth_odometry", "fields": ("source_stamp_ns", "position", "frame_id", "time_basis", "source_clock")},
        ),
        "identity_fields": ("session_id", "request_id", "bundle_generation", "sample_id"),
        "validity_fields": ("time_basis", "source_clock", "truth_frame_witness"),
        "qualification_checks": (
            "source_time_valid", "frame_transform_valid", "reference_lineage_valid",
            "coverage_sufficient", "capture_integrity_valid",
        ),
        "qualification_required": True,
    },
    {
        "metric": "tracking.velocity_error_mps",
        "unit": "m/s",
        "time_basis": "source_stamp_ns",
        "required_sources": (
            {"source": "pva", "fields": ("source_stamp_ns", "velocity", "frame_id", "time_basis", "source_clock"),
             "identity_fields": ("session_id", "request_id", "bundle_generation", "sample_id")},
            {"source": "ground_truth_odometry", "fields": ("source_stamp_ns", "linear_velocity", "frame_id", "time_basis", "source_clock")},
        ),
        "identity_fields": ("session_id", "request_id", "bundle_generation", "sample_id"),
        "validity_fields": ("time_basis", "source_clock", "truth_frame_witness"),
        "qualification_checks": (
            "source_time_valid", "frame_transform_valid", "reference_lineage_valid",
            "coverage_sufficient", "capture_integrity_valid",
        ),
        "qualification_required": True,
    },
    {
        "metric": "motion.stop_go",
        "unit": "s",
        "time_basis": "pva.source_stamp_ns",
        "required_sources": (
            {"source": "pva", "fields": ("source_stamp_ns", "time_basis", "source_clock", "velocity", "frame_id")},
        ),
        "qualification_required": True,
    },
    {
        "metric": "motion.chattering",
        "unit": "m/s",
        "time_basis": "source_stamp_ns",
        "required_sources": (
            {"source": "pva", "fields": ("source_stamp_ns", "time_basis", "source_clock", "velocity", "frame_id", "trajectory_flag")},
        ),
        "qualification_required": True,
    },
    {
        "metric": "motion.stitching",
        "unit": "mixed",
        "time_basis": "planner_trace.timestamp_s",
        "required_sources": (
            {"source": "planner_trace", "fields": ("cycle_id", "bundle_id", "record_key")},
        ),
        "qualification_required": False,
    },
    {
        "metric": "localization.ate_rpe",
        "unit": "m",
        "time_basis": "source_stamp_ns",
        "required_sources": (
            {"source": "ground_truth_odometry", "fields": ("source_stamp_ns", "time_basis", "source_clock", "position", "frame_id")},
            {"source": "corrected_or_propagated_odometry", "fields": ("source_stamp_ns", "time_basis", "source_clock", "position", "frame_id")},
        ),
        "qualification_required": False,
    },
    {
        "metric": "mission.completion",
        "unit": "boolean",
        "time_basis": "scenario lifecycle event",
        "required_sources": (
            {"source": "scenario.json", "fields": ("mission_complete_observed", "outcome")},
            {"source": "navigation_mode_status", "fields": ("waypoint_accepted", "accepted_waypoint_index")},
        ),
        "qualification_required": True,
    },
    {
        "metric": "safety.collision",
        "unit": "count",
        "time_basis": "scenario collision envelope",
        "required_sources": (
            {"source": "scenario.json", "fields": ("collision_count", "minimum_collision_clearance_m")},
        ),
        "qualification_required": True,
    },
    {
        "metric": "tracking.adapter_reference_vs_px4_state",
        "unit": "m",
        "time_basis": "px4_source_stamp",
        "required_sources": (
            {"source": "px4_input_trace", "fields": ("trace_timestamp_ns", "trace_sequence")},
            {"source": "px4_telemetry", "fields": ("source_stamp_ns", "time_basis", "source_clock")},
            {"source": "px4_clock_mapping_witness", "fields": ("status", "source_clock")},
        ),
        "qualification_required": False,
    },
    {
        "metric": "timing.pva_observer_interarrival_ms",
        "unit": "ms",
        "time_basis": "observer_record_steady_ns",
        "required_sources": (
            {"source": "pva", "fields": ("observer_record_steady_ns",)},
        ),
        "qualification_required": False,
    },
    {
        "metric": "timing.px4_setpoint_update_duration_ms",
        "unit": "ms",
        "time_basis": "px4_input_trace steady_clock",
        "required_sources": (
            {"source": "px4_input_trace", "fields": ("trace_timestamp_ns", "setpoint_update_duration_ns")},
        ),
        "qualification_required": False,
    },
)


def _rows_for_source(inputs: dict[str, Any], source: str) -> list[dict[str, Any]]:
    def rows(value: Any) -> list[dict[str, Any]]:
        # Preserve valid rows when a recorder emitted one malformed row; the
        # malformed row must not make otherwise useful diagnostics disappear.
        return [item for item in value if isinstance(item, dict)] \
            if isinstance(value, (list, tuple)) else []

    if source == "pva":
        return rows(inputs.get("pva", []))
    if source == "planner_trace":
        return rows(inputs.get("planner_trace", []))
    if source == "ground_truth_odometry":
        streams = inputs.get("streams", {})
        return rows(streams.get("ground_truth_odometry", [])) if isinstance(streams, dict) else []
    if source == "corrected_or_propagated_odometry":
        streams = inputs.get("streams", {})
        if not isinstance(streams, dict):
            return []
        return rows(streams.get("corrected_odometry", [])) or rows(streams.get("propagated_odometry", []))
    if source == "px4_input_trace":
        return rows(inputs.get("px4_input_trace", []))
    if source == "px4_telemetry":
        streams = inputs.get("streams", {})
        if not isinstance(streams, dict):
            return []
        return rows(streams.get("px4_odometry", [])) or rows(streams.get("local_position", []))
    if source == "navigation_mode_status":
        return [
            event.get("payload", {})
            for event in inputs.get("scenario_events", [])
            if event.get("kind") == "navigation_mode_status"
            and isinstance(event.get("payload"), dict)
        ]
    if source == "scenario.json":
        scenario = inputs.get("scenario", {})
        return [scenario] if isinstance(scenario, dict) else []
    if source == "px4_clock_mapping_witness":
        metadata = inputs.get("metadata", {})
        contract = metadata.get("evidence_contract", {}) if isinstance(metadata, dict) else {}
        mapping = contract.get("px4_to_ros_mapping", {}) if isinstance(contract, dict) else {}
        return [mapping] if isinstance(mapping, dict) else []
    return []


_INTEGER_FIELDS = {
    "source_stamp_ns", "trace_timestamp_ns", "timestamp_ns", "execution_stamp_ns",
    "observer_record_steady_ns", "world_observation_stamp_ns", "authorization_steady_ns", "request_id",
    "bundle_generation", "sample_id", "cycle_id", "bundle_id", "planning_cycle_id",
    "localization_epoch", "goal_epoch", "trace_sequence", "record_sequence",
}
_NONNEGATIVE_INTEGER_FIELDS = {
    "trajectory_flag", "accepted_waypoint_index", "collision_count",
    "setpoint_update_duration_ns",
}
_NONNEGATIVE_NUMBER_FIELDS = {"minimum_collision_clearance_m"}
_VECTOR_FIELDS = {"position", "velocity", "linear_velocity"}
_BOOLEAN_FIELDS = {"mission_complete_observed", "waypoint_accepted"}
_STRING_FIELDS = {"time_basis", "source_clock", "outcome", "status", "setpoint_kind"}
_IDENTITY_FIELDS = {
    "request_id", "bundle_generation", "sample_id", "cycle_id", "bundle_id",
    "planning_cycle_id", "localization_epoch", "goal_epoch", "trace_sequence",
}


def _finite_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _field_issue(field: str, value: Any, *, identity: bool = False) -> str | None:
    """Return a stable reason for malformed evidence, without coercion."""
    if value is None:
        return "MISSING"
    if identity or field in _IDENTITY_FIELDS:
        if field == "session_id":
            return None if isinstance(value, str) and value.strip() else "IDENTITY_INVALID"
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            return "IDENTITY_INVALID"
        return None
    if field in _INTEGER_FIELDS:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            return "SOURCE_TIME_INVALID" if field.endswith("stamp_ns") or field == "trace_timestamp_ns" else "TYPE_OR_RANGE_INVALID"
        return None
    if field in _NONNEGATIVE_INTEGER_FIELDS:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return "TYPE_OR_RANGE_INVALID"
        return None
    if field in _NONNEGATIVE_NUMBER_FIELDS:
        return None if _finite_number(value) and float(value) >= 0.0 else "NONFINITE_OR_TYPE_INVALID"
    if field in _VECTOR_FIELDS:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return "TYPE_INVALID"
        if any(not _finite_number(item) for item in value):
            return "NONFINITE"
        return None
    if field == "record_key":
        if (
            not isinstance(value, (list, tuple)) or len(value) != 2
            or any(isinstance(item, bool) or not isinstance(item, int) or item <= 0 for item in value)
        ):
            return "IDENTITY_INVALID"
        return None
    if field == "timestamp_s":
        return None if _finite_number(value) and float(value) >= 0.0 else "SOURCE_TIME_INVALID"
    if field in _BOOLEAN_FIELDS:
        return None if isinstance(value, bool) else "TYPE_INVALID"
    if field in _STRING_FIELDS or field.endswith("_frame_id") or field == "frame_id":
        if not isinstance(value, str) or not value.strip():
            return "FRAME_INVALID" if field.endswith("frame_id") or field == "frame_id" else "TYPE_OR_PROVENANCE_INVALID"
        if field == "source_clock" and value.strip().lower() in {"unknown", "unset", "invalid"}:
            return "SOURCE_CLOCK_UNVERIFIED"
        if field == "time_basis" and value.strip() != "source_stamp":
            return "SOURCE_TIME_INVALID"
        return None
    # Scalar metric values must be numeric and finite when their name carries
    # a physical unit.  Do not accept numeric strings or silently replace NaN.
    if field.endswith(("_m", "_mps", "_mps2", "_mps3", "_ms", "_ns")):
        return None if _finite_number(value) else "NONFINITE_OR_TYPE_INVALID"
    return None


def _source_check(inputs: dict[str, Any], requirement: dict[str, Any]) -> dict[str, Any]:
    source = str(requirement["source"])
    rows = _rows_for_source(inputs, source)
    fields = [str(field) for field in requirement.get("fields", ())]
    identity_fields = [str(field) for field in requirement.get("identity_fields", ())]
    all_fields = list(dict.fromkeys(fields + identity_fields))
    missing_fields = [
        field for field in all_fields
        if not any(field in row and row.get(field) is not None for row in rows)
    ]
    partially_missing_fields = [
        field for field in all_fields
        if rows and any(field not in row or row.get(field) is None for row in rows)
    ]
    invalid_fields: set[str] = set()
    invalid_reasons: set[str] = set()
    invalid_rows: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        row_reasons: dict[str, str] = {}
        for field in fields:
            issue = _field_issue(field, row.get(field))
            if issue not in (None, "MISSING"):
                invalid_fields.add(field)
                invalid_reasons.add(issue)
                row_reasons[field] = issue
        for field in identity_fields:
            issue = _field_issue(field, row.get(field), identity=True)
            if issue not in (None, "MISSING"):
                invalid_fields.add(field)
                invalid_reasons.add(issue)
                row_reasons[field] = issue
        if row_reasons:
            invalid_rows.append({"row_index": row_index, "fields": row_reasons})
    invalid_fields_sorted = sorted(invalid_fields)
    return {
        "source": source,
        "row_count": len(rows),
        "required_fields": all_fields,
        "missing_fields": missing_fields,
        "partially_missing_fields": partially_missing_fields,
        "identity_fields": identity_fields,
        "invalid_fields": invalid_fields_sorted,
        "invalid_reasons": sorted(invalid_reasons),
        "invalid_rows": invalid_rows,
        "status": "AVAILABLE" if rows and not missing_fields and not partially_missing_fields and not invalid_fields_sorted else "NOT_EVALUABLE",
    }


def build_evidence_contract(inputs: dict[str, Any]) -> dict[str, Any]:
    """Return explicit availability for every metric without filling gaps."""
    metrics: list[dict[str, Any]] = []
    qualification_missing: list[str] = []
    for definition in METRIC_CONTRACTS:
        sources = [_source_check(inputs, requirement) for requirement in definition["required_sources"]]
        missing = [
            f"{item['source']}: {','.join(item['missing_fields']) or ','.join(item.get('invalid_fields', [])) or 'no rows'}"
            for item in sources if item["status"] != "AVAILABLE"
        ]
        status = "AVAILABLE" if not missing else "NOT_EVALUABLE"
        if definition.get("qualification_required") and missing:
            qualification_missing.append(f"{definition['metric']} source incomplete")
        metrics.append({
            "metric": definition["metric"],
            "unit": definition["unit"],
            "time_basis": definition["time_basis"],
            "identity_fields": list(definition.get("identity_fields", ())),
            "validity_fields": list(definition.get("validity_fields", ())),
            "qualification_checks": list(definition.get("qualification_checks", ())),
            "qualification_required": bool(definition.get("qualification_required", False)),
            "status": status,
            "sources": sources,
            "reason": None if not missing else "MISSING_SOURCE_OR_REQUIRED_FIELD",
        })
    return {
        "schema_version": EVIDENCE_CONTRACT_SCHEMA_VERSION,
        "status": "COMPLETE" if not qualification_missing else "INCOMPLETE",
        "qualification_missing": sorted(set(qualification_missing)),
        "metrics": metrics,
    }

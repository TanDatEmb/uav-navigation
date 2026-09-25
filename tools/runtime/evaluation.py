"""Offline navigation-quality evaluator.

This module is deliberately separate from ROS callbacks and flight-control
logic.  It consumes the session-owned evidence artifacts, computes metrics once
from raw records, and returns a versioned assessment for ``report.py`` and the
HTML renderer.  A missing timestamp, frame witness, epoch continuity or writer
record is represented as ``NOT_EVALUABLE``; missing data is never converted to
zero.
"""

from __future__ import annotations

import bisect
from enum import Enum
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable

from evidence_contract import build_evidence_contract
from command_diagnostics import join_execution_diagnostics
from planner_trace import collect_planner_trace_records, planner_trace_summary


EVALUATION_SCHEMA_VERSION = 2
DEFAULT_MAX_MATCH_GAP_S = 0.15
DEFAULT_STOP_ENTER_MPS = 0.10
DEFAULT_STOP_EXIT_MPS = 0.20
DEFAULT_STOP_MIN_DURATION_S = 0.20
DEFAULT_C0_SPEED_MIN_MPS = 1.0
DEFAULT_C0_SPEED_MAX_MPS = 5.0
C0_SW_POLICY_VERSION = "C0_SW_V1"
C0_SW_POLICY_PROVENANCE = (
    "user-approved task 2026-09-25: C0-SW software-first decision"
)
_SUCCESS_DISPOSITIONS = {
    "PUBLISHED", "AUTHORIZED", "EXPORTED", "ACTIVATED", "OBSERVED", "SUCCESS", "ACTIVE",
}
_TERMINAL_REJECT_DISPOSITIONS = {"REJECTED", "FAILED", "CANCELED", "CANCELLED", "SUPERSEDED"}
_LIFECYCLE_PHASES = {
    "request", "result", "retained", "heading_admitted", "authorize",
    "export", "activate", "publish", "supersede", "recovery_retry",
    "planner_emergency_outcome",
}


class SourceTimestampPolicy(Enum):
    """Source-time contract for an evidence stream, not a flight-time gate."""

    STRICTLY_INCREASING = "STRICTLY_INCREASING"
    NON_DECREASING = "NON_DECREASING"
    DUPLICATES_ALLOWED = "DUPLICATES_ALLOWED"
    DUPLICATES_ALLOWED_FOR_HEARTBEAT = "DUPLICATES_ALLOWED_FOR_HEARTBEAT"
    EVENT_SEQUENCE_AUTHORITATIVE = "EVENT_SEQUENCE_AUTHORITATIVE"
    NO_SOURCE_TIME = "NO_SOURCE_TIME"


STREAM_SOURCE_TIMESTAMP_POLICY = {
    "navigation_command": SourceTimestampPolicy.DUPLICATES_ALLOWED_FOR_HEARTBEAT,
    "ground_truth_odometry": SourceTimestampPolicy.STRICTLY_INCREASING,
    "corrected_odometry": SourceTimestampPolicy.STRICTLY_INCREASING,
    "propagated_odometry": SourceTimestampPolicy.STRICTLY_INCREASING,
}


def _load(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _integer(value: Any) -> int | None:
    result = _number(value)
    return int(result) if result is not None and result.is_integer() else None


def _vector(value: Any, size: int = 3) -> tuple[float, ...] | None:
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    result = tuple(_number(value[index]) for index in range(size))
    return result if all(item is not None for item in result) else None


def _norm(value: Iterable[float]) -> float:
    return math.sqrt(sum(float(item) * float(item) for item in value))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def _summary(values: list[float], *, unit: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "count": len(values),
        "mean": statistics.fmean(values) if values else None,
        "rmse": math.sqrt(statistics.fmean([value * value for value in values])) if values else None,
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "maximum": max(values) if values else None,
    }
    if unit is not None:
        result["unit"] = unit
    return result


def _source_stamp_ns(event: dict[str, Any], payload: dict[str, Any]) -> tuple[int | None, str]:
    for key in ("stamp_ns", "source_stamp_ns"):
        stamp = _integer(payload.get(key))
        if stamp is not None and stamp > 0:
            return stamp, "source_stamp"
    stamp = _integer(event.get("sim_time_ns"))
    if stamp is not None and stamp > 0:
        return stamp, "observer_sim_time_legacy"
    return None, "missing"


def _source_time_status(
    rows: list[dict[str, Any]],
    *,
    policy: SourceTimestampPolicy = SourceTimestampPolicy.STRICTLY_INCREASING,
) -> tuple[bool, list[str]]:
    """Validate provenance, not merely presence of a numeric timestamp."""
    if policy in {SourceTimestampPolicy.DUPLICATES_ALLOWED_FOR_HEARTBEAT,
                  SourceTimestampPolicy.EVENT_SEQUENCE_AUTHORITATIVE,
                  SourceTimestampPolicy.NO_SOURCE_TIME}:
        raise ValueError("this policy requires stream-specific identity validation")
    reasons: list[str] = []
    if not rows:
        return False, ["SOURCE_TIME_UNAVAILABLE"]
    previous_stamp: int | None = None
    previous_epoch: Any = None
    for row in rows:
        basis = str(row.get("time_basis", "unknown"))
        stamp = _integer(row.get("source_stamp_ns"))
        if basis == "observer_sim_time_legacy":
            reasons.append("OBSERVER_TIME_ONLY")
        elif basis != "source_stamp" or stamp is None or stamp <= 0:
            reasons.append("SOURCE_TIMESTAMP_INVALID")
        if row.get("source_clock") in (None, "", "unknown"):
            reasons.append("CLOCK_RELATION_UNVERIFIED")
        epoch = row.get("localization_epoch")
        if previous_epoch is not None and epoch != previous_epoch:
            reasons.append("SOURCE_TIME_EPOCH_CHANGE")
            previous_stamp = None
        if stamp is not None and stamp > 0 and previous_stamp is not None:
            if stamp == previous_stamp and policy == SourceTimestampPolicy.STRICTLY_INCREASING:
                reasons.append("SOURCE_TIMESTAMP_DUPLICATE")
            elif stamp < previous_stamp:
                reasons.append("SOURCE_TIMESTAMP_REGRESSION")
        if stamp is not None and stamp > 0:
            previous_stamp = stamp
        previous_epoch = epoch
    return not reasons, sorted(set(reasons))


_REFERENCE_HEARTBEAT_IGNORED_FIELDS = frozenset({
    # A repeated immutable command may receive a new transport sample ID.
    "sample_id", "trajectory_id",
    # Each heartbeat is freshly authorized; its diagnostic steady timestamp
    # is an event timestamp rather than a change to the reference/certificate.
    "execution_authorization_steady_ns",
    # These identify the observer row, never the executable reference.
    "arrival_wall_ns", "arrival_steady_ns", "record_sequence",
    "record_sim_time_ns", "observer_record_steady_ns",
})


def _same_reference_heartbeat(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return {
        key: value for key, value in left.items()
        if key not in _REFERENCE_HEARTBEAT_IGNORED_FIELDS
    } == {
        key: value for key, value in right.items()
        if key not in _REFERENCE_HEARTBEAT_IGNORED_FIELDS
    }


def _canonical_reference_heartbeats(
    rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], int, list[str]]:
    """Collapse only exact same-state command heartbeats at one ROS source tick.

    A fresh sample ID may keep the steady-time adapter lease alive while ROS
    time has not advanced. Raw rows remain in the session and lifecycle
    evidence; this copy is only the time-indexed tracking reference.
    """
    canonical: list[dict[str, Any]] = []
    collapsed = 0
    reasons: list[str] = []
    for row in rows:
        if not canonical:
            canonical.append(row)
            continue
        previous = canonical[-1]
        if (row.get("source_stamp_ns") != previous.get("source_stamp_ns") or
                row.get("localization_epoch") != previous.get("localization_epoch")):
            canonical.append(row)
            continue
        previous_id = _integer(previous.get("sample_id"))
        current_id = _integer(row.get("sample_id"))
        if (previous_id is None or current_id is None or
                current_id <= previous_id or
                not _same_reference_heartbeat(row, previous)):
            reasons.append("SOURCE_TIMESTAMP_DUPLICATE_CONFLICT")
            canonical.append(row)
            continue
        collapsed += 1
        # Keep the latest sample ID so another identical heartbeat can be
        # checked for strict sample ordering.
        canonical[-1] = row
    return canonical, collapsed, sorted(set(reasons))


def _source_clock_relation_status(
    reference: list[dict[str, Any]], measured: list[dict[str, Any]]
) -> tuple[bool, list[str]]:
    reference_clocks = {
        str(row.get("source_clock")) for row in reference
        if row.get("source_clock") not in (None, "", "unknown")
    }
    measured_clocks = {
        str(row.get("source_clock")) for row in measured
        if row.get("source_clock") not in (None, "", "unknown")
    }
    if len(reference_clocks) != 1 or len(measured_clocks) != 1:
        return False, ["CLOCK_RELATION_UNVERIFIED"]
    if reference_clocks != measured_clocks:
        return False, ["CLOCK_RELATION_UNVERIFIED"]
    return True, []


def _record_identity(item: dict[str, Any]) -> tuple[Any, ...]:
    """Identity used for segment boundaries; missing values stay explicit."""
    return tuple(item.get(key) for key in (
        "session_id", "localization_epoch", "goal_epoch", "request_id",
        "bundle_generation", "analytic_sample_role",
    ))


def _present_identity(value: Any) -> bool:
    if value is None or value == "":
        return False
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return math.isfinite(float(value)) and value > 0
    return True


def _lifecycle_transaction_key(event: dict[str, Any]) -> tuple[Any, ...]:
    """Return the producer-declared transaction owner, never a guessed phase key."""
    request_id = event.get("bundle_owner_request_id", event.get("request_id"))
    source = _integer(event.get("bundle_source"))
    generation = _integer(event.get("bundle_generation"))
    # Retained-position heading rebind and emergency brake create new
    # executable generations outside a planner solve. Their own immutable
    # source + generation is the producer identity; inventing a nearby
    # planning cycle would misattribute the command.
    declared_cycle = event.get("bundle_owner_cycle_id")
    cycle_id = (("no_execution", event.get("sample_id"))
                if event.get("phase") == "authorize" and
                event.get("disposition") == "REJECTED" and
                event.get("goal_epoch") == 0 and
                event.get("bundle_generation") == 0 else
                ("terminal_monitor", event.get("captured_bundle_generation"),
                 event.get("producer_event_sequence"))
                if event.get("phase") == "retained" and
                event.get("purpose") == 2 else
                ("candidate_source", source, generation)
                if source in {2, 3} and generation and
                not _present_identity(declared_cycle) else declared_cycle)
    return (
        event.get("runtime_instance_id"),
        event.get("session_id"),
        event.get("localization_epoch"),
        event.get("goal_epoch"),
        request_id,
        cycle_id,
    )


def _lifecycle_consistency_signature(event: dict[str, Any]) -> tuple[Any, ...]:
    """Fields that must not disagree for one producer event identity."""
    return tuple(event.get(field) for field in (
        "disposition", "runtime_instance_id", "session_id",
        "localization_epoch", "goal_epoch", "request_id",
        "causal_planning_cycle_id", "bundle_owner_request_id",
        "bundle_owner_cycle_id", "bundle_generation", "sample_id",
        "trace_sequence", "adapter_trace_sequence", "source_stamp_ns",
        "authorization_boundary", "authorization_steady_ns",
        "world_generation", "world_revision", "world_observation_stamp_ns",
        "setpoint_kind", "setpoint_boundary",
    ))


def reduce_lifecycle(
    events: Iterable[dict[str, Any]],
    px4_input_trace: Iterable[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Reduce producer evidence by identity, independent of recorder order.

    This is an evidence consistency reducer, not a second flight-control FSM.
    A transaction is eligible only when its successful phases carry the same
    runtime/session/request/cycle/bundle lineage and publish has an attributed
    adapter trace. Repeated equal observations are deduplicated; contradictory
    dispositions are retained as a conflict.
    """
    normalized = [dict(item) for item in events if isinstance(item, dict)]
    normalized.extend(
        dict(
            item,
            phase="publish",
            disposition=str(item.get("disposition") or "OBSERVED"),
            attribution="px4_input_trace",
        )
        for item in (px4_input_trace or [])
        if isinstance(item, dict)
        and item.get("attribution") == "px4_input_trace"
        # Local velocity hold has no Core command sample or bundle. It is a
        # separate adapter safety boundary, never an incomplete Core command.
        and _present_identity(item.get("sample_id"))
        and _present_identity(item.get("bundle_generation"))
    )
    normalized.extend(
        dict(item, phase="planner_emergency_outcome",
             bundle_source=None, bundle_generation=None)
        for item in tuple(normalized)
        if item.get("phase") == "retained" and
        item.get("disposition_code") == 5 and
        item.get("purpose") == 0 and
        _present_identity(item.get("planning_cycle_id")) and
        _present_identity(item.get("after_bundle_generation"))
    )
    normalized.extend(
        dict(item, purpose=0,
             producer_projection="emergency_created_by_terminal_monitor")
        for item in tuple(normalized)
        if item.get("phase") == "retained" and
        item.get("disposition_code") == 5 and
        item.get("purpose") == 2 and
        _present_identity(item.get("after_bundle_generation"))
    )
    for event in normalized:
        if (event.get("phase") == "retained" and
                event.get("disposition_code") == 5 and
                _present_identity(event.get("after_bundle_generation"))):
            event["bundle_source"] = 3  # CandidateSource::kEmergency
            event["bundle_generation"] = event["after_bundle_generation"]
            # This retained validation is an independent emergency producer,
            # even if its diagnostic trace also names a scheduling tick.
            event["bundle_owner_cycle_id"] = None
            event["bundle_owner_attribution"] = "producer_declared"
    # A command can be authorized by a later retained-command validation than
    # the planning cycle which produced its immutable bundle. Build ownership
    # only from export witnesses; treating causal_planning_cycle_id as the
    # bundle owner silently relabels an old active command after every failed
    # renewal.
    export_owners: dict[tuple[Any, ...], set[Any]] = {}
    for event in normalized:
        if str(event.get("phase", "")) != "export":
            continue
        owner_cycle = event.get(
            "bundle_owner_cycle_id", event.get("causal_planning_cycle_id")
        )
        bundle_key = tuple(event.get(field) for field in (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "bundle_generation",
        ))
        if all(_present_identity(value) for value in bundle_key) and _present_identity(
            owner_cycle
        ):
            export_owners.setdefault(bundle_key, set()).add(owner_cycle)

    authorization_owners: dict[tuple[Any, ...], set[tuple[Any, ...]]] = {}
    def sample_key(event: dict[str, Any]) -> tuple[Any, ...]:
        return tuple(event.get(field) for field in (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "bundle_generation", "sample_id",
            "world_generation", "world_revision", "world_observation_stamp_ns",
        ))

    for event in normalized:
        if str(event.get("phase", "")) != "authorize":
            continue
        owner_key = _lifecycle_transaction_key(event)[5]
        if (event.get("bundle_owner_attribution") == "producer_declared"
                and _present_identity(owner_key)) and all(
            _present_identity(value) for value in sample_key(event)
        ):
            authorization_owners.setdefault(sample_key(event), set()).add((
                event.get("bundle_source"), event.get("bundle_owner_cycle_id"),
                event.get("bundle_generation")))

    for event in normalized:
        phase = str(event.get("phase", ""))
        if phase in {"request", "export"}:
            owner_cycle = event.get("bundle_owner_cycle_id")
            if not _present_identity(owner_cycle):
                owner_cycle = event.get("causal_planning_cycle_id")
            event["bundle_owner_cycle_id"] = owner_cycle
            event.setdefault("bundle_owner_attribution", "producer_declared")
            continue
        if (
            phase == "authorize"
            and str(event.get("disposition", "")).upper()
            in _TERMINAL_REJECT_DISPOSITIONS
        ):
            # A rejected authorization is the terminal outcome of the
            # producer request; no bundle was exported and therefore no
            # export-owner witness can exist. Keep its declared causal cycle
            # solely to close that rejected transaction.
            owner_cycle = event.get("bundle_owner_cycle_id")
            if not _present_identity(owner_cycle):
                owner_cycle = event.get("causal_planning_cycle_id")
            event["bundle_owner_cycle_id"] = owner_cycle
            event.setdefault("bundle_owner_attribution", "producer_declared")
            continue
        if phase not in {"authorize", "activate", "publish"}:
            continue
        if (phase in {"authorize", "activate"} and
                _integer(event.get("bundle_source")) in {2, 3} and
                _present_identity(event.get("bundle_generation"))):
            event["bundle_owner_attribution"] = "producer_declared"
            continue
        direct_owner = event.get("bundle_owner_cycle_id")
        if (_present_identity(direct_owner) and
                event.get("bundle_owner_attribution") == "producer_declared"):
            event["bundle_owner_attribution"] = "producer_declared"
            continue
        if phase == "publish":
            exact_authorization_owners = authorization_owners.get(sample_key(event), set())
            if len(exact_authorization_owners) == 1:
                source, cycle, generation = next(iter(exact_authorization_owners))
                event["bundle_source"] = source
                event["bundle_owner_cycle_id"] = cycle
                if event.get("bundle_generation") != generation:
                    event["bundle_owner_attribution"] = "identity_conflict"
                    continue
                event["bundle_owner_attribution"] = "exact_authorization"
                continue
            if len(exact_authorization_owners) > 1:
                event["bundle_owner_cycle_id"] = None
                event["bundle_owner_attribution"] = "ambiguous_authorization"
                continue
        bundle_key = tuple(event.get(field) for field in (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "bundle_generation",
        ))
        owners = export_owners.get(bundle_key, set())
        if len(owners) == 1:
            event["bundle_owner_cycle_id"] = next(iter(owners))
            event["bundle_owner_attribution"] = "resolved_from_export"
        else:
            event["bundle_owner_cycle_id"] = None
            event["bundle_owner_attribution"] = (
                "missing_export" if not owners else "ambiguous_export"
            )
    transactions: dict[tuple[Any, ...], dict[str, Any]] = {}
    conflicts: list[dict[str, Any]] = []
    unresolved: list[dict[str, Any]] = []
    unbound_events: list[dict[str, Any]] = []
    seen: dict[tuple[Any, ...], tuple[Any, ...]] = {}

    for event in normalized:
        phase = str(event.get("phase", ""))
        if phase not in _LIFECYCLE_PHASES:
            continue
        if phase == "request" and not _present_identity(
            event.get("causal_planning_cycle_id")
        ):
            # Legacy goal-topic observations are not planner-cycle requests.
            # Preserve them separately instead of silently dropping the row
            # or attaching it to the nearest subsequent planner cycle.
            unbound_events.append({
                "phase": phase,
                "reason": "REQUEST_CYCLE_UNAVAILABLE",
                "request_boundary": event.get("request_boundary"),
                "request_id": event.get("request_id"),
            })
            continue
        if phase == "activate" and not _present_identity(
            _lifecycle_transaction_key(event)[5]
        ):
            # Without a unique export witness, command-timer activation has no
            # producer cycle and must not be guessed from observer order.
            unbound_events.append({
                "phase": phase,
                "reason": "ACTIVATION_OWNER_UNRESOLVED",
                "request_id": event.get("request_id"),
                "bundle_generation": event.get("bundle_generation"),
            })
            continue
        disposition = str(event.get("disposition", "")).upper()
        tx_key = _lifecycle_transaction_key(event)
        transaction = transactions.setdefault(tx_key, {
            "identity": {
                "runtime_instance_id": tx_key[0],
                "session_id": tx_key[1],
                "localization_epoch": tx_key[2],
                "goal_epoch": tx_key[3],
                "request_id": tx_key[4],
                "causal_planning_cycle_id": (
                    tx_key[5] if not isinstance(tx_key[5], tuple) else None),
                "producer_kind": (
                    "PLANNING_CYCLE" if not isinstance(tx_key[5], tuple)
                    else "NO_EXECUTION_SIGNAL" if tx_key[5][0] == "no_execution"
                    else "TERMINAL_MONITOR" if tx_key[5][0] == "terminal_monitor"
                    else "HEADING_REBIND" if tx_key[5][1] == 2
                    else "EMERGENCY_BRAKE"),
                "producer_id": (
                    tx_key[5] if not isinstance(tx_key[5], tuple)
                    else tx_key[5][1] if tx_key[5][0] == "no_execution"
                    else tx_key[5][1:] if tx_key[5][0] == "terminal_monitor"
                    else tx_key[5][2]),
            },
            "events": {},
            "events_all": {},
            "status": "INCOMPLETE",
            "evidence_outcome": "MISSING_EVIDENCE",
            "reasons": [],
            "terminal_outcome": None,
        })
        owner_attribution = event.get("bundle_owner_attribution")
        if owner_attribution in {"missing_export", "ambiguous_export", "ambiguous_authorization"}:
            transaction["reasons"].append(
                "BUNDLE_OWNER_" + str(owner_attribution).upper()
            )
        event_identity = (
            phase,
            tx_key,
            event.get("bundle_generation"),
            event.get("sample_id"),
            event.get("trace_sequence", event.get("adapter_trace_sequence")),
        )
        signature = _lifecycle_consistency_signature(event)
        if event_identity in seen:
            if seen[event_identity] != signature:
                conflicts.append({
                    "phase": phase,
                    "identity": transaction["identity"],
                    "reason": "CONTRADICTORY_EVENT_PAYLOAD",
                })
                transaction["reasons"].append("CONFLICTING_EVIDENCE")
            else:
                continue
        else:
            seen[event_identity] = signature
        transaction["events"].setdefault(phase, dict(event))
        transaction["events_all"].setdefault(phase, []).append(dict(event))

    valid_reference_ids: set[tuple[Any, ...]] = set()
    producer_owned_reference_ids: set[tuple[Any, ...]] = set()
    producer_authorized_reference_ids: set[tuple[Any, ...]] = set()
    valid_transactions: list[dict[str, Any]] = []
    activation_events = [
        item for item in normalized if str(item.get("phase", "")) == "activate"
    ]
    for transaction in transactions.values():
        identity = transaction["identity"]
        phase_events = transaction["events"]
        if "activate" not in phase_events:
            bundle_generation = phase_events.get("export", {}).get("bundle_generation")
            for candidate in activation_events:
                if (
                    candidate.get("runtime_instance_id") == identity.get("runtime_instance_id")
                    and candidate.get("session_id") == identity.get("session_id")
                    and candidate.get("request_id") == identity.get("request_id")
                    and candidate.get("bundle_generation") == bundle_generation
                    and candidate.get("localization_epoch") == identity.get("localization_epoch")
                    and candidate.get("goal_epoch") == identity.get("goal_epoch")
                ):
                    phase_events["activate"] = dict(candidate)
                    break
        identity_fields = (
            "runtime_instance_id", "session_id", "localization_epoch",
            "request_id", "producer_id",
        ) if identity["producer_kind"] == "NO_EXECUTION_SIGNAL" else (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "producer_id",
        )
        if any(not _present_identity(identity.get(field)) for field in identity_fields):
            transaction["reasons"].append("LIFECYCLE_IDENTITY_MISSING")
        request = phase_events.get("request")
        result = phase_events.get("result")
        retained = phase_events.get("retained")
        supersede = phase_events.get("supersede")
        recovery_retry = phase_events.get("recovery_retry")
        planner_emergency = phase_events.get("planner_emergency_outcome")
        authorize = phase_events.get("authorize")
        export = phase_events.get("export")
        activate = phase_events.get("activate")
        publish = phase_events.get("publish")
        heading_admitted = phase_events.get("heading_admitted")
        producer_kind = identity["producer_kind"]
        if producer_kind == "NO_EXECUTION_SIGNAL":
            if (authorize and authorize.get("disposition") == "REJECTED" and
                    authorize.get("authorization_boundary") ==
                        "execution_timeline_publish_if_current" and
                    _present_identity(authorize.get("authorization_steady_ns")) and
                    authorize.get("goal_epoch") == 0 and
                    authorize.get("bundle_generation") == 0 and
                    authorize.get("sample_id") == identity["producer_id"] and
                    not transaction["reasons"]):
                transaction["status"] = "VALID_REJECT"
                transaction["terminal_outcome"] = "NO_EXECUTION_AUTHORITY"
                transaction["evidence_outcome"] = "INTENTIONALLY_ABSENT"
            else:
                transaction["reasons"].append("NO_EXECUTION_REJECTION_WITNESS_INVALID")
            continue
        if producer_kind == "TERMINAL_MONITOR":
            monitor_generation = retained.get("captured_bundle_generation") if retained else None
            monitor_sequence = retained.get("producer_event_sequence") if retained else None
            disposition = retained.get("disposition_code") if retained else None
            identity_complete = (
                _present_identity(monitor_generation) and
                _present_identity(monitor_sequence) and
                _present_identity(retained.get("state_ingress_sequence")) and
                _present_identity(retained.get("final_state_source_ros_ns")) and
                _present_identity(retained.get("final_state_receive_steady_ns"))) if retained else False
            if not identity_complete:
                transaction["reasons"].append("TERMINAL_MONITOR_CAUSAL_WITNESS_MISSING")
            elif (disposition == 8 and
                  retained.get("owner_snapshot_current") == 1 and
                  retained.get("callback_request_current") == 1 and
                  retained.get("monitor_window_current") == 1 and
                  retained.get("after_command_available") == 1 and
                  retained.get("after_failure_latched") == 0 and
                  retained.get("after_bundle_generation") == monitor_generation and
                  retained.get("final_freshness_reason") == 0 and
                  retained.get("final_witness_age_bounded") == 1 and
                  retained.get("final_body_known_free") == 1 and
                  retained.get("final_anchor_valid") == 1 and
                  not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "CERTIFIED_COMMAND_PRESERVED"
                transaction["evidence_outcome"] = "RESOLVED"
                continue
            elif (disposition in {2, 3} and
                  (retained.get("owner_snapshot_current") == 0 or
                   retained.get("callback_request_current") == 0 or
                   retained.get("monitor_window_current") == 0) and
                  not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "STALE_MONITOR_DISCARDED"
                transaction["evidence_outcome"] = "SUPERSEDED"
                continue
            elif (disposition == 4 and
                  retained.get("after_command_available") == 0 and
                  retained.get("after_failure_latched") == 1 and
                  not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "MONITOR_FAIL_CLOSED"
                transaction["evidence_outcome"] = "RESOLVED"
                continue
            elif (disposition == 5 and
                  _present_identity(retained.get("after_bundle_generation")) and
                  retained.get("after_bundle_generation") != monitor_generation and
                  retained.get("callback_request_current") == 1 and
                  retained.get("after_command_available") == 1 and
                  retained.get("after_failure_latched") == 0 and
                  retained.get("final_freshness_reason") == 0 and
                  retained.get("final_witness_age_bounded") == 1 and
                  not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "EMERGENCY_COMMITTED"
                transaction["evidence_outcome"] = "RESOLVED"
                continue
            transaction["reasons"].append("TERMINAL_MONITOR_OUTCOME_UNVERIFIED")
            continue
        if supersede is not None:
            old_generation = supersede.get("bundle_generation")
            replacement_generation = supersede.get("replacement_bundle_generation")
            source_present = heading_admitted is not None or (
                request is not None and result is not None and export is not None)
            revision_cleared = (
                replacement_generation == 0 and
                _integer(supersede.get("admission_goal_epoch")) is not None and
                supersede["admission_goal_epoch"] > identity["goal_epoch"])
            replaced = (_present_identity(replacement_generation) and
                        old_generation != replacement_generation)
            if (source_present and _present_identity(old_generation) and
                    (replaced or revision_cleared) and
                    _integer(supersede.get("current_snapshot_version")) is not None and
                    _integer(supersede.get("previous_snapshot_version")) is not None and
                    supersede["current_snapshot_version"] >
                        supersede["previous_snapshot_version"] and
                    not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "SUPERSEDED_PENDING"
                transaction["evidence_outcome"] = "SUPERSEDED"
                continue
        if (request and result and planner_emergency and export is None and
                result.get("planner_status") == 4 and
                result.get("planner_disposition") == 4 and
                result.get("runtime_admission_attempted") is False and
                planner_emergency.get("disposition_code") == 5 and
                planner_emergency.get("callback_request_current") == 1 and
                planner_emergency.get("after_command_available") == 1 and
                planner_emergency.get("after_failure_latched") == 0 and
                _present_identity(planner_emergency.get("after_bundle_generation")) and
                not transaction["reasons"]):
            transaction["status"] = "VALID_TERMINAL"
            transaction["terminal_outcome"] = "EMERGENCY_COMMITTED"
            transaction["evidence_outcome"] = "RESOLVED"
            continue
        if request and result and recovery_retry and not transaction["reasons"]:
            disposition = recovery_retry.get("disposition")
            if (disposition == "SUPERSEDED" and
                    recovery_retry.get("identity_current") is False):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "STALE_RETRY_DISCARDED"
                transaction["evidence_outcome"] = "SUPERSEDED"
                continue
            if (disposition == "RETRY_SCHEDULED" and
                    recovery_retry.get("identity_current") is True and
                    recovery_retry.get("timeout") is False and
                    recovery_retry.get("after_failed") is False):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "RECOVERY_RETRY_SCHEDULED"
                transaction["evidence_outcome"] = "INTENTIONALLY_ABSENT"
                continue
            if (disposition == "FAIL_CLOSED" and
                    recovery_retry.get("identity_current") is True and
                    recovery_retry.get("timeout") is True and
                    recovery_retry.get("after_failed") is True):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "RECOVERY_TIMEOUT_FAIL_CLOSED"
                transaction["evidence_outcome"] = "RESOLVED"
                continue
        request_disposition = str(request.get("disposition", "")).upper() if request else ""
        if request_disposition in _TERMINAL_REJECT_DISPOSITIONS:
            transaction["terminal_outcome"] = request_disposition
            transaction["status"] = "VALID_REJECT" if not transaction["reasons"] else "CONFLICTING"
            transaction["evidence_outcome"] = (
                "CONFLICTING_EVIDENCE" if transaction["reasons"] else
                "SUPERSEDED" if request_disposition == "SUPERSEDED" else
                "INTENTIONALLY_ABSENT"
            )
            continue
        if authorize and str(authorize.get("disposition", "")).upper() in _TERMINAL_REJECT_DISPOSITIONS:
            transaction["terminal_outcome"] = str(authorize.get("disposition")).upper()
            transaction["status"] = "VALID_REJECT" if not transaction["reasons"] else "CONFLICTING"
            transaction["evidence_outcome"] = (
                "CONFLICTING_EVIDENCE" if transaction["reasons"] else
                "SUPERSEDED" if transaction["terminal_outcome"] == "SUPERSEDED" else
                "INTENTIONALLY_ABSENT"
            )
            continue
        if request and result and retained and export is None:
            # A replacement solve can deliberately produce no new bundle.
            # Resolve only from the actual retained-decision producer event,
            # with the exact desired solve cycle and the exact incumbent
            # disposition. Planner classification alone is not a validator.
            disposition = retained.get("disposition_code")
            if (result.get("planner_disposition") in {3, 4}
                    and result.get("runtime_admission_attempted") is False
                    and disposition in {6, 7, 8}
                    and retained.get("after_command_available") == 1
                    and retained.get("after_failure_latched") == 0
                    and retained.get("owner_snapshot_current") == 1
                    and retained.get("callback_request_current") == 1
                    and not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "RETAINED_INCUMBENT"
                transaction["evidence_outcome"] = "INTENTIONALLY_ABSENT"
                continue
            if disposition in {2, 3} and not transaction["reasons"]:
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = (
                    "STALE_DISCARDED" if disposition == 2 else "SUPERSEDED"
                )
                transaction["evidence_outcome"] = transaction["terminal_outcome"]
                continue
            if (disposition == 4 and retained.get("owner_snapshot_current") == 1
                    and retained.get("callback_request_current") == 1
                    and retained.get("after_command_available") == 0
                    and retained.get("after_failure_latched") == 1
                    and not transaction["reasons"]):
                transaction["status"] = "VALID_TERMINAL"
                transaction["terminal_outcome"] = "FAIL_CLOSED"
                transaction["evidence_outcome"] = "RESOLVED"
                continue
        required = ({
            "heading_admitted": heading_admitted,
            "activate": activate,
            "authorize": authorize,
            "publish": publish,
        } if producer_kind == "HEADING_REBIND" else {
            "retained": retained,
            "authorize": authorize,
            "publish": publish,
        } if producer_kind == "EMERGENCY_BRAKE" else {
            "request": request,
            "authorize": authorize,
            "export": export,
            "activate": activate,
            "publish": publish,
        })
        missing = [phase for phase, value in required.items() if value is None]
        if missing:
            transaction["reasons"].extend(
                f"LIFECYCLE_PHASE_INCOMPLETE:{phase}" for phase in missing
            )
        for phase, value in required.items():
            if value is not None and str(value.get("disposition", "")).upper() not in (
                _SUCCESS_DISPOSITIONS | {"ADMITTED"}
            ):
                transaction["reasons"].append(f"LIFECYCLE_{phase.upper()}_NOT_SUCCESSFUL")
        if producer_kind == "EMERGENCY_BRAKE" and retained is not None:
            if (retained.get("disposition_code") != 5 or
                    retained.get("after_bundle_generation") != identity["producer_id"] or
                    retained.get("after_command_available") != 1 or
                    retained.get("after_failure_latched") != 0):
                transaction["reasons"].append("EMERGENCY_COMMIT_WITNESS_INVALID")
        if producer_kind == "HEADING_REBIND" and heading_admitted is not None:
            if (heading_admitted.get("bundle_generation") != identity["producer_id"] or
                    not _present_identity(heading_admitted.get("parent_bundle_generation"))):
                transaction["reasons"].append("HEADING_REBIND_ORIGIN_INVALID")
        if export is not None and not _present_identity(export.get("bundle_generation")):
            transaction["reasons"].append("BUNDLE_IDENTITY_MISSING")
        bundle_generations = {
            item.get("bundle_generation")
            for phase in ("authorize", "export", "activate", "publish", "heading_admitted", "retained")
            for item in transaction["events_all"].get(phase, [])
            if _present_identity(item.get("bundle_generation"))
        }
        if not bundle_generations:
            transaction["reasons"].append("BUNDLE_IDENTITY_MISSING")
        elif len(bundle_generations) > 1:
            transaction["reasons"].append("BUNDLE_IDENTITY_CONFLICT")
        if publish is not None:
            if not _present_identity(publish.get("sample_id")):
                transaction["reasons"].append("COMMAND_IDENTITY_MISSING")
            if not _present_identity(publish.get("adapter_trace_sequence", publish.get("trace_sequence"))):
                transaction["reasons"].append("ADAPTER_TRACE_SEQUENCE_MISSING")
            if not publish.get("setpoint_kind", publish.get("setpoint_boundary")):
                transaction["reasons"].append("SETPOINT_KIND_MISSING")
        authorize_events = transaction["events_all"].get("authorize", [])
        publish_events = transaction["events_all"].get("publish", [])
        valid_publishes: list[dict[str, Any]] = []
        for publish_event in publish_events:
            matching_authorizations = [
                item for item in authorize_events
                if item.get("sample_id") == publish_event.get("sample_id")
                and item.get("bundle_generation") == publish_event.get("bundle_generation")
                and str(item.get("disposition", "")).upper() == "AUTHORIZED"
                and item.get("authorization_boundary") ==
                    "execution_timeline_publish_if_current"
                and _present_identity(item.get("authorization_steady_ns"))
            ]
            if not matching_authorizations:
                transaction["reasons"].append("COMMAND_AUTHORIZATION_LINEAGE_MISSING")
                continue
            publish_world = (
                publish_event.get("world_generation"),
                publish_event.get("world_revision"),
                publish_event.get("world_observation_stamp_ns"),
            )
            if any(not _present_identity(value) for value in publish_world):
                transaction["reasons"].append("COMMAND_WORLD_IDENTITY_MISSING")
                continue
            if not any(
                publish_world == (
                    item.get("world_generation"),
                    item.get("world_revision"),
                    item.get("world_observation_stamp_ns"),
                )
                for item in matching_authorizations
            ):
                transaction["reasons"].append("COMMAND_WORLD_IDENTITY_MISMATCH")
                continue
            valid_publishes.append(publish_event)
        if not transaction["reasons"]:
            transaction["status"] = "VALID"
            transaction["evidence_outcome"] = "RESOLVED"
            valid_transactions.append(transaction)
            for authorization in authorize_events:
                if (str(authorization.get("disposition", "")).upper() ==
                        "AUTHORIZED" and
                        authorization.get("bundle_owner_attribution") ==
                            "producer_declared" and
                        all(_present_identity(authorization.get(field)) for field in (
                            "sample_id", "bundle_generation", "world_generation",
                            "world_revision", "world_observation_stamp_ns",
                        ))):
                    producer_authorized_reference_ids.add((
                        identity.get("runtime_instance_id"),
                        identity.get("session_id"),
                        identity.get("localization_epoch"),
                        identity.get("goal_epoch"),
                        identity.get("request_id"),
                        authorization.get("bundle_generation"),
                        authorization.get("sample_id"),
                        authorization.get("world_generation"),
                        authorization.get("world_revision"),
                        authorization.get("world_observation_stamp_ns"),
                    ))
            for publish_event in valid_publishes:
                effective_generation = (
                    export.get("bundle_generation") if export else
                    identity["producer_id"])
                valid_reference_ids.add((
                    identity.get("request_id"),
                    effective_generation,
                    publish_event.get("sample_id"),
                ))
                origin_confirmed = (
                    activate and activate.get("bundle_owner_attribution") ==
                        "producer_declared" and
                    (export is not None or heading_admitted is not None)
                ) or (producer_kind == "EMERGENCY_BRAKE" and retained is not None)
                if (authorize and origin_confirmed and
                        authorize.get("bundle_owner_attribution") == "producer_declared"
                        and publish_event.get("bundle_owner_attribution") ==
                            "exact_authorization"):
                    producer_owned_reference_ids.add((
                        identity.get("runtime_instance_id"),
                        identity.get("session_id"),
                        identity.get("localization_epoch"),
                        identity.get("goal_epoch"),
                        identity.get("request_id"),
                        effective_generation,
                        publish_event.get("sample_id"),
                        publish_event.get("world_generation"),
                        publish_event.get("world_revision"),
                        publish_event.get("world_observation_stamp_ns"),
                    ))
        else:
            transaction["evidence_outcome"] = (
                "CONFLICTING_EVIDENCE" if "CONFLICTING_EVIDENCE" in transaction["reasons"]
                else "MISSING_EVIDENCE"
            )
            unresolved.append(transaction)

    # Early terminal branches must not bypass unresolved accounting. Derive
    # this from final transaction status instead of loop fallthrough.
    unresolved = [
        transaction for transaction in transactions.values()
        if transaction["status"] not in {"VALID", "VALID_TERMINAL", "VALID_REJECT"}
    ]
    critical_unbound = [item for item in unbound_events if item["phase"] == "activate"]
    return {
        "status": (
            "CONFLICTING" if conflicts else
            "INCOMPLETE" if unresolved or critical_unbound else
            "VALID" if valid_transactions else
            "INCOMPLETE"
        ),
        "transactions": sorted(
            transactions.values(),
            key=lambda item: tuple(str(item["identity"].get(field) or "") for field in (
                "request_id", "causal_planning_cycle_id", "goal_epoch",
            )),
        ),
        "valid_transaction_count": len(valid_transactions),
        "valid_reference_ids": [list(item) for item in sorted(valid_reference_ids, key=str)],
        "producer_owned_reference_ids": [
            list(item) for item in sorted(producer_owned_reference_ids, key=str)
        ],
        "producer_authorized_reference_ids": [
            list(item) for item in sorted(producer_authorized_reference_ids, key=str)
        ],
        "conflicts": conflicts,
        "unresolved": unresolved,
        "unbound_events": unbound_events,
        "reasons": sorted(set(
            reason for item in transactions.values() for reason in item["reasons"]
        ) | ({"CONFLICTING_EVIDENCE"} if conflicts else set())
          | {item["reason"] for item in critical_unbound}),
        "order_independent": True,
    }


def _read_jsonl(
    path: Path,
    *,
    issues: list[str] | None = None,
    label: str,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
            except ValueError:
                if issues is not None and line.strip():
                    issues.append(f"MALFORMED_JSONL:{label}:{line_number}")
                continue
            if isinstance(value, dict):
                value.setdefault("_line_number", line_number)
                rows.append(value)
            elif issues is not None:
                issues.append(f"NON_OBJECT_JSONL:{label}:{line_number}")
    return rows


def _mission_waypoints(session: Path, scenario: dict[str, Any]) -> list[tuple[float, float, float]]:
    candidates = [session / "resolved_mission.yaml", session / "resolved_mission.yml"]
    configured = scenario.get("mission_file")
    if configured:
        configured_path = Path(str(configured))
        candidates.append(
            configured_path if configured_path.is_absolute() else session / configured_path
        )
    for path in candidates:
        try:
            import yaml
            value = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (ImportError, OSError, ValueError):
            continue
        raw = value.get("mission", {}).get("waypoints", []) if isinstance(value, dict) else []
        if isinstance(raw, list):
            result = []
            for item in raw:
                point = _vector(item.get("position")) if isinstance(item, dict) else None
                if point is not None:
                    result.append(tuple(float(v) for v in point))
            if result:
                return result
    return []


def _normalize_sample(row: dict[str, Any]) -> dict[str, Any] | None:
    payload = row.get("payload")
    if not isinstance(payload, dict):
        return None
    stamp_ns, time_basis = _source_stamp_ns(row, payload)
    if stamp_ns is None:
        return None
    result = dict(payload)
    result.update({
        "source_stamp_ns": stamp_ns,
        "time_basis": time_basis,
        "source_clock": payload.get("source_clock") or "unknown",
        "arrival_wall_ns": row.get("arrival_wall_ns"),
        "arrival_steady_ns": row.get("arrival_steady_ns"),
        "record_sequence": row.get("record_sequence"),
        "stream": row.get("stream"),
        "accepted_by_monitor": row.get("accepted_by_monitor") is True,
    })
    return result


def _normalize_pva(event: dict[str, Any], session_id: str) -> dict[str, Any] | None:
    payload = event.get("payload")
    if not isinstance(payload, dict):
        return None
    stamp_ns, time_basis = _source_stamp_ns(event, payload)
    if stamp_ns is None:
        return None
    result = dict(payload)
    result.update({
        "source_stamp_ns": stamp_ns,
        "time_basis": time_basis,
        "source_clock": payload.get("source_clock") or "unknown",
        "record_sim_time_ns": event.get("sim_time_ns"),
        "observer_record_steady_ns": event.get("observer_record_steady_ns"),
        "record_sequence": event.get("record_sequence"),
        "session_id": session_id,
    })
    return result


def _writer_integrity(
    label: str,
    value: dict[str, Any],
    *,
    require_terminal: bool,
) -> list[str]:
    reasons: list[str] = []
    if "capture_complete" not in value:
        reasons.append("CAPTURE_NOT_FINALIZED")
    elif value.get("capture_complete") is not True:
        reasons.append("CAPTURE_NOT_FINALIZED")
    counter_fields = (
        "submitted_records", "accepted_records", "written_records",
        "pending_records", "dropped_records", "snapshot_rejected_records",
        "queue_full_drop_records", "closed_rejected_records",
        "serialization_error_count", "write_error_count",
    )
    counters: dict[str, int] = {}
    for field in counter_fields:
        raw = value.get(field)
        parsed = None if isinstance(raw, bool) else _integer(raw)
        if parsed is None or parsed < 0:
            reasons.append("WRITER_COUNTER_INVALID")
        else:
            counters[field] = parsed
    for field, reason in (
        ("snapshot_rejected_records", "SNAPSHOT_REJECTED"),
        ("queue_full_drop_records", "QUEUE_FULL_DROP"),
        ("closed_rejected_records", "CLOSED_REJECTED"),
        ("serialization_error_count", "SERIALIZATION_ERROR"),
        ("write_error_count", "WRITE_ERROR"),
    ):
        if counters.get(field, 0) > 0:
            reasons.append(reason)
    if counters.get("dropped_records", 0) > 0:
        reasons.append("EVIDENCE_DROP")
    if all(field in counters for field in counter_fields):
        submitted = counters["submitted_records"]
        accepted = counters["accepted_records"]
        written = counters["written_records"]
        dropped = counters["dropped_records"]
        pending = counters["pending_records"]
        if submitted != accepted + dropped:
            reasons.append("WRITER_SUBMISSION_ACCOUNTING_MISMATCH")
        if accepted != written or pending != max(0, accepted - written):
            reasons.append("WRITER_ACCOUNTING_MISMATCH")
    category_counters = value.get("records_by_category")
    if not isinstance(category_counters, dict):
        reasons.append("WRITER_CATEGORY_ACCOUNTING_INVALID")
    else:
        category_totals = {
            field: 0 for field in (
                "submitted_records", "accepted_records", "written_records",
                "dropped_records",
            )
        }
        for counters in category_counters.values():
            if not isinstance(counters, dict):
                reasons.append("WRITER_CATEGORY_ACCOUNTING_INVALID")
                continue
            parsed_counters: dict[str, int] = {}
            for field in category_totals:
                raw = counters.get(field)
                parsed = None if isinstance(raw, bool) else _integer(raw)
                if parsed is None or parsed < 0:
                    reasons.append("WRITER_CATEGORY_ACCOUNTING_INVALID")
                    break
                parsed_counters[field] = parsed
            if len(parsed_counters) != len(category_totals):
                continue
            submitted = parsed_counters["submitted_records"]
            accepted = parsed_counters["accepted_records"]
            written = parsed_counters["written_records"]
            dropped = parsed_counters["dropped_records"]
            if submitted != accepted + dropped or accepted != written:
                reasons.append("WRITER_CATEGORY_ACCOUNTING_MISMATCH")
            for field, count in parsed_counters.items():
                category_totals[field] += count
        if all(field in counters for field in counter_fields) and (
            category_totals["submitted_records"] != counters["submitted_records"]
            or category_totals["accepted_records"] != counters["accepted_records"]
            or category_totals["written_records"] != counters["written_records"]
            or category_totals["dropped_records"] != counters["dropped_records"]
        ):
            reasons.append("WRITER_CATEGORY_TOTAL_MISMATCH")
    if require_terminal and not value.get("terminal_marker"):
        reasons.append("TERMINAL_MARKER_MISSING")
    return [f"{label.upper()}_{reason}" for reason in sorted(set(reasons))]


def _has_capture_terminal_marker(scenario: dict[str, Any], events: list[dict[str, Any]]) -> bool:
    # A summary can be partially written or copied from another run. Require
    # the terminal event from the append-only recorder stream.
    terminal_names = {
        "mission_complete_observed", "external_mode_completed",
        "expected_fail_closed", "mission_aborted", "mission_cancelled",
        "scenario_finished", "capture_complete",
    }
    return any(
        event.get("kind") == "event"
        and isinstance(event.get("payload"), dict)
        and str(event["payload"].get("name", "")) in terminal_names
        for event in events
    )


def _evidence_category(record: dict[str, Any]) -> str:
    kind = record.get("kind")
    stream = record.get("stream")
    if isinstance(kind, str) and kind:
        if kind == "sample" and isinstance(stream, str) and stream:
            return f"sample:{stream}"
        return kind
    return "__unknown__"


def _artifact_count_reasons(
    label: str,
    writer: dict[str, Any],
    records: list[dict[str, Any]],
) -> list[str]:
    reasons: list[str] = []
    written = _integer(writer.get("written_records"))
    if written is not None and written != len(records):
        reasons.append(f"{label}_WRITER_ARTIFACT_COUNT_MISMATCH")
    categories = writer.get("records_by_category")
    if not isinstance(categories, dict):
        return reasons
    observed: dict[str, int] = {}
    for record in records:
        category = _evidence_category(record)
        observed[category] = observed.get(category, 0) + 1
    declared: dict[str, int] = {}
    for category, counters in categories.items():
        if not isinstance(counters, dict):
            continue
        count = _integer(counters.get("written_records"))
        if count is not None:
            declared[str(category)] = count
    if observed != declared:
        reasons.append(f"{label}_WRITER_CATEGORY_ARTIFACT_MISMATCH")
    return reasons


def reduce_world_transactions(
    events: Iterable[dict[str, Any]],
    producer_counts: Iterable[dict[str, Any]],
    writer: dict[str, Any],
    *,
    required: bool = False,
    expected_event_kinds: Iterable[str] = (),
    expected_transaction_count: int | None = None,
) -> dict[str, Any]:
    """Reduce producer-declared World evidence by exact identity.

    This is an evidence consistency reducer only. It does not assess or feed
    WorldModel/ExecutionAuthority decisions. No event is associated by nearest
    timestamp; the producer sequence and declared transaction key own joins.
    """
    normalized = [dict(item) for item in events if isinstance(item, dict)]
    counts = [dict(item) for item in producer_counts if isinstance(item, dict)]
    expected_kinds = tuple(str(item) for item in expected_event_kinds)
    by_sequence: dict[tuple[str, int], tuple[Any, ...]] = {}
    conflicts: list[dict[str, Any]] = []
    unresolved: list[dict[str, Any]] = []
    transactions: dict[str, dict[str, Any]] = {}
    for event in normalized:
        sequence = _integer(event.get("producer_event_sequence"))
        runtime_instance = str(event.get("producer_runtime_instance_id") or "")
        kind = str(event.get("event_kind", ""))
        tx_key = str(event.get("transaction_key", ""))
        signature = tuple(event.get(key) for key in (
            "event_kind", "disposition", "transaction_key",
            "prior_world_generation", "prior_world_revision",
            "prior_world_source_stamp_ns", "next_world_generation",
            "next_world_revision", "next_world_source_stamp_ns",
            "before_timeline_version", "after_timeline_version",
            "before_active_generation", "after_active_generation",
            "before_pending_generation", "after_pending_generation",
        ))
        if sequence is None or sequence <= 0 or not runtime_instance or not kind or not tx_key:
            unresolved.append({"producer_event_sequence": sequence,
                              "reason": "WORLD_EVENT_IDENTITY_MISSING"})
            continue
        identity_fields = (
            "before_timeline_version", "after_timeline_version",
            "next_world_localization_epoch", "next_world_generation",
            "next_world_revision", "next_world_source_stamp_ns",
        )
        identity = {field: _integer(event.get(field)) for field in identity_fields}
        if (any(value is None or value <= 0 for field, value in identity.items()
                if field != "next_world_source_stamp_ns") or
                identity["next_world_source_stamp_ns"] is None or
                identity["next_world_source_stamp_ns"] < 0):
            unresolved.append({"producer_event_sequence": sequence,
                               "reason": "WORLD_EVENT_REFERENCE_MISSING"})
        expected_key = ":".join(str(identity[field]) for field in (
            "before_timeline_version", "next_world_localization_epoch",
            "next_world_generation", "next_world_revision",
            "next_world_source_stamp_ns"))
        if tx_key != expected_key:
            conflicts.append({"producer_event_sequence": sequence,
                              "reason": "WORLD_TRANSACTION_KEY_MISMATCH",
                              "transaction_key": tx_key,
                              "expected_transaction_key": expected_key})
        for prefix in ("before", "after"):
            active_generation = _integer(event.get(f"{prefix}_active_generation"))
            pending_generation = _integer(event.get(f"{prefix}_pending_generation"))
            if active_generation is None or active_generation < 0 or \
                    pending_generation is None or pending_generation < 0:
                unresolved.append({"producer_event_sequence": sequence,
                                   "reason": f"WORLD_{prefix.upper()}_EXECUTION_IDENTITY_MISSING"})
                continue
            if active_generation > 0 and any(
                (_integer(event.get(f"{prefix}_active_{field}")) or 0) <= 0
                for field in ("goal_epoch", "request_id")
            ):
                unresolved.append({"producer_event_sequence": sequence,
                                   "reason": f"WORLD_{prefix.upper()}_ACTIVE_IDENTITY_INCOMPLETE"})
            if pending_generation > 0 and any(
                (_integer(event.get(f"{prefix}_pending_{field}")) or 0) <= 0
                for field in ("goal_epoch", "request_id")
            ):
                unresolved.append({"producer_event_sequence": sequence,
                                   "reason": f"WORLD_{prefix.upper()}_PENDING_IDENTITY_INCOMPLETE"})
        sequence_key = (runtime_instance, sequence)
        previous = by_sequence.get(sequence_key)
        if previous is not None:
            if previous != signature:
                conflicts.append({"producer_event_sequence": sequence,
                                  "reason": "WORLD_EVENT_SEQUENCE_CONFLICT"})
            continue
        by_sequence[sequence_key] = signature
        item = transactions.setdefault(tx_key, {
            "transaction_key": tx_key,
            "events": [],
            "status": "RESOLVED",
        })
        item["events"].append(kind)
        if kind == "WORLD_PUBLICATION_SUPERSEDED":
            item["status"] = "SUPERSEDED"
        elif kind == "WORLD_COMMAND_SUSPENDED":
            item["status"] = "SUSPENDED"
        elif kind == "WORLD_COMMAND_RECERTIFIED":
            item["status"] = "RECERTIFIED"
        elif kind == "WORLD_COMMAND_RESUMED":
            item["status"] = "RESUMED"
        elif kind == "WORLD_PUBLICATION_FAILED" or kind == "WORLD_RECERTIFICATION_REJECTED":
            item["status"] = "INVALIDATED"
        elif kind == "WORLD_PUBLICATION_COMMITTED":
            item["status"] = (
                "INVALIDATED" if "INVALIDATED" in str(event.get("disposition", ""))
                else "RESOLVED"
            )

    sequences_by_runtime: dict[str, list[int]] = {}
    for runtime_instance, sequence in by_sequence:
        sequences_by_runtime.setdefault(runtime_instance, []).append(sequence)
    sequence_gaps: list[dict[str, Any]] = []
    for runtime_instance, sequences in sequences_by_runtime.items():
        ordered = sorted(sequences)
        sequence_gaps.extend({
            "runtime_instance_id": runtime_instance,
            "missing_sequences": [left + 1, right - 1],
        } for left, right in zip(ordered, ordered[1:]) if right > left + 1)
    counter_values_by_runtime: dict[str, list[int]] = {}
    for item in counts:
        runtime_instance = str(item.get("producer_runtime_instance_id") or "")
        value = _integer(item.get("events_produced"))
        if not runtime_instance or value is None or value < 0:
            unresolved.append({"reason": "WORLD_PRODUCER_COUNTER_IDENTITY_MISSING"})
            continue
        counter_values_by_runtime.setdefault(runtime_instance, []).append(value)
    for runtime_instance, values in counter_values_by_runtime.items():
        if any(right < left for left, right in zip(values, values[1:])):
            unresolved.append({"reason": "WORLD_PRODUCER_COUNTER_REGRESSION",
                               "runtime_instance_id": runtime_instance})
    producer_runtime_ids = set(sequences_by_runtime) | set(counter_values_by_runtime)
    produced_by_runtime = {
        runtime: max(values) for runtime, values in counter_values_by_runtime.items()
    }
    produced_total = max(produced_by_runtime.values(), default=None)
    all_observed_sequences = [sequence for values in sequences_by_runtime.values()
                              for sequence in values]
    first_observed_sequence = min(all_observed_sequences, default=None)
    last_observed_sequence = max(all_observed_sequences, default=None)
    category = writer.get("records_by_category", {}).get("world_transaction", {})
    count_category = writer.get("records_by_category", {}).get(
        "world_transaction_producer_count", {})
    written = _integer(category.get("written_records"))
    submitted = _integer(category.get("submitted_records"))
    dropped = _integer(category.get("dropped_records"))
    writer_errors = (_integer(writer.get("serialization_error_count")) or 0) + (
        _integer(writer.get("write_error_count")) or 0)
    expected_missing = sorted(set(expected_kinds) - {
        str(event.get("event_kind", "")) for event in normalized
    })
    reference_missing = sum(
        not all((_integer(event.get(field)) or 0) > 0 for field in (
            "next_world_localization_epoch", "next_world_generation",
            "next_world_revision", "next_world_source_stamp_ns",
        ))
        for event in normalized
    )
    reference_conflicts = sum(
        item.get("reason") in {
            "WORLD_TRANSACTION_KEY_MISMATCH", "WORLD_EVENT_SEQUENCE_CONFLICT",
            "WORLD_EVENT_AHEAD_OF_PRODUCER_COUNT",
        }
        for item in conflicts
    )
    if sequence_gaps:
        unresolved.extend({"reason": "WORLD_PRODUCER_SEQUENCE_GAP", **gap}
                          for gap in sequence_gaps)
    if written is not None and written != len(normalized):
        unresolved.append({"reason": "WORLD_WRITER_ARTIFACT_COUNT_MISMATCH"})
    if dropped:
        unresolved.append({"reason": "WORLD_EVIDENCE_DROPPED", "count": dropped})
    if writer_errors:
        unresolved.append({"reason": "WORLD_EVIDENCE_WRITER_ERROR", "count": writer_errors})
    count_written = _integer(count_category.get("written_records"))
    count_dropped = _integer(count_category.get("dropped_records"))
    if count_written is not None and count_written != len(counts):
        unresolved.append({"reason": "WORLD_PRODUCER_COUNT_ARTIFACT_MISMATCH"})
    if count_dropped:
        unresolved.append({"reason": "WORLD_PRODUCER_COUNT_DROPPED",
                           "count": count_dropped})
    if required:
        if not expected_kinds:
            unresolved.append({"reason": "WORLD_EXPECTED_EVENT_KINDS_MISSING"})
        unresolved.extend({"reason": "REQUIRED_WORLD_EVENT_MISSING", "event_kind": kind}
                          for kind in expected_missing)
        if produced_total is None or not counts:
            unresolved.append({"reason": "WORLD_PRODUCER_COUNT_WITNESS_MISSING"})
        if len(producer_runtime_ids) != 1:
            unresolved.append({"reason": "WORLD_RUNTIME_INSTANCE_CHANGED_OR_MISSING",
                               "runtime_instance_ids": sorted(producer_runtime_ids)})
        for runtime_instance, produced in produced_by_runtime.items():
            observed = sorted(sequences_by_runtime.get(runtime_instance, []))
            if observed and observed[-1] < produced:
                unresolved.append({"reason": "WORLD_EVENT_TAIL_MISSING",
                                   "runtime_instance_id": runtime_instance,
                                   "last_event_sequence": observed[-1],
                                   "producer_count": produced})
            elif observed and produced < observed[-1]:
                conflicts.append({"reason": "WORLD_EVENT_AHEAD_OF_PRODUCER_COUNT",
                                  "runtime_instance_id": runtime_instance,
                                  "last_event_sequence": observed[-1],
                                  "producer_count": produced})
        if expected_transaction_count is None or expected_transaction_count < 0:
            unresolved.append({"reason": "WORLD_EXPECTED_TRANSACTION_COUNT_MISSING"})
        elif len(transactions) < expected_transaction_count:
            unresolved.append({"reason": "WORLD_REQUIRED_TRANSACTION_MISSING",
                               "expected": expected_transaction_count,
                               "observed": len(transactions)})
    event_written = _integer(category.get("written_records"))
    event_submitted = _integer(category.get("submitted_records"))
    event_accepted = _integer(category.get("accepted_records"))
    event_dropped = _integer(category.get("dropped_records"))
    count_written = _integer(count_category.get("written_records"))
    count_submitted = _integer(count_category.get("submitted_records"))
    count_accepted = _integer(count_category.get("accepted_records"))
    count_dropped = _integer(count_category.get("dropped_records"))
    if required:
        if any(value is None for value in (
            event_written, event_submitted, event_accepted, event_dropped,
            count_written, count_submitted, count_accepted, count_dropped,
        )):
            unresolved.append({"reason": "WORLD_WRITER_CATEGORY_ACCOUNTING_MISSING"})
        if event_written is not None and event_written != len(normalized):
            unresolved.append({"reason": "WORLD_WRITER_ARTIFACT_COUNT_MISMATCH"})
        if count_written is not None and count_written != len(counts):
            unresolved.append({"reason": "WORLD_PRODUCER_COUNT_ARTIFACT_MISMATCH"})
        for label, submitted_value, accepted_value, written_value, dropped_value in (
            ("EVENT", event_submitted, event_accepted, event_written, event_dropped),
            ("COUNT", count_submitted, count_accepted, count_written, count_dropped),
        ):
            if None not in (submitted_value, accepted_value, written_value, dropped_value) and (
                submitted_value != accepted_value + dropped_value or
                accepted_value != written_value
            ):
                unresolved.append({"reason": f"WORLD_{label}_WRITER_ACCOUNTING_MISMATCH"})
    return {
        "required": bool(required),
        "status": "CONFLICTING" if conflicts else
                  "INCOMPLETE" if required and unresolved else "RESOLVED",
        "required_world_transactions": (
            expected_transaction_count if required and
            expected_transaction_count is not None else 0),
        "resolved_world_transactions": len(transactions),
        "required_world_unresolved": len(unresolved) if required else 0,
        "required_world_conflicts": len(conflicts) if required else 0,
        "required_world_reference_missing": reference_missing if required else 0,
        "required_world_reference_conflicts": reference_conflicts if required else 0,
        "required_world_events_produced": produced_total,
        "required_world_events_observed": len(normalized),
        "first_observed_producer_sequence": first_observed_sequence,
        "last_observed_producer_sequence": last_observed_sequence,
        "required_world_events_submitted": submitted,
        "required_world_events_written": written,
        "required_world_events_dropped": dropped,
        "required_world_events_serialization_failures":
            _integer(writer.get("serialization_error_count")) or 0,
        "required_world_events_writer_failures":
            _integer(writer.get("write_error_count")) or 0,
        "sequence_gaps": sequence_gaps,
        "transactions": list(transactions.values()),
        "unresolved": unresolved,
        "conflicts": conflicts,
    }


def load_evaluation_inputs(
    session: Path,
    config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load raw session-owned sources without using display-decimated data."""
    session = Path(session)
    scenario = _load(session / "scenario.json", {})
    if not isinstance(scenario, dict):
        scenario = {}
    metadata = _load(session / "metadata.json", {})
    if not isinstance(metadata, dict):
        metadata = {}
    monitor = _load(session / "monitor.json", {})
    if not isinstance(monitor, dict):
        monitor = {}
    read_issues: list[str] = []
    scenario_events = _read_jsonl(
        session / "scenario.jsonl", issues=read_issues, label="scenario"
    )
    read_issues.extend(join_execution_diagnostics(scenario_events))
    sample_rows = _read_jsonl(
        session / "samples.jsonl", issues=read_issues, label="monitor"
    )
    session_id = str(metadata.get("run_id") or metadata.get("session_id") or session.name)
    pva_event_count = sum(
        event.get("kind") == "pva_command" for event in scenario_events
    )
    pva = [
        item for event in scenario_events
        if event.get("kind") == "pva_command"
        for item in [_normalize_pva(event, session_id)]
        if item is not None
    ]
    streams: dict[str, list[dict[str, Any]]] = {}
    monitor_sample_counts: dict[str, dict[str, int]] = {}
    for row in sample_rows:
        if row.get("kind") != "sample":
            continue
        stream = str(row.get("stream") or "unknown")
        counters = monitor_sample_counts.setdefault(stream, {
            "recorded_rows": 0,
            "monitor_accepted_rows": 0,
            "monitor_rejected_rows": 0,
            "normalized_rows": 0,
            "normalization_rejected_rows": 0,
        })
        counters["recorded_rows"] += 1
        if row.get("accepted_by_monitor") is not True:
            counters["monitor_rejected_rows"] += 1
            if "accepted_by_monitor" not in row:
                counters.setdefault("acceptance_witness_missing_rows", 0)
                counters["acceptance_witness_missing_rows"] += 1
            continue
        counters["monitor_accepted_rows"] += 1
        item = _normalize_sample(row)
        if item is None:
            counters["normalization_rejected_rows"] += 1
            continue
        counters["normalized_rows"] += 1
        streams.setdefault(stream, []).append(item)
    # Preserve recorder order. Metric helpers may sort local copies, while the
    # source-time validator must still be able to detect duplicate or regressed
    # producer timestamps instead of having load-time sorting hide them.
    lifecycle = [
        event.get("payload", {})
        for event in scenario_events
        if event.get("kind") == "lifecycle"
        and isinstance(event.get("payload"), dict)
    ]
    px4_input_trace = [
        event.get("payload", {})
        for event in scenario_events
        if event.get("kind") == "px4_input_trace"
        and isinstance(event.get("payload"), dict)
    ]
    world_transactions = [
        event.get("payload", {}) for event in scenario_events
        if event.get("kind") == "world_transaction"
        and isinstance(event.get("payload"), dict)
    ]
    world_producer_counts = [
        event.get("payload", {}) for event in scenario_events
        if event.get("kind") == "world_transaction_producer_count"
        and isinstance(event.get("payload"), dict)
    ]
    actual_config = dict(config) if isinstance(config, dict) else {}
    # The report builder supplies its runtime/evaluation config, which can be
    # non-empty while omitting the per-run scenario overlay.  Always merge the
    # immutable session manifest so run-specific evidence requirements (for
    # example required World transactions) are not silently lost.
    try:
        import yaml
        raw = yaml.safe_load((session / "scenario_config.yaml").read_text(encoding="utf-8"))
        session_config = raw if isinstance(raw, dict) else {}
    except (ImportError, OSError, ValueError):
        session_config = {}
    if session_config:
        merged_config = dict(actual_config)
        configured_scenario = actual_config.get("scenario", {})
        session_scenario = session_config.get("scenario", {})
        merged_scenario = dict(configured_scenario) if isinstance(configured_scenario, dict) else {}
        if isinstance(session_scenario, dict):
            merged_scenario.update(session_scenario)
        merged_config["scenario"] = merged_scenario
        actual_config = merged_config
    configured_scenario = actual_config.get("scenario", {}) if isinstance(actual_config, dict) else {}
    if not isinstance(configured_scenario, dict):
        configured_scenario = {}
    evaluation_config = actual_config.get("evaluation", {}) if isinstance(actual_config, dict) else {}
    if not isinstance(evaluation_config, dict):
        evaluation_config = {}
    waypoint_source = dict(configured_scenario)
    waypoint_source.update({key: value for key, value in scenario.items() if value is not None})
    tracking_coverage_policy = scenario.get(
        "tracking_coverage_policy",
        configured_scenario.get("tracking_coverage_policy", evaluation_config.get("tracking_coverage_policy")),
    )
    tracking_acceptance_policy = scenario.get(
        "tracking_acceptance_policy",
        configured_scenario.get(
            "tracking_acceptance_policy",
            evaluation_config.get("tracking_acceptance_policy"),
        ),
    )
    evaluation_window = scenario.get(
        "evaluation_window",
        configured_scenario.get("evaluation_window", evaluation_config.get("evaluation_window")),
    )
    writer = scenario.get("evidence_writer", {})
    if not isinstance(writer, dict):
        writer = {}
    monitor_writer = monitor.get("evidence_writer", {})
    if not isinstance(monitor_writer, dict):
        monitor_writer = {}
    lifecycle_reduction = reduce_lifecycle(lifecycle, px4_input_trace)
    world_required = bool(
        scenario.get("world_evidence_required", False) or
        configured_scenario.get("world_evidence_required", False)
    )
    expected_world_kinds = scenario.get(
        "required_world_event_kinds",
        configured_scenario.get("required_world_event_kinds", ()),
    )
    if not isinstance(expected_world_kinds, (list, tuple)):
        expected_world_kinds = ()
    world_transaction_reduction = reduce_world_transactions(
        world_transactions, world_producer_counts, writer,
        required=world_required, expected_event_kinds=expected_world_kinds,
        expected_transaction_count=_integer(scenario.get(
            "required_world_transactions",
            configured_scenario.get("required_world_transactions"))),
    )
    completeness_reasons: list[str] = list(read_issues)
    if not (session / "scenario.jsonl").is_file():
        completeness_reasons.append("scenario event stream is missing")
    if not (session / "samples.jsonl").is_file():
        completeness_reasons.append("monitor sample stream is missing")
    if len(pva) != pva_event_count:
        completeness_reasons.append("PVA_NORMALIZATION_REJECTED")
    for stream in ("ground_truth_odometry",):
        counters = monitor_sample_counts.get(stream, {})
        if int(counters.get("monitor_rejected_rows", 0)) > 0:
            completeness_reasons.append(
                f"MONITOR_REJECTED_REQUIRED_SAMPLE:{stream}"
            )
        if int(counters.get("normalization_rejected_rows", 0)) > 0:
            completeness_reasons.append(
                f"MONITOR_UNUSABLE_REQUIRED_SAMPLE:{stream}"
            )
        if int(counters.get("acceptance_witness_missing_rows", 0)) > 0:
            completeness_reasons.append(
                f"MONITOR_ACCEPTANCE_WITNESS_MISSING:{stream}"
            )
        monitor_stream = monitor.get("streams", {}).get(stream, {})
        if isinstance(monitor_stream, dict):
            for field, reason in (
                ("timestamp_duplicate_count", "SOURCE_TIMESTAMP_DUPLICATE"),
                ("timestamp_regression_count", "SOURCE_TIMESTAMP_REGRESSION"),
                ("timestamp_epoch_discard_count", "SOURCE_TIMESTAMP_EPOCH_DISCARD"),
                ("invalid_source_timestamp_count", "SOURCE_TIMESTAMP_INVALID"),
            ):
                if int(_number(monitor_stream.get(field)) or 0) > 0:
                    completeness_reasons.append(f"{reason}:{stream}")
    for event in scenario_events:
        if event.get("kind") != "evidence_gap" or not isinstance(event.get("payload"), dict):
            continue
        reason = str(event["payload"].get("reason") or "unspecified")
        completeness_reasons.append(f"EVIDENCE_GAP:{reason}")
    if lifecycle_reduction["status"] == "CONFLICTING":
        completeness_reasons.append("CONFLICTING_EVIDENCE")
    if lifecycle_reduction["status"] == "INCOMPLETE":
        completeness_reasons.append("LIFECYCLE_ATTRIBUTION_INCOMPLETE")
    if any(
        int(_number(item.get("trace_drop_count")) or 0) > 0
        for item in px4_input_trace
    ):
        completeness_reasons.append("PX4_INPUT_TRACE_QUEUE_DROP")
    if any(
        int(_number(item.get("trace_publish_error_count")) or 0) > 0
        for item in px4_input_trace
    ):
        completeness_reasons.append("PX4_INPUT_TRACE_PUBLISH_ERROR")
    required_trace_counters = (
        "trace_enqueued_count", "trace_published_before_count",
        "trace_drop_count", "trace_publish_error_count",
    )
    if px4_input_trace and any(
        _integer(item.get(field)) is None
        for item in px4_input_trace for field in required_trace_counters
    ):
        completeness_reasons.append("PX4_INPUT_TRACE_ACCOUNTING_MISSING")
    elif px4_input_trace:
        maximum_enqueued = max(
            int(item["trace_enqueued_count"]) for item in px4_input_trace
        )
        maximum_published = max(
            int(item["trace_published_before_count"]) + 1
            for item in px4_input_trace
        )
        if maximum_enqueued != maximum_published:
            completeness_reasons.append("PX4_INPUT_TRACE_TAIL_LOSS")
    trace_sequences = sorted({
        sequence for item in px4_input_trace
        for sequence in [_integer(item.get("trace_sequence"))]
        if sequence is not None and sequence > 0
    })
    if trace_sequences and trace_sequences[0] != 1:
        completeness_reasons.append("PX4_INPUT_TRACE_SEQUENCE_PREFIX_MISSING")
    if any(right != left + 1 for left, right in zip(trace_sequences, trace_sequences[1:])):
        completeness_reasons.append("PX4_INPUT_TRACE_SEQUENCE_GAP")
    if lifecycle and not lifecycle_reduction.get("valid_transaction_count"):
        # A valid reject transaction is complete for its outcome, while an
        # un-attributed success must not be promoted to an active reference.
        if not any(
            item.get("status") == "VALID_REJECT"
            for item in lifecycle_reduction.get("transactions", [])
        ):
            completeness_reasons.append("LIFECYCLE_ATTRIBUTION_INCOMPLETE")
    if not _has_capture_terminal_marker(scenario, scenario_events):
        completeness_reasons.append("TERMINAL_MARKER_MISSING")
    writer_integrity_reasons = []
    for label, value, records in (
        ("scenario", writer, scenario_events),
        ("monitor", monitor_writer, sample_rows),
    ):
        reasons = _writer_integrity(label, value, require_terminal=False)
        reasons.extend(_artifact_count_reasons(label.upper(), value, records))
        completeness_reasons.extend(reasons)
        writer_integrity_reasons.extend(reasons)
    capture_integrity_valid = bool(writer or monitor_writer) and not writer_integrity_reasons
    capture_integrity_valid = capture_integrity_valid and _has_capture_terminal_marker(
        scenario, scenario_events
    )
    capture_integrity_valid = capture_integrity_valid and not any(
        reason.startswith("PX4_INPUT_TRACE_") for reason in completeness_reasons
    )
    return {
        "session": session,
        "session_id": session_id,
        "scenario": scenario,
        "metadata": metadata,
        "monitor": monitor,
        "config": actual_config,
        "scenario_events": scenario_events,
        "lifecycle": lifecycle,
        "lifecycle_reduction": lifecycle_reduction,
        "world_transactions": world_transactions,
        "world_transaction_reduction": world_transaction_reduction,
        "world_evidence_required": world_required,
        "px4_input_trace": px4_input_trace,
        "streams": streams,
        "monitor_sample_counts": monitor_sample_counts,
        "pva": pva,
        "planner_trace": collect_planner_trace_records(scenario, streams.get("mapping_diagnostics", []) + streams.get("diagnostics", [])),
        "waypoints": _mission_waypoints(session, waypoint_source),
        "tracking_coverage_policy": tracking_coverage_policy,
        "tracking_acceptance_policy": tracking_acceptance_policy,
        "evaluation_window": evaluation_window,
        "capture_integrity_valid": capture_integrity_valid,
        "writer": {"scenario": writer, "monitor": monitor_writer},
        "completeness_reasons": sorted(set(completeness_reasons)),
    }


def _segment_rows(rows: list[dict[str, Any]], max_gap_s: float) -> list[dict[str, Any]]:
    if not rows:
        return []
    ordered = sorted(rows, key=lambda item: int(item["source_stamp_ns"]))
    segments: list[dict[str, Any]] = []
    current: list[dict[str, Any]] = []
    previous_stamp: int | None = None
    previous_identity: tuple[Any, ...] | None = None
    for item in ordered:
        stamp = int(item["source_stamp_ns"])
        identity = _record_identity(item)
        boundary = (
            current and (
                previous_stamp is None or stamp < previous_stamp or
                (stamp - previous_stamp) / 1e9 > max_gap_s or
                (previous_identity is not None and identity != previous_identity)
            )
        )
        if boundary:
            segments.append({
                "identity": previous_identity,
                "start_ns": int(current[0]["source_stamp_ns"]),
                "end_ns": int(current[-1]["source_stamp_ns"]),
                "sample_count": len(current),
                "samples": current,
            })
            current = []
        current.append(item)
        previous_stamp = stamp
        previous_identity = identity
    if current:
        segments.append({
            "identity": previous_identity,
            "start_ns": int(current[0]["source_stamp_ns"]),
            "end_ns": int(current[-1]["source_stamp_ns"]),
            "sample_count": len(current),
            "samples": current,
        })
    for index, segment in enumerate(segments):
        segment["segment_id"] = f"segment-{index}"
    return segments


def build_execution_segments(inputs: dict[str, Any], max_gap_s: float = DEFAULT_MAX_MATCH_GAP_S) -> dict[str, Any]:
    pva = list(inputs.get("pva", []))
    return {
        "reference_navigation": _segment_rows(pva, max_gap_s),
        "adapter_setpoint": _segment_rows(inputs.get("adapter_setpoint", []), max_gap_s),
        "px4_telemetry": _segment_rows(inputs.get("px4_telemetry", []), max_gap_s),
        "policy": {"max_gap_s": max_gap_s, "identity_fields": [
            "session_id", "localization_epoch", "goal_epoch", "request_id",
            "bundle_generation", "analytic_sample_role",
        ]},
    }


def _bracket(samples: list[dict[str, Any]], target_ns: int, max_gap_s: float) -> tuple[dict[str, Any], dict[str, Any], float] | None:
    ordered = sorted(samples, key=lambda item: int(item["source_stamp_ns"]))
    stamps = [int(item["source_stamp_ns"]) for item in ordered]
    index = bisect.bisect_left(stamps, target_ns)
    if index < len(ordered) and stamps[index] == target_ns:
        return ordered[index], ordered[index], 0.0
    if index <= 0 or index >= len(ordered):
        return None
    left, right = ordered[index - 1], ordered[index]
    gap = int(right["source_stamp_ns"]) - int(left["source_stamp_ns"])
    if gap <= 0 or gap / 1e9 > max_gap_s:
        return None
    # Do not interpolate through an identity/reset boundary.
    if _record_identity(left) != _record_identity(right):
        return None
    return left, right, (target_ns - int(left["source_stamp_ns"])) / float(gap)


def _interpolate_vector(bracket: tuple[dict[str, Any], dict[str, Any], float], field: str) -> tuple[float, ...] | None:
    left, right, alpha = bracket
    first = _vector(left.get(field))
    second = _vector(right.get(field))
    if first is None or second is None:
        return None
    return tuple((1.0 - alpha) * first[index] + alpha * second[index] for index in range(3))


def _interpolate_velocity(
    bracket: tuple[dict[str, Any], dict[str, Any], float],
) -> tuple[float, ...] | None:
    # Navigation commands use ``velocity`` while odometry evidence retains the
    # producer-owned ``linear_velocity`` name.  Do not silently lose the
    # velocity metric merely because the two typed sources use different field
    # names.
    for field in ("velocity", "linear_velocity"):
        value = _interpolate_vector(bracket, field)
        if value is not None:
            return value
    return None


def _metric_from_errors(errors: list[float], *, unit: str, source: str, time_basis: str, frame: str | None, matched: int, total: int, max_gap_s: float | None = None) -> dict[str, Any]:
    return {
        "reference": source,
        "time_basis": time_basis,
        "frame": frame,
        "unit": unit,
        **_summary(errors, unit=unit),
        "matched_sample_count": matched,
        "unmatched_sample_count": max(0, total - matched),
        "coverage_ratio": matched / total if total else None,
        "maximum_gap_s": max_gap_s,
        "status": "AVAILABLE" if matched else "NOT_EVALUABLE",
        "reason": None if matched else "NO_VALID_BRACKETED_SAMPLES",
    }


def _tracking_pair(
    reference: list[dict[str, Any]],
    measured: list[dict[str, Any]],
    max_gap_s: float,
    measured_to_reference: dict[str, Any],
) -> tuple[list[float], list[float], int, list[float], list[int]]:
    errors: list[float] = []
    vector_errors: list[float] = []
    gaps: list[float] = []
    matched_stamps: list[int] = []
    matched = 0
    for command in reference:
        bracket = _bracket(measured, int(command["source_stamp_ns"]), max_gap_s)
        if bracket is None:
            continue
        measured_position = _interpolate_vector(bracket, "position")
        command_position = _vector(command.get("position"))
        measured_velocity = _interpolate_velocity(bracket)
        command_velocity = _vector(command.get("velocity"))
        if measured_position is not None:
            measured_position = _apply_frame_transform(
                measured_position, measured_to_reference, is_position=True
            )
        if measured_velocity is not None:
            measured_velocity = _apply_frame_transform(
                measured_velocity, measured_to_reference, is_position=False
            )
        if command_position is not None and measured_position is not None:
            errors.append(_norm(measured_position[index] - command_position[index] for index in range(3)))
        if command_velocity is not None and measured_velocity is not None:
            vector_errors.append(_norm(measured_velocity[index] - command_velocity[index] for index in range(3)))
        if command_position is not None and measured_position is not None or command_velocity is not None and measured_velocity is not None:
            matched += 1
            gaps.append((int(bracket[1]["source_stamp_ns"]) - int(bracket[0]["source_stamp_ns"])) / 1e9)
            matched_stamps.append(int(command["source_stamp_ns"]))
    return errors, vector_errors, matched, gaps, matched_stamps


def _matrix3(value: Any) -> list[list[float]] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    matrix: list[list[float]] = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != 3:
            return None
        values = [_number(item) for item in row]
        if any(item is None for item in values):
            return None
        matrix.append([float(item) for item in values])
    return matrix


def _rotation_matrix_valid(matrix: list[list[float]], tolerance: float = 1e-6) -> bool:
    gram = [
        [sum(matrix[k][i] * matrix[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    orthonormal = all(
        abs(gram[i][j] - (1.0 if i == j else 0.0)) <= tolerance
        for i in range(3) for j in range(3)
    )
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    return orthonormal and abs(determinant - 1.0) <= tolerance


def _apply_frame_transform(
    vector: tuple[float, ...],
    transform: dict[str, Any],
    *,
    is_position: bool,
) -> tuple[float, ...]:
    matrix = transform["rotation"]
    translation = transform["translation"]
    if transform["direction"] == "target_to_source":
        rotated = tuple(
            sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)
        )
        return tuple(
            rotated[index] + (translation[index] if is_position else 0.0)
            for index in range(3)
        )
    shifted = tuple(
        vector[index] - (translation[index] if is_position else 0.0)
        for index in range(3)
    )
    return tuple(
        sum(matrix[column][row] * shifted[column] for column in range(3))
        for row in range(3)
    )


def _frame_witness(
    reference: list[dict[str, Any]],
    measured: list[dict[str, Any]],
    scenario: dict[str, Any],
) -> tuple[str | None, str | None, dict[str, Any] | None]:
    ref_frames = {str(item.get("frame_id")) for item in reference if item.get("frame_id")}
    measured_frames = {str(item.get("frame_id")) for item in measured if item.get("frame_id")}
    if not ref_frames or not measured_frames:
        return None, "FRAME_WITNESS_UNAVAILABLE", None
    if len(ref_frames) != 1 or len(measured_frames) != 1:
        return None, "FRAME_WITNESS_MISMATCH", None
    witness = scenario.get("truth_frame_witness")
    if not isinstance(witness, dict):
        return None, "FRAME_WITNESS_UNAVAILABLE", None
    if witness.get("valid") is not True:
        return None, "FRAME_WITNESS_INVALID", None
    transform = witness.get("T_L_G") if isinstance(witness.get("T_L_G"), dict) else witness
    source_frame = witness.get("source_frame") or witness.get("lio_frame_id")
    target_frame = witness.get("target_frame") or witness.get("gazebo_frame_id")
    if not source_frame or not target_frame:
        return None, "FRAME_WITNESS_FIELDS_MISSING", None
    if not (
        (str(source_frame) in ref_frames and str(target_frame) in measured_frames)
        or (str(source_frame) in measured_frames and str(target_frame) in ref_frames)
        or (ref_frames == measured_frames == {str(source_frame), str(target_frame)})
    ):
        return None, "FRAME_WITNESS_MISMATCH", None
    point_semantics = witness.get("point_semantics") or witness.get("reference_event")
    if not point_semantics:
        return None, "FRAME_WITNESS_POINT_UNSPECIFIED", None
    translation = (
        transform.get("translation")
        or transform.get("translation_lio_from_gazebo")
    ) if isinstance(transform, dict) else None
    if (
        not isinstance(translation, (list, tuple))
        or len(translation) != 3
        or _vector(translation) is None
    ):
        return None, "FRAME_TRANSFORM_TRANSLATION_INVALID", None
    matrix = (
        transform.get("rotation_matrix")
        or transform.get("rotation_matrix_lio_from_gazebo")
    ) if isinstance(transform, dict) else None
    matrix_value = _matrix3(matrix)
    if matrix_value is None:
        return None, "FRAME_TRANSFORM_ROTATION_INVALID", None
    if not _rotation_matrix_valid(matrix_value):
        return None, "FRAME_TRANSFORM_ROTATION_INVALID", None
    epochs = {
        item.get("localization_epoch") for item in reference + measured
        if _present_identity(item.get("localization_epoch"))
    }
    witness_epoch = witness.get("localization_epoch", witness.get("lio_localization_epoch"))
    if not _present_identity(witness_epoch) or any(int(epoch) != int(witness_epoch) for epoch in epochs):
        return None, "FRAME_WITNESS_EPOCH_INVALID", None
    validity_scope = witness.get("validity_scope")
    if validity_scope not in {"localization_epoch", "source_interval"}:
        return None, "FRAME_WITNESS_LIFETIME_MISSING", None
    if validity_scope == "source_interval":
        valid_from = _integer(witness.get("valid_from_source_stamp_ns"))
        valid_until = _integer(witness.get("valid_until_source_stamp_ns"))
        timestamps = [
            _integer(item.get("source_stamp_ns")) for item in reference + measured
        ]
        if (
            valid_from is None or valid_until is None or valid_until < valid_from
            or any(stamp is None or stamp < valid_from or stamp > valid_until for stamp in timestamps)
        ):
            return None, "FRAME_WITNESS_OUTSIDE_LIFETIME", None
    if not witness.get("provenance") and not witness.get("reference_event"):
        return None, "FRAME_WITNESS_PROVENANCE_MISSING", None
    reference_frame = next(iter(ref_frames))
    measured_frame = next(iter(measured_frames))
    if reference_frame == measured_frame:
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        translation_value = _vector(translation)
        if (
            translation_value is None
            or any(abs(value) > 1e-6 for value in translation_value)
            or any(
                abs(matrix_value[row][column] - identity[row][column]) > 1e-6
                for row in range(3) for column in range(3)
            )
        ):
            return None, "FRAME_WITNESS_SAME_FRAME_NONIDENTITY", None
        direction = "target_to_source"
        matrix_value = identity
        translation_value = (0.0, 0.0, 0.0)
    elif reference_frame == str(source_frame) and measured_frame == str(target_frame):
        direction = "target_to_source"
        translation_value = _vector(translation)
    elif reference_frame == str(target_frame) and measured_frame == str(source_frame):
        direction = "source_to_target"
        translation_value = _vector(translation)
    else:
        return None, "FRAME_WITNESS_MISMATCH", None
    return reference_frame, None, {
        "rotation": matrix_value,
        "translation": translation_value,
        "direction": direction,
    }


def _tracking_coverage(
    reference: list[dict[str, Any]],
    matched_stamps: list[int],
    inputs: dict[str, Any],
) -> tuple[dict[str, Any], list[str]]:
    policy = inputs.get("tracking_coverage_policy")
    required = ("min_coverage_ratio", "max_uncovered_interval_s", "max_pairing_gap_s")
    if not isinstance(policy, dict) or any(_number(policy.get(key)) is None for key in required):
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "TRACKING_COVERAGE_POLICY_UNAVAILABLE",
        }, ["TRACKING_COVERAGE_POLICY_UNAVAILABLE"]
    if not isinstance(policy.get("provenance"), str) or not policy["provenance"].strip():
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "TRACKING_COVERAGE_POLICY_PROVENANCE_MISSING",
        }, ["TRACKING_COVERAGE_POLICY_PROVENANCE_MISSING"]
    if not isinstance(policy.get("version"), str) or not policy["version"].strip():
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "TRACKING_COVERAGE_POLICY_VERSION_MISSING",
        }, ["TRACKING_COVERAGE_POLICY_VERSION_MISSING"]
    minimum_ratio = float(policy["min_coverage_ratio"])
    maximum_uncovered = float(policy["max_uncovered_interval_s"])
    maximum_pairing_gap = float(policy["max_pairing_gap_s"])
    if not (
        0.0 < minimum_ratio <= 1.0
        and maximum_uncovered >= 0.0
        and maximum_pairing_gap > 0.0
    ):
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "TRACKING_COVERAGE_POLICY_INVALID",
        }, ["TRACKING_COVERAGE_POLICY_INVALID"]
    window = inputs.get("evaluation_window")
    if not isinstance(window, dict):
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "COVERAGE_WINDOW_UNAVAILABLE",
        }, ["COVERAGE_WINDOW_UNAVAILABLE"]
    start = _integer(window.get("start_ns", window.get("source_start_ns")))
    end = _integer(window.get("end_ns", window.get("source_end_ns")))
    if start is None or end is None or end <= start:
        return {
            "required_duration_s": None, "valid_duration_s": None,
            "coverage_ratio": None, "longest_uncovered_interval_s": None,
            "uncovered_intervals": [], "status": "NOT_EVALUABLE",
            "reason": "COVERAGE_WINDOW_INVALID",
        }, ["COVERAGE_WINDOW_INVALID"]
    pairing_gap = maximum_pairing_gap * 1e9
    valid = sorted({
        stamp for stamp in matched_stamps if start <= stamp <= end
    })
    valid_intervals: list[tuple[int, int]] = []
    for left, right in zip(valid, valid[1:]):
        if right - left <= pairing_gap:
            valid_intervals.append((left, right))
    uncovered: list[tuple[int, int]] = []
    cursor = start
    for left, right in valid_intervals:
        if left > cursor:
            uncovered.append((cursor, left))
        cursor = max(cursor, right)
    if cursor < end:
        uncovered.append((cursor, end))
    required_duration_s = (end - start) / 1e9
    uncovered_duration_s = sum(right - left for left, right in uncovered) / 1e9
    valid_duration_s = max(0.0, required_duration_s - uncovered_duration_s)
    ratio = valid_duration_s / required_duration_s if required_duration_s > 0.0 else None
    longest_uncovered = max(
        (right - left) / 1e9 for left, right in uncovered
    ) if uncovered else 0.0
    sufficient = (
        ratio is not None
        and ratio >= minimum_ratio
        and longest_uncovered <= maximum_uncovered
    )
    result = {
        "required_duration_s": required_duration_s,
        "valid_duration_s": valid_duration_s,
        "coverage_ratio": ratio,
        "longest_uncovered_interval_s": longest_uncovered,
        "uncovered_intervals": [
            {"start_ns": left, "end_ns": right, "duration_s": (right - left) / 1e9}
            for left, right in uncovered
        ],
        "evaluation_window": {"start_ns": start, "end_ns": end},
        "status": "AVAILABLE" if sufficient else "NOT_EVALUABLE",
        "reason": None if sufficient else "TRACKING_COVERAGE_INSUFFICIENT",
    }
    return result, [] if sufficient else ["TRACKING_COVERAGE_INSUFFICIENT"]


def _reference_lineage_status(
    inputs: dict[str, Any], reference: list[dict[str, Any]]
) -> tuple[bool, list[str]]:
    if inputs.get("reference_lineage_valid") is True:
        return True, []
    reduction = inputs.get("lifecycle_reduction")
    if not isinstance(reduction, dict):
        return False, ["REFERENCE_LINEAGE_UNAVAILABLE"]
    valid_ids = {
        tuple(item) for item in reduction.get("valid_reference_ids", [])
        if isinstance(item, list) and len(item) == 3
    }
    if not valid_ids:
        return False, ["REFERENCE_LINEAGE_UNAVAILABLE"]
    for item in reference:
        identity = (
            item.get("request_id"),
            item.get("bundle_generation", item.get("trajectory_generation")),
            item.get("sample_id", item.get("trajectory_id")),
        )
        if identity not in valid_ids:
            return False, ["REFERENCE_LINEAGE_MISMATCH"]
    return True, []


def evaluate_tracking(inputs: dict[str, Any], max_gap_s: float = DEFAULT_MAX_MATCH_GAP_S) -> dict[str, Any]:
    raw_reference = [item for item in inputs.get("pva", []) if item.get("executable", True) is not False]
    reference, collapsed_heartbeats, heartbeat_reasons = _canonical_reference_heartbeats(raw_reference)
    truth = inputs.get("streams", {}).get("ground_truth_odometry", [])
    corrected = inputs.get("streams", {}).get("corrected_odometry", [])
    propagated = inputs.get("streams", {}).get("propagated_odometry", [])
    metrics: dict[str, Any] = {}
    reasons: list[str] = []
    # Duplicate PVA source ticks are admitted only after the exact immutable
    # heartbeat and strictly advancing sample-ID check above. The canonical
    # reference then has to advance in source time like any other reference.
    reference_time_valid, reference_time_reasons = _source_time_status(
        reference, policy=SourceTimestampPolicy.STRICTLY_INCREASING)
    reference_time_reasons = sorted(set(reference_time_reasons + heartbeat_reasons))
    reference_time_valid = reference_time_valid and not heartbeat_reasons
    lineage_valid, lineage_reasons = _reference_lineage_status(inputs, raw_reference)
    policy = inputs.get("tracking_coverage_policy")
    pairing_gap = (
        _number(policy.get("max_pairing_gap_s"))
        if isinstance(policy, dict) else None
    )
    diagnostic_gap = pairing_gap if pairing_gap is not None else max_gap_s
    for name, measured, measured_stream in (
        ("tracking.navigation_reference_vs_truth", truth, "ground_truth_odometry"),
        ("tracking.navigation_reference_vs_lio", corrected or propagated,
         "corrected_odometry" if corrected else "propagated_odometry"),
    ):
        frame, frame_reason, frame_transform = _frame_witness(
            reference, measured, inputs.get("scenario", {})
        )
        measured_time_valid, measured_time_reasons = _source_time_status(
            measured, policy=STREAM_SOURCE_TIMESTAMP_POLICY[measured_stream])
        clock_relation_valid, clock_relation_reasons = _source_clock_relation_status(
            reference, measured
        )
        if frame_reason:
            metric_reasons = sorted(set(
                [frame_reason]
                + reference_time_reasons
                + measured_time_reasons
                + clock_relation_reasons
                + lineage_reasons
            ))
            metrics[name] = {
                "reference": "published_navigation_command",
                "time_basis": "command_source_stamp",
                "frame": None,
                "unit": "m",
                "status": "NOT_EVALUABLE",
                "reason": frame_reason,
                "matched_sample_count": 0,
                "unmatched_sample_count": len(reference),
                "coverage_ratio": 0.0 if reference else None,
                "maximum_gap_s": None,
                "qualification_checks": {
                    "source_time_valid": (
                        reference_time_valid and measured_time_valid
                        and clock_relation_valid
                    ),
                    "frame_transform_valid": False,
                    "reference_lineage_valid": lineage_valid,
                    "coverage_sufficient": False,
                    "capture_integrity_valid": inputs.get("capture_integrity_valid") is True,
                },
                "qualification_role": "diagnostic_only",
                "qualification_reasons": metric_reasons,
            }
            reasons.extend(metric_reasons)
            continue
        position_errors, velocity_errors, matched, gaps, matched_stamps = _tracking_pair(
            reference, measured, diagnostic_gap, frame_transform
        )
        coverage, coverage_reasons = _tracking_coverage(reference, matched_stamps, inputs)
        source_time_valid = (
            reference_time_valid and measured_time_valid and clock_relation_valid
        )
        frame_transform_valid = frame_reason is None
        capture_integrity_valid = inputs.get("capture_integrity_valid") is True
        check_reasons = (
            reference_time_reasons + measured_time_reasons +
            clock_relation_reasons + lineage_reasons +
            ([] if frame_transform_valid else [frame_reason or "FRAME_WITNESS_INVALID"]) +
            coverage_reasons +
            ([] if capture_integrity_valid else ["CAPTURE_NOT_FINALIZED"])
        )
        metrics[name] = _metric_from_errors(
            position_errors, unit="m", source="published_navigation_command",
            time_basis="command_source_stamp", frame=frame, matched=matched,
            total=len(reference), max_gap_s=max(gaps, default=None),
        )
        coverage_status = coverage.get("status")
        coverage_reason = coverage.get("reason")
        metrics[name].update(coverage)
        metrics[name]["coverage_status"] = coverage_status
        metrics[name]["coverage_reason"] = coverage_reason
        metrics[name]["status"] = "AVAILABLE" if matched else "NOT_EVALUABLE"
        metrics[name]["matched_sample_ratio"] = (
            matched / len(reference) if reference else None
        )
        metrics[name]["qualification_checks"] = {
            "source_time_valid": source_time_valid,
            "frame_transform_valid": frame_transform_valid,
            "reference_lineage_valid": lineage_valid,
            "coverage_sufficient": not coverage_reasons,
            "capture_integrity_valid": capture_integrity_valid,
        }
        metrics[name]["qualification_role"] = (
            "qualification_candidate" if not check_reasons else "diagnostic_only"
        )
        metrics[name]["qualification_reasons"] = sorted(set(check_reasons))
        metrics[name + ".velocity"] = _metric_from_errors(
            velocity_errors, unit="m/s", source="published_navigation_command",
            time_basis="command_source_stamp", frame=frame, matched=matched,
            total=len(reference), max_gap_s=max(gaps, default=None),
        )
        metrics[name + ".velocity"].update(coverage)
        metrics[name + ".velocity"]["coverage_status"] = coverage_status
        metrics[name + ".velocity"]["coverage_reason"] = coverage_reason
        metrics[name + ".velocity"]["status"] = "AVAILABLE" if velocity_errors else "NOT_EVALUABLE"
        metrics[name + ".velocity"]["qualification_checks"] = dict(
            metrics[name]["qualification_checks"]
        )
        metrics[name + ".velocity"]["qualification_role"] = metrics[name]["qualification_role"]
        metrics[name + ".velocity"]["qualification_reasons"] = sorted(set(check_reasons))
        reasons.extend(check_reasons)
    # Adapter/PX4 timestamps currently use a distinct PX4 clock unless a
    # session-owned mapping witness is present. Never silently join by arrival.
    metrics["tracking.adapter_reference_vs_px4_state"] = {
        "reference": "adapter_px4_input_trace",
        "time_basis": "px4_source_stamp",
        "frame": None,
        "unit": "m",
        "status": "NOT_EVALUABLE",
        "reason": "PX4_CLOCK_MAPPING_OR_ADAPTER_TRACE_UNAVAILABLE",
        "matched_sample_count": 0,
        "unmatched_sample_count": len(reference),
        "coverage_ratio": 0.0 if reference else None,
        "maximum_gap_s": None,
    }
    return {
        "metrics": metrics, "reasons": sorted(set(reasons)),
        "max_gap_s": diagnostic_gap,
        "source_timestamp_policies": {
            stream: policy.value
            for stream, policy in STREAM_SOURCE_TIMESTAMP_POLICY.items()
        },
        "reference_heartbeat_collapsed_count": collapsed_heartbeats,
        "raw_reference_count": len(raw_reference),
        "canonical_reference_count": len(reference),
    }


def _raw_velocity(item: dict[str, Any]) -> tuple[float, float, float] | None:
    value = item.get("velocity")
    if value is None:
        value = item.get("linear_velocity")
    vector = _vector(value)
    return tuple(float(v) for v in vector) if vector is not None else None


def _stop_events(pva: list[dict[str, Any]], truth: list[dict[str, Any]], config: dict[str, Any]) -> list[dict[str, Any]]:
    enter = _number(config.get("stop_enter_threshold_mps")) or DEFAULT_STOP_ENTER_MPS
    exit_value = _number(config.get("stop_exit_threshold_mps")) or DEFAULT_STOP_EXIT_MPS
    minimum = _number(config.get("stop_min_duration_s")) or DEFAULT_STOP_MIN_DURATION_S
    ordered = sorted(pva, key=lambda item: int(item["source_stamp_ns"]))
    active: dict[str, Any] | None = None
    events: list[dict[str, Any]] = []
    previous_speed: float | None = None
    for item in ordered:
        velocity = _raw_velocity(item)
        if velocity is None:
            continue
        stamp_ns = int(item["source_stamp_ns"])
        speed = _norm(velocity)
        if active is None and speed <= enter:
            active = {
                "start_ns": stamp_ns,
                "samples": [item],
                "command_speed_before_mps": previous_speed,
            }
            continue
        if active is not None:
            active["samples"].append(item)
            if speed >= exit_value:
                end_ns = stamp_ns
                duration_s = (end_ns - int(active["start_ns"])) / 1e9
                if duration_s >= minimum:
                    first = active["samples"][0]
                    actual = []
                    for truth_item in truth:
                        if int(truth_item["source_stamp_ns"]) < int(active["start_ns"]) or int(truth_item["source_stamp_ns"]) > end_ns:
                            continue
                        velocity_value = _raw_velocity(truth_item)
                        if velocity_value is not None:
                            actual.append(_norm(velocity_value))
                    events.append({
                        "start_ns": int(active["start_ns"]),
                        "end_ns": end_ns,
                        "duration_s": duration_s,
                        "command_speed_before_mps": active.get("command_speed_before_mps"),
                        "command_speed_during_mps": _norm(_raw_velocity(first) or (0.0, 0.0, 0.0)),
                        "actual_speed_before_mps": None,
                        "actual_speed_during_mps": statistics.fmean(actual) if actual else None,
                        "bundle_generation": first.get("bundle_generation"),
                        "role": first.get("trajectory_flag"),
                        "mode": first.get("setpoint_kind", "TRACKING"),
                        "reason_evidence": "raw_command_velocity_hysteresis",
                        "classification": "expected" if first.get("trajectory_flag") in (0, None) else "safety-related",
                    })
                active = None
        previous_speed = speed
    return events


def _measured_acceleration(truth: list[dict[str, Any]]) -> tuple[list[float], list[float]]:
    vector_values: list[float] = []
    speed_values: list[float] = []
    ordered = sorted(truth, key=lambda item: int(item["source_stamp_ns"]))
    for left, right in zip(ordered, ordered[1:]):
        first = _raw_velocity(left)
        second = _raw_velocity(right)
        dt = (int(right["source_stamp_ns"]) - int(left["source_stamp_ns"])) / 1e9
        if first is None or second is None or dt <= 0.0:
            continue
        vector_values.append(_norm((second[index] - first[index]) / dt for index in range(3)))
        speed_values.append(abs(_norm(second) - _norm(first)) / dt)
    return vector_values, speed_values


def _command_vector_metric(pva: list[dict[str, Any]], field: str, unit: str) -> dict[str, Any]:
    values = [
        _norm(vector)
        for item in pva
        for vector in [_vector(item.get(field))]
        if vector is not None
    ]
    values = [value for value in values if math.isfinite(value)]
    return {"source": "published_navigation_command", "unit": unit, **_summary(values, unit=unit), "status": "AVAILABLE" if values else "NOT_EVALUABLE", "reason": None if values else "NO_VALID_RAW_COMMAND_SAMPLES"}


def _command_transitions(pva: list[dict[str, Any]]) -> list[dict[str, Any]]:
    ordered = sorted(pva, key=lambda item: int(item["source_stamp_ns"]))
    transitions: list[dict[str, Any]] = []
    for previous, current in zip(ordered, ordered[1:]):
        if previous.get("bundle_generation") == current.get("bundle_generation"):
            continue
        fields: dict[str, float | None] = {}
        for field, unit in (("position", "m"), ("velocity", "m/s"), ("acceleration", "m/s2"), ("jerk", "m/s3")):
            left, right = _vector(previous.get(field)), _vector(current.get(field))
            fields[field + "_jump_" + unit.replace("/", "_")] = _norm(right[index] - left[index] for index in range(3)) if left is not None and right is not None else None
        transitions.append({
            "timestamp_ns": int(current["source_stamp_ns"]),
            "previous_bundle_generation": previous.get("bundle_generation"),
            "bundle_generation": current.get("bundle_generation"),
            "previous_role": previous.get("trajectory_flag"),
            "role": current.get("trajectory_flag"),
            "observed_command_transition": fields,
        })
    return transitions


def _chattering(pva: list[dict[str, Any]], truth: list[dict[str, Any]]) -> dict[str, Any]:
    def total_variation(rows: list[dict[str, Any]], velocity_key: str) -> float:
        ordered = sorted(rows, key=lambda item: int(item["source_stamp_ns"]))
        result = 0.0
        for left, right in zip(ordered, ordered[1:]):
            a, b = _vector(left.get(velocity_key)), _vector(right.get(velocity_key))
            if a is not None and b is not None:
                result += _norm(b[index] - a[index] for index in range(3))
        return result
    roles = [item.get("trajectory_flag") for item in pva]
    role_switches = sum(left != right for left, right in zip(roles, roles[1:]))
    return {
        "command_velocity_total_variation_mps": total_variation(pva, "velocity"),
        "actual_velocity_total_variation_mps": total_variation(truth, "linear_velocity"),
        "role_switch_count": role_switches,
        "status": "AVAILABLE" if pva else "NOT_EVALUABLE",
        "reason": None if pva else "NO_VALID_RAW_COMMAND_SAMPLES",
    }


def evaluate_motion_quality(inputs: dict[str, Any]) -> dict[str, Any]:
    pva = [item for item in inputs.get("pva", []) if item.get("executable", True) is not False]
    truth = inputs.get("streams", {}).get("ground_truth_odometry", [])
    evaluator_config = inputs.get("config", {}).get("evaluation", {})
    if not isinstance(evaluator_config, dict):
        evaluator_config = {}
    measured_acceleration, speed_change_rate = _measured_acceleration(truth)
    transitions = _command_transitions(pva)
    jerk = _command_vector_metric(pva, "jerk", "m/s3")
    acceleration = _command_vector_metric(pva, "acceleration", "m/s2")
    acceleration["name"] = "sampled_command_acceleration"
    scenario = inputs.get("scenario", {})
    completion_time_s = _number(
        scenario.get("mission_completion_time_s", scenario.get("duration_s"))
    ) if scenario.get("mission_complete_observed") is True else None
    return {
        "status": "AVAILABLE" if pva else "NOT_EVALUABLE",
        "stop_go": {
            "events": _stop_events(pva, truth, evaluator_config),
            "detector": {
                "enter_threshold_mps": _number(evaluator_config.get("stop_enter_threshold_mps")) or DEFAULT_STOP_ENTER_MPS,
                "exit_threshold_mps": _number(evaluator_config.get("stop_exit_threshold_mps")) or DEFAULT_STOP_EXIT_MPS,
                "minimum_duration_s": _number(evaluator_config.get("stop_min_duration_s")) or DEFAULT_STOP_MIN_DURATION_S,
                "source": "offline_evaluator_config_snapshot",
            },
        },
        "sampled_command_jerk": jerk,
        "sampled_command_acceleration": acceleration,
        "measured_vector_acceleration": {"source": "ground_truth_odometry", "unit": "m/s2", **_summary(measured_acceleration, unit="m/s2"), "status": "AVAILABLE" if measured_acceleration else "NOT_EVALUABLE"},
        "speed_change_rate": {"source": "ground_truth_odometry", "unit": "m/s2", **_summary(speed_change_rate, unit="m/s2"), "status": "AVAILABLE" if speed_change_rate else "NOT_EVALUABLE"},
        "stitching": {"observed_command_transition": transitions, "planned_splice_residual": {
            field: {
                "source": "planner_trace",
                "unit": unit,
                **_summary(
                    [float(item[field]) for item in inputs.get("planner_trace", [])
                     if _number(item.get(field)) is not None],
                    unit=unit,
                ),
                "status": "AVAILABLE" if any(
                    _number(item.get(field)) is not None
                    for item in inputs.get("planner_trace", [])
                ) else "NOT_EVALUABLE",
            }
            for field, unit in (
                ("splice_position_residual_m", "m"),
                ("splice_velocity_residual_mps", "m/s"),
                ("splice_acceleration_residual_mps2", "m/s2"),
                ("splice_jerk_residual_mps3", "m/s3"),
            )
        }},
        "chattering": _chattering(pva, truth),
        "completion_time_s": {
            "source": "scenario mission completion event",
            "unit": "s",
            "value": completion_time_s,
            "status": "AVAILABLE" if completion_time_s is not None else "NOT_EVALUABLE",
            "reason": None if completion_time_s is not None else "MISSION_COMPLETION_TIME_UNAVAILABLE",
        },
    }


def evaluate_planning(inputs: dict[str, Any]) -> dict[str, Any]:
    records = [item for item in inputs.get("planner_trace", []) if isinstance(item, dict)]
    summary = planner_trace_summary(records)
    unique_solves = {
        (item.get("cycle_id"), item.get("bundle_id"), item.get("request_id"))
        for item in records
    }
    missing_terminal = sum(
        bool(item.get("transaction_completeness") not in (None, "complete", "COMPLETE"))
        for item in records
    )
    outcomes = {
        "candidate_accepted": sum(bool(item.get("candidate_accepted") or item.get("commit_observed_this_cycle")) for item in records),
        "staged": sum(bool(item.get("staged") or item.get("stage_result_code") is not None) for item in records),
        "activated": sum(bool(item.get("activated") or item.get("activation_observed")) for item in records),
    }
    return {
        "status": "AVAILABLE" if records else "NOT_EVALUABLE",
        "solve_count": len(unique_solves - {(None, None, None)}),
        "record_count": len(records),
        "outcome_counts": outcomes,
        "missing_terminal_event_count": missing_terminal,
        "timing_by_outcome": summary,
        "records": records,
    }


def evaluate_timing(inputs: dict[str, Any]) -> dict[str, Any]:
    """Summarize observed timing without inferring missing producer phases."""
    pva_times = sorted(
        int(item["observer_record_steady_ns"])
        for item in inputs.get("pva", [])
        if _integer(item.get("observer_record_steady_ns")) is not None
    )
    pva_intervals_ms = [
        (right - left) / 1e6
        for left, right in zip(pva_times, pva_times[1:])
        if right >= left
    ]
    px4_durations_ms = [
        duration / 1e6
        for item in inputs.get("px4_input_trace", [])
        for duration in [_number(item.get("setpoint_update_duration_ns"))]
        if duration is not None and duration >= 0.0
    ]
    return {
        "pva_observer_interarrival_ms": {
            "source": "pva observer_record_steady_ns",
            **_summary(pva_intervals_ms, unit="ms"),
            "status": "AVAILABLE" if pva_intervals_ms else "NOT_EVALUABLE",
            "reason": None if pva_intervals_ms else "NO_CONSECUTIVE_OBSERVER_TIMESTAMPS",
        },
        "px4_setpoint_update_duration_ms": {
            "source": "px4_input_trace setpoint_update_duration_ns",
            **_summary(px4_durations_ms, unit="ms"),
            "status": "AVAILABLE" if px4_durations_ms else "NOT_EVALUABLE",
            "reason": None if px4_durations_ms else "NO_VALID_PX4_UPDATE_DURATION",
        },
        "px4_state_use": _px4_state_use_timing(inputs.get("px4_input_trace", [])),
    }


def _px4_state_use_timing(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Observed state tuple at PX4 update, not a freshness/qualification verdict.

    Duration clocks stay local and monotonic; signed ROS ages retain pauses and
    backward jumps. Repeated uses of one state are retained (per-update weight),
    not misrepresented as a census of receiver callbacks or DDS latency.
    """
    steady_fields = (
        "state_callback_enter_steady_ns", "state_lock_requested_steady_ns",
        "state_lock_acquired_steady_ns", "state_receive_steady_ns",
        "state_snapshot_steady_ns", "update_start_steady_ns",
        "update_end_steady_ns",
    )
    ros_fields = (
        "state_source_stamp_ros_ns", "state_callback_enter_ros_ns",
        "state_snapshot_ros_ns", "update_start_ros_ns",
    )
    intervals = (
        "callback_validation_ms", "receiver_mutex_wait_ms",
        "post_lock_to_receive_ms", "receive_to_snapshot_ms",
        "snapshot_to_update_ms", "px4_update_ms",
    )
    values = {name: [] for name in intervals}
    values.update({"source_age_at_callback_ms": [], "source_age_at_update_ms": []})
    missing = invalid = epoch_mismatch = ros_backward = 0
    states: set[tuple[int, int]] = set()

    def positive_integer(raw: Any) -> int | None:
        # Diagnostic values arrive as decimal strings. Avoid float conversion
        # losing integer identity or accepting booleans/fractional timestamps.
        if isinstance(raw, bool) or not isinstance(raw, (str, int)):
            return None
        if isinstance(raw, str) and not raw.isdecimal():
            return None
        number = int(raw)
        return number if 0 < number <= (1 << 64) - 1 else None

    for record in records:
        trace = record.get("trace_values", {}) if isinstance(record, dict) else {}
        if not isinstance(trace, dict):
            missing += 1
            continue
        present = trace.get("state_input_present")
        if present is not True and present != "true":
            missing += 1
            continue
        fields = (*steady_fields, *ros_fields, "state_sequence", "state_localization_epoch")
        if any(trace.get(field) in (None, "NOT_RECORDED") for field in fields):
            missing += 1
            continue
        parsed = {field: positive_integer(trace[field]) for field in fields}
        if any(value is None for value in parsed.values()):
            invalid += 1
            continue
        # Timestamp producers are signed int64, unlike the uint64 identities.
        if any(parsed[field] > (1 << 63) - 1 for field in (*steady_fields, *ros_fields)):
            invalid += 1
            continue
        steady = [parsed[field] for field in steady_fields]
        if any(right < left for left, right in zip(steady, steady[1:])):
            invalid += 1
            continue
        states.add((parsed["state_localization_epoch"], parsed["state_sequence"]))
        command_epoch = positive_integer(trace.get("localization_epoch"))
        if command_epoch is not None and command_epoch != parsed["state_localization_epoch"]:
            epoch_mismatch += 1
        for name, left, right in zip(intervals, steady, steady[1:]):
            values[name].append((right - left) / 1e6)
        source = parsed["state_source_stamp_ros_ns"]
        values["source_age_at_callback_ms"].append(
            (parsed["state_callback_enter_ros_ns"] - source) / 1e6)
        values["source_age_at_update_ms"].append(
            (parsed["update_start_ros_ns"] - source) / 1e6)
        if (parsed["state_snapshot_ros_ns"] < parsed["state_callback_enter_ros_ns"] or
                parsed["update_start_ros_ns"] < parsed["state_snapshot_ros_ns"]):
            ros_backward += 1
    usable = len(values["px4_update_ms"])
    return {
        "status": "AVAILABLE" if usable else "NOT_EVALUABLE",
        "reason": None if usable else "NO_VALID_DIRECT_STATE_USE_TIMESTAMPS",
        "record_count": len(records), "usable_record_count": usable,
        "missing_record_count": missing, "invalid_record_count": invalid,
        "distinct_state_count": len(states),
        "command_state_epoch_mismatch_count": epoch_mismatch,
        "ros_backward_interval_count": ros_backward,
        "negative_source_age_at_callback_count": sum(
            age < 0 for age in values["source_age_at_callback_ms"]),
        "negative_source_age_at_update_count": sum(
            age < 0 for age in values["source_age_at_update_ms"]),
        "sample_weighting": "per PX4 update; repeated state uses retained",
        "scope": "local receiver-to-update observation; no DDS/producer/PX4 acceptance proof",
        "metrics": {name: _summary(samples, unit="ms") for name, samples in values.items()},
    }


def _ate_rpe(estimate: list[dict[str, Any]], truth: list[dict[str, Any]], max_gap_s: float) -> dict[str, Any]:
    errors: list[float] = []
    pairs: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]] = []
    for item in estimate:
        bracket = _bracket(truth, int(item["source_stamp_ns"]), max_gap_s)
        if bracket is None:
            continue
        expected = _interpolate_vector(bracket, "position")
        if expected is not None:
            estimate_value = _vector(item.get("estimate_position"))
            if estimate_value is not None:
                errors.append(_norm(estimate_value[index] - expected[index] for index in range(3)))
                pairs.append((int(item["source_stamp_ns"]), estimate_value, expected))
    rpe: list[float] = []
    for (_, first_est, first_truth), (_, last_est, last_truth) in zip(pairs, pairs[1:]):
        rpe.append(_norm((last_est[index] - first_est[index]) - (last_truth[index] - first_truth[index]) for index in range(3)))
    return {"ate": _summary(errors, unit="m"), "rpe": _summary(rpe, unit="m"), "matched_sample_count": len(pairs), "status": "AVAILABLE" if pairs else "NOT_EVALUABLE"}


def evaluate_localization(inputs: dict[str, Any]) -> dict[str, Any]:
    truth = inputs.get("streams", {}).get("ground_truth_odometry", [])
    outputs: dict[str, Any] = {}
    for name in ("corrected_odometry", "propagated_odometry"):
        estimate = inputs.get("streams", {}).get(name, [])
        rows: list[dict[str, Any]] = []
        for item in estimate:
            bracket = _bracket(truth, int(item["source_stamp_ns"]), DEFAULT_MAX_MATCH_GAP_S)
            if bracket is None:
                continue
            truth_position = _interpolate_vector(bracket, "position")
            estimate_position = _vector(item.get("position"))
            if truth_position is not None and estimate_position is not None:
                rows.append(dict(item, estimate_position=list(estimate_position)))
        outputs[name] = _ate_rpe(rows, truth, DEFAULT_MAX_MATCH_GAP_S)
        outputs[name]["source"] = name
    return {
        "streams": outputs,
        "status": "AVAILABLE" if any(
            item.get("status") == "AVAILABLE" for item in outputs.values()
        ) else "NOT_EVALUABLE",
    }


def _guidance_deviation(truth: list[dict[str, Any]], waypoints: list[tuple[float, float, float]]) -> dict[str, Any]:
    if len(waypoints) < 2:
        return {"status": "NOT_EVALUABLE", "reason": "MISSION_GUIDANCE_UNAVAILABLE", "unit": "m", "source": "mission_guidance_polyline"}
    values: list[float] = []
    for item in truth:
        point = _vector(item.get("position"))
        if point is None:
            continue
        distances = []
        for start, end in zip(waypoints, waypoints[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            denominator = dx * dx + dy * dy
            alpha = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator)) if denominator > 1e-12 else 0.0
            distances.append(math.hypot(point[0] - (start[0] + alpha * dx), point[1] - (start[1] + alpha * dy)))
        values.append(min(distances))
    return {"source": "mission_guidance_polyline", "unit": "m", **_summary(values, unit="m"), "status": "AVAILABLE" if values else "NOT_EVALUABLE", "reason": None if values else "NO_VALID_TRUTH_SAMPLES", "qualification_role": "descriptive_only"}


def _dimension(status: str, reasons: list[str], **extra: Any) -> dict[str, Any]:
    return {"status": status, "reasons": sorted(set(str(item) for item in reasons)), **extra}


def evaluate_software_qualification(inputs: dict[str, Any]) -> dict[str, Any]:
    """Evaluate the separately approved C0-SW evidence scope.

    Eligibility means the software outcome can be assessed. A demonstrated
    software failure remains eligible and is reported as FAIL; integrated
    tracking and measured-motion quality retain their independent verdicts.
    """
    metadata = inputs.get("metadata", {})
    scenario = inputs.get("scenario", {})
    reduction = inputs.get("lifecycle_reduction", {})
    world_reduction = inputs.get("world_transaction_reduction", {})
    references = [item for item in inputs.get("pva", [])
                  if item.get("executable", True) is not False]
    reasons: list[str] = []
    if metadata.get("qualification_scope") != "C0_SW":
        reasons.append("C0_SW_SCOPE_NOT_DECLARED")
    if metadata.get("qualification_policy_version") != C0_SW_POLICY_VERSION:
        reasons.append("C0_SW_POLICY_VERSION_MISSING")
    if metadata.get("qualification_policy_provenance") != C0_SW_POLICY_PROVENANCE:
        reasons.append("C0_SW_POLICY_PROVENANCE_MISSING")
    configuration = metadata.get("runtime_configuration", {})
    tracking_requested = metadata.get("tracking_experiment", {}).get("mode")
    if tracking_requested != "off":
        reasons.append("TRACKING_REQUESTED_NOT_OFF")
    for boundary in ("mapping", "external_mode"):
        effective = configuration.get(boundary, {}).get("effective", {})
        if (effective.get("mode") != "off" or effective.get("enabled") is not False
                or effective.get("suppress_braking") is not False
                or effective.get("suppress_estimator_health_response") is not False):
            reasons.append(f"CONFIGURATION_NOT_TRUTHFUL:{boundary}")
    injection = configuration.get("planner_fault_injection", {}).get("effective", {})
    if not injection or any(value not in (False, 0, None) for value in injection.values()):
        reasons.append("PLANNER_FAULT_INJECTION_ACTIVE_OR_UNKNOWN")
    requested_speed = _number(metadata.get("requested_cruise_speed_mps"))
    if requested_speed is None or not (
        DEFAULT_C0_SPEED_MIN_MPS <= requested_speed <= DEFAULT_C0_SPEED_MAX_MPS
    ):
        reasons.append("OUTSIDE_C0_SPEED_SCOPE")

    writer_reasons = [
        f"{label}:{reason}"
        for label, writer in inputs.get("writer", {}).items()
        for reason in _writer_integrity(label, writer, require_terminal=False)
    ]
    if not inputs.get("writer"):
        writer_reasons.append("REQUIRED_WRITER_ACCOUNTING_MISSING")
    if not inputs.get("scenario_events"):
        writer_reasons.append("SCENARIO_EVENT_STREAM_MISSING")
    reasons.extend(writer_reasons)
    unresolved = len(reduction.get("unresolved", []))
    conflicts = len(reduction.get("conflicts", []))
    unbound_activations = sum(
        item.get("phase") == "activate" for item in reduction.get("unbound_events", [])
    )
    missing_results = sum(
        "request" in transaction.get("events", {}) and
        "result" not in transaction.get("events", {})
        for transaction in reduction.get("transactions", [])
    )
    if unresolved:
        reasons.append("C0_SW_REQUIRED_LIFECYCLE_UNRESOLVED")
    if conflicts:
        reasons.append("C0_SW_REQUIRED_LIFECYCLE_CONFLICT")
    if unbound_activations:
        reasons.append("C0_SW_ACTIVATION_OWNER_MISSING")
    if missing_results:
        reasons.append("C0_SW_PLANNER_RESULT_WITNESS_MISSING")
    if inputs.get("world_evidence_required"):
        if world_reduction.get("required_world_unresolved", 0):
            reasons.append("C0_SW_REQUIRED_WORLD_TRANSACTION_UNRESOLVED")
        if world_reduction.get("required_world_conflicts", 0):
            reasons.append("C0_SW_REQUIRED_WORLD_TRANSACTION_CONFLICT")
        if world_reduction.get("required_world_reference_missing", 0):
            reasons.append("C0_SW_REQUIRED_WORLD_REFERENCE_MISSING")
        if world_reduction.get("required_world_reference_conflicts", 0):
            reasons.append("C0_SW_REQUIRED_WORLD_REFERENCE_CONFLICT")
        if world_reduction.get("required_world_events_dropped", 0):
            reasons.append("C0_SW_REQUIRED_WORLD_EVIDENCE_DROPPED")
    valid_references = {
        tuple(item) for item in reduction.get("producer_owned_reference_ids", [])
        if isinstance(item, list) and len(item) == 10
    }
    authorized_references = {
        tuple(item) for item in reduction.get("producer_authorized_reference_ids", [])
        if isinstance(item, list) and len(item) == 10
    }
    rejections = [
        event.get("payload", {}) for event in inputs.get("scenario_events", [])
        if event.get("kind") == "command_rejection" and
        isinstance(event.get("payload"), dict)
    ]
    rejection_keys = {
        tuple(item.get(field) for field in (
            "mode_activation_id", "localization_epoch", "goal_epoch",
            "request_id", "bundle_generation", "sample_id",
        ))
        for item in rejections if item.get("command_present") is True
    }
    rejected_command_keys = {key[1:] for key in rejection_keys}
    no_execution_signals = [
        transaction.get("events", {}).get("authorize", {})
        for transaction in reduction.get("transactions", [])
        if transaction.get("identity", {}).get("producer_kind") ==
            "NO_EXECUTION_SIGNAL"
    ]
    unpaired_no_execution_signals = sum(
        tuple(item.get(field) for field in (
            "localization_epoch", "goal_epoch", "request_id",
            "bundle_generation", "sample_id",
        )) not in rejected_command_keys
        for item in no_execution_signals
    )
    if unpaired_no_execution_signals:
        reasons.append("C0_SW_NO_EXECUTION_ADAPTER_REJECTION_MISSING")
    admissions = [
        event.get("payload", {}) for event in inputs.get("scenario_events", [])
        if event.get("kind") == "command_admission"
        and isinstance(event.get("payload"), dict)
    ]
    adapter_identity_fields = (
        "mode_activation_id", "localization_epoch", "goal_epoch",
        "request_id", "bundle_generation", "sample_id",
    )
    admission_exact_keys = {
        tuple(item.get(field) for field in adapter_identity_fields)
        for item in admissions
    }
    if any(any(item.get(field) is None for field in (
            "stage", "reason_code", "disposition")) for item in rejections):
        reasons.append("C0_SW_ADAPTER_REJECTION_UNTYPED")
    missing_references = sum(
        (reference_id := (
            item.get("runtime_instance_id"), item.get("session_id"),
            item.get("localization_epoch"), item.get("goal_epoch"),
            item.get("request_id"), item.get("bundle_generation"),
            item.get("sample_id"), item.get("world_generation"),
            item.get("world_revision"), item.get("world_observation_stamp_ns"),
        )) not in valid_references and not (
            reference_id in authorized_references and
            tuple(item.get(field) for field in adapter_identity_fields) in
                (rejection_keys | admission_exact_keys)
        )
        for item in references
    )
    admitted_without_setpoint_trace = sum(
        tuple(item.get(field) for field in (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "bundle_generation", "sample_id",
            "world_generation", "world_revision", "world_observation_stamp_ns",
        )) not in valid_references and
        tuple(item.get(field) for field in adapter_identity_fields) in
            admission_exact_keys
        for item in references
    )
    if not references or missing_references:
        reasons.append("C0_SW_REQUIRED_REFERENCE_LINEAGE_MISSING")
    reference_payloads: dict[tuple[Any, ...], set[tuple[Any, ...]]] = {}
    for item in references:
        sample_owner = tuple(item.get(field) for field in (
            "runtime_instance_id", "session_id", "localization_epoch",
            "goal_epoch", "request_id", "bundle_generation", "sample_id",
        ))
        world = tuple(item.get(field) for field in (
            "world_generation", "world_revision", "world_observation_stamp_ns",
        ))
        reference_payloads.setdefault(sample_owner, set()).add(world)
    conflicting_references = sum(
        len(worlds) > 1 for worlds in reference_payloads.values())
    if conflicting_references:
        reasons.append("C0_SW_REQUIRED_REFERENCE_LINEAGE_CONFLICT")
    command_key_fields = (
        "localization_epoch", "goal_epoch", "request_id",
        "bundle_generation", "sample_id",
    )
    command_keys = {
        tuple(item.get(field) for field in command_key_fields)
        for item in references
    }
    admission_keys = {
        tuple(item.get(field) for field in command_key_fields)
        for item in admissions
    }
    adapter_command_keys = {
        tuple(item.get(field) for field in command_key_fields)
        for item in inputs.get("px4_input_trace", [])
        if _present_identity(item.get("sample_id")) and
        _present_identity(item.get("bundle_generation"))
    }
    missing_adapter_receipts = len(adapter_command_keys - admission_keys)
    unbound_adapter_receipts = len(admission_keys - command_keys)
    if missing_adapter_receipts:
        reasons.append("C0_SW_ADAPTER_ADMISSION_RECEIPT_MISSING")
    if unbound_adapter_receipts:
        reasons.append("C0_SW_ADAPTER_ADMISSION_WITHOUT_CORE_REFERENCE")
    retained_sequences = sorted({
        _integer(item.get("producer_event_sequence"))
        for item in inputs.get("lifecycle", [])
        if item.get("phase") == "retained"
    } - {None})
    if retained_sequences and (
        retained_sequences[0] != 1 or
        any(right != left + 1 for left, right in zip(
            retained_sequences, retained_sequences[1:]))
    ):
        reasons.append("RETAINED_DECISION_PRODUCER_SEQUENCE_GAP")
    if any(
        (_integer(item.get("failed_before_event")) or 0) > 0 or
        (_integer(item.get("suppressed_before_event")) or 0) > 0
        for item in inputs.get("lifecycle", []) if item.get("phase") == "retained"
    ):
        reasons.append("RETAINED_DECISION_PRODUCER_LOSS")

    outcome = scenario.get("outcome")
    expected_indices = list(range(len(inputs.get("waypoints", []))))
    accepted_indices = [
        _integer(item.get("accepted_waypoint_index"))
        for item in scenario.get("waypoint_acceptance_events", [])
        if isinstance(item, dict) and item.get("waypoint_accepted") is True
    ]
    if (scenario.get("mission_complete_observed") is True and
            outcome == "COMPLETE" and expected_indices and
            accepted_indices == expected_indices):
        product_logic = "PASS"
    elif outcome == "PAUSED_SAFETY_STOP":
        # A mode label and observed Hold alone do not prove that the adapter
        # used the right measured state, identity and safety gate. Preserve a
        # software safety stop as assessable only after its causal gate witness
        # is joined, rather than turning a physical failure into a false PASS.
        product_logic = "NOT_EVALUABLE"
        reasons.append("SAFETY_STOP_GATE_DECISION_WITNESS_NOT_ASSESSED")
    elif outcome == "COMPLETE" and scenario.get("mission_complete_observed") is True:
        product_logic = "NOT_EVALUABLE"
        reasons.append("MISSION_ACCEPTANCE_LINEAGE_INCOMPLETE")
    elif outcome in {"COMPLETE", "FAILED"}:
        product_logic = "FAIL"
    else:
        product_logic = "NOT_EVALUABLE"
    if product_logic == "NOT_EVALUABLE":
        reasons.append("PRODUCT_OUTCOME_UNATTRIBUTED")
    authority = "PASS" if not (
        unresolved or conflicts or unbound_activations or missing_results or
        missing_references or conflicting_references or
        missing_adapter_receipts or unbound_adapter_receipts or
        unpaired_no_execution_signals
    ) else "NOT_EVALUABLE"
    evidence = "PASS" if not writer_reasons and not (
        unresolved or conflicts or missing_results or missing_references or
        conflicting_references or missing_adapter_receipts or
        unbound_adapter_receipts or unpaired_no_execution_signals
    ) else "NOT_EVALUABLE"
    temporal = "PASS" if (
        product_logic == "PASS" and authority == "PASS" and evidence == "PASS" and
        not missing_adapter_receipts
    ) else "NOT_EVALUABLE"
    eligible = not reasons
    return {
        "qualification_scope": "C0_SW",
        "qualification_policy_version": C0_SW_POLICY_VERSION,
        "qualification_policy_provenance": metadata.get("qualification_policy_provenance"),
        "software_qualification_eligible": eligible,
        "assessment_status": (
            "NOT_EVALUABLE" if not eligible else
            "FAIL" if product_logic == "FAIL" else
            "PASS" if all(status == "PASS" for status in (
                product_logic, authority, temporal, evidence)) else
            "NOT_EVALUABLE"),
        "blocking_reasons": sorted(set(reasons)),
        "required_lifecycle_unresolved": unresolved,
        "required_lifecycle_conflicting": conflicts,
        "required_world_transactions": world_reduction.get(
            "required_world_transactions", 0),
        "resolved_world_transactions": world_reduction.get(
            "resolved_world_transactions", 0),
        "required_world_unresolved": world_reduction.get(
            "required_world_unresolved", 0),
        "required_world_conflicts": world_reduction.get(
            "required_world_conflicts", 0),
        "required_world_reference_missing": world_reduction.get(
            "required_world_reference_missing", 0),
        "required_world_reference_conflicts": world_reduction.get(
            "required_world_reference_conflicts", 0),
        "required_world_events_produced": world_reduction.get(
            "required_world_events_produced"),
        "required_world_events_written": world_reduction.get(
            "required_world_events_written"),
        "required_world_events_dropped": world_reduction.get(
            "required_world_events_dropped"),
        "required_world_events_serialization_failures": world_reduction.get(
            "required_world_events_serialization_failures", 0),
        "required_world_events_writer_failures": world_reduction.get(
            "required_world_events_writer_failures", 0),
        "required_reference_count": len(references),
        "required_reference_missing": missing_references,
        "references_admitted_without_setpoint_trace": admitted_without_setpoint_trace,
        "required_reference_conflicting": conflicting_references,
        "adapter_admission_receipts": len(admission_keys),
        "no_execution_signals": len(no_execution_signals),
        "no_execution_adapter_rejections_missing": unpaired_no_execution_signals,
        "adapter_receipts_missing": missing_adapter_receipts,
        "adapter_receipts_unbound": unbound_adapter_receipts,
        "axes": {
            "PRODUCT_LOGIC": product_logic,
            "AUTHORITY_IDENTITY": authority,
            "TEMPORAL_SAFETY": temporal,
            "EVIDENCE_COMPLETENESS": evidence,
            "FAULT_HANDLING": "SUITE_EVIDENCE_REQUIRED",
            "FLIGHT_PERFORMANCE": "DEFERRED_C0_IFP",
            "ENVIRONMENT_VALIDITY": "DEFERRED_SIMULATION_VALIDITY",
        },
    }


def _tracking_acceptance_status(
    position_metric: dict[str, Any],
    velocity_metric: dict[str, Any],
    policy: Any,
) -> tuple[str, list[str]]:
    required = {
        "position_error_p95_max_m": (position_metric, "p95"),
        "position_error_max_m": (position_metric, "maximum"),
        "velocity_error_p95_max_mps": (velocity_metric, "p95"),
        "velocity_error_max_mps": (velocity_metric, "maximum"),
    }
    if not isinstance(policy, dict):
        return "NOT_EVALUABLE", ["TRACKING_ACCEPTANCE_POLICY_UNAVAILABLE"]
    if not isinstance(policy.get("provenance"), str) or not policy["provenance"].strip():
        return "NOT_EVALUABLE", ["TRACKING_ACCEPTANCE_POLICY_PROVENANCE_MISSING"]
    if not isinstance(policy.get("version"), str) or not policy["version"].strip():
        return "NOT_EVALUABLE", ["TRACKING_ACCEPTANCE_POLICY_VERSION_MISSING"]
    limits: dict[str, float] = {}
    for key in required:
        value = _number(policy.get(key))
        if value is None or value < 0.0:
            return "NOT_EVALUABLE", ["TRACKING_ACCEPTANCE_POLICY_INVALID"]
        limits[key] = value
    reasons = []
    for key, (metric, statistic) in required.items():
        observed = _number(metric.get(statistic))
        if observed is None:
            return "NOT_EVALUABLE", ["TRACKING_ACCEPTANCE_METRIC_UNAVAILABLE"]
        if observed > limits[key]:
            reasons.append(f"TRACKING_{key.upper()}_EXCEEDED")
    return ("FAIL", reasons) if reasons else ("PASS", [])


def evaluate_session(inputs: dict[str, Any]) -> dict[str, Any]:
    scenario = inputs.get("scenario", {})
    tracking = evaluate_tracking(inputs)
    motion = evaluate_motion_quality(inputs)
    planning = evaluate_planning(inputs)
    localization = evaluate_localization(inputs)
    timing = evaluate_timing(inputs)
    completeness_reasons = list(inputs.get("completeness_reasons", []))
    evidence_contract = build_evidence_contract(inputs)
    completeness_reasons.extend(evidence_contract.get("qualification_missing", []))
    capture_integrity_valid = inputs.get("capture_integrity_valid")
    if capture_integrity_valid is None:
        writer = inputs.get("writer")
        if isinstance(writer, dict) and writer:
            writer_reasons = []
            for label, value in writer.items():
                if isinstance(value, dict):
                    writer_reasons.extend(_writer_integrity(
                        str(label), value, require_terminal=False
                    ))
            capture_integrity_valid = bool(writer) and not writer_reasons
        else:
            # The public evaluator must not manufacture recorder completeness.
            # Unit fixtures which intentionally model a complete capture set
            # this input explicitly.
            capture_integrity_valid = False
    if not capture_integrity_valid and "CAPTURE_NOT_FINALIZED" not in completeness_reasons:
        completeness_reasons.append("CAPTURE_NOT_FINALIZED")
    pva = inputs.get("pva", [])
    truth = inputs.get("streams", {}).get("ground_truth_odometry", [])
    configured_scenario = inputs.get("config", {}).get("scenario", {})
    if not isinstance(configured_scenario, dict):
        configured_scenario = {}
    requested_speed = _number(
        scenario.get(
            "requested_cruise_speed_mps",
            inputs.get("metadata", {}).get(
                "requested_cruise_speed_mps",
                configured_scenario.get("requested_cruise_speed_mps"),
            ),
        )
    )
    if requested_speed is None:
        requested_speed = _number(inputs.get("metadata", {}).get("requested_speed_mps"))
    speed_reasons: list[str] = []
    if requested_speed is None:
        speed_reasons.append("REQUESTED_SPEED_UNAVAILABLE")
    elif not DEFAULT_C0_SPEED_MIN_MPS <= requested_speed <= DEFAULT_C0_SPEED_MAX_MPS:
        speed_reasons.append("SPEED_OUTSIDE_C0_SCOPE")
    experiment = inputs.get("metadata", {}).get(
        "tracking_experiment", configured_scenario.get("tracking_experiment", {})
    )
    bypass = scenario.get(
        "experimental_bypasses",
        inputs.get("metadata", {}).get(
            "experimental_bypasses", configured_scenario.get("experimental_bypasses", {})
        ),
    )
    if experiment and experiment.get("mode", "off") != "off":
        speed_reasons.append("EXPERIMENTAL_TRACKING_MODE")
    if isinstance(bypass, dict) and bypass:
        speed_reasons.append("EXPERIMENTAL_BYPASS_PRESENT")
    if speed_reasons:
        for metric in tracking["metrics"].values():
            if isinstance(metric, dict) and "qualification_role" in metric:
                metric["qualification_role"] = "diagnostic_only"
    mission_reasons: list[str] = []
    # ``outcome`` is a recorder summary and may be optimistic after an early
    # process exit.  Only the explicit completion event can satisfy this gate.
    mission_complete = scenario.get("mission_complete_observed") is True
    if not mission_complete:
        mission_reasons.append("MISSION_COMPLETION_NOT_OBSERVED")
    if scenario.get("waypoint_acceptance_events") is None and scenario.get("mission_waypoint_count", 0):
        mission_reasons.append("WAYPOINT_ACCEPTANCE_EVIDENCE_MISSING")
    if scenario.get("waypoint_acceptance_events") is not None:
        expected = list(range(int(_number(scenario.get("mission_waypoint_count")) or 0)))
        accepted = [int(item.get("accepted_waypoint_index")) for item in scenario.get("waypoint_acceptance_events", []) if isinstance(item, dict) and item.get("waypoint_accepted", True) and _integer(item.get("accepted_waypoint_index")) is not None]
        if expected and accepted != expected:
            mission_reasons.append("WAYPOINT_ACCEPTANCE_INCOMPLETE")
    mission_status = "PASS" if mission_complete and not mission_reasons else "NOT_EVALUABLE" if not mission_complete else "FAIL"
    collision_count = _number(scenario.get("collision_count"))
    safety_reasons = [] if collision_count == 0 else ["COLLISION_EVIDENCE_MISSING" if collision_count is None else "COLLISION_ENVELOPE_BREACHED"]
    safety_status = "PASS" if collision_count == 0 else "NOT_EVALUABLE" if collision_count is None else "FAIL"
    evidence_dimension_status = "PASS" if not completeness_reasons else "NOT_EVALUABLE"
    # Ground truth is the independent witness.  A command compared only with
    # LIO/propagated state is internal consistency, not proof of real tracking
    # quality when the estimator itself may have drifted.
    truth_tracking = tracking["metrics"].get("tracking.navigation_reference_vs_truth", {})
    truth_velocity_tracking = tracking["metrics"].get(
        "tracking.navigation_reference_vs_truth.velocity", {}
    )
    tracking_checks = truth_tracking.get("qualification_checks", {})
    # Optional command-vs-LIO diagnostics retain their own reasons but cannot
    # invalidate the independent ground-truth tracking dimension.
    tracking_reasons = list(truth_tracking.get("qualification_reasons", []))
    tracking_evidence_valid = (
        truth_tracking.get("status") == "AVAILABLE"
        and truth_tracking.get("matched_sample_count", 0) > 0
        and isinstance(tracking_checks, dict)
        and all(tracking_checks.get(key) is True for key in (
            "source_time_valid", "frame_transform_valid",
            "reference_lineage_valid", "coverage_sufficient",
            "capture_integrity_valid",
        ))
    )
    if tracking_evidence_valid:
        tracking_status, acceptance_reasons = _tracking_acceptance_status(
            truth_tracking,
            truth_velocity_tracking,
            inputs.get("tracking_acceptance_policy"),
        )
        tracking_reasons.extend(acceptance_reasons)
    else:
        tracking_status = "NOT_EVALUABLE"
    motion_reasons: list[str] = []
    motion_status = "NOT_EVALUABLE"
    if motion.get("status") == "AVAILABLE":
        # Motion metrics are descriptive until W9 pins an explicit acceptance
        # contract. Presence of jerk/chattering numbers is not a PASS.
        motion_reasons.append("MOTION_ACCEPTANCE_POLICY_UNAVAILABLE")
    qualification_reasons = (
        completeness_reasons + speed_reasons + tracking_reasons + motion_reasons
    )
    if not all(status == "PASS" for status in (
            mission_status, safety_status, tracking_status, motion_status,
            evidence_dimension_status)):
        qualification_reasons.append("REQUIRED_DIMENSION_NOT_PASS")
    dimension_statuses = (
        mission_status, safety_status, tracking_status, motion_status,
        evidence_dimension_status,
    )
    assessment_status = (
        "FAIL" if "FAIL" in dimension_statuses
        else "PASS" if all(status == "PASS" for status in dimension_statuses)
        else "NOT_EVALUABLE"
    )
    explicit_evidence_status = "COMPLETE" if not completeness_reasons else "INCOMPLETE"
    result = {
        "schema_version": EVALUATION_SCHEMA_VERSION,
        "profile": "navigation_quality",
        "qualification_scope": "C0_IFP",
        "assessment_status": assessment_status,
        "evidence_status": explicit_evidence_status,
        "qualification_eligible": not qualification_reasons,
        "integrated_flight_qualification_eligible": not qualification_reasons,
        "blocking_reasons": sorted(set(qualification_reasons)),
        "qualification_reasons": sorted(set(qualification_reasons)),
        "scope": {
            "speed_band": "C0",
            "requested_speed_mps": requested_speed,
            "allowed_speed_range_mps": [DEFAULT_C0_SPEED_MIN_MPS, DEFAULT_C0_SPEED_MAX_MPS],
            "qualification_excludes": [
                "6/8 m/s characterization",
                "12 m/s outside C0",
                "tracking experiments",
                "writer drops",
                "missing frame/time evidence",
            ],
        },
        "tracking_coverage_policy": inputs.get("tracking_coverage_policy"),
        "tracking_acceptance_policy": inputs.get("tracking_acceptance_policy"),
        "source_timestamp_policies": tracking["source_timestamp_policies"],
        "evaluation_window": inputs.get("evaluation_window"),
        "tracking_reference_accounting": {
            "raw_command_count": tracking["raw_reference_count"],
            "canonical_source_tick_count": tracking["canonical_reference_count"],
            "exact_heartbeat_collapse_count": tracking["reference_heartbeat_collapsed_count"],
        },
        "dimensions": {
            "mission": _dimension(mission_status, mission_reasons),
            "safety": _dimension(safety_status, safety_reasons),
            "tracking": _dimension(tracking_status, tracking_reasons),
            "motion_quality": _dimension(motion_status, motion_reasons),
            "evidence": _dimension(evidence_dimension_status, completeness_reasons),
        },
        "evidence_contract": evidence_contract,
        "lifecycle": [
            dict(item) for item in inputs.get("lifecycle", [])
            if isinstance(item, dict)
        ],
        "lifecycle_reduction": inputs.get("lifecycle_reduction", {}),
        "world_transactions": [
            dict(item) for item in inputs.get("world_transactions", [])
            if isinstance(item, dict)
        ],
        "world_transaction_reduction": inputs.get(
            "world_transaction_reduction", {}),
        "metrics": {
            **tracking["metrics"],
            "mission_guidance_deviation_xy_m": _guidance_deviation(truth, inputs.get("waypoints", [])),
            "motion_quality": motion,
            "planning": planning,
            "localization": localization,
            "timing": timing,
        },
        "completeness": {
            "status": explicit_evidence_status,
            "reasons": sorted(set(completeness_reasons)),
            "raw_pva_count": len(pva),
            "raw_ground_truth_count": len(truth),
            "monitor_sample_counts": inputs.get("monitor_sample_counts", {}),
            "display_decimation_is_not_used_for_metrics": True,
        },
    }
    result["software_qualification"] = evaluate_software_qualification(inputs)
    result["software_qualification_eligible"] = result[
        "software_qualification"]["software_qualification_eligible"]
    return result

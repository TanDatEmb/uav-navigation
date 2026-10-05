#!/usr/bin/env python3
"""Read-only triage of external-mode session artifacts (W3-T1).

For every session directory it reconstructs, from artifacts only, the chain
"last good solve -> failing solves -> emergency/hold -> adapter safety stop"
and classifies the *initiating* cause.  It never runs SITL, never writes into
the session directory and never proposes or applies a threshold: every number
printed is copied from the session, every label is a deterministic function of
those numbers (rules are listed in ``classify``).

Sources used (all inside one session directory):
  report.json            runtime verdict, scenario outcome, LIO / residual summary
  planning_timeline.jsonl  planner_trace decision records, command_rejection
  execution_timeline.jsonl ground-truth odometry (speed)
  logs/navigation_runtime_node_*.log   WARN/ERROR lines
  logs/px4_navigation_external_mode_node_*.log  first adapter ERROR

Enum tables below are copied from the product headers at the base SHA; the
test asserts they stay aligned with the header text when the headers exist.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable

NOT_MEASURED = "NOT_MEASURED"

# src/planning/navigation_planning/include/navigation_planning/planning_outcome.hpp
OUTCOME_NAMES = [
    "RefinedCompleteBundle", "BaselineCompleteBundle", "DeadlineWithCompleteBundle",
    "RetainedCommittedBundle", "MapEvidenceInsufficient", "NoCompleteBundle",
    "StaleResult", "CancelledByHigherPriorityRequest", "InvalidRequest",
]
STAGE_NAMES = [
    "none", "input", "world_freshness", "route_window", "a_star", "corridor",
    "nominal_seed", "nominal_refinement", "backup_seed", "backup_refinement",
    "dynamic_certificate", "flatness_certificate", "world_certificate",
    "commit_recertification", "deadline",
]
REASON_NAMES = [
    "none", "anchor_out_of_map", "anchor_unknown", "anchor_occupied",
    "main_path_unavailable", "main_corridor_unavailable", "nominal_dynamics",
    "nominal_flatness", "backup_known_free_insufficient", "backup_dynamics",
    "backup_flatness", "world_changed", "no_complete_bundle_at_deadline",
    "stale_result", "superseded", "invalid_input", "main_known_free_insufficient",
    "candidate_export_invalid", "backup_world_blocked",
    "sensing_horizon_insufficient",
    "stop_outside_recovery_envelope", "stop_synthesis_failed", "route_regression",
]
# src/mapping/navigation_world_model/include/navigation_world_model/world_commit_authorizer.hpp
WORLD_COMMIT_NAMES = [
    "kNotAttempted", "kCommitted", "kNoPublishedWorld", "kWorldAdvanced",
    "kSuperseded", "kCancelled", "kCandidateRejected",
]
WORLD_ADVANCED = 3
# src/px4/px4_navigation_external_mode (NavigationCommandRejection.msg)
DISPOSITION_SAFETY_STOP = 3

# Outcomes that are a planner failure (4..7); 8 (InvalidRequest) at cycle 1 is the
# pre-goal startup record and is reported separately, never as a failing solve.
FAILING_OUTCOMES = {4, 5, 6, 7}
SUCCESS_OUTCOMES = {0, 1, 2}

WARN_RE = re.compile(r"\[(WARN|ERROR|INFO)\] \[(\d+\.\d+)\] \[[\w_]+\]: (.*)")


def _name(table: list[str], value: Any) -> str:
    try:
        index = int(float(value))
    except (TypeError, ValueError):
        return NOT_MEASURED
    return table[index] if 0 <= index < len(table) else f"unknown({index})"


def _f(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _jsonl(path: Path) -> Iterable[dict[str, Any]]:
    try:
        handle = path.open(encoding="utf-8", errors="replace")
    except OSError:
        return
    with handle:
        for line in handle:
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict):
                yield value


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _log_lines(session: Path, pattern: str) -> list[tuple[str, float, str]]:
    rows: list[tuple[str, float, str]] = []
    for path in sorted((session / "logs").glob(pattern)):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = WARN_RE.search(line)
            if match:
                rows.append((match.group(1), float(match.group(2)), match.group(3)))
    return rows


def _experiment_name(value: Any) -> str:
    if isinstance(value, dict):
        return str(value.get("name") or NOT_MEASURED)
    return str(value) if value else NOT_MEASURED


def decision_records(session: Path) -> list[dict[str, Any]]:
    """One record per planner solve generation (planner_trace with solve_generation)."""
    out: list[dict[str, Any]] = []
    for item in _jsonl(session / "planning_timeline.jsonl"):
        record = item.get("record", item)
        if record.get("kind") != "planner_trace":
            continue
        values = (record.get("payload") or {}).get("values") or {}
        if "solve_generation" not in values:
            continue
        out.append({
            "sim_s": (record.get("sim_time_ns") or 0) / 1e9,
            "cycle": int(float(values.get("planning_cycle_id", -1))),
            "solve_generation": int(float(values["solve_generation"])),
            "outcome": int(float(values.get("planning_outcome", -1))),
            "stage": int(float(values.get("planning_failure_stage", 0))),
            "reason": int(float(values.get("planning_failure_reason", 0))),
            "candidate_rejection_cause": int(float(values.get(
                "planner_backend_candidate_rejection_cause", 0))),
            "candidate_rejection_cause_name": values.get(
                "planner_backend_candidate_rejection_cause_name"),
            "replan_code": int(float(values.get("replan_code", 0))),
            "commit_decision": int(float(values.get("commit_decision", 0))),
            "solve_deadline_exceeded": values.get("solve_deadline_exceeded") == "1",
            "planning_latency_ms": _f(values.get("planning_latency_ms")),
            "anchor_error_m": _f(values.get("anchor_error_m")),
            "projected_anchor_error_m": _f(values.get("projected_anchor_error_m")),
            "retained_tracking_limit_m": _f(values.get("retained_tracking_limit_m")),
            "tracking_certificate_exceeded": values.get("tracking_certificate_exceeded") == "1",
            "emergency_candidate_commit_result": int(float(
                values.get("emergency_candidate_commit_result", 0))),
            "time_to_backup_start_s": _f(values.get("time_to_backup_start_s")),
            "phase_bridge_usable": values.get("phase_execution_bridge_usable") == "1",
            "path_relative_accepted": values.get("path_relative_tracking_accepted") == "1",
            "path_relative_error_m": _f(values.get("path_relative_error_m")),
            "path_relative_phase_offset_s": _f(values.get("path_relative_phase_offset_s")),
            "anchor_error_time_aligned_m": _f(values.get("anchor_error_time_aligned_m")),
            "pinned_world_revision": int(float(values.get("pinned_world_revision", 0))),
            "certificate_world_revision": int(float(values.get("certificate_world_revision", 0))),
            "planner_disposition": values.get("planner_disposition"),
            "runtime_admission_attempted": values.get("runtime_admission_attempted") == "1",
            "execution_boundary_rejection": int(float(values.get("execution_boundary_rejection", 0))),
            "planning_start_mode": values.get("planning_start_mode"),
        })
    out.sort(key=lambda r: (r["sim_s"], r["solve_generation"]))
    return out


REJECT_WARN_KINDS = (
    ("MAIN route-regression certificate", "ROUTE_REGRESSION_CERTIFICATE"),
    ("rejected by latest WorldModel", "LATEST_WORLD_CERTIFICATE_BLOCKED"),
    ("final yaw-rate certificate", "YAW_RATE_CERTIFICATE"),
    ("final yaw-acceleration certificate", "YAW_ACCELERATION_CERTIFICATE"),
    ("authorization rejected with reason", "COMMIT_GATE_REJECTED"),
    ("no mission command identity", "NO_COMMAND_IDENTITY"),
    ("no WorldModel commit authorizer", "NO_COMMIT_AUTHORIZER"),
    ("no published WorldModel", "NO_PUBLISHED_WORLD"),
)
TYPED_REJECT_KINDS = {
    "ROUTE_REGRESSION": "ROUTE_REGRESSION_CERTIFICATE",
    "WORLD_VALIDATION": "LATEST_WORLD_CERTIFICATE_BLOCKED",
    "YAW_RATE_CERTIFICATE": "YAW_RATE_CERTIFICATE",
    "YAW_ACCELERATION_CERTIFICATE": "YAW_ACCELERATION_CERTIFICATE",
    "INVALID_COMMAND_IDENTITY": "NO_COMMAND_IDENTITY",
    "NO_COMMIT_AUTHORIZER": "NO_COMMIT_AUTHORIZER",
    "NO_PUBLISHED_WORLD": "NO_PUBLISHED_WORLD",
    "COMMIT_TRANSACTION": "COMMIT_GATE_REJECTED",
}


def reject_subtypes(session: Path, decisions: list[dict[str, Any]]) -> dict[str, Any]:
    """Attach a rejection subtype to every replan_code==-7 decision record.

    Prefer the typed producer field introduced by CP-W5-21.  Legacy sessions
    without that field retain the ordered WARN fallback and stay UNMATCHED when
    the warning/decision counts do not match.
    """
    targets = [d for d in decisions if d["replan_code"] == -7]
    warns: list[str] = []
    path = session / "logs" / "mapping.log"
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        text = ""
    for line in text.splitlines():
        if "[planner WARN]" not in line and "[planner ERROR]" not in line:
            continue
        for needle, kind in REJECT_WARN_KINDS:
            if needle in line:
                warns.append(kind)
                break
    typed = [
        d.get("candidate_rejection_cause_name")
        for d in targets
    ]
    typed_available = len(targets) > 0 and all(
        isinstance(value, str) and value not in {"", "none"} for value in typed
    )
    matched = typed_available or len(warns) == len(targets)
    for index, record in enumerate(targets):
        record["reject_subtype"] = (
            TYPED_REJECT_KINDS.get(typed[index].upper(), typed[index].upper())
            if typed_available else warns[index]
            if matched else "UNMATCHED"
        )
    return {"reject_warn_count": len(warns), "reject_record_count": len(targets), "matched": matched}


def rejection_records(session: Path) -> tuple[list[dict[str, Any]], float | None]:
    rejections: list[dict[str, Any]] = []
    first_pva_failure: float | None = None
    for item in _jsonl(session / "planning_timeline.jsonl"):
        record = item.get("record", item)
        kind = record.get("kind")
        payload = record.get("payload") or {}
        if kind == "command_rejection":
            rejections.append({
                "sim_s": (record.get("sim_time_ns") or 0) / 1e9,
                "stage": payload.get("stage"), "reason_code": payload.get("reason_code"),
                "disposition": payload.get("disposition"),
                "bundle_generation": payload.get("bundle_generation"),
                "tracking_longitudinal_error_m": _f(payload.get("tracking_longitudinal_error_m")),
                "tracking_lateral_error_m": _f(payload.get("tracking_lateral_error_m")),
            })
        elif kind == "pva_command_failure" and first_pva_failure is None:
            first_pva_failure = (record.get("sim_time_ns") or 0) / 1e9
    return rejections, first_pva_failure


def speed_profile(session: Path) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for item in _jsonl(session / "execution_timeline.jsonl"):
        record = item.get("record", item)
        if record.get("kind") != "sample" or record.get("stream") != "ground_truth_odometry":
            continue
        payload = record.get("payload") or {}
        velocity = payload.get("linear_velocity")
        stamp = payload.get("stamp_ns")
        if isinstance(velocity, list) and len(velocity) == 3 and stamp is not None:
            pts.append((stamp / 1e9, math.sqrt(sum(float(c) ** 2 for c in velocity))))
    return pts


ENVELOPE_RE = re.compile(
    r"tracking envelope exceeded: longitudinal=([\d.]+)/([\d.]+) m reverse=([\d.]+)/([\d.]+) m "
    r"lateral=([\d.]+)/([\d.]+) m measured_enu=\[([-\d.,]+)\] command_enu=\[([-\d.,]+)\] .*?"
    r"command_velocity_enu=\[([-\d.,]+)\] "
    r"command_acceleration_enu=\[([-\d.,]+)\].*?generation=(\d+) role=(\w+) "
    r"trajectory_time=([\d.]+) s .*?stamp=([\d.]+)")


def _ahead_of_command(session: Path, adapter: dict[str, Any], stop_sim_s: float) -> float | None:
    """Signed distance (measured - command) projected on the ground-truth velocity direction
    0.5 s before the stop (positive = vehicle is ahead of the command along its travel)."""
    best: tuple[float, list[float]] | None = None
    for item in _jsonl(session / "execution_timeline.jsonl"):
        record = item.get("record", item)
        if record.get("kind") != "sample" or record.get("stream") != "ground_truth_odometry":
            continue
        payload = record.get("payload") or {}
        stamp = payload.get("stamp_ns")
        velocity = payload.get("linear_velocity")
        if stamp is None or not isinstance(velocity, list):
            continue
        t = stamp / 1e9
        if t <= stop_sim_s - 0.5 and (best is None or t > best[0]):
            best = (t, [float(c) for c in velocity])
    if best is None:
        return None
    norm = math.sqrt(sum(c * c for c in best[1]))
    if norm < 0.5:
        return None
    delta = [m - c for m, c in zip(adapter["measured_enu"], adapter["command_enu"])]
    return round(sum(d * v / norm for d, v in zip(delta, best[1])), 3)


def adapter_first_error(session: Path) -> dict[str, Any]:
    for level, wall, message in _log_lines(session, "px4_navigation_external_mode_node_*.log"):
        if level != "ERROR":
            continue
        envelope = ENVELOPE_RE.search(message)
        if envelope:
            lon, lon_limit, rev, rev_limit, lat, lat_limit = (float(envelope.group(i)) for i in range(1, 7))
            axis = max((("longitudinal", lon), ("reverse", rev), ("lateral", lat)), key=lambda x: x[1])[0]
            accel = [float(x) for x in envelope.group(10).split(",")]
            return {
                "kind": "ADAPTER_TRACKING_ENVELOPE", "wall_s": wall, "axis": axis,
                "longitudinal_m": lon, "reverse_m": rev, "lateral_m": lat,
                "limit_m": lon_limit, "role": envelope.group(12),
                "bundle_generation": int(envelope.group(11)),
                "trajectory_time_s": float(envelope.group(13)),
                "command_stamp_s": float(envelope.group(14)),
                "measured_enu": [float(x) for x in envelope.group(7).split(",")],
                "command_enu": [float(x) for x in envelope.group(8).split(",")],
                "command_accel_norm_mps2": round(math.sqrt(sum(a * a for a in accel)), 3),
            }
        if "command stale" in message:
            return {"kind": "ADAPTER_COMMAND_STALE", "wall_s": wall, "message": message[:160]}
        if "odometry stale" in message:
            return {"kind": "ADAPTER_ODOMETRY_STALE", "wall_s": wall, "message": message[:160]}
        return {"kind": "ADAPTER_OTHER_ERROR", "wall_s": wall, "message": message[:160]}
    return {"kind": "NONE"}


def wall_to_sim_s(session: Path) -> list[tuple[float, float]]:
    """(wall_s, sim_s) pairs from world_transaction witnesses, for estimating sim time of a log line."""
    pairs: list[tuple[float, float]] = []
    for item in _jsonl(session / "planning_timeline.jsonl"):
        record = item.get("record", item)
        if record.get("kind") != "world_transaction":
            continue
        payload = record.get("payload") or {}
        wall = payload.get("observer_receive_ros_ns")
        sim = payload.get("event_header_ros_ns")
        if isinstance(wall, (int, float)) and isinstance(sim, (int, float)):
            pairs.append((wall / 1e9, sim / 1e9))
    pairs.sort()
    return pairs


def estimate_sim_s(pairs: list[tuple[float, float]], wall_s: float) -> float | None:
    if not pairs:
        return None
    nearest = min(pairs, key=lambda p: abs(p[0] - wall_s))
    return round(nearest[1] + (wall_s - nearest[0]), 2)


RUNTIME_ERROR_KINDS = (
    ("rejected planned STOPPED_HOLD endpoint", "RUNTIME_STOPPED_HOLD_ENDPOINT_REJECTED"),
    ("PlanFromRest recovery timeout", "RUNTIME_PLANFROMREST_TIMEOUT"),
    ("hot replan failed without a valid safety suffix", "RUNTIME_NO_SAFETY_SUFFIX"),
    ("publishing rejected command planner_failed=1", "RUNTIME_PLANNER_FAILED_LATCHED"),
)


def runtime_first_error(session: Path) -> dict[str, Any]:
    for level, wall, message in _log_lines(session, "navigation_runtime_node_*.log"):
        if level != "ERROR":
            continue
        for needle, kind in RUNTIME_ERROR_KINDS:
            if needle in message:
                detail = re.search(r"anchor_error_m=([\d.]+) anchor_limit_m=([\d.]+)", message)
                out: dict[str, Any] = {"kind": kind, "wall_s": wall}
                if detail:
                    out["anchor_error_m"] = float(detail.group(1))
                    out["anchor_limit_m"] = float(detail.group(2))
                return out
        return {"kind": "RUNTIME_OTHER_ERROR", "wall_s": wall, "message": message[:160]}
    return {"kind": "NONE"}


def runtime_log_summary(session: Path) -> dict[str, Any]:
    lines = _log_lines(session, "navigation_runtime_node_*.log")
    counters: collections.Counter[str] = collections.Counter()
    first: dict[str, float] = {}
    patterns = {
        "emergency_anchor": "one-shot emergency altitude anchor",
        "hot_replan_failed": "hot replan failed",
        "stopped_hold_endpoint_rejected": "rejected planned STOPPED_HOLD endpoint",
        "planner_failed_publish_rejected": "publishing rejected command planner_failed=1",
        "planfromrest_timeout": "PlanFromRest recovery timeout",
        "main_reserve_rejected": "insufficient MAIN reserve",
        "localization_epoch_changed": "Localization epoch changed",
        "emergency_certified_stop": "emergency brake reached a certified stop",
    }
    for level, wall, message in lines:
        for key, needle in patterns.items():
            if needle in message:
                counters[key] += 1
                first.setdefault(key, wall)
    return {"counts": dict(counters), "first_wall_s": first}


def _tx_sim_s(transaction: dict[str, Any]) -> float | None:
    for phase in ("authorize", "request", "retained", "publish"):
        event = (transaction.get("events") or {}).get(phase) or {}
        stamp = event.get("source_stamp_ns")
        if isinstance(stamp, (int, float)):
            return stamp / 1e9
    return None


def lifecycle_outcomes(report: dict[str, Any], stop_sim_s: float | None) -> dict[str, Any]:
    """Terminal outcomes of the lifecycle reduction, split at the adapter stop.

    NO_EXECUTION_AUTHORITY is produced for NO_EXECUTION_SIGNAL transactions (evaluation.py
    629): one per rejected 50 Hz command sample once the runtime has latched planner_failed.
    It is therefore a *consequence* observable only after the failure.
    """
    transactions = (((report.get("evaluation") or {}).get("lifecycle_reduction") or {})
                    .get("transactions") or [])
    by_outcome: collections.Counter[str] = collections.Counter()
    pre: collections.Counter[str] = collections.Counter()
    post: collections.Counter[str] = collections.Counter()
    for item in transactions:
        outcome = str(item.get("terminal_outcome"))
        by_outcome[outcome] += 1
        stamp = _tx_sim_s(item)
        if stop_sim_s is None or stamp is None:
            continue
        (pre if stamp <= stop_sim_s else post)[outcome] += 1
    first_tx = next((t.get("terminal_outcome") for t in transactions
                     if t.get("terminal_outcome") is not None), None)
    return {"transaction_count": len(transactions), "by_terminal_outcome": dict(by_outcome),
            "pre_stop_by_terminal_outcome": dict(pre),
            "post_stop_by_terminal_outcome": dict(post),
            "first_listed_terminal_outcome": first_tx}


def estimator_summary(report: dict[str, Any]) -> dict[str, Any]:
    lio = report.get("lio") or {}
    tracking = report.get("tracking") or {}
    px4 = report.get("px4") or {}
    ate = ((tracking.get("lio_vs_ground_truth") or {}).get("ate") or {}).get("norm") or {}
    return {
        "lio_state": lio.get("state", NOT_MEASURED),
        "lio_navigation_valid": lio.get("navigation_valid", NOT_MEASURED),
        "lio_observability_rejection_count": lio.get("observability_rejection_count", NOT_MEASURED),
        "lio_absolute_guard_trigger_count": ((lio.get("map_maintenance") or {})
                                             .get("absolute_guard_trigger_count", NOT_MEASURED)),
        "lio_ate_max_m": ate.get("maximum", NOT_MEASURED),
        "px4_estimator_fault_events": px4.get("estimator_fault_events", NOT_MEASURED),
        "px4_dead_reckoning_events": px4.get("dead_reckoning_events", NOT_MEASURED),
    }


def classify(chain: dict[str, Any]) -> dict[str, Any]:
    """Deterministic labels.  Inputs are numbers extracted above.

    initiator = class of the FIRST failing solve of the final failing streak (the failing solves
    between the last successful solve and the stop):
      COMMIT_GATE_WORLD_ADVANCED   commit_decision==kWorldAdvanced        -> ADR-020 G-1
      CERT_LATEST_WORLD_BLOCKED    rc -7, planner WARN "rejected by latest WorldModel"
                                   (candidate tube hits an OCCUPIED cell of the newest world)
      CERT_ROUTE_REGRESSION        rc -7, planner WARN "MAIN route-regression certificate"
      CERT_REJECTED_OTHER          rc -7, any other / UNMATCHED subtype
      SOLVE_KNOWN_FREE             reason in {backup_known_free_insufficient,
                                   main_known_free_insufficient, backup_world_blocked}
      SOLVE_DYNAMICS               reason in {nominal_dynamics, nominal_flatness, backup_dynamics,
                                   backup_flatness, stop_outside_recovery_envelope,
                                   stop_synthesis_failed}
      SOLVE_DEADLINE               reason no_complete_bundle_at_deadline / stage deadline  -> G-4
      SOLVE_OTHER                  any other failing solve
      NO_SOLVE_FAILURE             empty streak
    The commit_recertification/world_changed label of planner_trace is NOT used: it is emitted for
    every rc -7 regardless of cause (see reject_subtypes).
    """
    streak = chain["failing_streak"]
    if not streak:
        return {"initiator": "NO_SOLVE_FAILURE"}
    head = streak[0]
    reason = _name(REASON_NAMES, head["reason"])
    stage = _name(STAGE_NAMES, head["stage"])
    if head["commit_decision"] == WORLD_ADVANCED:
        initiator = "COMMIT_GATE_WORLD_ADVANCED"
    elif head["replan_code"] == -7:
        initiator = {
            "LATEST_WORLD_CERTIFICATE_BLOCKED": "CERT_LATEST_WORLD_BLOCKED",
            "ROUTE_REGRESSION_CERTIFICATE": "CERT_ROUTE_REGRESSION",
        }.get(head.get("reject_subtype", ""), "CERT_REJECTED_OTHER")
    elif reason in {"backup_known_free_insufficient", "main_known_free_insufficient",
                    "backup_world_blocked"}:
        initiator = "SOLVE_KNOWN_FREE"
    elif reason in {"nominal_dynamics", "nominal_flatness", "backup_dynamics",
                    "backup_flatness", "stop_outside_recovery_envelope", "stop_synthesis_failed"}:
        initiator = "SOLVE_DYNAMICS"
    elif reason == "no_complete_bundle_at_deadline" or stage == "deadline" \
            or head["solve_deadline_exceeded"]:
        initiator = "SOLVE_DEADLINE"
    else:
        initiator = "SOLVE_OTHER"
    return {"initiator": initiator}


def triage_session(session: Path) -> dict[str, Any]:
    report = _json(session / "report.json")
    scenario = _json(session / "scenario.json")
    metadata = _json(session / "metadata.json")
    decisions = decision_records(session)
    reject_match = reject_subtypes(session, decisions)
    rejections, first_pva_failure = rejection_records(session)
    adapter = adapter_first_error(session)
    runtime_error = runtime_first_error(session)
    runtime_log = runtime_log_summary(session)

    stop_rejection = next((r for r in rejections
                           if r["disposition"] == DISPOSITION_SAFETY_STOP), None)
    stop_sim_s = stop_rejection["sim_s"] if stop_rejection else first_pva_failure
    stop_source = ("command_rejection(SAFETY_STOP)" if stop_rejection else
                   "first pva_command_failure" if first_pva_failure is not None else NOT_MEASURED)
    if stop_sim_s is None:
        wall = (runtime_error.get("wall_s") if runtime_error["kind"] != "NONE"
                else adapter.get("wall_s"))
        if wall is not None:
            estimate = estimate_sim_s(wall_to_sim_s(session), wall)
            if estimate is not None:
                stop_sim_s, stop_source = estimate, "estimated from first ERROR log line via world_transaction clock pairs"

    real = [d for d in decisions if d["outcome"] != 8]  # drop startup InvalidRequest record
    before_stop = [d for d in real if stop_sim_s is None or d["sim_s"] <= stop_sim_s]
    streak: list[dict[str, Any]] = []
    for record in reversed(before_stop):
        if record["outcome"] in FAILING_OUTCOMES or record["replan_code"] < 0:
            streak.append(record)
        else:
            break
    streak.reverse()
    last_success = next((d for d in reversed(before_stop)
                         if d["outcome"] in SUCCESS_OUTCOMES and d["replan_code"] > 0), None)

    hist: collections.Counter[str] = collections.Counter()
    for d in real:
        if d["outcome"] in FAILING_OUTCOMES or d["replan_code"] < 0:
            hist[f"{_name(STAGE_NAMES, d['stage'])}/{_name(REASON_NAMES, d['reason'])}"] += 1
    commit_hist = collections.Counter(_name(WORLD_COMMIT_NAMES, d["commit_decision"]) for d in real)
    reject_hist = collections.Counter(d.get("reject_subtype") for d in real if d["replan_code"] == -7)
    emergencies = [d for d in real if d["emergency_candidate_commit_result"] == 1]
    first_emergency = emergencies[0] if emergencies else None

    speeds = speed_profile(session)

    def speed_at(t: float) -> float | None:
        if not speeds:
            return None
        return min(speeds, key=lambda point: abs(point[0] - t))[1]

    emergency_events = [{
        "sim_s": round(d["sim_s"], 2),
        "speed_mps": (round(speed_at(d["sim_s"]), 2) if speed_at(d["sim_s"]) is not None else None),
        "anchor_error_m": d["anchor_error_m"],
        "tracking_certificate_exceeded": d["tracking_certificate_exceeded"],
        "followed_by_stop_within_3s": (isinstance(stop_sim_s, float)
                                       and 0.0 <= stop_sim_s - d["sim_s"] <= 3.0),
    } for d in emergencies]
    failing_solves = [d for d in real if d["outcome"] in FAILING_OUTCOMES or d["replan_code"] < 0]
    pre_stop_speeds = [v for t, v in speeds if stop_sim_s is None or t <= stop_sim_s]
    speed_at_stop = pre_stop_speeds[-1] if pre_stop_speeds else None
    speed_max = max(pre_stop_speeds) if pre_stop_speeds else None

    if adapter.get("kind") == "ADAPTER_TRACKING_ENVELOPE" and speeds and isinstance(stop_sim_s, float):
        adapter["vehicle_ahead_of_command_m"] = _ahead_of_command(session, adapter, stop_sim_s)

    scenario_outcome = ((report.get("mission_outcome") or {}).get("scenario") or {}).get("outcome")
    chain = {
        "session": session.name,
        "runtime_verdict": report.get("runtime_verdict", NOT_MEASURED),
        "scenario_outcome": scenario_outcome or NOT_MEASURED,
        "mission_reasons": ((report.get("mission_outcome") or {}).get("acceptance") or {}).get("reasons"),
        "backup_evidence_experiment": _experiment_name(
            scenario.get("backup_evidence_experiment") or metadata.get("backup_evidence_experiment")),
        "adapter_first_error": adapter,
        "runtime_first_error": runtime_error,
        "stop_sim_s": stop_sim_s if stop_sim_s is not None else NOT_MEASURED,
        "stop_sim_source": stop_source,
        "solve_count": len(real),
        "success_count": sum(1 for d in real if d["outcome"] in SUCCESS_OUTCOMES and d["replan_code"] > 0),
        "failure_count": sum(hist.values()),
        "failure_hist_stage_reason": dict(hist),
        "commit_decision_hist": dict(commit_hist),
        "reject_subtype_hist": dict(reject_hist),
        "reject_subtype_match": reject_match,
        "commit_world_advanced_count": commit_hist.get("kWorldAdvanced", 0),
        "deadline_exceeded_count": sum(1 for d in real if d["solve_deadline_exceeded"]),
        "last_success_sim_s": last_success["sim_s"] if last_success else NOT_MEASURED,
        "failing_streak": streak,
        "failing_streak_len": len(streak),
        "first_failure_sim_s": streak[0]["sim_s"] if streak else NOT_MEASURED,
        "first_failure_stage_reason": (
            f"{_name(STAGE_NAMES, streak[0]['stage'])}/{_name(REASON_NAMES, streak[0]['reason'])}"
            if streak else NOT_MEASURED),
        "first_failure_replan_code": streak[0]["replan_code"] if streak else NOT_MEASURED,
        "first_failure_time_to_backup_start_s": (streak[0]["time_to_backup_start_s"] if streak
                                                 else NOT_MEASURED),
        "first_emergency_sim_s": first_emergency["sim_s"] if first_emergency else NOT_MEASURED,
        "first_emergency_anchor_error_m": first_emergency["anchor_error_m"] if first_emergency else NOT_MEASURED,
        "first_emergency_retained_tracking_limit_m": (first_emergency["retained_tracking_limit_m"]
                                                      if first_emergency else NOT_MEASURED),
        "first_emergency_tracking_certificate_exceeded": (first_emergency["tracking_certificate_exceeded"]
                                                          if first_emergency else NOT_MEASURED),
        "emergency_count": len(emergencies),
        "emergency_events": emergency_events,
        "failing_solves_detail": [{k: d[k] for k in (
            "sim_s", "replan_code", "anchor_error_m", "retained_tracking_limit_m",
            "phase_bridge_usable", "path_relative_accepted", "path_relative_phase_offset_s",
            "emergency_candidate_commit_result", "tracking_certificate_exceeded")} for d in failing_solves],
        "failing_solve_tracking_certificate_exceeded_count": sum(
            1 for d in failing_solves if d["tracking_certificate_exceeded"]),
        "speed_at_stop_mps": speed_at_stop if speed_at_stop is not None else NOT_MEASURED,
        "speed_max_before_stop_mps": speed_max if speed_max is not None else NOT_MEASURED,
        "runtime_log": runtime_log,
        "lifecycle": lifecycle_outcomes(
            report, stop_sim_s if isinstance(stop_sim_s, float) else None),
        "estimator": estimator_summary(report),
    }
    chain.update(classify(chain))
    return chain


def flat_row(chain: dict[str, Any]) -> dict[str, Any]:
    adapter = chain["adapter_first_error"]
    stop = chain["stop_sim_s"] if isinstance(chain["stop_sim_s"], float) else None
    prior = [e for e in chain["emergency_events"] if stop is None or e["sim_s"] <= stop]
    last_emergency = prior[-1] if prior else None
    return {
        "session": chain["session"],
        "runtime_verdict": chain["runtime_verdict"],
        "scenario_outcome": chain["scenario_outcome"],
        "policy": chain["backup_evidence_experiment"],
        "initiator": chain["initiator"],
        "terminal_trigger": adapter["kind"] if adapter["kind"] == "ADAPTER_TRACKING_ENVELOPE"
        else f"{chain['runtime_first_error']['kind']}(adapter:{adapter['kind']})",
        "runtime_first_error": chain["runtime_first_error"]["kind"],
        "runtime_stopped_hold_anchor_error_m": chain["runtime_first_error"].get("anchor_error_m", ""),
        "stop_sim_source": chain["stop_sim_source"],
        "terminal_axis": adapter.get("axis", ""),
        "terminal_role": adapter.get("role", ""),
        "terminal_error_m": max(adapter.get("longitudinal_m", 0.0), adapter.get("reverse_m", 0.0),
                                adapter.get("lateral_m", 0.0)) if adapter["kind"] == "ADAPTER_TRACKING_ENVELOPE" else "",
        "terminal_command_accel_mps2": adapter.get("command_accel_norm_mps2", ""),
        "terminal_vehicle_ahead_of_command_m": adapter.get("vehicle_ahead_of_command_m", ""),
        "stop_sim_s": chain["stop_sim_s"],
        "last_success_sim_s": chain["last_success_sim_s"],
        "first_failure_sim_s": chain["first_failure_sim_s"],
        "first_failure_stage_reason": chain["first_failure_stage_reason"],
        "first_failure_replan_code": chain["first_failure_replan_code"],
        "failing_streak_len": chain["failing_streak_len"],
        "solve_count": chain["solve_count"],
        "success_count": chain["success_count"],
        "failure_count": chain["failure_count"],
        "commit_world_advanced_count": chain["commit_world_advanced_count"],
        "reject_subtype_hist": json.dumps(chain["reject_subtype_hist"], sort_keys=True),
        "pre_stop_lifecycle": json.dumps(chain["lifecycle"]["pre_stop_by_terminal_outcome"], sort_keys=True),
        "post_stop_lifecycle": json.dumps(chain["lifecycle"]["post_stop_by_terminal_outcome"], sort_keys=True),
        "first_listed_terminal_outcome": chain["lifecycle"]["first_listed_terminal_outcome"],
        "deadline_exceeded_count": chain["deadline_exceeded_count"],
        "emergency_count": chain["emergency_count"],
        "last_emergency_before_stop_sim_s": last_emergency["sim_s"] if last_emergency else "",
        "last_emergency_before_stop_speed_mps": last_emergency["speed_mps"] if last_emergency else "",
        "last_emergency_before_stop_anchor_error_m": last_emergency["anchor_error_m"] if last_emergency else "",
        "first_emergency_anchor_error_m": chain["first_emergency_anchor_error_m"],
        "first_emergency_retained_tracking_limit_m": chain["first_emergency_retained_tracking_limit_m"],
        "speed_at_stop_mps": chain["speed_at_stop_mps"],
        "speed_max_before_stop_mps": chain["speed_max_before_stop_mps"],
        "lio_state": chain["estimator"]["lio_state"],
        "lio_ate_max_m": chain["estimator"]["lio_ate_max_m"],
        "px4_estimator_fault_events": chain["estimator"]["px4_estimator_fault_events"],
        "failure_hist": json.dumps(chain["failure_hist_stage_reason"], sort_keys=True),
    }


def aggregate(chains: list[dict[str, Any]]) -> dict[str, Any]:
    """Counts over the supplied sessions (no thresholds, no pass/fail judgement)."""
    def tally(values: Iterable[Any]) -> dict[str, int]:
        return dict(collections.Counter(str(v) for v in values))

    blocked = [c for c in chains if c["scenario_outcome"] == "PAUSED_SAFETY_STOP"]
    solves = sum(c["solve_count"] for c in chains)
    failures = sum(c["failure_count"] for c in chains)
    emergencies = [e for c in chains for e in c["emergency_events"]]
    return {
        "sessions": len(chains),
        "paused_safety_stop_sessions": len(blocked),
        "initiator_of_paused_safety_stop": tally(c["initiator"] for c in blocked),
        "terminal_of_paused_safety_stop": tally(
            c["adapter_first_error"]["kind"] if c["adapter_first_error"]["kind"] == "ADAPTER_TRACKING_ENVELOPE"
            else c["runtime_first_error"]["kind"] for c in blocked),
        "solves": solves,
        "failing_solves": failures,
        "commit_decision_hist_all": dict(sum((collections.Counter(c["commit_decision_hist"]) for c in chains),
                                             collections.Counter())),
        "reject_subtype_hist_all": dict(sum((collections.Counter(c["reject_subtype_hist"]) for c in chains),
                                            collections.Counter())),
        "failure_hist_all": dict(sum((collections.Counter(c["failure_hist_stage_reason"]) for c in chains),
                                     collections.Counter())),
        "emergencies": len(emergencies),
        "paused_sessions_with_emergency_within_3s_before_stop": sum(
            1 for c in blocked if any(e["followed_by_stop_within_3s"] for e in c["emergency_events"])),
        "paused_sessions_whose_first_emergency_is_within_3s_before_stop": sum(
            1 for c in blocked if c["emergency_events"] and c["emergency_events"][0]["followed_by_stop_within_3s"]),
        "paused_sessions_failing_solve_with_tracking_certificate_exceeded": sum(
            1 for c in blocked if c["failing_solve_tracking_certificate_exceeded_count"] > 0),
        "sessions_with_world_advanced_in_final_streak": sum(
            1 for c in blocked if any(d["commit_decision"] == WORLD_ADVANCED for d in c["failing_streak"])),
        "sessions_with_any_world_advanced": sum(1 for c in chains if c["commit_world_advanced_count"] > 0),
        "failing_solves_by_bridge_and_anchor": failing_solve_bridge_table(chains),
    }


def failing_solve_bridge_table(chains: list[dict[str, Any]]) -> dict[str, int]:
    """Cross-tab of failing solves by retained-command MAIN bridge availability.

    A failing solve with a finite retained anchor error is counted under
    bridge_usable / bridge_unavailable and, within each, by whether the same decision record
    committed the one-shot emergency brake.
    """
    out: collections.Counter[str] = collections.Counter()
    for chain in chains:
        for record in chain["failing_solves_detail"]:
            if record["anchor_error_m"] is None:
                out["anchor_error_not_measured"] += 1
                continue
            key = "bridge_usable" if record["phase_bridge_usable"] else "bridge_unavailable"
            exceeded = "anchor_above_retained_limit" if (
                record["retained_tracking_limit_m"] is not None
                and record["anchor_error_m"] > record["retained_tracking_limit_m"]) else "anchor_within_limit"
            out[f"{key}/{exceeded}/emergency_committed={record['emergency_candidate_commit_result'] == 1}"] += 1
    return dict(out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sessions", nargs="+", type=Path,
                        help="session directories (external-mode-check-*)")
    parser.add_argument("--json", type=Path, help="write full per-session chains as JSON")
    parser.add_argument("--csv", type=Path, help="write flat per-session table as CSV")
    args = parser.parse_args(argv)
    chains = [triage_session(path) for path in args.sessions]
    if args.json:
        args.json.write_text(json.dumps({"aggregate": aggregate(chains), "sessions": chains},
                                        indent=1, sort_keys=True, default=str), encoding="utf-8")
    rows = [flat_row(c) for c in chains]
    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    if not args.json and not args.csv:
        json.dump(rows, sys.stdout, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

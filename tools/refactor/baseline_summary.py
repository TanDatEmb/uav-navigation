#!/usr/bin/env python3
"""Summarize W3-C1 sessions without re-judging product behavior.

The session-owned report remains the verdict authority.  This tool only joins
explicit fields from report/scenario/monitor/runtime/provenance artifacts and
uses ``NOT_MEASURED`` when a producer did not emit a requested measurement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable

from tools.refactor import triage_sessions


NOT_MEASURED = "NOT_MEASURED"
POLICY_NAMES = {
    "raycasting_on_backup_strict": "SAFE",
    "raycasting_on_backup_unknown": "FAST",
    "raycasting_off_backup_strict": "SAFE",
    "raycasting_off_backup_unknown": "FAST",
}
METRIC_ALIASES = {
    "adr_m3_mapping_update_us": (
        "mapping_total_update_us", "mapping_callback_total_us",
        "world_model_update_us", "mapping_update_us",
    ),
    "adr_m4_solve_us": (
        "solve_us", "solver_latency_us", "planning_trajectory_optimization_us",
    ),
    "adr_m4_non_solve_us": (
        "non_solve_us", "planning_worker_non_solve_us",
    ),
    "adr_m6_certify_us": (
        "certify_us", "authorize_and_stage_us", "authorizeAndStage_us",
    ),
    "runtime_admission_us": ("runtime_admission_us",),
    "bundle_duration_us": (
        "bundle_duration_us", "declared_duration_us", "committed_bundle_duration_us",
    ),
    "phase_ms": ("phase_ms", "mapping_commit_phase_ms"),
}


def _load(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _walk(value: Any) -> Iterable[tuple[str, Any]]:
    if isinstance(value, dict):
        for key, child in value.items():
            yield str(key), child
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def _first(data: Iterable[Any], names: Iterable[str]) -> Any:
    wanted = set(names)
    for item in data:
        for key, value in _walk(item):
            if key in wanted and value is not None:
                return value
    return None


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        value = value.strip().lower()
        if value in {"true", "yes", "on", "1"}:
            return True
        if value in {"false", "no", "off", "0"}:
            return False
    return None


def _jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return rows
    for line in lines:
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict):
            rows.append(value)
    return rows


def _policy(scenario: dict[str, Any], metadata: dict[str, Any], report: dict[str, Any]) -> tuple[str, str]:
    experiment = _first((scenario, metadata, report), ("backup_evidence_experiment",))
    name = ""
    allow_unknown: bool | None = None
    if isinstance(experiment, dict):
        name = str(experiment.get("name") or experiment.get("id") or "")
        allow_unknown = _bool(experiment.get("backup_allow_unknown"))
        if allow_unknown is None:
            policy = str(experiment.get("backup_unknown_policy") or "").lower()
            if policy == "allow_unknown":
                allow_unknown = True
            elif policy == "require_known_free":
                allow_unknown = False
    elif isinstance(experiment, str):
        name = experiment
    if allow_unknown is None and name in POLICY_NAMES:
        allow_unknown = POLICY_NAMES[name] == "FAST"
    if allow_unknown is None:
        return "UNCLASSIFIED", name
    return ("FAST" if allow_unknown else "SAFE"), name


def _metric_values(session: Path, sources: list[Any]) -> dict[str, list[float]]:
    values = {name: [] for name in METRIC_ALIASES}
    raw: list[Any] = list(sources)
    raw.extend(_jsonl(session / "samples.jsonl"))
    raw.extend(_jsonl(session / "scenario.jsonl"))
    for item in raw:
        for metric, aliases in METRIC_ALIASES.items():
            found = []
            for key, value in _walk(item):
                if key in aliases:
                    number = _number(value)
                    if number is not None:
                        found.append(number)
            if found:
                values[metric].extend(found)
    # Explicit phase durations may be emitted as start/end steady timestamps.
    for item in raw:
        flat = dict(_walk(item))
        start = _number(flat.get("runtime_admission_started_steady_ns"))
        end = _number(flat.get("runtime_admission_finished_steady_ns"))
        if start is not None and end is not None and end >= start:
            values["runtime_admission_us"].append((end - start) / 1000.0)
        start = _number(flat.get("planner_solve_started_steady_ns"))
        end = _number(flat.get("planner_solve_finished_steady_ns"))
        if start is not None and end is not None and end >= start:
            values["adr_m4_solve_us"].append((end - start) / 1000.0)
    return values


def _px4_binary_sha(sources: list[Any]) -> Any:
    explicit = _first(sources, ("px4_binary_sha256", "px4_sha256"))
    if explicit is not None:
        return explicit
    for source in sources:
        if not isinstance(source, dict):
            continue
        stack = [source]
        while stack:
            value = stack.pop()
            if isinstance(value, dict):
                path = str(value.get("path") or value.get("resolved_path") or "")
                if path.endswith("/bin/px4") and isinstance(value.get("sha256"), str):
                    return value["sha256"]
                stack.extend(value.values())
            elif isinstance(value, list):
                stack.extend(value)
    return None


def distribution(values: list[float]) -> dict[str, Any] | str:
    if not values:
        return NOT_MEASURED
    ordered = sorted(values)

    def at(fraction: float) -> float:
        return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]

    return {
        "n": len(ordered),
        "p50": at(0.50),
        "p95": at(0.95),
        "p99": at(0.99),
        "max": max(ordered),
    }


def _manifest_source(sources: list[Any]) -> dict[str, Any]:
    for source in sources:
        if not isinstance(source, dict):
            continue
        for key in ("build_provenance", "provenance", "captured_provenance"):
            value = source.get(key)
            if not isinstance(value, dict):
                continue
            candidates = [value]
            if isinstance(value.get("manifest"), dict):
                candidates.append(value["manifest"])
            if isinstance(value.get("source"), dict):
                candidates.append(value)
            for candidate in candidates:
                manifest_source = candidate.get("source")
                if isinstance(manifest_source, dict) and "git_head" in manifest_source:
                    return manifest_source
    return {}


def _status(policy: str, report: dict[str, Any], required: list[Path], provenance_dirty: bool) -> str:
    if provenance_dirty or policy == "UNCLASSIFIED" or any(not path.is_file() for path in required):
        return "NOT_EVALUABLE"
    assessment = report.get("evaluation")
    if isinstance(assessment, dict):
        assessment_status = str(assessment.get("assessment_status") or "").upper()
        if assessment_status == "NOT_EVALUABLE":
            return "NOT_EVALUABLE"
        if assessment_status == "FAIL":
            return "FAIL"
        if assessment_status == "PASS":
            return "PASS"
    verdict = str(report.get("runtime_verdict") or report.get("verdict") or "").upper()
    if verdict == "FAIL":
        return "FAIL"
    if verdict == "PASS":
        return "PASS"
    return "NOT_EVALUABLE"


def summarize_session(session: Path, *, matrix: str = "UNKNOWN", run_idx: int | str = "UNKNOWN") -> dict[str, Any]:
    session = Path(session).resolve()
    metadata = _load(session / "metadata.json")
    scenario = _load(session / "scenario.json")
    monitor = _load(session / "monitor.json")
    runtime = _load(session / "runtime.json")
    report = _load(session / "report.json")
    policy, experiment = _policy(scenario, metadata, report)
    explicit_matrix = _first((metadata, scenario), ("matrix", "baseline_matrix"))
    if explicit_matrix is not None:
        matrix = str(explicit_matrix)
    explicit_idx = _first((metadata, scenario), ("run_idx", "run_index", "baseline_run_index"))
    if explicit_idx is not None:
        run_idx = explicit_idx
    speed = _first((scenario, metadata), ("requested_cruise_speed_mps", "speed_mps", "target_speed_mps"))
    scene = _first((scenario, metadata), ("requested_scene", "map_scene", "resolved_profile", "scene")) or "UNKNOWN"
    experiment_id = _first((metadata, scenario), ("experiment_id",)) or session.name
    match = re.search(r"W3-C1-(M[12])-[^-]+-[^-]+-(\d+)", str(experiment_id))
    if match:
        matrix, run_idx = match.group(1), int(match.group(2))
    sources = [metadata, scenario, monitor, runtime, report]
    values = _metric_values(session, sources)
    manifest_source = _manifest_source(sources)
    provenance = manifest_source or None
    nav_sha = manifest_source.get("git_head", NOT_MEASURED)
    px4_sha = _px4_binary_sha(sources) or NOT_MEASURED
    infra = _first(sources, ("infrastructure_invalid",))
    infra_value = _bool(infra)
    provenance_dirty = manifest_source.get("git_dirty") is True or _first(
        sources, ("provenance_dirty",)
    ) is True
    required = [session / name for name in ("metadata.json", "scenario.json", "monitor.json", "runtime.json", "report.json")]
    status = _status(policy, report, required, provenance_dirty)
    try:
        cause = str(triage_sessions.triage_session(session).get("initiator", NOT_MEASURED))
    except (OSError, ValueError, TypeError, KeyError):
        cause = NOT_MEASURED
    row: dict[str, Any] = {
        "session": str(session), "experiment_id": str(experiment_id), "matrix": matrix,
        "scene": scene, "speed_mps": speed if speed is not None else NOT_MEASURED,
        "policy": policy, "backup_evidence_experiment": experiment or NOT_MEASURED,
        "run_idx": run_idx, "runtime_verdict": report.get("runtime_verdict", report.get("verdict", NOT_MEASURED)),
        "classification_status": status,
        "outcome": _first((report, scenario, metadata), ("outcome", "terminal_outcome")) or NOT_MEASURED,
        "infrastructure_invalid": infra_value if infra_value is not None else NOT_MEASURED,
        "cause": cause,
        "artifact_status": "COMPLETE" if all(path.is_file() for path in required) else "INCOMPLETE",
        "provenance_status": "AVAILABLE" if provenance is not None else NOT_MEASURED,
        "nav_build_sha": nav_sha, "px4_binary_sha256": px4_sha,
        "provenance_dirty": provenance_dirty,
        "metrics_json": json.dumps({name: distribution(items) for name, items in values.items()}, sort_keys=True),
    }
    for metric, items in values.items():
        summary = distribution(items)
        if isinstance(summary, dict):
            row[metric] = summary["p50"]
            for key in ("n", "p50", "p95", "p99", "max"):
                row[f"{metric}_{key}"] = summary[key]
        else:
            row[metric] = NOT_MEASURED
            for key in ("n", "p50", "p95", "p99", "max"):
                row[f"{metric}_{key}"] = NOT_MEASURED
    return row


def _csv_value(value: Any) -> Any:
    if value is None:
        return NOT_MEASURED
    return value


def write_summary(rows: list[dict[str, Any]], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    run_fields = list(rows[0].keys()) if rows else ["session"]
    runs_path = output_dir / "baseline_runs.csv"
    with runs_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=run_fields)
        writer.writeheader()
        writer.writerows({key: _csv_value(row.get(key)) for key in run_fields} for row in rows)

    metrics = list(METRIC_ALIASES)
    distribution_fields = ["matrix", "scene", "speed_mps", "policy", "metric", "n", "p50", "p95", "p99", "max", "status"]
    distribution_path = output_dir / "baseline_distribution.csv"
    groups: dict[tuple[str, str, str, str, str], list[float]] = {}
    group_keys: set[tuple[str, str, str, str, str]] = set()
    for row in rows:
        for metric in metrics:
            key = (str(row["matrix"]), str(row["scene"]), str(row["speed_mps"]), str(row["policy"]), metric)
            group_keys.add(key)
            values = json.loads(row["metrics_json"]).get(metric, NOT_MEASURED)
            if isinstance(values, dict):
                # Per-session summaries are kept in baseline_runs; distribution
                # aggregation remains explicit and never treats missing data as 0.
                value = values.get("p50")
                if isinstance(value, (int, float)):
                    groups.setdefault(key, []).append(float(value))
    with distribution_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=distribution_fields)
        writer.writeheader()
        for key in sorted(group_keys):
            summary = distribution(groups[key]) if key in groups else NOT_MEASURED
            payload = {"matrix": key[0], "scene": key[1], "speed_mps": key[2], "policy": key[3], "metric": key[4]}
            if isinstance(summary, dict):
                payload.update(summary, status="MEASURED")
            else:
                payload.update({"n": 0, "p50": NOT_MEASURED, "p95": NOT_MEASURED, "p99": NOT_MEASURED, "max": NOT_MEASURED, "status": NOT_MEASURED})
            writer.writerow(payload)
    return runs_path, distribution_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--matrix", default="UNKNOWN")
    args = parser.parse_args()
    rows = [summarize_session(path, matrix=args.matrix, run_idx=index + 1) for index, path in enumerate(args.sessions)]
    write_summary(rows, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

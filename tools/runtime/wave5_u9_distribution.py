"""Extract the Wave 5 U9-prime braking-seed distribution.

The extractor is deliberately read-only and fail-closed: a session is included
only when its metadata identifies the requested scene/speed and its timeline
contains a finite positive ``backup_last_seed_duration_s`` sample.  The final
such sample is the session witness, matching the planner-trace field used by
the Wave 5 matrix report.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable


def _finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _finite_positive(value: Any) -> float | None:
    parsed = _finite_float(value)
    return parsed if parsed is not None and parsed > 0.0 else None


def _integer(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _boolean(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes"}


def nearest_rank(values: Iterable[float], quantile: float) -> float | None:
    """Return the nearest-rank quantile used by the five-run matrix."""

    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    rank = max(1, math.ceil(quantile * len(ordered)))
    return ordered[rank - 1]


def _timeline_seed_samples(timeline: Path) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    if not timeline.is_file():
        return samples
    with timeline.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            values = record.get("record", {}).get("payload", {}).get("values", {})
            duration = _finite_positive(values.get("backup_last_seed_duration_s"))
            if duration is None:
                continue
            samples.append(
                {
                    "duration_s": duration,
                    "attempts_used": _integer(
                        values.get("backup_last_seed_attempts_used")
                    ),
                    "maximum_attempts": _integer(
                        values.get("backup_last_seed_maximum_attempts")
                    ),
                    "steady_cruise": _boolean(
                        values.get("backup_last_seed_steady_cruise")
                    ),
                    "failure": _integer(values.get("backup_last_seed_failure")),
                    "entry_acceleration_mps2": _finite_float(
                        values.get("backup_last_seed_entry_acceleration_mps2")
                    ),
                    "entry_jerk_mps3": _finite_float(
                        values.get("backup_last_seed_entry_jerk_mps3")
                    ),
                    "initial_velocity_mps": _finite_float(
                        values.get("backup_last_seed_initial_velocity_mps")
                    ),
                }
            )
    return samples


def extract_session(session: Path) -> dict[str, Any] | None:
    """Extract one final U9-prime witness, or ``None`` if incomplete."""

    try:
        metadata = json.loads((session / "metadata.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    identity = metadata.get("scenario_identity", {})
    requested = identity.get("requested", {})
    scene = requested.get("scene")
    speed = _finite_positive(metadata.get("requested_cruise_speed_mps"))
    if not isinstance(scene, str) or not scene or speed is None:
        return None
    samples = _timeline_seed_samples(session / "planning_timeline.jsonl")
    if not samples:
        return None
    scenario: dict[str, Any] = {}
    try:
        scenario = json.loads((session / "scenario.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    witness = samples[-1]
    return {
        "artifact": session.name,
        "scene": scene,
        "speed_mps": speed,
        "duration_s": witness["duration_s"],
        "attempts_used": witness["attempts_used"],
        "maximum_attempts": witness["maximum_attempts"],
        "steady_cruise": witness["steady_cruise"],
        "failure": witness["failure"],
        "entry_acceleration_mps2": witness["entry_acceleration_mps2"],
        "entry_jerk_mps3": witness["entry_jerk_mps3"],
        "initial_velocity_mps": witness["initial_velocity_mps"],
        "timeline_sample_count": len(samples),
        "mission_complete_observed": scenario.get("mission_complete_observed"),
        "outcome": scenario.get("outcome"),
        "collision_count": scenario.get("collision_count"),
    }


def collect_sessions(root: Path) -> list[dict[str, Any]]:
    return [
        record
        for session in sorted(root.glob("external-mode-check-20261004T*"))
        if session.is_dir()
        for record in [extract_session(session)]
        if record is not None
    ]


def _numeric_values(records: list[dict[str, Any]], key: str) -> list[float]:
    return [
        float(record[key])
        for record in records
        if isinstance(record.get(key), (int, float))
        and math.isfinite(float(record[key]))
    ]


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[tuple[str, float], list[dict[str, Any]]] = {}
    for record in records:
        groups.setdefault((record["scene"], record["speed_mps"]), []).append(record)
    output: list[dict[str, Any]] = []
    for (scene, speed), group in sorted(groups.items()):
        durations = _numeric_values(group, "duration_s")
        attempts = [int(record["attempts_used"]) for record in group]
        output.append(
            {
                "scene": scene,
                "speed_mps": speed,
                "n": len(group),
                "duration_s_p50": nearest_rank(durations, 0.50),
                "duration_s_p95": nearest_rank(durations, 0.95),
                "duration_s_min": min(durations),
                "duration_s_max": max(durations),
                "attempts_min": min(attempts),
                "attempts_max": max(attempts),
                "steady_cruise_true": sum(
                    1 for record in group if record["steady_cruise"]
                ),
                "entry_acceleration_mps2_p95": nearest_rank(
                    _numeric_values(group, "entry_acceleration_mps2"), 0.95
                ),
                "entry_jerk_mps3_p95": nearest_rank(
                    _numeric_values(group, "entry_jerk_mps3"), 0.95
                ),
                "initial_velocity_mps_p50": nearest_rank(
                    _numeric_values(group, "initial_velocity_mps"), 0.50
                ),
                "initial_velocity_mps_p95": nearest_rank(
                    _numeric_values(group, "initial_velocity_mps"), 0.95
                ),
                "initial_velocity_mps_min": min(
                    _numeric_values(group, "initial_velocity_mps")
                ),
                "initial_velocity_mps_max": max(
                    _numeric_values(group, "initial_velocity_mps")
                ),
                "at_rest_seed_count": sum(
                    1
                    for record in group
                    if isinstance(record.get("initial_velocity_mps"), (int, float))
                    and abs(float(record["initial_velocity_mps"])) <= 1.0e-9
                ),
                "failure_codes": sorted(
                    {int(record["failure"]) for record in group}
                ),
                "mission_complete_count": sum(
                    1
                    for record in group
                    if record.get("mission_complete_observed") is True
                ),
                "collision_count_total": sum(
                    int(record["collision_count"] or 0)
                    for record in group
                    if isinstance(record.get("collision_count"), int)
                ),
            }
        )
    return {
        "session_count": len(records),
        "groups": output,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(".artifacts/runtime"))
    args = parser.parse_args()
    print(json.dumps(summarize(collect_sessions(args.root)), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

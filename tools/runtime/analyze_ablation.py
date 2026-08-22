#!/usr/bin/env python3
"""Summarize paired External Mode controller ablation artifacts.

The report intentionally reads only benchmark_metrics.json and scenario.jsonl
from completed sessions.  It never infers a PASS from component counters: the
mission, waypoint, safety and localization gates remain the source of truth.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _read_session(path: Path) -> dict[str, Any]:
    metrics = json.loads((path / "benchmark_metrics.json").read_text(encoding="utf-8"))
    finite_position = 0
    setpoint_count = 0
    scenario = path / "scenario.jsonl"
    if scenario.is_file():
        for line in scenario.read_text(encoding="utf-8").splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("kind") != "setpoint":
                continue
            payload = event.get("payload", {})
            setpoint_count += 1
            finite_position += int(bool(payload.get("position_finite")))
    mission_id = ""
    for event_name in ("mission", "acceptance"):
        value = metrics.get(event_name, {})
        if isinstance(value, dict) and value.get("mission_id"):
            mission_id = str(value["mission_id"])
            break
    return {
        "session": str(path),
        "mission_id": mission_id,
        "outcome": metrics.get("safety", {}).get("outcome"),
        "mission_complete": metrics.get("acceptance", {}).get("mission_complete_observed"),
        "waypoints_complete": metrics.get("acceptance", {}).get("waypoint_acceptance_complete"),
        "reasons": metrics.get("acceptance", {}).get("reasons", []),
        "collision_count": metrics.get("safety", {}).get("collision_count", 0),
        "minimum_clearance_m": metrics.get("safety", {}).get("minimum_collision_clearance_m"),
        "lio_residual_max_m": metrics.get("localization", {}).get("max_position_residual_m"),
        "lio_residual_p95_m": metrics.get("localization", {}).get("p95_position_residual_m"),
        "cross_track_p95_m": metrics.get("tracking", {}).get("cross_track_error_m", {}).get("p95"),
        "measured_speed_max_mps": metrics.get("control", {}).get("measured_speed_mps", {}).get("maximum"),
        "setpoint_speed_max_mps": metrics.get("control", {}).get("setpoint_speed_mps", {}).get("maximum"),
        "setpoint_count": setpoint_count,
        "finite_position_setpoint_count": finite_position,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"sessions": [_read_session(path) for path in args.sessions]}
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

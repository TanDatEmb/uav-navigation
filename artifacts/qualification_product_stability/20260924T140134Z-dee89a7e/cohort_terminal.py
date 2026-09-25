#!/usr/bin/env python3
"""Reproduce terminal metrics from the pinned 10-run predecessor cohort."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/runtime"))
from analyze_terminal_recovery import analyze, _rows, _closest  # noqa: E402


def run(index_path: Path, output_path: Path) -> None:
    attempts = json.loads(index_path.read_text(encoding="utf-8"))["attempts"]
    values = []
    for attempt in attempts:
        session = Path(attempt["sessions_created"][0])
        commands = [row for row in _rows(session / "scenario.jsonl", kind="pva_command")
                    if row["payload"].get("request_id") == 5]
        states = list(_rows(session / "samples.jsonl", stream="propagated_odometry"))
        final_generation = int(commands[-1]["payload"]["bundle_generation"])
        first_completion = next(row for row in commands
                                if int(row["payload"]["bundle_generation"]) == final_generation
                                and row["payload"].get("trajectory_status") == 3)
        measured = _closest(states, "timestamp_ns", int(first_completion["sim_time_ns"]))
        command = first_completion["payload"]
        state = measured["payload"]
        failures = analyze(session)["episodes"]
        mission_complete = [row for row in _rows(session / "scenario.jsonl", kind="mission_progress")
                            if row["payload"].get("event") == 1]
        completion = mission_complete[-1]["payload"] if mission_complete else {}
        values.append({
            "run": attempt["index"], "session": str(session),
            "result": attempt["mission_outcome"]["result"],
            "generation": final_generation,
            "analytic_completion_ros_ns": first_completion["sim_time_ns"],
            "analytic_anchor_error_m": math.dist(command["position"], state["position"]),
            "analytic_speed_mps": math.dist(state["linear_velocity"], [0, 0, 0]),
            "analytic_state_skew_ms": (int(first_completion["sim_time_ns"]) -
                                       int(measured["timestamp_ns"])) / 1e6,
            "command_state_source_age_ms": (int(first_completion["sim_time_ns"]) -
                                            int(command.get("state_source_stamp_ns") or 0)) / 1e6,
            "terminal_rejection_error_m": failures[-1]["reported_anchor_error_m"] if failures else "",
            "terminal_rejection_speed_mps": failures[-1]["failure_state_speed_mps"] if failures else "",
            "mission_acceptance_error_m": completion.get("acceptance_position_error_m", ""),
            "mission_acceptance_speed_mps": completion.get("acceptance_speed_mps", ""),
            "max_accepted_state_gap_ms": attempt["timing_summary"]["maximum_accepted_receive_gap_ms"],
        })
    with output_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(values[0]))
        writer.writeheader()
        writer.writerows(values)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: cohort_terminal.py INDEX_JSON OUTPUT_CSV")
    run(Path(sys.argv[1]), Path(sys.argv[2]))

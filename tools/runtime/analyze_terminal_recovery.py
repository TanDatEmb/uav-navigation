#!/usr/bin/env python3
"""Offline, diagnostic-only terminal endpoint/vehicle timeline from one SITL session.

The planner's analytic completion and a measured stop are different facts.  This
tool joins recorded command, PX4 input, and propagated odometry without changing
any admission, recovery, or evaluation rule.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
from typing import Any


REJECTION = re.compile(
    r"\[(\d+\.\d+)\].*execution boundary rejected planned STOPPED_HOLD"
    r".*anchor_error_m=([0-9.]+) anchor_limit_m=([0-9.]+)"
)


def _rows(path: Path, *, kind: str | None = None, stream: str | None = None):
    with path.open(encoding="utf-8") as source:
        for line in source:
            row = json.loads(line)
            if kind is not None and row.get("kind") != kind:
                continue
            if stream is not None and row.get("stream") != stream:
                continue
            yield row


def _closest(rows: list[dict[str, Any]], key: str, value: int) -> dict[str, Any]:
    return min(rows, key=lambda row: abs(int(row[key]) - value))


def analyze(session: Path) -> dict[str, Any]:
    command = list(_rows(session / "scenario.jsonl", kind="pva_command"))
    px4 = list(_rows(session / "scenario.jsonl", kind="px4_input_trace"))
    state = list(_rows(session / "samples.jsonl", stream="propagated_odometry"))
    truth = list(_rows(session / "samples.jsonl", stream="ground_truth_odometry"))
    frame_witness = next(_rows(session / "scenario.jsonl", kind="truth_frame_witness"), None)
    if not command or not state:
        raise ValueError("command and propagated odometry are required")
    failures: list[dict[str, Any]] = []
    for line in (session / "logs/mapping.log").open(encoding="utf-8"):
        match = REJECTION.search(line)
        if match:
            failures.append({
                "wall_ns": round(float(match.group(1)) * 1e9),
                "reported_anchor_error_m": float(match.group(2)),
                "anchor_limit_m": float(match.group(3)),
            })
    episodes = []
    for failure in failures:
        failed_wall = failure["wall_ns"]
        preceding = [row for row in command if int(row.get("observer_record_steady_ns") or 0) > 0
                     and int(row["payload"].get("trajectory_status") or 0) == 3]
        # Monotonic observer clock is not wall time.  Use the sample's ROS
        # timestamp, then select the matching command generation by mission
        # episode (the final command before the source-time gate rejection).
        closest_state = _closest(state, "arrival_wall_ns", failed_wall)
        source_ns = int(closest_state["timestamp_ns"])
        candidates = [row for row in preceding if int(row["sim_time_ns"]) <= source_ns + 100_000_000]
        if not candidates:
            raise ValueError("no completed command precedes rejected terminal state")
        terminal = candidates[-1]
        identity = terminal["payload"]
        generation = int(identity["bundle_generation"])
        request = int(identity["request_id"])
        same = [row for row in command if int(row["payload"].get("bundle_generation") or 0) == generation
                and int(row["payload"].get("request_id") or 0) == request]
        complete = next((row for row in same if int(row["payload"].get("trajectory_status") or 0) == 3), None)
        if complete is None:
            raise ValueError("no analytic completion in rejected generation")
        endpoint = identity["position"]
        sampled = closest_state["payload"]
        error = math.dist(endpoint, sampled["position"])
        speed = math.dist(sampled["linear_velocity"], [0.0, 0.0, 0.0])
        command_stamp = int(terminal["sim_time_ns"])
        timeline = []
        complete_ns = int(complete["sim_time_ns"])
        for offset_ms in (-2000, -1000, -500, -200, -100, 0, 100, 200, 500, 1000):
            target = complete_ns + offset_ms * 1_000_000
            observed = _closest(state, "timestamp_ns", target)
            at_or_before = [row for row in command if int(row["sim_time_ns"]) <= target]
            current = at_or_before[-1] if at_or_before else command[0]
            measured = observed["payload"]
            timeline.append({
                "offset_ms": offset_ms,
                "state_source_ns": int(observed["timestamp_ns"]),
                "command_ros_ns": int(current["sim_time_ns"]),
                "command_status": current["payload"].get("trajectory_status"),
                "command_generation": current["payload"].get("bundle_generation"),
                "command_position": current["payload"]["position"],
                "command_velocity": current["payload"]["velocity"],
                "measured_position": measured["position"],
                "measured_speed_mps": math.dist(measured["linear_velocity"], [0, 0, 0]),
                "endpoint_error_m": math.dist(endpoint, measured["position"]),
            })
        switch = None
        prior = [row for row in command if int(row["sim_time_ns"]) < int(same[0]["sim_time_ns"])]
        if prior:
            predecessor = prior[-1]
            switch = {
                "old_generation": int(predecessor["payload"]["bundle_generation"]),
                "new_generation": generation,
                "command_position_step_m": math.dist(
                    predecessor["payload"]["position"], same[0]["payload"]["position"]),
                "sample_interval_ms": (int(same[0]["sim_time_ns"]) -
                                       int(predecessor["sim_time_ns"])) / 1e6,
            }
        px4_same = [row for row in px4 if int(row["payload"].get("bundle_generation") or 0) == generation]
        # Select the source-time terminal window, but measure stream cadence
        # on the independent steady observer clock.  Simulation time alone
        # would hide a host/transport pause.
        post_complete_command_ns = [int(row["observer_record_steady_ns"]) for row in same
                                    if complete_ns <= int(row["sim_time_ns"]) <= source_ns + 20_000_000]
        post_complete_px4_ns = [int(row["observer_record_steady_ns"]) for row in px4_same
                                if complete_ns <= int(row["sim_time_ns"]) <= source_ns + 20_000_000]
        def maximum_interval_ms(stamps: list[int]) -> float | None:
            return max(((right - left) / 1e6 for left, right in zip(stamps, stamps[1:])),
                       default=None)
        truth_crosscheck = None
        if truth and frame_witness:
            rotation = frame_witness["payload"]["T_L_G"]["rotation_matrix_lio_from_gazebo"]
            truth_before = _closest(truth, "timestamp_ns", complete_ns)
            truth_after = _closest(truth, "timestamp_ns", source_ns)
            state_before = _closest(state, "timestamp_ns", complete_ns)
            gazebo_delta = [after - before for before, after in zip(
                truth_before["payload"]["position"], truth_after["payload"]["position"])]
            local_truth_delta = [sum(rotation[index][axis] * gazebo_delta[axis]
                                     for axis in range(3)) for index in range(3)]
            measured_delta = [after - before for before, after in zip(
                state_before["payload"]["position"], sampled["position"])]
            local_truth_at_failure = [base + delta for base, delta in zip(
                state_before["payload"]["position"], local_truth_delta)]
            truth_crosscheck = {
                "method": "evaluation-only truth displacement rotated into LIO, aligned at first analytic completion",
                "truth_before_source_ns": int(truth_before["timestamp_ns"]),
                "truth_after_source_ns": int(truth_after["timestamp_ns"]),
                "truth_local_endpoint_error_m": math.dist(endpoint, local_truth_at_failure),
                "truth_speed_at_analytic_completion_mps": math.dist(
                    truth_before["payload"]["linear_velocity"], [0, 0, 0]),
                "truth_speed_at_rejection_mps": math.dist(
                    truth_after["payload"]["linear_velocity"], [0, 0, 0]),
                "lio_vs_truth_displacement_residual_m": math.dist(
                    measured_delta, local_truth_delta),
            }
        episodes.append({
            "request_id": request, "generation": generation,
            "endpoint": endpoint, "analytic_complete_ros_ns": complete_ns,
            "analytic_complete_status": complete["payload"].get("trajectory_status"),
            "failure_wall_ns": failed_wall,
            "failure_state_source_ns": source_ns,
            "failure_state_arrival_wall_ns": int(closest_state["arrival_wall_ns"]),
            "failure_state_position": sampled["position"],
            "failure_state_speed_mps": speed,
            "reconstructed_anchor_error_m": error,
            "reported_anchor_error_m": failure["reported_anchor_error_m"],
            "anchor_limit_m": failure["anchor_limit_m"],
            "last_command_ros_ns": command_stamp,
            "last_command_source_ns": int(identity.get("state_source_stamp_ns") or 0),
            "px4_setpoint_boundaries": sorted(set(
                str(row["payload"].get("setpoint_boundary")) for row in px4_same)),
            "px4_trace_drops_max": max((int(row["payload"].get("trace_drop_count") or 0)
                                          for row in px4_same), default=0),
            "post_completion_command_max_gap_ms": maximum_interval_ms(post_complete_command_ns),
            "post_completion_px4_input_max_gap_ms": maximum_interval_ms(post_complete_px4_ns),
            "generation_switch": switch,
            "truth_crosscheck": truth_crosscheck,
            "timeline": timeline,
            "classification": "ACTUAL_TRACKING_ERROR" if error > failure["anchor_limit_m"]
                              else "UNRESOLVED",
        })
    return {"session": str(session), "failure_count": len(episodes), "episodes": episodes}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.session)
    serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    else:
        print(serialized, end="")


if __name__ == "__main__":
    main()

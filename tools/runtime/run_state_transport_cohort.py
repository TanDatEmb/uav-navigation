#!/usr/bin/env python3
"""Run and retain every diagnostic true-tracking-off long_featured attempt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import runner
from analyze_temporal_layers import analyze as analyze_temporal_layers
from state_transport_analysis import analyze_session


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--label", default="qualification-state-transport")
    parser.add_argument("--gazebo-native-diagnostic", action="store_true",
                        help="record bounded native Gazebo and host witnesses")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("count must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    attempts: list[dict] = []
    for index in range(1, args.count + 1):
        before = set(runner._runtime_session_paths(runner.ARTIFACT_ROOT))
        run_status = runner.run_sim(
            True, control_interface="external_mode", map_profile="long_featured",
            map_seed=0, tracking_experiment_mode="off", sitl_dynamics_profile="off",
            state_transport_trace=True,
            gazebo_native_diagnostic=args.gazebo_native_diagnostic,
            experiment_id=f"{args.label}-{index:02d}",
        )
        created = set(runner._runtime_session_paths(runner.ARTIFACT_ROOT)) - before
        record: dict = {"index": index, "runner_status": run_status,
                        "sessions_created": [str(path) for path in sorted(created)]}
        if len(created) == 1:
            session = created.pop()
            report_path = session / "report.json"
            if report_path.exists():
                report = json.loads(report_path.read_text())
                mission = report.get("mission_outcome") or {}
                acceptance = mission.get("acceptance") or {}
                record["mission_outcome"] = {
                    "result": (mission.get("scenario") or {}).get("outcome"),
                    "mission_complete_observed": acceptance.get("mission_complete_observed"),
                    "accepted_indices": acceptance.get("waypoint_acceptance_indices"),
                }
                evaluation = report.get("evaluation") or {}
                record["qualification_eligible"] = evaluation.get("qualification_eligible")
                record["assessment_status"] = evaluation.get("assessment_status")
                record["qualification_reasons"] = evaluation.get("qualification_reasons")
            try:
                timing = analyze_session(session)
                (session / "state_transport_analysis.json").write_text(
                    json.dumps(timing, indent=2, sort_keys=True) + "\n")
                record["timing_summary"] = {
                    key: timing.get(key) for key in (
                        "producer_count", "callback_count", "accepted_count",
                        "rejected_count", "matched_accepted_count",
                        "trace_complete_for_accepted", "unknown_tail_count",
                        "maximum_accepted_receive_gap_ms")}
                layers = analyze_temporal_layers(session)
                (session / "temporal_layer_analysis.json").write_text(
                    json.dumps(layers, indent=2, sort_keys=True) + "\n")
                record["temporal_layer_summary"] = {
                    "native_observer_status": layers["native_observer_status"],
                    "event_count": layers["event_count"],
                    "first_stalled_layers": [event["first_stalled_layer"]
                                             for event in layers["events"]],
                }
            except (OSError, ValueError, KeyError) as error:
                record["timing_analysis_error"] = str(error)
        attempts.append(record)
        args.output.write_text(json.dumps({"attempts": attempts}, indent=2,
                                          sort_keys=True) + "\n")
        print(json.dumps(record, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

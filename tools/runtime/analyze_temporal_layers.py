#!/usr/bin/env python3
"""Join independent, diagnostic-only SITL timing witnesses for state gaps.

The output deliberately reports UNRESOLVED when a bridge callback, native
sensor, or host scheduling witness is absent.  A gap at a downstream layer is
not proof that that layer caused it.
"""

from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path
from typing import Any

from state_transport_analysis import _clock_events_from_rosbag


TRIGGER_MS = 150.0  # Investigation trigger; not an admission/safety limit.


def _load_rows(path: Path, streams: set[str]) -> dict[str, list[dict[str, Any]]]:
    result = {stream: [] for stream in streams}
    with path.open(encoding="utf-8") as source:
        for line in source:
            row = json.loads(line)
            name = row.get("stream")
            if name in result:
                result[name].append(row)
    for rows in result.values():
        rows.sort(key=lambda row: int(row.get("arrival_wall_ns") or 0))
    return result


def _gap_at(rows: list[dict[str, Any]], midpoint_ns: int) -> dict[str, Any] | None:
    if len(rows) < 2:
        return None
    arrivals = [int(row.get("arrival_wall_ns") or 0) for row in rows]
    index = bisect.bisect_right(arrivals, midpoint_ns)
    if index < 1 or index >= len(rows):
        return None
    left, right = rows[index - 1], rows[index]
    source_before = int(left.get("timestamp_ns") or 0)
    source_after = int(right.get("timestamp_ns") or 0)
    return {
        "last_progress_wall_ns": arrivals[index - 1],
        "next_progress_wall_ns": arrivals[index],
        "gap_ms": (arrivals[index] - arrivals[index - 1]) / 1e6,
        "source_progress_ms": (source_after - source_before) / 1e6
                              if source_before and source_after else None,
    }


def _native_gap(summary: dict[str, Any], stream: str, midpoint_ns: int) -> dict[str, Any] | None:
    native = summary.get("gazebo_native", {}).get(stream, {})
    events = native.get("arrival_gap_events", [])
    matching = [event for event in events
                if int(event.get("before_arrival_ns") or 0) <= midpoint_ns <=
                int(event.get("after_arrival_ns") or 0)]
    if not matching:
        return None
    event = max(matching, key=lambda item: int(item.get("gap_ns") or 0))
    before = int(event.get("before_source_ns") or 0)
    after = int(event.get("after_source_ns") or 0)
    return {
        "last_progress_wall_ns": int(event["before_arrival_ns"]),
        "next_progress_wall_ns": int(event["after_arrival_ns"]),
        "gap_ms": int(event["gap_ns"]) / 1e6,
        "source_progress_ms": (after - before) / 1e6 if before and after else None,
    }


def _ros_clock_gap(events: list[dict[str, Any]], midpoint_ns: int) -> dict[str, Any] | None:
    # The monitor records bounded gap events but not every high-rate /clock
    # sample in samples.jsonl.  The independent bag recorder supplies exact
    # arrival/source pairs for this diagnostic join.
    matching = [event for event in events if
                int(event["previous_arrival_wall_ns"]) <= midpoint_ns <=
                int(event["arrival_wall_ns"])]
    if not matching:
        return None
    event = max(matching, key=lambda item: float(item["gap_ms"]))
    return {
        "last_progress_wall_ns": int(event["previous_arrival_wall_ns"]),
        "next_progress_wall_ns": int(event["arrival_wall_ns"]),
        "gap_ms": float(event["gap_ms"]),
        "source_progress_ms": (int(event["source_stamp_ns"]) -
                               int(event["previous_source_stamp_ns"])) / 1e6,
    }


def classify(witness: dict[str, dict[str, Any] | None], native_ready: bool,
             native_imu_covered: bool = False) -> tuple[str, str]:
    """Return only source-supported first-layer claims; otherwise unresolved."""
    def stalled(name: str) -> bool:
        item = witness.get(name)
        return bool(item and float(item["gap_ms"]) > TRIGGER_MS)

    def source_progress_deficit(name: str) -> bool:
        item = witness.get(name)
        return bool(item and item.get("source_progress_ms") is not None and
                    float(item["gap_ms"]) - float(item["source_progress_ms"]) > TRIGGER_MS)

    if native_ready and not stalled("observer_loop") and \
            stalled("gazebo_stats") and stalled("gazebo_clock") and \
            source_progress_deficit("gazebo_stats") and \
            source_progress_deficit("gazebo_clock"):
        return "GAZEBO_SIMULATION_STALL", \
            "Gazebo native stats and clock show >150 ms sim-progress deficit with observer scheduled"
    if stalled("observer_loop") and stalled("gazebo_clock"):
        return "UNRESOLVED", "native observer scheduling also stalled; host-wide cause not excluded"
    if native_ready and stalled("gazebo_clock") and not stalled("gazebo_stats"):
        return "CLOCK_PUBLICATION_STALL", "native world stats progressed while native clock stopped"
    if native_ready and not stalled("observer_loop") and \
            not stalled("gazebo_clock") and stalled("ros_clock"):
        return "UNRESOLVED", "clock gap lies between native publication and ROS recording; bridge, DDS and recorder are not separated"
    if native_imu_covered and not stalled("observer_loop") and \
            not stalled("gazebo_clock") and stalled("native_imu") and stalled("imu"):
        return "UNRESOLVED", "first observed gap is native IMU; generation and native transport are not separated"
    if native_imu_covered and not stalled("observer_loop") and \
            not stalled("native_imu") and stalled("imu"):
        return "UNRESOLVED", "IMU gap lies between native and ROS observations; bridge, DDS and monitor are not separated"
    if not stalled("ros_clock") and not stalled("imu") and stalled("worker") and \
            stalled("publisher"):
        return "FAST_LIO_WORKER_STALL", "clock and IMU progressed while worker/publisher stopped"
    if not stalled("publisher") and stalled("adapter_callback"):
        return "DDS_OR_EXECUTOR_DELAY", "publisher progressed while adapter callback stopped"
    if not stalled("adapter_callback") and stalled("accepted_state"):
        return "ADAPTER_SEMANTIC_REJECTION_STARVATION", "callbacks progressed but acceptance stopped"
    return "UNRESOLVED", "insufficient independent first-layer witness"


def analyze(session: Path, trigger_ms: float = TRIGGER_MS) -> dict[str, Any]:
    names = {"imu", "lidar", "simulation_clock", "corrected_odometry",
             "propagated_odometry", "odometry_producer_trace",
             "odometry_adapter_ingress_trace"}
    rows = _load_rows(session / "samples.jsonl", names)
    callbacks = rows["odometry_adapter_ingress_trace"]
    accepted = [row for row in callbacks
                if int(row.get("payload", {}).get("disposition") or 0) == 1]
    native_path = session / "gazebo_native_summary.json"
    native = json.loads(native_path.read_text(encoding="utf-8")) if native_path.exists() else {}
    native_ready = native.get("status") == "OK"
    ros_clock_events = _clock_events_from_rosbag(session)
    events = []
    for before, after in zip(accepted, accepted[1:]):
        if before["payload"].get("localization_epoch") != after["payload"].get("localization_epoch"):
            continue
        gap_ms = (int(after["payload"]["accepted_receive_steady_ns"]) -
                  int(before["payload"]["accepted_receive_steady_ns"])) / 1e6
        if gap_ms <= trigger_ms:
            continue
        midpoint = (int(before["arrival_wall_ns"]) + int(after["arrival_wall_ns"])) // 2
        native_streams = native.get("gazebo_native", {})
        native_covered = native_ready and all(
            int(native_streams.get(name, {}).get("first_arrival_ns") or 0) <= midpoint <=
            int(native_streams.get(name, {}).get("last_arrival_ns") or 0) and
            int(native_streams.get(name, {}).get("arrival_gap_event_overflow") or 0) == 0
            for name in ("world_stats", "world_clock", "observer_loop"))
        imu_native = native_streams.get("native_imu", {})
        native_imu_covered = native_covered and \
            int(imu_native.get("first_arrival_ns") or 0) <= midpoint <= \
            int(imu_native.get("last_arrival_ns") or 0) and \
            int(imu_native.get("arrival_gap_event_overflow") or 0) == 0
        witness = {
            "observer_loop": _native_gap(native, "observer_loop", midpoint),
            "gazebo_stats": _native_gap(native, "world_stats", midpoint),
            "gazebo_clock": _native_gap(native, "world_clock", midpoint),
            "gazebo_lidar": _native_gap(native, "native_lidar", midpoint),
            "native_imu": _native_gap(native, "native_imu", midpoint),
            "ros_clock": _ros_clock_gap(ros_clock_events, midpoint),
            "imu": _gap_at(rows["imu"], midpoint),
            "lidar": _gap_at(rows["lidar"], midpoint),
            "fast_lio_ingress": _gap_at(rows["corrected_odometry"], midpoint),
            "worker": _gap_at(rows["odometry_producer_trace"], midpoint),
            "publisher": _gap_at(rows["propagated_odometry"], midpoint),
            "adapter_callback": _gap_at(callbacks, midpoint),
            "accepted_state": _gap_at(accepted, midpoint),
        }
        category, reason = classify(witness, native_covered, native_imu_covered)
        events.append({
            "before_sequence": int(before["payload"].get("sequence") or 0),
            "after_sequence": int(after["payload"].get("sequence") or 0),
            "accepted_gap_ms": gap_ms,
            "interval_wall_ns": [int(before["arrival_wall_ns"]),
                                 int(after["arrival_wall_ns"])],
            "first_stalled_layer": category,
            "classification_reason": reason,
            "native_interval_covered": native_covered,
            "native_imu_interval_covered": native_imu_covered,
            "witness": witness,
        })
    return {
        "session": str(session), "trigger_ms": trigger_ms,
        "trigger_is_diagnostic_only": True,
        "native_observer_status": native.get("status", "NOT_RUN"),
        "accepted_count": len(accepted), "event_count": len(events), "events": events,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--trigger-ms", type=float, default=TRIGGER_MS)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = json.dumps(analyze(args.session, args.trigger_ms), indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    else:
        print(output, end="")


if __name__ == "__main__":
    main()

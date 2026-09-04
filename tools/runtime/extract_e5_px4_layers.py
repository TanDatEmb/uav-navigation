#!/usr/bin/env python3
"""Extract high-rate PX4 input and External Mode evidence from an E5 rosbag.

This is an offline decoder only.  It does not publish, replay, or alter any
runtime state.  PX4 message timestamps are retained in simulation time and
bag timestamps are retained as transport provenance.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def finite_triplet(values: object) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return [float("nan")] * 3
    return [float(value) for value in values]


def write_input(bag: Path, output: Path, topic_type: dict[str, str]) -> int:
    rows: list[dict[str, object]] = []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    msg_type = get_message(topic_type["/fmu/in/trajectory_setpoint"])
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic != "/fmu/in/trajectory_setpoint":
            continue
        msg = deserialize_message(data, msg_type)
        p = finite_triplet(msg.position)
        v = finite_triplet(msg.velocity)
        a = finite_triplet(msg.acceleration)
        rows.append({
            "bag_timestamp_ns": int(bag_stamp),
            "timestamp_us": int(msg.timestamp),
            "timestamp_ns": int(msg.timestamp) * 1000,
            "position_ned": p,
            "velocity_ned": v,
            "acceleration_ned": a,
            "jerk_ned": finite_triplet(msg.jerk),
            "yaw": float(msg.yaw),
            "yawspeed": float(msg.yawspeed),
        })
    fields = ["bag_timestamp_ns", "timestamp_us", "timestamp_ns",
              "position_ned", "velocity_ned", "acceleration_ned",
              "jerk_ned", "yaw", "yawspeed"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            for key in ("position_ned", "velocity_ned", "acceleration_ned", "jerk_ned"):
                row[key] = ",".join(f"{x:.17g}" for x in row[key])
            writer.writerow(row)
    return len(rows)


def write_mode(bag: Path, output: Path, topic_type: dict[str, str]) -> int:
    rows: list[dict[str, object]] = []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    msg_type = get_message(topic_type["/navigation/mode_status"])
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic != "/navigation/mode_status":
            continue
        msg = deserialize_message(data, msg_type)
        header_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        rows.append({
            "bag_timestamp_ns": int(bag_stamp),
            "header_stamp_ns": header_ns,
            "timestamp_ns": header_ns if header_ns > 0 else int(bag_stamp),
            "mission_id": msg.mission_id,
            "waypoint_index": int(msg.waypoint_index),
            "request_id": int(msg.request_id),
            "state": int(msg.state),
            "state_name": {0: "ACTIVE", 1: "BRAKING", 2: "PAUSED", 3: "COMPLETE", 4: "FAILED"}.get(int(msg.state), "UNKNOWN"),
            "reason": int(msg.reason),
            "reason_name": {0: "NONE", 1: "SAFETY_STOP", 2: "OPERATOR_TAKEOVER", 3: "ODOMETRY_STALE", 4: "TRAJECTORY_INVALID"}.get(int(msg.reason), "UNKNOWN"),
            "external_mode_state": int(msg.external_mode_state),
            "external_mode_state_name": {0: "TRACK_TRAJECTORY", 1: "WAIT_AIRBORNE", 2: "WAIT_HEALTH", 3: "WAIT_FIRST_COMMAND", 4: "MISSION_HOLD", 5: "COMPLETED_HOLD", 6: "RECOVERY_HOLD", 7: "FAILSAFE_HOLD", 8: "HANDOVER_HOLD"}.get(int(msg.external_mode_state), "UNKNOWN"),
            "external_mode_reason": msg.external_mode_reason,
            "waypoint_accepted": bool(msg.waypoint_accepted),
            "accepted_waypoint_index": int(msg.accepted_waypoint_index),
            "acceptance_position_error_m": float(msg.acceptance_position_error_m),
            "acceptance_speed_mps": float(msg.acceptance_speed_mps),
        })
    fields = list(rows[0].keys()) if rows else ["timestamp_ns"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {"/fmu/in/trajectory_setpoint", "/navigation/mode_status"}
    missing = sorted(required - topics.keys())
    if missing:
        raise SystemExit("missing topics: " + ", ".join(missing))
    count_input = write_input(args.bag, args.output_dir / "px4_input_trajectory_setpoint.csv", topics)
    count_mode = write_mode(args.bag, args.output_dir / "navigation_mode_status.csv", topics)
    print(f"trajectory_setpoint_rows={count_input} mode_status_rows={count_mode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

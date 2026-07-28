#!/usr/bin/env python3
"""Fail-closed live contract checker for the M1 Gazebo harness."""

import argparse
import json
from pathlib import Path
import sys
import time


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--min-lidar-messages", type=int, default=20)
    parser.add_argument("--min-imu-messages", type=int, default=200)
    parser.add_argument("--lidar-topic", default="/lidar/points")
    parser.add_argument("--imu-topic", default="/lidar/imu")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--lidar-frame", default="lidar_link")
    parser.add_argument("--imu-frame", default="imu_link")
    parser.add_argument("--require-dynamic-odom-tf", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    report = {"schema_version": 1, "passed": False, "checks": {}, "failures": []}
    try:
        import rclpy
        from builtin_interfaces.msg import Time
        from rclpy.duration import Duration
        from sensor_msgs.msg import Imu, PointCloud2
        from rosgraph_msgs.msg import Clock
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        report["failures"].append(f"ROS 2 Python dependencies unavailable: {error}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        return 2

    state = {
        "lidar": {"count": 0, "last_stamp_ns": None, "regressions": 0, "frames": set()},
        "imu": {"count": 0, "last_stamp_ns": None, "regressions": 0, "frames": set()},
        "clock": {"count": 0, "last_stamp_ns": None, "regressions": 0},
    }

    def observe(name, stamp, frame=None):
        entry = state[name]
        value = _stamp_ns(stamp)
        if value <= 0:
            report["failures"].append(f"{name} has a zero/negative timestamp")
        if entry["last_stamp_ns"] is not None and value <= entry["last_stamp_ns"]:
            entry["regressions"] += 1
        entry["last_stamp_ns"] = value
        entry["count"] += 1
        if frame is not None:
            entry["frames"].add(frame)

    rclpy.init()
    node = rclpy.create_node("m1_sim_contract_checker")
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    node.create_subscription(PointCloud2, args.lidar_topic,
                             lambda msg: observe("lidar", msg.header.stamp, msg.header.frame_id), 20)
    node.create_subscription(Imu, args.imu_topic,
                             lambda msg: observe("imu", msg.header.stamp, msg.header.frame_id), 200)
    node.create_subscription(Clock, args.clock_topic,
                             lambda msg: observe("clock", msg.clock), 20)
    try:
        deadline = time.monotonic() + args.duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        topic_types = dict(node.get_topic_names_and_types())
        expected = {
            args.lidar_topic: "sensor_msgs/msg/PointCloud2",
            args.imu_topic: "sensor_msgs/msg/Imu",
            args.clock_topic: "rosgraph_msgs/msg/Clock",
        }
        for topic, message_type in expected.items():
            if message_type not in topic_types.get(topic, []):
                report["failures"].append(f"{topic} does not advertise {message_type}")
        requirements = {"lidar": args.min_lidar_messages, "imu": args.min_imu_messages, "clock": 1}
        for name, minimum in requirements.items():
            entry = state[name]
            if entry["count"] < minimum:
                report["failures"].append(f"{name} count {entry['count']} < {minimum}")
            if entry["regressions"]:
                report["failures"].append(f"{name} has {entry['regressions']} timestamp regressions")
        if state["lidar"]["frames"] != {args.lidar_frame}:
            report["failures"].append(f"lidar frames are {sorted(state['lidar']['frames'])}")
        if state["imu"]["frames"] != {args.imu_frame}:
            report["failures"].append(f"imu frames are {sorted(state['imu']['frames'])}")
        static_pairs = [("base_link", args.lidar_frame), ("base_link", args.imu_frame)]
        if args.require_dynamic_odom_tf:
            static_pairs.append(("odom", "base_link"))
        for parent, child in static_pairs:
            try:
                buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
            except Exception as error:  # tf2 exception classes differ across Jazzy patch versions.
                report["failures"].append(f"missing TF {parent}->{child}: {error}")
        report["checks"] = {
            name: {key: (sorted(value) if isinstance(value, set) else value) for key, value in entry.items()}
            for name, entry in state.items()
        }
        report["passed"] = not report["failures"]
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

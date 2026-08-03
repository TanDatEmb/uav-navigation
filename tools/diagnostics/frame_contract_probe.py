#!/usr/bin/env python3
"""Record runtime frame and timestamp evidence without changing the graph.

The probe discovers versioned PX4 VehicleOdometry topics, subscribes only to
the known read-only evidence topics, and writes JSON artifacts. It is not a
qualification gate by itself; the caller decides whether missing samples are
an environment failure or an expected pre-start observation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import time
from typing import Any


RAW_TOPIC_RE = re.compile(r"^/fmu/out/vehicle_odometry(?:_v\d+)?$")
FIXED_TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/lidar/imu": "sensor_msgs/msg/Imu",
    "/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/px4/estimator_odometry": "nav_msgs/msg/Odometry",
    "/px4/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
    "/lio/odometry_corrected": "nav_msgs/msg/Odometry",
    "/lio/odometry_propagated": "nav_msgs/msg/Odometry",
    "/lio/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
    "/navigation/odometry_supervisor/status":
        "navigation_interfaces/msg/OdometrySupervisorStatus",
    "/navigation/odometry_supervisor/diagnostics":
        "diagnostic_msgs/msg/DiagnosticArray",
}


def _command(args: list[str]) -> tuple[int, str, str]:
    try:
        result = subprocess.run(args, capture_output=True, text=True, check=False)
    except OSError as error:
        return 127, "", str(error)
    return result.returncode, result.stdout, result.stderr


def discover() -> dict[str, Any]:
    ros_code, ros_out, ros_err = _command(["ros2", "topic", "list", "-t"])
    ros_topics: dict[str, list[str]] = {}
    for line in ros_out.splitlines():
        match = re.match(r"^(\S+)\s+\[(.+)\]$", line.strip())
        if match:
            ros_topics[match.group(1)] = [item.strip() for item in match.group(2).split(",")]
    raw_candidates = sorted(topic for topic in ros_topics if RAW_TOPIC_RE.fullmatch(topic))
    selected = raw_candidates[0] if raw_candidates else None
    gz_code, gz_out, gz_err = _command(["gz", "topic", "-l"])
    gz_topics = sorted(line.strip() for line in gz_out.splitlines() if line.strip())
    return {
        "ros2_topic_list_returncode": ros_code,
        "ros2_topic_list_stderr": ros_err.strip(),
        "ros_topics": ros_topics,
        "raw_px4_candidates": raw_candidates,
        "raw_px4_selected": selected,
        "gazebo_topic_list_returncode": gz_code,
        "gazebo_topic_list_stderr": gz_err.strip(),
        "gazebo_topics": gz_topics,
        "fixed_topic_types": FIXED_TOPICS,
    }


def _stamp_ns(message: Any) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        stamp = getattr(message, "clock", None)
    if stamp is None:
        return 0
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))


def summarize(topic: str, message: Any) -> dict[str, Any]:
    result: dict[str, Any] = {"topic": topic, "stamp_ns": _stamp_ns(message)}
    header = getattr(message, "header", None)
    if header is not None:
        result["frame_id"] = str(getattr(header, "frame_id", ""))
    if hasattr(message, "child_frame_id"):
        result["child_frame_id"] = str(message.child_frame_id)
    if hasattr(message, "pose_frame"):
        result.update({
            "px4_timestamp": int(getattr(message, "timestamp", 0)),
            "px4_timestamp_sample": int(getattr(message, "timestamp_sample", 0)),
            "pose_frame": int(message.pose_frame),
            "velocity_frame": int(message.velocity_frame),
            "reset_counter": int(getattr(message, "reset_counter", 0)),
            "position": [float(value) for value in message.position],
            "velocity": [float(value) for value in message.velocity],
            "q": [float(value) for value in message.q],
        })
    if hasattr(message, "pose"):
        pose = message.pose.pose
        result["position"] = [float(pose.position.x), float(pose.position.y),
                               float(pose.position.z)]
        result["orientation_xyzw"] = [float(pose.orientation.x), float(pose.orientation.y),
                                       float(pose.orientation.z), float(pose.orientation.w)]
        result["linear_velocity"] = [float(message.twist.twist.linear.x),
                                      float(message.twist.twist.linear.y),
                                      float(message.twist.twist.linear.z)]
    if hasattr(message, "status"):
        result["diagnostic_status_names"] = [str(item.name) for item in message.status]
        result["diagnostic_values"] = {
            str(item.name): {str(value.key): str(value.value) for value in item.values}
            for item in message.status
        }
    if hasattr(message, "width"):
        result.update({"width": int(message.width), "height": int(message.height),
                       "point_step": int(message.point_step),
                       "is_dense": bool(message.is_dense)})
    return result


def record(output: Path, duration_s: float) -> int:
    output.mkdir(parents=True, exist_ok=False)
    discovery = discover()
    (output / "discovery.json").write_text(
        json.dumps(discovery, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    records: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from navigation_interfaces.msg import OdometrySupervisorStatus
        from px4_msgs.msg import VehicleOdometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
    except ImportError as error:
        errors.append(f"ROS_IMPORT_ERROR: {error}")
        rclpy = None

    if rclpy is not None:
        message_types = {
            "/clock": Clock, "/lidar/imu": Imu, "/lidar/points": PointCloud2,
            "/px4/estimator_odometry": Odometry, "/px4/diagnostics": DiagnosticArray,
            "/lio/odometry_corrected": Odometry, "/lio/odometry_propagated": Odometry,
            "/lio/diagnostics": DiagnosticArray,
            "/navigation/odometry_supervisor/status": OdometrySupervisorStatus,
            "/navigation/odometry_supervisor/diagnostics": DiagnosticArray,
        }
        selected = discovery["raw_px4_selected"]
        if selected:
            message_types[selected] = VehicleOdometry
        rclpy.init(args=[])
        node = rclpy.create_node("frame_contract_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = []
        counts: dict[str, int] = {}

        def callback(message: Any, topic: str = "") -> None:
            counts[topic] = counts.get(topic, 0) + 1
            item = summarize(topic, message)
            item["wall_time_ns"] = time.time_ns()
            records.append(item)

        for topic, message_type in message_types.items():
            if topic not in discovery["ros_topics"]:
                continue
            try:
                subscriptions.append(node.create_subscription(
                    message_type, topic, lambda message, name=topic: callback(message, name), qos))
            except Exception as error:  # discovery remains useful if one type is incompatible
                errors.append(f"SUBSCRIBE_ERROR {topic}: {error}")
        deadline = time.monotonic() + max(0.1, duration_s)
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
    else:
        counts = {}
    with (output / "samples.jsonl").open("w", encoding="utf-8") as stream:
        for item in records:
            stream.write(json.dumps(item, sort_keys=True) + "\n")
    required = list(FIXED_TOPICS)
    missing = [topic for topic in required if not any(item["topic"] == topic for item in records)]
    summary = {
        "schema_version": 1,
        "duration_s": duration_s,
        "sample_count": len(records),
        "sample_counts": counts,
        "required_topics_missing_samples": missing,
        "errors": errors,
        "raw_px4_selected": discovery["raw_px4_selected"],
        "probe_complete": not errors,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if not errors else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    return record(args.output.resolve(), args.duration)


if __name__ == "__main__":
    raise SystemExit(main())

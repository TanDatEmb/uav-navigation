#!/usr/bin/env python3
"""Publish deterministic velocity commands for the Gazebo M1 sensor harness.

The generated trace is a command record, not a ground-truth trajectory. Sensor
and TF validity must be checked separately with verify_m1_sim_contract.py.
"""

import argparse
import json
from pathlib import Path
import sys


PROFILES = {
    "static": [(12.0, (0.0, 0.0, 0.0, 0.0))],
    "yaw": [(6.3, (0.0, 0.0, 0.0, 1.0)), (6.3, (0.0, 0.0, 0.0, -1.0))],
    "translation": [(4.0, (1.0, 0.0, 0.0, 0.0)), (4.0, (-1.0, 0.0, 0.0, 0.0))],
    "vertical": [(3.0, (0.0, 0.0, 0.5, 0.0)), (3.0, (0.0, 0.0, -0.5, 0.0))],
    "square": [
        (3.0, (1.0, 0.0, 0.0, 0.0)),
        (1.571, (0.0, 0.0, 0.0, 1.0)),
        (3.0, (1.0, 0.0, 0.0, 0.0)),
        (1.571, (0.0, 0.0, 0.0, 1.0)),
        (3.0, (1.0, 0.0, 0.0, 0.0)),
        (1.571, (0.0, 0.0, 0.0, 1.0)),
        (3.0, (1.0, 0.0, 0.0, 0.0)),
        (1.571, (0.0, 0.0, 0.0, 1.0)),
    ],
}


def trace(profile: str, topic: str) -> dict:
    segments = []
    elapsed = 0.0
    for duration, command in PROFILES[profile]:
        segments.append(
            {
                "start_s": elapsed,
                "duration_s": duration,
                "linear_xyz_m_s": list(command[:3]),
                "angular_z_rad_s": command[3],
            }
        )
        elapsed += duration
    return {
        "schema_version": 1,
        "profile": profile,
        "command_topic": topic,
        "duration_s": elapsed,
        "ground_truth_claim": False,
        "segments": segments,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", choices=sorted(PROFILES))
    parser.add_argument("--topic", default="/sim/cmd_vel")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    output = trace(args.profile, args.topic)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    if args.dry_run:
        return 0

    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ImportError as error:
        print(f"ROS 2 Python dependencies unavailable: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("m1_scenario_runner")
    publisher = node.create_publisher(Twist, args.topic, 10)
    try:
        for segment in output["segments"]:
            message = Twist()
            message.linear.x, message.linear.y, message.linear.z = segment["linear_xyz_m_s"]
            message.angular.z = segment["angular_z_rad_s"]
            deadline = node.get_clock().now().nanoseconds + int(segment["duration_s"] * 1e9)
            while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
        publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

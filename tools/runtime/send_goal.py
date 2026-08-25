#!/usr/bin/env python3
"""Publish one or more stamped navigation goals to the runtime."""

from __future__ import annotations

import argparse
import time

import rclpy
from navigation_contracts.msg import NavigationGoal
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--frame", default="lio_odom")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()
    if args.repeat <= 0 or args.period <= 0.0:
        parser.error("--repeat and --period must be positive")

    rclpy.init()
    node = Node("navigation_goal_harness")
    publisher = node.create_publisher(NavigationGoal, "navigation/goal", 10)
    message = NavigationGoal()
    message.header.frame_id = args.frame
    message.mission_id = "manual_goal"
    message.waypoint_index = 0
    message.acceptance_radius_m = 0.5
    try:
        for index in range(args.repeat):
            message.request_id = index + 1
            message.header.stamp = node.get_clock().now().to_msg()
            message.target.x = args.x
            message.target.y = args.y
            message.target.z = args.z
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            if index + 1 < args.repeat:
                time.sleep(args.period)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

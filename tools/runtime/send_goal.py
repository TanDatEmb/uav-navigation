#!/usr/bin/env python3
"""Publish one or more stamped navigation goals to the runtime."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
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
    publisher = node.create_publisher(PoseStamped, "navigation/goal", 10)
    message = PoseStamped()
    message.header.frame_id = args.frame
    message.pose.position.x = args.x
    message.pose.position.y = args.y
    message.pose.position.z = args.z
    message.pose.orientation.w = 1.0
    try:
        for index in range(args.repeat):
            message.header.stamp = node.get_clock().now().to_msg()
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

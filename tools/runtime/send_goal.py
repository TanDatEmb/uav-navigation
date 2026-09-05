#!/usr/bin/env python3
"""Publish one or more stamped navigation goals to the runtime."""

from __future__ import annotations

import argparse
import math
import time


def _populate_route_snapshot(
    message: object, *, waypoint_id: str = "manual_goal"
) -> None:
    """Populate the canonical one-waypoint route for a manual goal.

    The runtime accepts a goal only when the compatibility mirrors agree with
    this immutable route snapshot.  Keep this helper deliberately fail-closed
    so a partially constructed message cannot be published as a lifecycle
    event.
    """
    try:
        frame_id = str(message.header.frame_id)
        mission_id = str(message.mission_id)
        waypoint_index = int(message.waypoint_index)
        request_id = int(message.request_id)
        acceptance_radius_m = float(message.acceptance_radius_m)
        target = message.target
        coordinates = (float(target.x), float(target.y), float(target.z))
        behavior_stop = int(message.BEHAVIOR_STOP)
        behavior = int(message.behavior)
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("manual goal is missing route contract fields") from error
    if (not frame_id or not mission_id or not waypoint_id or waypoint_index != 0 or
            request_id <= 0 or not math.isfinite(acceptance_radius_m) or
            acceptance_radius_m <= 0.0 or behavior != behavior_stop or
            not all(math.isfinite(value) for value in coordinates)):
        raise ValueError("manual goal cannot satisfy the canonical route contract")

    try:
        route = message.route
        route.mission_id = mission_id
        route.frame_id = frame_id
        route.route_revision = 1
        route.request_id = request_id
        route.active_waypoint_index = waypoint_index
        route.waypoint_positions.clear()
        route.waypoint_positions.append(target)
        route.waypoint_ids.clear()
        route.waypoint_ids.append(waypoint_id)
        route.waypoint_acceptance_radii_m.clear()
        route.waypoint_acceptance_radii_m.append(acceptance_radius_m)
        route.waypoint_behaviors.clear()
        route.waypoint_behaviors.append(behavior_stop)
        route.measured_progress_valid = True
        route.measured_segment_index = 0
        route.measured_progress_arc_m = 0.0
        route.measured_projection_arc_m = 0.0
        route.measured_lateral_error_m = 0.0
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("manual goal is missing route snapshot fields") from error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--frame", default="lio_odom")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()
    if (not math.isfinite(args.x) or not math.isfinite(args.y) or
            not math.isfinite(args.z)):
        parser.error("goal coordinates must be finite")
    if not args.frame:
        parser.error("--frame must not be empty")
    if (args.repeat <= 0 or not math.isfinite(args.period) or
            args.period <= 0.0):
        parser.error("--repeat and --period must be positive and finite")

    import rclpy
    from navigation_contracts.msg import NavigationGoal
    from rclpy.node import Node

    rclpy.init()
    node = Node("navigation_goal_harness")
    publisher = node.create_publisher(NavigationGoal, "navigation/goal", 10)
    message = NavigationGoal()
    message.header.frame_id = args.frame
    message.mission_id = "manual_goal"
    message.waypoint_index = 0
    message.acceptance_radius_m = 0.5
    message.behavior = NavigationGoal.BEHAVIOR_STOP
    message.has_next_target = False
    message.target.x = args.x
    message.target.y = args.y
    message.target.z = args.z
    try:
        for index in range(args.repeat):
            message.request_id = index + 1
            message.header.stamp = node.get_clock().now().to_msg()
            _populate_route_snapshot(message)
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

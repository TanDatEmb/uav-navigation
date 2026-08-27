#!/usr/bin/env python3
"""Exercise one bounded planner backend goal during recorded-dataset replay.

The recorded vehicle does not execute the generated command.  This helper is
therefore a planner/runtime benchmark only, never a controller or mission
acceptance test.  It publishes one goal relative to a fresh propagated state,
observes READY commands for a bounded source-time window, then sends an
explicit OPERATOR_TAKEOVER terminal status to remove the synthetic goal.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Sequence


def relative_goal(
    position: Sequence[float], velocity: Sequence[float], distance_m: float
) -> tuple[float, float, float]:
    """Return a horizontal goal ``distance_m`` ahead of the measured motion."""
    if len(position) < 3 or len(velocity) < 3:
        raise ValueError("position and velocity must contain three values")
    values = [float(item) for item in (*position[:3], *velocity[:3], distance_m)]
    if not all(math.isfinite(item) for item in values):
        raise ValueError("shadow planning inputs must be finite")
    if distance_m <= 0.0:
        raise ValueError("shadow goal distance must be positive")
    horizontal_speed = math.hypot(values[3], values[4])
    if horizontal_speed <= 1.0e-3:
        raise ValueError("horizontal velocity is too small to orient the shadow goal")
    scale = float(distance_m) / horizontal_speed
    return (
        values[0] + values[3] * scale,
        values[1] + values[4] * scale,
        values[2],
    )


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--goal-distance-m", type=float, default=5.0)
    parser.add_argument("--warmup-source-s", type=float, default=2.0)
    parser.add_argument("--observe-source-s", type=float, default=2.0)
    parser.add_argument("--wall-timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if (
        not math.isfinite(args.goal_distance_m)
        or not math.isfinite(args.warmup_source_s)
        or not math.isfinite(args.observe_source_s)
        or not math.isfinite(args.wall_timeout_s)
        or args.goal_distance_m <= 0.0
        or args.warmup_source_s < 0.0
        or args.observe_source_s <= 0.0
        or args.wall_timeout_s <= 0.0
    ):
        parser.error("distance/observation/timeout must be finite and positive")

    # Keep unit tests independent of ROS installation/import state.
    import rclpy
    from navigation_contracts.msg import (
        NavigationCommand,
        NavigationGoal,
        NavigationModeStatus,
        PropagatedOdometry,
    )
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    result_path = args.output / "dataset_shadow_planning.json"
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": "STARTING",
        "goal_distance_m": args.goal_distance_m,
        "warmup_source_s": args.warmup_source_s,
        "observe_source_s": args.observe_source_s,
        "goal_published": False,
        "ready_command_count": 0,
        "unique_ready_generations": [],
    }
    _atomic_json(result_path, result)

    rclpy.init()
    node = Node("dataset_shadow_planning")
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    terminal_status_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    goal_publisher = node.create_publisher(NavigationGoal, "/navigation/goal", reliable)
    status_publisher = node.create_publisher(
        NavigationModeStatus, "/navigation/mode_status", terminal_status_qos
    )

    first_odom_stamp_ns: int | None = None
    latest_odom: Any | None = None
    goal_message: Any | None = None
    ready_started_stamp_ns: int | None = None
    ready_generations: set[int] = set()
    ready_command_count = 0
    emergency_command_count = 0
    last_goal_publish_wall = 0.0

    def on_odometry(message: Any) -> None:
        nonlocal first_odom_stamp_ns, latest_odom
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if stamp_ns <= 0:
            return
        if first_odom_stamp_ns is None:
            first_odom_stamp_ns = stamp_ns
        latest_odom = message

    def on_command(message: Any) -> None:
        nonlocal ready_started_stamp_ns, ready_command_count, emergency_command_count
        if goal_message is None:
            return
        if int(message.status) == int(NavigationCommand.STATUS_READY):
            ready_command_count += 1
            generation = int(message.bundle_generation)
            if generation > 0:
                ready_generations.add(generation)
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            if ready_started_stamp_ns is None and stamp_ns > 0:
                ready_started_stamp_ns = stamp_ns
        elif int(message.status) == int(NavigationCommand.STATUS_REJECTED):
            emergency_command_count += 1

    def on_propagated_odometry(message: Any) -> None:
        # The propagated topic carries the product-owned epoch/sequence
        # envelope.  The nested odometry is the state used for goal geometry.
        if int(message.localization_epoch) <= 0 or int(message.sequence) <= 0:
            return
        on_odometry(message.odometry)

    node.create_subscription(
        PropagatedOdometry,
        "/lio/odometry_propagated",
        on_propagated_odometry,
        best_effort,
    )
    node.create_subscription(NavigationCommand, "/navigation/navigation_command", on_command, best_effort)

    def publish_teardown(odometry: Any) -> None:
        if goal_message is None:
            return
        discovery_deadline = time.monotonic() + 1.0
        while (
            status_publisher.get_subscription_count() <= 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
        terminal = NavigationModeStatus()
        terminal.header = odometry.header
        terminal.mission_id = goal_message.mission_id
        terminal.waypoint_index = goal_message.waypoint_index
        terminal.request_id = goal_message.request_id
        # This is an explicit synthetic teardown, not mission completion.
        terminal.state = NavigationModeStatus.FAILED
        terminal.reason = NavigationModeStatus.OPERATOR_TAKEOVER
        terminal.waypoint_accepted = False
        for _ in range(3):
            status_publisher.publish(terminal)
            rclpy.spin_once(node, timeout_sec=0.05)

    wall_deadline = time.monotonic() + args.wall_timeout_s
    exit_code = 1
    try:
        while time.monotonic() < wall_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if latest_odom is None or first_odom_stamp_ns is None:
                continue
            odom_stamp_ns = int(latest_odom.header.stamp.sec) * 1_000_000_000 + int(
                latest_odom.header.stamp.nanosec
            )
            source_elapsed_s = (odom_stamp_ns - first_odom_stamp_ns) / 1e9
            if goal_message is None:
                horizontal_speed = math.hypot(
                    float(latest_odom.twist.twist.linear.x),
                    float(latest_odom.twist.twist.linear.y),
                )
                if (
                    source_elapsed_s < args.warmup_source_s
                    or horizontal_speed < 0.25
                    or goal_publisher.get_subscription_count() <= 0
                ):
                    continue
                target = relative_goal(
                    (
                        latest_odom.pose.pose.position.x,
                        latest_odom.pose.pose.position.y,
                        latest_odom.pose.pose.position.z,
                    ),
                    (
                        latest_odom.twist.twist.linear.x,
                        latest_odom.twist.twist.linear.y,
                        latest_odom.twist.twist.linear.z,
                    ),
                    args.goal_distance_m,
                )
                goal_message = NavigationGoal()
                goal_message.header = latest_odom.header
                goal_message.mission_id = "dataset_shadow_planning"
                goal_message.waypoint_index = 0
                goal_message.request_id = 1
                goal_message.target.x, goal_message.target.y, goal_message.target.z = target
                goal_message.acceptance_radius_m = 0.20
                goal_message.behavior = NavigationGoal.BEHAVIOR_STOP
                goal_message.has_next_target = False
                goal_publisher.publish(goal_message)
                last_goal_publish_wall = time.monotonic()
                result.update(
                    {
                        "status": "GOAL_PUBLISHED",
                        "goal_published": True,
                        "goal_stamp_ns": odom_stamp_ns,
                        "planning_frame": str(latest_odom.header.frame_id),
                        "start_position": [
                            latest_odom.pose.pose.position.x,
                            latest_odom.pose.pose.position.y,
                            latest_odom.pose.pose.position.z,
                        ],
                        "start_horizontal_speed_mps": horizontal_speed,
                        "target_position": list(target),
                    }
                )
                _atomic_json(result_path, result)
                continue

            # Republish the same logical request while waiting for discovery;
            # onGoal treats this identity as idempotent.
            if ready_started_stamp_ns is None:
                if emergency_command_count > 0:
                    result["failure"] = (
                        "planner backend emitted EMER before committing a READY shadow command"
                    )
                    publish_teardown(latest_odom)
                    result["teardown"] = (
                        "FAILED/OPERATOR_TAKEOVER synthetic shadow-goal cancellation"
                    )
                    break
                if time.monotonic() - last_goal_publish_wall >= 0.5:
                    goal_publisher.publish(goal_message)
                    last_goal_publish_wall = time.monotonic()
                continue
            if emergency_command_count > 0:
                result["failure"] = "planner backend emitted EMER during shadow planning window"
                publish_teardown(latest_odom)
                result["teardown"] = (
                    "FAILED/OPERATOR_TAKEOVER synthetic shadow-goal cancellation"
                )
                break
            if (odom_stamp_ns - ready_started_stamp_ns) / 1e9 < args.observe_source_s:
                continue

            publish_teardown(latest_odom)
            result.update(
                {
                    "status": "PASS",
                    "ready_command_count": ready_command_count,
                    "unique_ready_generations": sorted(ready_generations),
                    "emergency_command_count": emergency_command_count,
                    "first_ready_stamp_ns": ready_started_stamp_ns,
                    "last_observed_odom_stamp_ns": odom_stamp_ns,
                    "teardown": "FAILED/OPERATOR_TAKEOVER synthetic shadow-goal cancellation",
                    "flight_acceptance": False,
                }
            )
            exit_code = 0
            break
        else:
            result["failure"] = "timed out waiting for bounded shadow planning evidence"
    except Exception as error:
        result["failure"] = str(error)
    finally:
        result["ready_command_count"] = ready_command_count
        result["unique_ready_generations"] = sorted(ready_generations)
        result["emergency_command_count"] = emergency_command_count
        if exit_code != 0:
            result["status"] = "FAIL"
        _atomic_json(result_path, result)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

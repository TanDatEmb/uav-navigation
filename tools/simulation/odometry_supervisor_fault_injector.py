#!/usr/bin/env python3
"""Synthetic ROS-native fault injector for the P0.8 supervisor.

This is test tooling only. It publishes the supervisor's ROS boundary topics,
answers epoch queries, and writes a replayable JSON artifact containing input,
residual, state, reason, and action timelines. It never changes production
estimator or bridge output.
"""

import argparse
import json
import math
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import OdometrySupervisorStatus
from navigation_interfaces.srv import SampleOdometryAtTime
from rclpy.node import Node


SCENARIOS = (
    "healthy",
    "single_position_jump",
    "slow_xy_drift",
    "slow_yaw_drift",
    "velocity_bias",
    "px4_stale",
    "lio_propagated_stale",
    "lio_corrected_stale",
    "px4_reset_generation",
    "clock_pause",
    "diagnostic_schema_corruption",
    "correlated_unhealthy",
)


def stamp_from_ns(value: int) -> Time:
    result = Time()
    result.sec = value // 1_000_000_000
    result.nanosec = value % 1_000_000_000
    return result


def ns_from_stamp(value: Time) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def key_values(values: dict[str, str]) -> list[KeyValue]:
    return [KeyValue(key=key, value=value) for key, value in values.items()]


def odometry(timestamp_ns: int, x: float = 0.0, yaw: float = 0.0,
             velocity_x: float = 0.0) -> Odometry:
    message = Odometry()
    message.header.stamp = stamp_from_ns(timestamp_ns)
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.twist.twist.linear.x = velocity_x
    return message


class SupervisorFaultInjector(Node):
    def __init__(self, scenario: str, duration_s: float, output: Path) -> None:
        super().__init__("p0_8_supervisor_fault_injector")
        self.scenario = scenario
        self.duration_s = duration_s
        self.output = output
        self.started_ns = self.get_clock().now().nanoseconds
        self.fault_start_s = 3.0
        self.finished = False
        self.inputs: list[dict] = []
        self.statuses: list[dict] = []
        self.propagated_pub = self.create_publisher(Odometry, "/lio/odometry_propagated", 20)
        self.corrected_pub = self.create_publisher(Odometry, "/lio/odometry_corrected", 20)
        self.px4_pub = self.create_publisher(Odometry, "/px4/odometry_ros", 20)
        self.lio_diag_pub = self.create_publisher(DiagnosticArray, "/lio/diagnostics", 10)
        self.px4_diag_pub = self.create_publisher(DiagnosticArray, "/px4/diagnostics", 10)
        self.status_sub = self.create_subscription(
            OdometrySupervisorStatus,
            "/navigation/odometry_supervisor/status",
            self.on_status,
            10,
        )
        self.query_srv = self.create_service(
            SampleOdometryAtTime, "/px4/sample_odometry_at_time", self.on_query
        )
        self.timer = self.create_timer(0.05, self.tick)

    def scenario_time(self) -> float:
        return (self.get_clock().now().nanoseconds - self.started_ns) * 1e-9

    def on_query(self, request, response):
        timestamp_ns = ns_from_stamp(request.sample_time)
        elapsed = max(0.0, (timestamp_ns - self.started_ns) * 1e-9)
        x, yaw, velocity = self.px4_values(elapsed)
        response.success = True
        response.reason = "synthetic epoch-aligned reference"
        response.odometry = odometry(timestamp_ns, x, yaw, velocity)
        response.interpolated = False
        response.reset_generation = 2 if self.scenario == "px4_reset_generation" and elapsed >= 2.0 else 1
        response.time_generation = 1
        response.component_validity_mask = 0xF
        response.covariance_availability_mask = 0x7
        return response

    def on_status(self, message: OdometrySupervisorStatus) -> None:
        self.statuses.append({
            "stamp_ns": ns_from_stamp(message.header.stamp),
            "evaluation_time_ns": ns_from_stamp(message.evaluation_time),
            "health": int(message.health),
            "reason_code": int(message.reason_code),
            "reason": message.reason,
            "comparison_valid": bool(message.comparison_valid),
            "external_odometry_allowed": bool(message.external_odometry_allowed),
            "reinitialization_requested": bool(message.reinitialization_requested),
            "position_error_m": float(message.position_error_m),
            "velocity_error_m_s": float(message.velocity_error_m_s),
            "yaw_error_rad": float(message.yaw_error_rad),
        })

    def publish_diagnostic(self, topic: str, values: dict[str, str], name: str) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = name
        status.level = DiagnosticStatus.OK
        status.message = "synthetic P0.8 input"
        status.values = key_values(values)
        message.status = [status]
        topic.publish(message)

    def px4_values(self, elapsed: float) -> tuple[float, float, float]:
        if self.scenario == "slow_xy_drift":
            return 0.04 * elapsed, 0.0, 0.0
        if self.scenario == "slow_yaw_drift":
            return 0.0, 0.025 * elapsed, 0.0
        if self.scenario == "velocity_bias":
            return 0.0, 0.0, 0.25
        return 0.0, 0.0, 0.0

    def lio_values(self, elapsed: float) -> tuple[float, float, float]:
        if self.scenario == "single_position_jump" and 1.0 <= elapsed < 1.05:
            return 2.0, 0.0, 0.0
        if self.scenario == "slow_xy_drift":
            return 0.04 * elapsed + 0.02 * elapsed, 0.0, 0.0
        if self.scenario == "slow_yaw_drift":
            return 0.0, 0.04 * elapsed, 0.0
        if self.scenario == "velocity_bias":
            return 0.0, 0.0, 0.75
        return 0.0, 0.0, 0.0

    def tick(self) -> None:
        elapsed = self.scenario_time()
        if elapsed >= self.duration_s:
            self.finish()
            return
        timestamp_ns = self.get_clock().now().nanoseconds
        lio_x, lio_yaw, lio_velocity = self.lio_values(elapsed)
        px4_x, px4_yaw, px4_velocity = self.px4_values(elapsed)
        propagated = odometry(timestamp_ns, lio_x, lio_yaw, lio_velocity)
        corrected = odometry(timestamp_ns, lio_x, lio_yaw, lio_velocity)
        px4 = odometry(timestamp_ns, px4_x, px4_yaw, px4_velocity)
        if self.scenario != "lio_propagated_stale" or elapsed < self.fault_start_s:
            self.propagated_pub.publish(propagated)
        if self.scenario != "lio_corrected_stale" or elapsed < self.fault_start_s:
            self.corrected_pub.publish(corrected)
        if self.scenario != "px4_stale" or elapsed < self.fault_start_s:
            self.px4_pub.publish(px4)

        lio_status = "Lost" if self.scenario == "correlated_unhealthy" else "Tracking"
        lio_values = {
            "diagnostic_schema_version": "2" if self.scenario == "diagnostic_schema_corruption" else "1",
            "status": lio_status,
            "navigation_valid": "false" if lio_status == "Lost" else "true",
            "initial_prior_source": "topic",
            "initial_prior_applied": "true",
            "initial_prior_fallback_applied": "false",
            "initial_prior_reason": "TOPIC_PRIOR_ACCEPTED",
        }
        px4_generation = "2" if self.scenario == "px4_reset_generation" and elapsed >= 2.0 else "1"
        px4_values_diag = {
            "diagnostic_schema_version": "1",
            "state": "running",
            "output_valid": "true",
            "continuity_valid": "true",
            "post_reset_stable": "true",
            "reset_generation": px4_generation,
            "time_generation": "1",
        }
        self.publish_diagnostic(self.lio_diag_pub, lio_values, "fast_lio/estimator")
        self.publish_diagnostic(self.px4_diag_pub, px4_values_diag, "px4_odometry_bridge")
        self.inputs.append({
            "stamp_ns": timestamp_ns,
            "elapsed_s": elapsed,
            "lio_position_x": lio_x,
            "px4_position_x": px4_x,
            "lio_yaw": lio_yaw,
            "px4_yaw": px4_yaw,
            "lio_velocity_x": lio_velocity,
            "px4_velocity_x": px4_velocity,
        })

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps({
            "scenario": self.scenario,
            "duration_s": self.duration_s,
            "inputs": self.inputs,
            "supervisor_status": self.statuses,
        }, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"wrote fault-injection artifact: {self.output}")
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SupervisorFaultInjector(args.scenario, args.duration, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

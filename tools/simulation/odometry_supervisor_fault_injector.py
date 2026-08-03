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
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import OdometrySupervisorStatus
from navigation_interfaces.srv import SampleOdometryAtTime
from rosgraph_msgs.msg import Clock
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


SCENARIOS = (
    "healthy",
    "single_position_jump",
    "slow_xy_drift",
    "slow_yaw_drift",
    "velocity_bias",
    "px4_stale",
    "px4_diagnostics_stale",
    "lio_propagated_stale",
    "lio_corrected_stale",
    "lio_diagnostics_stale",
    "px4_reset_generation",
    "px4_time_generation",
    "clock_pause",
    "diagnostic_schema_corruption",
    "lio_lost",
    "correlated_unhealthy",
)


EXPECTED_FINAL_HEALTH = {
    "healthy": {1},
    "single_position_jump": {1},
    "slow_xy_drift": {4},
    "slow_yaw_drift": {4},
    "velocity_bias": {3, 4},
    "px4_stale": {1},
    "px4_diagnostics_stale": {1},
    "lio_propagated_stale": {3},
    "lio_corrected_stale": {3},
    "lio_diagnostics_stale": {3},
    "px4_reset_generation": {1, 2},
    "px4_time_generation": {1, 2},
    "clock_pause": {1},
    "diagnostic_schema_corruption": {3},
    "lio_lost": {4},
    "correlated_unhealthy": {4},
}


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
             velocity_x: float = 0.0, frame_id: str = "lio_odom") -> Odometry:
    message = Odometry()
    message.header.stamp = stamp_from_ns(timestamp_ns)
    message.header.frame_id = frame_id
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.twist.twist.linear.x = velocity_x
    return message


class SupervisorFaultInjector(Node):
    def __init__(self, scenario: str, duration_s: float, output: Path,
                 use_sim_time: bool = False) -> None:
        super().__init__("p0_8_supervisor_fault_injector", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        self.scenario = scenario
        self.duration_s = duration_s
        self.output = output
        self.wall_started = time.monotonic()
        self.sim_time = bool(self.get_parameter("use_sim_time").value)
        self.clock_start_ns = 1_000_000_000 if self.sim_time else self.get_clock().now().nanoseconds
        self.fault_start_s = 5.0 if scenario == "clock_pause" else 3.0
        self.finished = False
        self.inputs: list[dict] = []
        self.statuses: list[dict] = []
        self.supervisor_diagnostics: list[dict] = []
        self.queries: list[dict] = []
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.propagated_pub = self.create_publisher(Odometry, "/lio/odometry_propagated", 20)
        self.corrected_pub = self.create_publisher(Odometry, "/lio/odometry_corrected", 20)
        self.px4_pub = self.create_publisher(Odometry, "/px4/estimator_odometry", 20)
        diagnostic_qos = 10
        if self.sim_time:
            diagnostic_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.lio_diag_pub = self.create_publisher(DiagnosticArray, "/lio/diagnostics", diagnostic_qos)
        self.px4_diag_pub = self.create_publisher(DiagnosticArray, "/px4/diagnostics", diagnostic_qos)
        self.status_sub = self.create_subscription(
            OdometrySupervisorStatus,
            "/navigation/odometry_supervisor/status",
            self.on_status,
            10,
        )
        self.supervisor_diag_sub = self.create_subscription(
            DiagnosticArray, "/navigation/odometry_supervisor/diagnostics",
            self.on_supervisor_diagnostics, 10)
        self.query_srv = self.create_service(
            SampleOdometryAtTime, "/px4/sample_odometry_at_time", self.on_query
        )
        self.timer = self.create_timer(0.10, self.tick,
                                      clock=RclpyClock(clock_type=ClockType.STEADY_TIME))

    def scenario_time(self) -> float:
        return time.monotonic() - self.wall_started

    def scenario_clock_ns(self, elapsed: float) -> int:
        if self.scenario == "clock_pause":
            # Start with the ROS clock already paused.  This isolates the
            # no-persistence-advance contract from DDS startup timing.
            elapsed = 0.0
        return self.clock_start_ns + int(elapsed * 1e9)

    def now_stamp(self, elapsed: float) -> Time:
        return stamp_from_ns(self.scenario_clock_ns(elapsed))

    def on_query(self, request, response):
        timestamp_ns = ns_from_stamp(request.sample_time)
        elapsed = max(0.0, (timestamp_ns - self.clock_start_ns) * 1e-9)
        if self.scenario == "px4_stale" and elapsed >= self.fault_start_s:
            response.success = False
            response.reason = "synthetic PX4 service timeout"
            self.queries.append({"requested_epoch_ns": timestamp_ns, "success": False,
                                 "reason": response.reason})
            return response
        x, yaw, velocity = self.px4_values(elapsed)
        response.success = True
        response.reason = "synthetic epoch-aligned reference"
        response.odometry = odometry(timestamp_ns, x, yaw, velocity, "px4_odom")
        response.interpolated = False
        response.reset_generation = 2 if self.scenario == "px4_reset_generation" and elapsed >= 2.0 else 1
        response.time_generation = 2 if self.scenario == "px4_time_generation" and elapsed >= 2.0 else 1
        response.component_validity_mask = 0xF
        response.covariance_availability_mask = 0x7
        self.queries.append({
            "requested_epoch_ns": timestamp_ns,
            "success": True,
            "reset_generation": int(response.reset_generation),
            "time_generation": int(response.time_generation),
            "component_validity_mask": int(response.component_validity_mask),
        })
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
            "comparison_epoch_ns": ns_from_stamp(message.comparison_epoch),
            "new_comparison_sample": bool(message.new_comparison_sample),
            "aligned_comparison_fresh": bool(message.aligned_comparison_fresh),
            "query_timeout_count": int(message.query_timeout_count),
            "query_generation_mismatch_count": int(message.query_generation_mismatch_count),
            "query_invalid_component_count": int(message.query_invalid_component_count),
            "px4_reset_generation": int(message.px4_reset_generation),
            "px4_time_generation": int(message.px4_time_generation),
        })

    def on_supervisor_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "odometry_supervisor":
                continue
            values = {item.key: item.value for item in status.values}
            self.supervisor_diagnostics.append({
                "stamp_ns": ns_from_stamp(message.header.stamp),
                "lio_diagnostics_valid": values.get("lio_diagnostics_valid"),
                "lio_diagnostics_schema_valid": values.get("lio_diagnostics_schema_valid"),
                "lio_diagnostics_stale": values.get("lio_diagnostics_stale"),
                "px4_diagnostics_valid": values.get("px4_diagnostics_valid"),
            })

    def publish_diagnostic(self, topic: str, values: dict[str, str], name: str) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.now_stamp(self.scenario_time())
        status = DiagnosticStatus()
        status.name = name
        status.level = DiagnosticStatus.OK
        status.message = "synthetic P0.8 input"
        status.values = key_values(values)
        message.status = [status]
        topic.publish(message)

    def px4_values(self, elapsed: float) -> tuple[float, float, float]:
        if self.scenario == "slow_xy_drift":
            return 0.0, 0.0, 0.0
        if self.scenario == "slow_yaw_drift":
            return 0.0, 0.0, 0.0
        if self.scenario == "velocity_bias":
            return 0.0, 0.0, 0.0
        return 0.0, 0.0, 0.0

    def lio_values(self, elapsed: float) -> tuple[float, float, float]:
        if self.scenario == "single_position_jump" and 1.0 <= elapsed < 1.05:
            return 2.0, 0.0, 0.0
        if self.scenario == "slow_xy_drift":
            return 0.20 * elapsed, 0.0, 0.0
        if self.scenario == "slow_yaw_drift":
            return 0.0, 0.10 * elapsed, 0.0
        if self.scenario == "velocity_bias":
            return 0.0, 0.0, 0.80
        return 0.0, 0.0, 0.0

    def tick(self) -> None:
        elapsed = self.scenario_time()
        if elapsed >= self.duration_s:
            self.finish()
            return
        timestamp_ns = self.scenario_clock_ns(elapsed)
        clock = Clock()
        clock.clock = stamp_from_ns(timestamp_ns)
        self.clock_pub.publish(clock)
        if self.scenario == "clock_pause" and elapsed >= self.fault_start_s:
            # Freeze the complete ROS-time input set at the pause epoch.  The
            # supervisor's wall timer may still run, but ROS-time freshness and
            # persistence must not advance.
            return
        lio_x, lio_yaw, lio_velocity = self.lio_values(elapsed)
        px4_x, px4_yaw, px4_velocity = self.px4_values(elapsed)
        propagated = odometry(timestamp_ns, lio_x, lio_yaw, lio_velocity)
        corrected = odometry(timestamp_ns, lio_x, lio_yaw, lio_velocity)
        px4 = odometry(timestamp_ns, px4_x, px4_yaw, px4_velocity, "px4_odom")
        if self.scenario != "lio_propagated_stale" or elapsed < self.fault_start_s:
            self.propagated_pub.publish(propagated)
        if self.scenario != "lio_corrected_stale" or elapsed < self.fault_start_s:
            self.corrected_pub.publish(corrected)
        if self.scenario != "px4_stale" or elapsed < self.fault_start_s:
            self.px4_pub.publish(px4)

        lio_status = "Lost" if self.scenario in {"correlated_unhealthy", "lio_lost"} else "Tracking"
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
        px4_time_generation = "2" if self.scenario == "px4_time_generation" and elapsed >= 2.0 else "1"
        px4_values_diag = {
            "diagnostic_schema_version": "2" if self.scenario == "diagnostic_schema_corruption" else "1",
            "state": "running",
            "output_valid": "true",
            "continuity_valid": "true",
            "post_reset_stable": "true",
            "reset_generation": px4_generation,
            "time_generation": px4_time_generation,
        }
        if self.scenario != "lio_diagnostics_stale" or elapsed < self.fault_start_s:
            self.publish_diagnostic(self.lio_diag_pub, lio_values, "fast_lio/estimator")
        if self.scenario != "px4_diagnostics_stale" or elapsed < self.fault_start_s:
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
        expected = EXPECTED_FINAL_HEALTH[self.scenario]
        observed = {entry["health"] for entry in self.statuses[-10:]}
        final_health = self.statuses[-1]["health"] if self.statuses else None
        oracle_pass = bool(self.statuses) and final_health in expected
        transitions = []
        for before, after in zip(self.statuses, self.statuses[1:]):
            if before["health"] != after["health"]:
                transitions.append({"from": before["health"], "to": after["health"],
                                    "at_ns": after["evaluation_time_ns"]})
        self.output.write_text(json.dumps({
            "scenario": self.scenario,
            "duration_s": self.duration_s,
            "expected_final_health": sorted(expected),
            "oracle": {"pass": oracle_pass, "final_health": final_health,
                        "observed_recent_health": sorted(observed),
                        "failure_reason": None if oracle_pass else "final health did not match scenario oracle"},
            "inputs": self.inputs,
            "queries": self.queries,
            "state_transitions": transitions,
            "supervisor_status": self.statuses,
            "supervisor_diagnostics": self.supervisor_diagnostics,
        }, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"wrote fault-injection artifact: {self.output}")
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--use-sim-time", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = SupervisorFaultInjector(args.scenario, args.duration, args.output,
                                   use_sim_time=args.use_sim_time)
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

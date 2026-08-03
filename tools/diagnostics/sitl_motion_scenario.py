#!/usr/bin/env python3
"""Drive a PX4 SITL vehicle through takeoff/yaw/translation and record evidence.

The setpoints are in PX4's documented local NED convention.  The recorder
keeps every stream in its native timestamp domain; it does not join samples by
callback arrival time.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time as RosTime
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommandAck,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def ros_time_ns(value: RosTime) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def finite(value: float) -> float | None:
    return float(value) if math.isfinite(float(value)) else None


class MotionScenario(Node):
    def __init__(self, output: Path, duration_s: float, wall_timeout_s: float,
                 scenario: str) -> None:
        super().__init__("p0_8_sitl_motion_scenario", parameter_overrides=[])
        self.output = output
        self.duration_s = duration_s
        self.wall_timeout_s = wall_timeout_s
        self.scenario = scenario
        self.started_wall = time.monotonic()
        self.start_sim_ns: int | None = None
        self.latest_sim_ns = 0
        self.last_phase = "PRESTART"
        self.command_last_sim_ns: dict[str, int] = {}
        self.counts: dict[str, int] = {}
        self.latest: dict[str, Any] = {}
        self.initial_heading_ned: float | None = None
        self.phase_events: list[dict[str, Any]] = []
        self.stream = output.open("w", encoding="utf-8")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.offboard_pub = self.create_publisher(OffboardControlMode,
                                                   "/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint,
                                                   "/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand,
                                                  "/fmu/in/vehicle_command", qos)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(Imu, "/lidar/imu", lambda m: self.record("lidar_imu", {
            "stamp_ns": ros_time_ns(m.header.stamp),
            "frame_id": m.header.frame_id,
            "angular_velocity": [finite(m.angular_velocity.x),
                                  finite(m.angular_velocity.y),
                                  finite(m.angular_velocity.z)],
            "linear_acceleration": [finite(m.linear_acceleration.x),
                                     finite(m.linear_acceleration.y),
                                     finite(m.linear_acceleration.z)],
        }), qos)
        self.create_subscription(VehicleLocalPosition,
                                 "/fmu/out/vehicle_local_position_v1",
                                 self.on_local_position, qos)
        self.create_subscription(VehicleStatus,
                                 "/fmu/out/vehicle_status_v1",
                                 lambda m: self.record("vehicle_status", {
                                     "timestamp_us": int(m.timestamp),
                                     "arming_state": int(m.arming_state),
                                     "nav_state": int(m.nav_state),
                                     "failsafe": bool(m.failsafe),
                                     "is_vtol": bool(m.is_vtol),
                                     "in_transition_mode": bool(m.in_transition_mode),
                                 }), qos)
        self.create_subscription(VehicleCommandAck,
                                 "/fmu/out/vehicle_command_ack",
                                 lambda m: self.record("vehicle_command_ack", {
                                     "timestamp_us": int(m.timestamp),
                                     "command": int(m.command),
                                     "result": int(m.result),
                                     "result_param1": int(m.result_param1),
                                     "result_param2": int(m.result_param2),
                                     "target_system": int(m.target_system),
                                     "target_component": int(m.target_component),
                                 }), qos)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 lambda m: self.record("vehicle_odometry", {
                                     "timestamp_us": int(m.timestamp),
                                     "timestamp_sample_us": int(m.timestamp_sample),
                                     "pose_frame": int(m.pose_frame),
                                     "velocity_frame": int(m.velocity_frame),
                                     "position": [finite(x) for x in m.position],
                                     "q": [finite(x) for x in m.q],
                                     "velocity": [finite(x) for x in m.velocity],
                                     "angular_velocity": [finite(x) for x in m.angular_velocity],
                                     "reset_counter": int(m.reset_counter),
                                 }), qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 lambda m: self.record("vehicle_attitude", {
                                     "timestamp_us": int(m.timestamp),
                                     "timestamp_sample_us": int(m.timestamp_sample),
                                     "q": [finite(x) for x in m.q],
                                     "delta_q_reset": [finite(x) for x in m.delta_q_reset],
                                     "quat_reset_counter": int(m.quat_reset_counter),
                                 }), qos)
        self.create_subscription(Odometry, "/px4/estimator_odometry",
                                 lambda m: self.record("px4_ros_odometry", self.odom(m)), qos)
        self.create_subscription(Odometry, "/lio/odometry_corrected",
                                 lambda m: self.record("lio_corrected", self.odom(m)), qos)
        self.create_subscription(Odometry, "/lio/odometry_propagated",
                                 lambda m: self.record("lio_propagated", self.odom(m)), qos)
        self.create_subscription(PointCloud2, "/lidar/points",
                                 lambda m: self.record("lidar_pointcloud", {
                                     "stamp_ns": ros_time_ns(m.header.stamp),
                                     "frame_id": m.header.frame_id,
                                     "height": int(m.height), "width": int(m.width),
                                     "point_step": int(m.point_step),
                                     "row_step": int(m.row_step),
                                     "is_dense": bool(m.is_dense),
                                     "fields": [field.name for field in m.fields],
                                     **self.pointcloud_summary(m),
                                 }), qos)
        self.create_subscription(DiagnosticArray, "/px4/diagnostics",
                                 lambda m: self.record_diagnostics("px4_diagnostics", m), qos)
        self.create_subscription(DiagnosticArray, "/lio/diagnostics",
                                 lambda m: self.record_diagnostics("lio_diagnostics", m), qos)
        self.create_subscription(DiagnosticArray,
                                 "/navigation/odometry_supervisor/diagnostics",
                                 lambda m: self.record_diagnostics("supervisor_diagnostics", m), qos)
        self.timer = self.create_timer(0.05, self.tick)

    @staticmethod
    def odom(message: Odometry) -> dict[str, Any]:
        return {
            "stamp_ns": ros_time_ns(message.header.stamp),
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "position": [finite(message.pose.pose.position.x),
                         finite(message.pose.pose.position.y),
                         finite(message.pose.pose.position.z)],
            "q_xyzw": [finite(message.pose.pose.orientation.x),
                        finite(message.pose.pose.orientation.y),
                        finite(message.pose.pose.orientation.z),
                        finite(message.pose.pose.orientation.w)],
            "linear_velocity": [finite(message.twist.twist.linear.x),
                                 finite(message.twist.twist.linear.y),
                                 finite(message.twist.twist.linear.z)],
            "angular_velocity": [finite(message.twist.twist.angular.x),
                                  finite(message.twist.twist.angular.y),
                                  finite(message.twist.twist.angular.z)],
        }

    @staticmethod
    def pointcloud_summary(message: PointCloud2) -> dict[str, Any]:
        samples: list[tuple[float, float, float]] = []
        for index, point in enumerate(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)):
            values = tuple(float(value) for value in point)
            if all(math.isfinite(value) for value in values) and index % 16 == 0:
                samples.append(values)
            if len(samples) >= 512:
                break
        if not samples:
            return {"sample_count": 0}
        return {
            "sample_count": len(samples),
            "sample_min": [min(point[index] for point in samples) for index in range(3)],
            "sample_max": [max(point[index] for point in samples) for index in range(3)],
            "sample_mean": [sum(point[index] for point in samples) / len(samples)
                            for index in range(3)],
        }

    def record(self, name: str, payload: dict[str, Any]) -> None:
        self.counts[name] = self.counts.get(name, 0) + 1
        self.latest[name] = payload
        self.stream.write(json.dumps({"kind": "sample", "stream": name,
                                      "callback_wall_ns": time.monotonic_ns(),
                                      "sim_time_ns": self.latest_sim_ns,
                                      "payload": payload}, sort_keys=True) + "\n")

    def record_diagnostics(self, name: str, message: DiagnosticArray) -> None:
        def level_value(status: Any) -> int:
            value = status.level
            return int(value[0]) if isinstance(value, (bytes, bytearray)) else int(value)

        self.record(name, {"stamp_ns": ros_time_ns(message.header.stamp),
                           "statuses": [{"name": status.name, "level": level_value(status),
                                         "message": status.message,
                                        "values": {item.key: item.value for item in status.values}}
                                        for status in message.status]})

    def on_local_position(self, message: VehicleLocalPosition) -> None:
        heading = finite(message.heading)
        if self.initial_heading_ned is None and heading is not None:
            self.initial_heading_ned = heading
        self.record("vehicle_local_position", {
            "timestamp_us": int(message.timestamp),
            "timestamp_sample_us": int(message.timestamp_sample),
            "x_ned_m": finite(message.x), "y_ned_m": finite(message.y),
            "z_ned_m": finite(message.z), "vx_ned_m_s": finite(message.vx),
            "vy_ned_m_s": finite(message.vy), "vz_ned_m_s": finite(message.vz),
            "heading_ned_rad": heading,
            "delta_xy_ned_m": [finite(x) for x in message.delta_xy],
            "delta_z_ned_m": finite(message.delta_z),
            "delta_vxy_ned_m_s": [finite(x) for x in message.delta_vxy],
            "delta_vz_ned_m_s": finite(message.delta_vz),
            "delta_heading_ned_rad": finite(message.delta_heading),
            "xy_reset_counter": int(message.xy_reset_counter),
            "z_reset_counter": int(message.z_reset_counter),
            "heading_reset_counter": int(message.heading_reset_counter),
            "xy_valid": bool(message.xy_valid), "z_valid": bool(message.z_valid),
        })

    def on_clock(self, message: Clock) -> None:
        self.latest_sim_ns = ros_time_ns(message.clock)
        if self.latest_sim_ns > 0 and self.start_sim_ns is None:
            self.start_sim_ns = self.latest_sim_ns

    def sim_elapsed_s(self) -> float:
        if self.start_sim_ns is None:
            return 0.0
        return max(0.0, (self.latest_sim_ns - self.start_sim_ns) / 1e9)

    def phase(self, elapsed_s: float) -> tuple[str, float, float, float]:
        # PX4 local position and trajectory setpoints are NED: negative z is up.
        if self.scenario == "takeoff":
            if elapsed_s < 3.0:
                return "GROUND", 0.0, 0.0, 0.0
            return "TAKEOFF_HOLD_YAW", 0.0, 0.0, -2.0
        if self.scenario == "yaw":
            if elapsed_s < 3.0:
                return "GROUND", 0.0, 0.0, 0.0
            if elapsed_s < 15.0:
                return "TAKEOFF_HOLD_YAW", 0.0, 0.0, -2.0
            return "YAW_PLUS_90", 0.0, 0.0, -2.0
        if elapsed_s < 3.0:
            return "GROUND", 0.0, 0.0, 0.0
        if elapsed_s < 15.0:
            return "TAKEOFF_HOLD_YAW", 0.0, 0.0, -2.0
        if elapsed_s < 27.0:
            return "YAW_PLUS_90", 0.0, 0.0, -2.0
        if elapsed_s < 39.0:
            return "TRANSLATE_XY", 2.0, 1.0, -2.0
        if elapsed_s < 51.0:
            return "YAW_MINUS_90", 2.0, 1.0, -2.0
        return "LAND", 0.0, 0.0, 0.0

    def publish_command(self, name: str, command: int, param1: float = 0.0,
                        param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.timestamp = self.latest_sim_ns // 1000
        message.command = command
        message.param1 = param1
        message.param2 = param2
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)
        self.phase_events.append({"kind": "command", "name": name,
                                  "sim_time_ns": self.latest_sim_ns,
                                  "command": command, "param1": param1,
                                  "param2": param2})

    def tick(self) -> None:
        if time.monotonic() - self.started_wall > self.wall_timeout_s:
            self.finish("WALL_TIMEOUT")
            return
        elapsed = self.sim_elapsed_s()
        if self.start_sim_ns is None:
            return
        name, x, y, z = self.phase(elapsed)
        if name != self.last_phase:
            self.last_phase = name
            self.phase_events.append({"kind": "phase", "name": name,
                                      "sim_time_ns": self.latest_sim_ns,
                                      "elapsed_s": elapsed,
                                      "setpoint_ned": [x, y, z]})
        mode = OffboardControlMode()
        mode.timestamp = self.latest_sim_ns // 1000
        mode.position = True
        self.offboard_pub.publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.latest_sim_ns // 1000
        setpoint.position = [float(x), float(y), float(z)]
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        initial_heading = self.initial_heading_ned if self.initial_heading_ned is not None else 0.0
        if name in {"GROUND", "TAKEOFF_HOLD_YAW"}:
            setpoint.yaw = initial_heading
        elif name in {"YAW_PLUS_90", "TRANSLATE_XY"}:
            setpoint.yaw = initial_heading + math.pi / 2.0
        elif name == "YAW_MINUS_90":
            setpoint.yaw = initial_heading - math.pi / 2.0
        else:
            setpoint.yaw = initial_heading
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)
        last_command = self.command_last_sim_ns.get("mode_arm", -10**18)
        if elapsed >= 2.0 and self.latest_sim_ns - last_command >= 1_000_000_000:
            self.publish_command("offboard_mode", VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                 1.0, 6.0)
            self.publish_command("arm", VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.command_last_sim_ns["mode_arm"] = self.latest_sim_ns
        if elapsed >= self.duration_s:
            self.finish("COMPLETE")

    def finish(self, reason: str) -> None:
        if not self.stream.closed:
            summary = {"kind": "summary", "reason": reason,
                       "duration_s": self.sim_elapsed_s(),
                       "phase_events": self.phase_events,
                       "counts": self.counts, "latest": self.latest,
                       "wall_elapsed_s": time.monotonic() - self.started_wall}
            self.stream.write(json.dumps(summary, sort_keys=True) + "\n")
            self.stream.flush()
            self.stream.close()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--wall-timeout-s", type=float, default=180.0)
    parser.add_argument("--scenario", choices=("full", "takeoff", "yaw"), default="full")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = MotionScenario(args.output, args.duration_s, args.wall_timeout_s, args.scenario)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if not node.stream.closed:
            node.finish("INTERRUPTED")
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

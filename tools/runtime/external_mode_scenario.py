#!/usr/bin/env python3
"""PX4-native External Mode acceptance scenario.

The M1 scenario injects a deterministic stationary PlannedTrajectory fixture
at the product-owned trajectory boundary and observes the resulting PX4
TrajectorySetpoint stream. It selects
the registered external mode from VehicleStatus, arms, enters the mode, then
hands control back to PX4 AUTO_LOITER and disarms on the ground. It does not
publish OffboardControlMode or use the Offboard mode path.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time
from typing import Any


def _time_ns(value: Any) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def _finite_vector(values: Any, size: int = 3) -> bool:
    try:
        return len(values) == size and all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


class ExternalModeScenario:
    def __init__(self, output: Path, config: dict[str, Any]) -> None:
        import rclpy
        from geometry_msgs.msg import Point, PoseStamped, Vector3
        from nav_msgs.msg import Odometry
        from navigation_interfaces.msg import PlannedTrajectory
        from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleStatus
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self.rclpy = rclpy
        self.Node = Node
        self.PoseStamped = PoseStamped
        self.Point = Point
        self.Vector3 = Vector3
        self.VehicleCommand = VehicleCommand
        self.VehicleStatus = VehicleStatus
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.with_suffix(".jsonl").open("w", encoding="utf-8")
        self.config = config.get("scenario", config)
        self.wall_start = time.monotonic()
        self.sim_start_ns: int | None = None
        self.sim_now_ns = 0
        self.latest_odom: dict[str, float] | None = None
        self.latest_status: dict[str, Any] = {}
        self.previous_nav_state: int | None = None
        self.external_mode_id: int | None = None
        self.mode_entered = False
        self.mode_exit_observed = False
        self.exit_requested = False
        self.exit_request_sim_ns: int | None = None
        self.trajectory_received = 0
        self.trajectory_success_count = 0
        self.latest_trajectory: dict[str, Any] = {}
        self.setpoint_count = 0
        self.finite_setpoint_count = 0
        self.ack_count = 0
        self.command_acks: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.last_command_ns: dict[str, int] = {}
        self.last_trajectory_ns = -10**18
        self.armed_seen = False
        self.failsafe_seen = False
        self.unexpected_rtl = False
        self.finished = False
        self.failure = ""

        self.node = Node("uav_navigation_external_mode_scenario")
        px4_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.trajectory_pub = self.node.create_publisher(PlannedTrajectory, "/navigation/trajectory", reliable_qos)
        self.command_pub = self.node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        self.node.create_subscription(Clock, "/clock", self._clock, px4_qos)
        self.node.create_subscription(Odometry, "/lio/odometry_propagated", self._odometry, reliable_qos)
        self.node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._status, px4_qos)
        self.node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._setpoint, px4_qos)
        self.node.create_subscription(PlannedTrajectory, "/navigation/trajectory", self._trajectory, reliable_qos)
        self.node.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._ack, px4_qos)
        self.timer = self.node.create_timer(0.05, self._tick)

    def _record(self, kind: str, payload: dict[str, Any]) -> None:
        self.stream.write(json.dumps({
            "kind": kind,
            "sim_time_ns": self.sim_now_ns,
            "payload": payload,
        }, sort_keys=True, allow_nan=False) + "\n")

    def _clock(self, message: Any) -> None:
        self.sim_now_ns = _time_ns(message.clock)
        if self.sim_now_ns > 0 and self.sim_start_ns is None:
            self.sim_start_ns = self.sim_now_ns

    def _odometry(self, message: Any) -> None:
        if message.header.frame_id != "lio_odom":
            return
        position = message.pose.pose.position
        values = (float(position.x), float(position.y), float(position.z))
        if all(math.isfinite(value) for value in values):
            self.latest_odom = {"x": values[0], "y": values[1], "z": values[2]}

    def _status(self, message: Any) -> None:
        nav_state = int(message.nav_state)
        armed = int(message.arming_state) == int(self.VehicleStatus.ARMING_STATE_ARMED)
        mask = int(message.can_set_nav_states_mask)
        if self.external_mode_id is None:
            for candidate in range(
                int(self.VehicleStatus.NAVIGATION_STATE_EXTERNAL1),
                int(self.VehicleStatus.NAVIGATION_STATE_EXTERNAL8) + 1,
            ):
                if mask & (1 << candidate):
                    self.external_mode_id = candidate
                    event = {"name": "external_mode_discovered", "mode_id": candidate}
                    self.events.append(event)
                    self._record("event", event)
                    break
        if self.external_mode_id is not None and nav_state == self.external_mode_id:
            if not self.mode_entered:
                event = {"name": "external_mode_entered", "mode_id": self.external_mode_id}
                self.events.append(event)
                self._record("event", event)
            self.mode_entered = True
        if self.mode_entered and self.exit_requested and int(message.executor_in_charge) == 0 and not self.mode_exit_observed:
            self.mode_exit_observed = True
            event = {"name": "external_mode_exit_observed", "nav_state": nav_state, "executor_in_charge": int(message.executor_in_charge)}
            self.events.append(event)
            self._record("event", event)
        elif self.mode_entered and nav_state != self.external_mode_id and not self.mode_exit_observed:
            self.mode_exit_observed = True
            event = {"name": "external_mode_exit_observed", "nav_state": nav_state}
            self.events.append(event)
            self._record("event", event)
        if self.mode_entered and nav_state == int(self.VehicleStatus.NAVIGATION_STATE_AUTO_RTL):
            self.unexpected_rtl = True
        self.failsafe_seen = self.failsafe_seen or bool(message.failsafe)
        self.armed_seen = self.armed_seen or armed
        self.latest_status = {
            "timestamp_us": int(message.timestamp),
            "arming_state": int(message.arming_state),
            "nav_state": nav_state,
            "failsafe": bool(message.failsafe),
            "pre_flight_checks_pass": bool(message.pre_flight_checks_pass),
            "executor_in_charge": int(message.executor_in_charge),
            "can_set_nav_states_mask": mask,
        }
        if nav_state != self.previous_nav_state:
            self._record("vehicle_status", self.latest_status)
        self.previous_nav_state = nav_state

    def _trajectory(self, message: Any) -> None:
        self.trajectory_received += 1
        if not bool(message.success):
            return
        self.trajectory_success_count += 1
        self.latest_trajectory = {
            "world_generation": int(message.world_generation),
            "world_revision": int(message.world_revision),
            "duration_s": float(message.duration_s),
            "point_count": len(message.position),
        }
        self._record("trajectory", self.latest_trajectory)

    def _setpoint(self, message: Any) -> None:
        self.setpoint_count += 1
        finite = (
            _finite_vector(message.position)
            and _finite_vector(message.velocity)
            and _finite_vector(message.acceleration)
        )
        if finite:
            self.finite_setpoint_count += 1
        if self.setpoint_count == 1 or self.setpoint_count % 50 == 0:
            self._record("setpoint", {
                "finite": finite,
                "position_ned": [float(value) for value in message.position],
                "velocity_ned": [float(value) for value in message.velocity],
                "acceleration_ned": [float(value) for value in message.acceleration],
            })

    def _ack(self, message: Any) -> None:
        self.ack_count += 1
        record = {"command": int(message.command), "result": int(message.result)}
        self.command_acks.append(record)
        self._record("command_ack", record)

    def sim_elapsed_s(self) -> float:
        if self.sim_start_ns is None:
            return 0.0
        return max(0.0, (self.sim_now_ns - self.sim_start_ns) / 1e9)

    def _command(self, name: str, command: int, p1: float = 0.0) -> None:
        message = self.VehicleCommand()
        message.timestamp = self.sim_now_ns // 1000
        message.command = command
        message.param1 = p1
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)
        event = {"name": name, "command": command, "param1": p1}
        self.events.append(event)
        self._record("command", event)

    def _retry(self, name: str, command: int, p1: float = 0.0) -> None:
        period_ns = int(float(self.config.get("command_retry_period_s", 1.0)) * 1e9)
        if self.sim_now_ns - self.last_command_ns.get(name, -10**18) >= period_ns:
            self._command(name, command, p1)
            self.last_command_ns[name] = self.sim_now_ns

    def _publish_fixture_trajectory(self) -> None:
        from navigation_interfaces.msg import PlannedTrajectory

        position = self.latest_odom or {"x": 0.0, "y": 0.0, "z": 0.0}
        trajectory = PlannedTrajectory()
        trajectory.header.frame_id = "lio_odom"
        trajectory.header.stamp.sec = self.sim_now_ns // 1_000_000_000
        trajectory.header.stamp.nanosec = self.sim_now_ns % 1_000_000_000
        trajectory.success = True
        trajectory.world_generation = 1
        trajectory.world_revision = 0
        trajectory.duration_s = 1.0
        trajectory.time_from_start = [0.0, 1.0]
        trajectory.position = [self.Point(), self.Point()]
        trajectory.velocity = [self.Vector3(), self.Vector3()]
        trajectory.acceleration = [self.Vector3(), self.Vector3()]
        for point in trajectory.position:
            point.x = position["x"]
            point.y = position["y"]
            point.z = position["z"]
        self.trajectory_pub.publish(trajectory)
        self.last_trajectory_ns = self.sim_now_ns

    def _tick(self) -> None:
        if self.finished:
            return
        if time.monotonic() - self.wall_start > float(self.config.get("wall_timeout_s", 120.0)):
            self.failure = "scenario wall timeout"
            self.finish("WALL_TIMEOUT")
            return
        if self.sim_start_ns is None:
            return
        elapsed = self.sim_elapsed_s()
        trajectory_period_ns = int(float(self.config.get("trajectory_publish_period_s", 0.2)) * 1e9)
        if self.config.get("trajectory_source", "fixture") == "fixture" and self.sim_now_ns - self.last_trajectory_ns >= trajectory_period_ns:
            self._publish_fixture_trajectory()

        if self.external_mode_id is None and elapsed > float(self.config.get("activation_timeout_s", 30.0)):
            self.failure = "PX4 did not advertise a registered External Mode"
            self.finish("ACTIVATION_TIMEOUT")
            return
        if self.trajectory_success_count == 0:
            if elapsed > float(self.config.get("activation_timeout_s", 30.0)):
                self.failure = "navigation runtime did not publish a successful trajectory"
                self.finish("TRAJECTORY_TIMEOUT")
            return
        if not self.mode_entered:
            if self.external_mode_id is None:
                if elapsed > float(self.config.get("activation_timeout_s", 30.0)):
                    self.failure = "PX4 did not advertise a registered External Mode"
                    self.finish("ACTIVATION_TIMEOUT")
                return
            self._retry("activate_external_mode", self.VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, float(self.external_mode_id))
            if elapsed > float(self.config.get("activation_timeout_s", 30.0)):
                self.failure = "PX4 did not enter the registered External Mode"
                self.finish("ACTIVATION_TIMEOUT")
            return
        if not self.armed_seen:
            if not bool(self.latest_status.get("pre_flight_checks_pass", False)):
                if elapsed > float(self.config.get("activation_timeout_s", 30.0)):
                    self.failure = "PX4 pre-flight checks did not become ready"
                    self.finish("PREFLIGHT_TIMEOUT")
                return
            self._retry("arm", self.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            return
        if not self.exit_requested and elapsed >= float(self.config.get("post_activation_s", 8.0)):
            exit_state = float(self.config.get("exit_nav_state", self.VehicleStatus.NAVIGATION_STATE_AUTO_LOITER))
            self._command("exit_external_mode", self.VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, exit_state)
            self.exit_requested = True
            self.exit_request_sim_ns = self.sim_now_ns
            return
        if self.mode_exit_observed and self.latest_status.get("arming_state") == int(self.VehicleStatus.ARMING_STATE_ARMED):
            self._retry("disarm", self.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            if self.exit_request_sim_ns is not None and self.sim_now_ns - self.exit_request_sim_ns > int(float(self.config.get("disarm_timeout_s", 10.0)) * 1e9):
                self.failure = "vehicle did not disarm after External Mode exit"
                self.finish("DISARM_TIMEOUT")
            return
        if self.mode_exit_observed and self.latest_status.get("arming_state") == int(self.VehicleStatus.ARMING_STATE_DISARMED):
            self.finish("COMPLETE")

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        failures = [self.failure] if self.failure else []
        if self.external_mode_id is None:
            failures.append("External Mode id was not discovered")
        if self.trajectory_success_count <= 0:
            failures.append("no successful PlannedTrajectory observed")
        if not self.mode_entered:
            failures.append("External Mode was never entered")
        if self.setpoint_count <= 0:
            failures.append("no PX4 TrajectorySetpoint observed")
        if self.setpoint_count > 0 and self.finite_setpoint_count != self.setpoint_count:
            failures.append("PX4 TrajectorySetpoint contained non-finite values")
        if not self.mode_exit_observed:
            failures.append("External Mode exit was not observed")
        if self.failsafe_seen:
            failures.append("PX4 reported failsafe during External Mode acceptance")
        if self.unexpected_rtl:
            failures.append("unexpected RTL observed during External Mode acceptance")
        summary = {
            "reason": reason,
            "duration_s": self.sim_elapsed_s(),
            "wall_elapsed_s": time.monotonic() - self.wall_start,
            "external_mode_id": self.external_mode_id,
            "trajectory_received": self.trajectory_received,
            "trajectory_success_count": self.trajectory_success_count,
            "latest_trajectory": self.latest_trajectory,
            "setpoint_count": self.setpoint_count,
            "finite_setpoint_count": self.finite_setpoint_count,
            "command_ack_count": self.ack_count,
            "command_acks": self.command_acks,
            "external_mode_entered": self.mode_entered,
            "external_mode_exit_observed": self.mode_exit_observed,
            "armed": self.armed_seen,
            "pre_flight_checks_pass": bool(self.latest_status.get("pre_flight_checks_pass", False)),
            "disarm_successful": self.latest_status.get("arming_state") == int(self.VehicleStatus.ARMING_STATE_DISARMED),
            "failsafe_seen": self.failsafe_seen,
            "unexpected_rtl": self.unexpected_rtl,
            "events": self.events,
            "failures": failures,
        }
        self.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.stream.close()


def run(output: Path, config_path: Path) -> int:
    import rclpy
    import yaml

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rclpy.init(args=[])
    scenario = ExternalModeScenario(output, config)

    def stop(_signum: int, _frame: Any) -> None:
        scenario.failure = "scenario interrupted"
        scenario.finish("INTERRUPTED")

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not scenario.finished:
            rclpy.spin_once(scenario.node, timeout_sec=0.1)
    finally:
        if not scenario.finished:
            scenario.finish("INTERRUPTED")
        scenario.node.destroy_node()
        rclpy.shutdown()
    summary = json.loads(output.read_text(encoding="utf-8"))
    return 0 if not summary.get("failures") else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    return run(args.output.resolve(), args.config.resolve())


if __name__ == "__main__":
    raise SystemExit(main())

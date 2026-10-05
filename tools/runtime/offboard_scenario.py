#!/usr/bin/env python3
"""Deterministic PX4 offboard acceptance scenario."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time
from typing import Any


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _time_ns(value: Any) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


class OffboardScenario:
    def __init__(self, output: Path, config: dict[str, Any]) -> None:
        import rclpy
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
            VehicleLandDetected,
            VehicleLocalPosition,
            VehicleStatus,
        )
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self.rclpy = rclpy
        self.VehicleCommand = VehicleCommand
        self.VehicleStatus = VehicleStatus
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.with_suffix(".jsonl").open("w", encoding="utf-8")
        self.config = config.get("scenario", config)
        self.wall_start = time.monotonic()
        self.sim_start_ns: int | None = None
        self.sim_now_ns = 0
        self.latest_status: dict[str, Any] = {}
        self.latest_position: dict[str, Any] = {}
        self.landed_seen = False
        self.grounded_seen = False
        self.disarm_forced = False
        self.events: list[dict[str, Any]] = []
        self.last_segment = "STARTUP"
        self.last_command_ns: dict[str, int] = {}
        self.offboard_seen = False
        self.previous_nav_state: int | None = None
        self.offboard_loss_count = 0
        self.armed_seen = False
        self.takeoff_seen = False
        self.land_commanded = False
        self.disarm_commanded = False
        self.finished = False
        self.failure = ""
        self.segments = self._segments()

        self.node = Node("uav_navigation_offboard_scenario")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.mode_pub = self.node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.node.create_subscription(Clock, "/clock", self._clock, qos)
        self.node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._status, qos)
        self.node.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_detected, qos)
        self.node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._position, qos)
        self.timer = self.node.create_timer(0.05, self._tick)

    def _segments(self) -> list[dict[str, Any]]:
        configured = self.config.get("segments")
        if configured:
            return list(configured)
        return [
            {"name": "GROUND", "duration_s": 3.0, "position_ned": [0.0, 0.0, 0.0]},
            {"name": "TAKEOFF", "duration_s": 8.0, "position_ned": [0.0, 0.0, -2.0]},
            {"name": "HOVER_1", "duration_s": 10.0, "position_ned": [0.0, 0.0, -2.0]},
            {"name": "TRANSLATE_X", "duration_s": 5.0, "position_ned": [3.0, 0.0, -2.0]},
            {"name": "HOVER_2", "duration_s": 5.0, "position_ned": [3.0, 0.0, -2.0]},
            {"name": "TRANSLATE_Y", "duration_s": 5.0, "position_ned": [3.0, 3.0, -2.0]},
            {"name": "HOVER_3", "duration_s": 5.0, "position_ned": [3.0, 3.0, -2.0]},
            {"name": "YAW", "duration_s": 5.0, "position_ned": [3.0, 3.0, -2.0], "yaw_rad": math.pi / 2.0},
            {"name": "RETURN", "duration_s": 8.0, "position_ned": [0.0, 0.0, -2.0]},
            {"name": "LAND", "duration_s": 8.0, "position_ned": [0.0, 0.0, 0.0]},
            {"name": "DISARM", "duration_s": 3.0, "position_ned": [0.0, 0.0, 0.0]},
        ]

    def _clock(self, message: Any) -> None:
        self.sim_now_ns = _time_ns(message.clock)
        if self.sim_now_ns and self.sim_start_ns is None:
            self.sim_start_ns = self.sim_now_ns

    def _status(self, message: Any) -> None:
        nav = int(message.nav_state)
        armed = int(message.arming_state) == int(self.VehicleStatus.ARMING_STATE_ARMED)
        if nav == int(self.VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.offboard_seen = True
        elif (self.previous_nav_state == int(self.VehicleStatus.NAVIGATION_STATE_OFFBOARD)
              and not self.land_commanded):
            self.offboard_loss_count += 1
        self.previous_nav_state = nav
        self.armed_seen = self.armed_seen or armed
        if armed and self.sim_elapsed_s() > 3.0:
            altitude = _finite(self.latest_position.get("z_ned_m"))
            self.takeoff_seen = self.takeoff_seen or (altitude is not None and altitude < -0.5)
        self.latest_status = {
            "timestamp_us": int(message.timestamp),
            "arming_state": int(message.arming_state),
            "nav_state": nav,
            "failsafe": bool(message.failsafe),
        }
        self._record("vehicle_status", self.latest_status)

    def _position(self, message: Any) -> None:
        self.latest_position = {
            "timestamp_us": int(message.timestamp),
            "x_ned_m": _finite(message.x), "y_ned_m": _finite(message.y), "z_ned_m": _finite(message.z),
            "vx_ned_m_s": _finite(message.vx), "vy_ned_m_s": _finite(message.vy), "vz_ned_m_s": _finite(message.vz),
            "heading_ned_rad": _finite(message.heading),
            "xy_valid": bool(message.xy_valid), "z_valid": bool(message.z_valid),
            "v_xy_valid": bool(message.v_xy_valid), "v_z_valid": bool(message.v_z_valid),
            "dead_reckoning": bool(message.dead_reckoning),
        }
        if (self.land_commanded and self.latest_position["z_ned_m"] is not None and
                abs(float(self.latest_position["z_ned_m"])) < 0.35 and
                all(self.latest_position[name] is not None and
                    abs(float(self.latest_position[name])) < 0.20
                    for name in ("vx_ned_m_s", "vy_ned_m_s", "vz_ned_m_s"))):
            self.grounded_seen = True
        self._record("vehicle_local_position", self.latest_position)

    def _land_detected(self, message: Any) -> None:
        self.landed_seen = bool(message.landed)
        if self.land_commanded and (bool(message.landed) or bool(message.maybe_landed) or bool(message.ground_contact)):
            self.grounded_seen = True
        self._record("vehicle_land_detected", {
            "landed": bool(message.landed),
            "maybe_landed": bool(message.maybe_landed),
            "ground_contact": bool(message.ground_contact),
        })

    def sim_elapsed_s(self) -> float:
        if self.sim_start_ns is None:
            return 0.0
        return max(0.0, (self.sim_now_ns - self.sim_start_ns) / 1e9)

    def _segment(self, elapsed_s: float) -> tuple[dict[str, Any], float]:
        elapsed = elapsed_s
        for segment in self.segments:
            duration = float(segment.get("duration_s", 0.0))
            if elapsed < duration:
                return segment, elapsed
            elapsed -= duration
        final = self.segments[-1]
        return final, float(final.get("duration_s", 0.0))

    def _record(self, kind: str, payload: dict[str, Any]) -> None:
        self.stream.write(json.dumps({"kind": kind, "sim_time_ns": self.sim_now_ns, "payload": payload}, sort_keys=True, allow_nan=False) + "\n")

    def _command(self, name: str, command: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0) -> None:
        message = self.VehicleCommand()
        message.timestamp = self.sim_now_ns // 1000
        message.command = command
        message.param1 = p1
        message.param2 = p2
        message.param3 = p3
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)
        event = {"kind": "command", "name": name, "sim_time_ns": self.sim_now_ns, "command": command}
        self.events.append(event)
        self._record("command", event)

    def _tick(self) -> None:
        if self.finished:
            return
        if time.monotonic() - self.wall_start > float(self.config.get("wall_timeout_s", 180.0)):
            self.failure = "scenario wall timeout"
            self.finish("WALL_TIMEOUT")
            return
        if self.sim_start_ns is None:
            return
        elapsed = self.sim_elapsed_s()
        segment, segment_elapsed = self._segment(elapsed)
        name = str(segment["name"])
        if name != self.last_segment:
            self.last_segment = name
            event = {"kind": "segment", "name": name, "sim_time_ns": self.sim_now_ns, "setpoint_ned": segment.get("position_ned", [0.0, 0.0, 0.0])}
            self.events.append(event)
            self._record("segment", event)
        mode = self.VehicleCommand
        from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
        offboard = OffboardControlMode()
        offboard.timestamp = self.sim_now_ns // 1000
        offboard.position = True
        self.mode_pub.publish(offboard)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.sim_now_ns // 1000
        # PX4 TrajectorySetpoint.position is always local NED (x north, y east,
        # z down).  Gazebo ground truth is ENU; the runtime report checks the
        # corresponding x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned mapping.
        position = [float(value) for value in segment.get("position_ned", [0.0, 0.0, 0.0])]
        setpoint.position = position
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = float(segment.get("yaw_rad", 0.0))
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)
        self._record("setpoint", {"segment": name, "position_ned": position, "yaw_rad": setpoint.yaw})

        if name not in {"LAND", "DISARM"} and elapsed >= 2.0 and self.sim_now_ns - self.last_command_ns.get("offboard_arm", -10**18) >= 1_000_000_000:
            self._command("offboard", mode.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._command("arm", mode.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_command_ns["offboard_arm"] = self.sim_now_ns
        if name == "LAND" and not self.land_commanded:
            self._command("land", mode.VEHICLE_CMD_NAV_LAND)
            self.land_commanded = True
        if name == "DISARM":
            if self.grounded_seen and not self.disarm_commanded:
                self._command("disarm", mode.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.disarm_commanded = True
                self.last_command_ns["disarm"] = self.sim_now_ns
            elif (self.grounded_seen and not self.disarm_forced and
                  self.sim_now_ns - self.last_command_ns.get("disarm", -10**18) >= 3_000_000_000):
                self._command("disarm_force", mode.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self.disarm_forced = True
                self.last_command_ns["disarm_force"] = self.sim_now_ns
            elif (self.grounded_seen and self.disarm_forced and
                  self.sim_now_ns - self.last_command_ns.get("disarm_force", -10**18) >= 1_000_000_000):
                self._command("disarm_force", mode.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self.last_command_ns["disarm_force"] = self.sim_now_ns
            if (self.grounded_seen and
                    self.latest_status.get("arming_state") == int(self.VehicleStatus.ARMING_STATE_DISARMED)):
                self.finish("COMPLETE")
            elif segment_elapsed >= float(segment.get("duration_s", 0.0)):
                self.failure = "vehicle did not disarm"
                self.finish("DISARM_TIMEOUT")

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        summary = {
            "reason": reason,
            "duration_s": self.sim_elapsed_s(),
            "wall_elapsed_s": time.monotonic() - self.wall_start,
            "offboard_entered": self.offboard_seen,
            "offboard_loss_count": self.offboard_loss_count,
            "armed": self.armed_seen,
            "takeoff_observed": self.takeoff_seen,
            "landing_commanded": self.land_commanded,
            "disarm_commanded": self.disarm_commanded,
            "grounded_observed": self.grounded_seen,
            "landing_successful": self.land_commanded and self.grounded_seen,
            "disarm_forced": self.disarm_forced,
            "disarm_successful": self.latest_status.get("arming_state") == int(self.VehicleStatus.ARMING_STATE_DISARMED),
            "events": self.events,
            "failures": [self.failure] if self.failure else [],
        }
        if not summary["offboard_entered"]:
            summary["failures"].append("OFFBOARD was never entered")
        if not summary["armed"]:
            summary["failures"].append("vehicle did not arm")
        if not summary["takeoff_observed"]:
            summary["failures"].append("vehicle did not reach takeoff altitude")
        if summary["offboard_loss_count"]:
            summary["failures"].append("unexpected OFFBOARD loss")
        if summary["landing_commanded"] and not summary["landing_successful"]:
            summary["failures"].append("vehicle did not reach grounded state")
        if reason == "COMPLETE" and not summary["disarm_successful"]:
            summary["failures"].append("vehicle did not disarm")
        self.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.stream.close()


def run(output: Path, config_path: Path) -> int:
    import yaml
    import rclpy

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rclpy.init(args=[])
    scenario = OffboardScenario(output, config)

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

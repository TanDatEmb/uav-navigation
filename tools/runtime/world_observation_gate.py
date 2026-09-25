#!/usr/bin/env python3
"""Test-only RegisteredScan relay with a simulation-time, mapping-only dropout.

This process is launched only by an explicit SITL runner option. It has no
writer to WorldModel, ExecutionAuthority, NavigationCommand, or PX4 interfaces.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from navigation_contracts.msg import (
    EstimatorHealth,
    NavigationCommandAdmission,
    NavigationCommand,
    NavigationModeStatus,
    RegisteredScan,
)
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class WorldObservationGate(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("world_observation_fault_gate")
        self.declare_parameter("input_topic", "/lio/mapping_observation")
        self.declare_parameter("output_topic", "/test/world_gate/mapping_observation")
        self.declare_parameter("command_topic", "/navigation/navigation_command")
        self.declare_parameter("ready_commands_before_fault", 100)
        self.declare_parameter("delay_after_trigger_ns", 1_000_000_000)
        self.declare_parameter("drop_duration_ns", 550_000_000)
        self.output_path = output_path
        self.stream = output_path.open("x", encoding="utf-8", buffering=1)
        self.output = self.create_publisher(
            RegisteredScan, str(self.get_parameter("output_topic").value), 10)
        self.subscription = self.create_subscription(
            RegisteredScan, str(self.get_parameter("input_topic").value),
            self._on_scan, 10)
        self.command_subscription = self.create_subscription(
            NavigationCommand, str(self.get_parameter("command_topic").value),
            self._on_command, 10)
        self.mode_status_subscription = self.create_subscription(
            NavigationModeStatus, "/navigation/mode_status", self._on_mode_status, 10)
        self.admission_subscription = self.create_subscription(
            NavigationCommandAdmission, "/navigation/command_admission",
            self._on_admission, 10)
        self.clock_subscription = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, "/lio/odometry_corrected", self._on_odom, 10)
        self.health_subscription = self.create_subscription(
            EstimatorHealth, "/lio/health", self._on_health, 10)
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray, "/navigation/diagnostics", self._on_diagnostics, 10)
        self.ready_commands = 0
        self.last_ready_sample_id = 0
        self.first_ready_stamp_ns: int | None = None
        self.fault_start_ns: int | None = None
        self.fault_end_ns: int | None = None
        self.first_drop_stamp_ns: int | None = None
        self.last_drop_stamp_ns: int | None = None
        self.first_forward_after_fault_ns: int | None = None
        self.input_count = 0
        self.forwarded_count = 0
        self.dropped_count = 0
        self.clock_count = 0
        self.odom_count = 0
        self.health_count = 0
        self.health_valid_count = 0
        self.diagnostic_count = 0
        self.core_diagnostic_count = 0
        self.adapter_diagnostic_count = 0
        self.mode_status_count = 0
        self.active_mode_status_count = 0
        self.command_admission_count = 0
        self.last_mode_state: int | None = None
        self.last_clock_ns: int | None = None
        self.clock_regressions = 0
        self._emit("GATE_READY", {
            "input_topic": str(self.get_parameter("input_topic").value),
            "output_topic": str(self.get_parameter("output_topic").value),
            "fault_duration_ns": int(self.get_parameter("drop_duration_ns").value),
            "delay_after_trigger_ns": int(
                self.get_parameter("delay_after_trigger_ns").value),
            "ready_command_threshold": int(
                self.get_parameter("ready_commands_before_fault").value),
            "odometry_topic": "/lio/odometry_corrected",
            "health_topic": "/lio/health",
            "core_diagnostics_topic": "/navigation/diagnostics",
            "adapter_liveness_topic": "/navigation/mode_status",
            "adapter_admission_topic": "/navigation/command_admission",
        })

    @staticmethod
    def _stamp_ns(stamp: Any) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _emit(self, event: str, payload: dict[str, Any]) -> None:
        record = {
            "event": event,
            "steady_ns": time.monotonic_ns(),
            "ros_now_ns": self.get_clock().now().nanoseconds,
            **payload,
        }
        self.stream.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    def _on_command(self, msg: NavigationCommand) -> None:
        if msg.status != NavigationCommand.STATUS_READY:
            return
        if int(msg.sample_id) <= self.last_ready_sample_id:
            return
        self.last_ready_sample_id = int(msg.sample_id)
        self.ready_commands += 1
        if self.first_ready_stamp_ns is None:
            self.first_ready_stamp_ns = self._stamp_ns(msg.header.stamp)
            self._emit("FIRST_READY_COMMAND", {
                "command_stamp_ns": self.first_ready_stamp_ns,
                "request_id": int(msg.request_id),
                "bundle_generation": int(msg.bundle_generation),
                "sample_id": int(msg.sample_id),
                "localization_epoch": int(msg.localization_epoch),
                "world_generation": int(msg.world_generation),
                "world_revision": int(msg.world_revision),
            })
        threshold = int(self.get_parameter("ready_commands_before_fault").value)
        if self.ready_commands == threshold:
            self.fault_start_ns = (
                self._stamp_ns(msg.header.stamp) +
                int(self.get_parameter("delay_after_trigger_ns").value))
            self.fault_end_ns = self.fault_start_ns + int(
                self.get_parameter("drop_duration_ns").value)
            self._emit("FAULT_SCHEDULED", {
                "ready_command_count": self.ready_commands,
                "trigger_sample_id": int(msg.sample_id),
                "trigger_command_stamp_ns": self._stamp_ns(msg.header.stamp),
                "fault_start_ros_ns": self.fault_start_ns,
                "fault_end_ros_ns": self.fault_end_ns,
            })

    def _on_mode_status(self, msg: NavigationModeStatus) -> None:
        self.mode_status_count += 1
        self.last_mode_state = int(msg.state)
        if msg.state == NavigationModeStatus.ACTIVE:
            self.active_mode_status_count += 1

    def _on_admission(self, _msg: NavigationCommandAdmission) -> None:
        self.command_admission_count += 1

    def _on_scan(self, msg: RegisteredScan) -> None:
        self.input_count += 1
        stamp_ns = self._stamp_ns(msg.header.stamp)
        dropping = (self.fault_start_ns is not None and self.fault_end_ns is not None
                    and self.fault_start_ns <= stamp_ns < self.fault_end_ns)
        identity = {
            "scan_sequence": int(msg.scan_sequence),
            "localization_epoch": int(msg.localization_epoch),
            "source_stamp_ns": stamp_ns,
            "input_index": self.input_count,
            "fault_start_ros_ns": self.fault_start_ns,
            "fault_end_ros_ns": self.fault_end_ns,
        }
        if dropping:
            self.dropped_count += 1
            self.first_drop_stamp_ns = self.first_drop_stamp_ns or stamp_ns
            self.last_drop_stamp_ns = stamp_ns
            self._emit("MAPPING_SCAN_DROPPED", {
                **identity, "forwarded_count": self.forwarded_count,
                "dropped_count": self.dropped_count,
                "clock_count": self.clock_count,
                "last_clock_ns": self.last_clock_ns,
                "odometry_count": self.odom_count,
                "health_count": self.health_count,
                "health_valid_count": self.health_valid_count,
                "diagnostic_count": self.diagnostic_count,
                "core_diagnostic_count": self.core_diagnostic_count,
                "adapter_diagnostic_count": self.adapter_diagnostic_count,
                "mode_status_count": self.mode_status_count,
                "active_mode_status_count": self.active_mode_status_count,
                "command_admission_count": self.command_admission_count,
                "last_mode_state": self.last_mode_state,
            })
            return
        self.output.publish(msg)
        self.forwarded_count += 1
        if self.fault_end_ns is not None and stamp_ns >= self.fault_end_ns:
            if self.first_forward_after_fault_ns is None:
                self.first_forward_after_fault_ns = stamp_ns
                self._emit("FAULT_ENDED_FORWARDING_RESTORED", {
                    **identity, "first_drop_stamp_ns": self.first_drop_stamp_ns,
                    "last_drop_stamp_ns": self.last_drop_stamp_ns,
                    "forwarded_count": self.forwarded_count,
                    "dropped_count": self.dropped_count,
                    "clock_count": self.clock_count,
                    "odometry_count": self.odom_count,
                    "health_count": self.health_count,
                    "health_valid_count": self.health_valid_count,
                    "diagnostic_count": self.diagnostic_count,
                    "core_diagnostic_count": self.core_diagnostic_count,
                    "adapter_diagnostic_count": self.adapter_diagnostic_count,
                    "mode_status_count": self.mode_status_count,
                    "active_mode_status_count": self.active_mode_status_count,
                    "command_admission_count": self.command_admission_count,
                    "last_mode_state": self.last_mode_state,
                })
        if self.input_count <= 3 or self.input_count % 100 == 0:
            self._emit("MAPPING_SCAN_FORWARDED", {
                **identity, "forwarded_count": self.forwarded_count,
                "dropped_count": self.dropped_count,
            })

    def _on_clock(self, msg: Clock) -> None:
        stamp_ns = self._stamp_ns(msg.clock)
        if self.last_clock_ns is not None and stamp_ns < self.last_clock_ns:
            self.clock_regressions += 1
        self.last_clock_ns = stamp_ns
        self.clock_count += 1

    def _on_odom(self, _msg: Odometry) -> None:
        self.odom_count += 1

    def _on_health(self, msg: EstimatorHealth) -> None:
        self.health_count += 1
        if (msg.state == EstimatorHealth.TRACKING and msg.navigation_valid and
                msg.correction_fresh and msg.propagation_valid):
            self.health_valid_count += 1

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        self.diagnostic_count += 1
        names = {str(status.name) for status in msg.status}
        if "navigation_runtime/planner" in names:
            self.core_diagnostic_count += 1
        if "navigation_external_mode/PX4_INPUT_SETPOINT" in names:
            self.adapter_diagnostic_count += 1

    def close(self) -> None:
        self._emit("GATE_FINAL", {
            "ready_commands": self.ready_commands,
            "last_ready_sample_id": self.last_ready_sample_id,
            "first_ready_stamp_ns": self.first_ready_stamp_ns,
            "fault_start_ros_ns": self.fault_start_ns,
            "fault_end_ros_ns": self.fault_end_ns,
            "first_drop_stamp_ns": self.first_drop_stamp_ns,
            "last_drop_stamp_ns": self.last_drop_stamp_ns,
            "first_forward_after_fault_ns": self.first_forward_after_fault_ns,
            "input_count": self.input_count,
            "forwarded_count": self.forwarded_count,
            "dropped_count": self.dropped_count,
            "clock_count": self.clock_count,
            "last_clock_ns": self.last_clock_ns,
            "clock_regressions": self.clock_regressions,
            "odometry_count": self.odom_count,
            "health_count": self.health_count,
            "health_valid_count": self.health_valid_count,
            "diagnostic_count": self.diagnostic_count,
            "core_diagnostic_count": self.core_diagnostic_count,
            "adapter_diagnostic_count": self.adapter_diagnostic_count,
            "mode_status_count": self.mode_status_count,
            "active_mode_status_count": self.active_mode_status_count,
            "command_admission_count": self.command_admission_count,
            "last_mode_state": self.last_mode_state,
            "steady_close_ns": time.monotonic_ns(),
        })
        self.stream.close()


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: world_observation_gate.py OUTPUT.jsonl [ROS args]", file=sys.stderr)
        return 2
    output = Path(sys.argv[1]).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv[2:])
    node = WorldObservationGate(output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

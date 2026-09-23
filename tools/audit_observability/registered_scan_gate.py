#!/usr/bin/env python3
"""Experiment-only scan input gate. Never touches odometry or flight authority."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from navigation_contracts.msg import NavigationCommand, RegisteredScan


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Gate(Node):
    def __init__(self, output: Path, fault_ms: int, ready_count: int):
        super().__init__("audit_registered_scan_gate")
        self.declare_parameter("use_sim_time", True)
        self.output = output.open("w", encoding="utf-8", buffering=1)
        self.fault_ns = fault_ms * 1_000_000
        self.ready_target = ready_count
        self.ready_seen = 0
        self.state = "FORWARD"
        self.fault_start_ros_ns = 0
        self.input_count = 0
        self.forwarded_count = 0
        self.dropped_count = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            RegisteredScan, "/navigation/audit_registered_scan", qos)
        self.scan_subscription = self.create_subscription(
            RegisteredScan, "/lio/mapping_observation", self.on_scan, qos)
        self.command_subscription = self.create_subscription(
            NavigationCommand, "/navigation/navigation_command", self.on_command, qos)
        self.timer = self.create_timer(0.02, self.maybe_end_fault)
        self.log("GATE_READY")

    def log(self, kind, message=None):
        record = dict(kind=kind, host_steady_ns=time.monotonic_ns(),
                      ros_now_ns=self.get_clock().now().nanoseconds,
                      input_count=self.input_count,
                      forwarded_count=self.forwarded_count,
                      dropped_count=self.dropped_count,
                      ready_seen=self.ready_seen)
        if message is not None:
            record.update(input_sequence=int(message.scan_sequence),
                          input_localization_epoch=int(message.localization_epoch),
                          input_source_stamp_ns=stamp_ns(message.header.stamp))
        self.output.write(json.dumps(record, sort_keys=True) + "\n")

    def on_command(self, message):
        if (message.status == NavigationCommand.STATUS_READY and
                message.execution_authorization == NavigationCommand.EXECUTION_AUTHORIZATION_GRANTED):
            self.ready_seen += 1

    def maybe_end_fault(self):
        if (self.state == "DROPPING" and
                self.get_clock().now().nanoseconds - self.fault_start_ros_ns >= self.fault_ns):
            self.state = "DONE"
            self.log("FAULT_END")

    def on_scan(self, message):
        self.input_count += 1
        self.maybe_end_fault()
        if self.state == "FORWARD" and self.ready_seen >= self.ready_target:
            self.state = "DROPPING"
            self.fault_start_ros_ns = self.get_clock().now().nanoseconds
            self.log("FAULT_START", message)
        if self.state == "DROPPING":
            self.dropped_count += 1
            return
        self.publisher.publish(message)
        self.forwarded_count += 1

    def close(self):
        self.log("GATE_FINAL")
        self.output.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fault-ms", type=int, required=True)
    parser.add_argument("--ready-count", type=int, default=20)
    args = parser.parse_args()
    if not 1 <= args.fault_ms <= 20_000 or args.ready_count < 1:
        parser.error("fault-ms must be 1..20000 and ready-count positive")
    rclpy.init()
    gate = Gate(args.output, args.fault_ms, args.ready_count)
    try:
        rclpy.spin(gate)
    finally:
        gate.close()
        gate.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

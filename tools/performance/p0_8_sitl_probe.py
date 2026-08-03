#!/usr/bin/env python3
"""Measured-clock probe for the P0.8 healthy SITL qualification.

The PX4 session launcher owns process lifecycle. This probe only samples the
ROS contracts and the session process groups, then exits after the requested
simulation-clock window.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics
import time
from typing import Any

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import OdometrySupervisorStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock


def ns(clock: Any) -> int:
    return int(clock.sec) * 1_000_000_000 + int(clock.nanosec)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    return values[min(len(values) - 1, max(0, int(q * len(values))))]


def diagnostic_values(message: DiagnosticArray, name: str) -> dict[str, str]:
    for status in message.status:
        if status.name == name:
            return {item.key: item.value for item in status.values}
    return {}


class Probe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(
            "p0_8_sitl_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.args = args
        self.mode = args.mode
        self.clock_ns = 0
        self.first_clock_ns = None
        self.measure_start_ns = None
        self.measure_end_ns = None
        self.latest_status: OdometrySupervisorStatus | None = None
        self.latest_lio: dict[str, str] = {}
        self.status_samples: list[dict[str, Any]] = []
        self.rows: list[dict[str, Any]] = []
        self.wall_started = time.monotonic()
        self.create_subscription(Clock, "/clock", self.on_clock, 50)
        self.create_subscription(OdometrySupervisorStatus,
                                 "/navigation/odometry_supervisor/status",
                                 self.on_status, 20)
        self.create_subscription(DiagnosticArray, "/lio/diagnostics", self.on_lio, 20)
        self.create_subscription(Odometry, "/lio/odometry_corrected", self.on_odometry, 20)
        self.timer = self.create_timer(1.0, self.sample)

    def on_clock(self, message: Clock) -> None:
        self.clock_ns = ns(message.clock)
        if self.first_clock_ns is None and self.clock_ns:
            self.first_clock_ns = self.clock_ns

    def on_status(self, message: OdometrySupervisorStatus) -> None:
        self.latest_status = message

    def on_lio(self, message: DiagnosticArray) -> None:
        self.latest_lio = diagnostic_values(message, "fast_lio/estimator")

    def on_odometry(self, _message: Odometry) -> None:
        pass

    def process_metrics(self) -> dict[str, float]:
        result: dict[str, float] = {}
        for role in ("supervisor", "bridge", "fast_lio"):
            pid_file = self.args.session / "pids" / f"{role}.pid"
            if not pid_file.is_file():
                result[f"{role}_cpu_percent"] = 0.0
                result[f"{role}_rss_bytes"] = 0
                result[f"{role}_threads"] = 0
                continue
            try:
                process = psutil.Process(int(pid_file.read_text().strip()))
                result[f"{role}_cpu_percent"] = float(process.cpu_percent(None))
                result[f"{role}_rss_bytes"] = int(process.memory_info().rss)
                result[f"{role}_threads"] = int(process.num_threads())
            except (OSError, ValueError, psutil.Error):
                result[f"{role}_cpu_percent"] = 0.0
                result[f"{role}_rss_bytes"] = 0
                result[f"{role}_threads"] = 0
        return result

    def sample(self) -> None:
        if self.first_clock_ns is None:
            if time.monotonic() - self.wall_started > self.args.wall_timeout_s:
                self.get_logger().error("simulation clock did not start")
                rclpy.shutdown()
            return
        elapsed_ns = self.clock_ns - self.first_clock_ns
        if self.measure_start_ns is None and elapsed_ns >= self.args.warmup_s * 1e9:
            self.measure_start_ns = self.clock_ns
            self.measure_end_ns = self.clock_ns + int(self.args.measure_s * 1e9)
        if self.measure_start_ns is None:
            return
        status = self.latest_status
        row: dict[str, Any] = {
            "sim_time_s": (self.clock_ns - self.measure_start_ns) / 1e9,
            "comparison_valid": int(bool(status and status.comparison_valid)),
            "monitoring_available": int(bool(status and status.monitoring_available)),
            "health": int(status.health) if status else -1,
            "query_count": int(status.query_sequence) if status else 0,
            "query_success_count": int(status.query_success_count) if status else 0,
            "query_failure_count": int(status.query_failure_count) if status else 0,
            "query_timeout_count": int(status.query_timeout_count) if status else 0,
            "query_generation_mismatch_count": int(status.query_generation_mismatch_count) if status else 0,
            "query_rtt_p50_ms": float(status.query_rtt_p50_ms) if status else 0.0,
            "query_rtt_p95_ms": float(status.query_rtt_p95_ms) if status else 0.0,
            "query_rtt_p99_ms": float(status.query_rtt_p99_ms) if status else 0.0,
            "query_rtt_max_ms": float(status.query_rtt_max_ms) if status else 0.0,
            "alignment_gap_ms": float(status.alignment_gap_ns) / 1e6 if status and status.alignment_gap_ns >= 0 else 0.0,
            "comparison_age_ms": float(status.aligned_comparison_age_ns) / 1e6 if status and status.aligned_comparison_age_ns >= 0 else 0.0,
            "state_transition_count": int(status.state_transition_count) if status else 0,
            "reinitialization_requests": int(status.reinitialization_request_sequence) if status else 0,
            "fast_lio_corrected_p95_us": float(self.latest_lio.get("p95_corrected_scan_end_to_end_us", 0.0)),
            "fast_lio_max_queue_depth": int(float(self.latest_lio.get("maximum_queue_depth", 0))),
        }
        row.update(self.process_metrics())
        self.rows.append(row)
        if self.clock_ns >= (self.measure_end_ns or 0):
            self.finish()

    def finish(self) -> None:
        if self.mode == "off":
            comparison_ratio = monitoring_ratio = 1.0
            timeouts = generations = 0
        else:
            comparison_ratio = statistics.mean(row["comparison_valid"] for row in self.rows) if self.rows else 0.0
            monitoring_ratio = statistics.mean(row["monitoring_available"] for row in self.rows) if self.rows else 0.0
            timeouts = max(row["query_timeout_count"] for row in self.rows) if self.rows else 0
            generations = max(row["query_generation_mismatch_count"] for row in self.rows) if self.rows else 0
        def values(key: str) -> list[float]:
            return [float(row[key]) for row in self.rows]
        result = {
            "schema_version": 1,
            "mode": self.mode,
            "warmup_s": self.args.warmup_s,
            "measurement_s": self.args.measure_s,
            "sample_count": len(self.rows),
            "comparison_valid_ratio": comparison_ratio,
            "monitoring_available_ratio": monitoring_ratio,
            "query_count": max((row["query_count"] for row in self.rows), default=0),
            "query_success_count": max((row["query_success_count"] for row in self.rows), default=0),
            "query_timeout_count": timeouts,
            "query_failure_count": max((row["query_failure_count"] for row in self.rows), default=0),
            "query_generation_mismatch_count": generations,
            "query_rtt_p50_ms": max((row["query_rtt_p50_ms"] for row in self.rows), default=0.0),
            "query_rtt_p95_ms": max((row["query_rtt_p95_ms"] for row in self.rows), default=0.0),
            "query_rtt_p99_ms": max((row["query_rtt_p99_ms"] for row in self.rows), default=0.0),
            "query_rtt_max_ms": max((row["query_rtt_max_ms"] for row in self.rows), default=0.0),
            "alignment_gap_p99_ms": percentile(values("alignment_gap_ms"), 0.99),
            "aligned_comparison_age_p99_ms": percentile(values("comparison_age_ms"), 0.99),
            "state_transition_count": max((row["state_transition_count"] for row in self.rows), default=0),
            "reinitialization_requests": max((row["reinitialization_requests"] for row in self.rows), default=0),
            "supervisor_cpu_mean_percent": statistics.mean(values("supervisor_cpu_percent")) if self.rows else 0.0,
            "supervisor_cpu_p95_percent": percentile(values("supervisor_cpu_percent"), 0.95),
            "supervisor_cpu_max_percent": max(values("supervisor_cpu_percent"), default=0.0),
            "supervisor_peak_rss_bytes": max((row["supervisor_rss_bytes"] for row in self.rows), default=0),
            "fast_lio_corrected_p95_us": max((row["fast_lio_corrected_p95_us"] for row in self.rows), default=0.0),
            "fast_lio_max_queue_depth": max((row["fast_lio_max_queue_depth"] for row in self.rows), default=0),
            "rows": self.rows,
        }
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=("off", "on"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmup-s", type=float, default=30.0)
    parser.add_argument("--measure-s", type=float, default=120.0)
    parser.add_argument("--wall-timeout-s", type=float, default=900.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

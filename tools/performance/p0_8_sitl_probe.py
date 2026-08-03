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

from p0_8_probe_metrics import StatusEventAccumulator


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


class ProcessMetricSampler:
    """Cache process handles and fail closed for missing/dead roles."""

    def __init__(self, session: Path, roles: tuple[str, ...]) -> None:
        self.session = session
        self.roles = roles
        self.processes: dict[str, psutil.Process] = {}
        self.identities: dict[str, tuple[int, float]] = {}
        self.cpu_primed: set[str] = set()

    def sample(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for role in self.roles:
            pid_file = self.session / "pids" / f"{role}.pid"
            state = "missing"
            values: dict[str, Any] = {
                "cpu_percent": None, "rss_bytes": None, "threads": None,
            }
            if pid_file.is_file():
                try:
                    pid = int(pid_file.read_text().strip())
                    candidate = psutil.Process(pid)
                    identity = (pid, float(candidate.create_time()))
                    if self.identities.get(role) != identity:
                        self.processes[role] = candidate
                        self.identities[role] = identity
                        self.cpu_primed.discard(role)
                    process = self.processes[role]
                    if not process.is_running() or process.status() == psutil.STATUS_ZOMBIE:
                        state = "dead"
                    else:
                        if role not in self.cpu_primed:
                            process.cpu_percent(None)
                            self.cpu_primed.add(role)
                            state = "primed"
                        else:
                            values["cpu_percent"] = float(process.cpu_percent(None))
                            state = "measured"
                        memory = process.memory_info()
                        values["rss_bytes"] = int(memory.rss)
                        values["threads"] = int(process.num_threads())
                except (OSError, ValueError, psutil.Error):
                    state = "dead"
            result[f"{role}_state"] = state
            for metric, value in values.items():
                result[f"{role}_{metric}"] = value
        return result


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
        self.status_events = StatusEventAccumulator()
        self.latest_lio: dict[str, str] = {}
        self.status_samples: list[dict[str, Any]] = []
        self.rows: list[dict[str, Any]] = []
        self.wall_started = time.monotonic()
        roles = ("bridge", "fast_lio", "supervisor") if self.mode == "on" else ("bridge", "fast_lio")
        self.process_sampler = ProcessMetricSampler(args.session, roles)
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
        event_ns = ns(message.header.stamp) or self.clock_ns
        self.status_events.record(message, event_ns)

    def on_lio(self, message: DiagnosticArray) -> None:
        self.latest_lio = diagnostic_values(message, "fast_lio/estimator")

    def on_odometry(self, _message: Odometry) -> None:
        pass

    def process_metrics(self) -> dict[str, Any]:
        result = self.process_sampler.sample()
        if self.mode == "off":
            result.update({
                "supervisor_state": "not_applicable",
                "supervisor_cpu_percent": None,
                "supervisor_rss_bytes": None,
                "supervisor_threads": None,
            })
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
            self.status_events.start(self.measure_start_ns, self.measure_end_ns, self.latest_status)
        if self.measure_start_ns is None:
            return
        status = self.latest_status if self.mode == "on" else None
        row: dict[str, Any] = {
            "sim_time_s": (self.clock_ns - self.measure_start_ns) / 1e9,
            "comparison_valid": int(status.comparison_valid) if status else None,
            "monitoring_available": int(status.monitoring_available) if status else None,
            "health": int(status.health) if status else None,
            "query_count": int(status.query_sequence) if status else None,
            "query_success_count": int(status.query_success_count) if status else None,
            "query_failure_count": int(status.query_failure_count) if status else None,
            "query_timeout_count": int(status.query_timeout_count) if status else None,
            "query_generation_mismatch_count": int(status.query_generation_mismatch_count) if status else None,
            "query_rtt_p50_ms": float(status.query_rtt_p50_ms) if status else None,
            "query_rtt_p95_ms": float(status.query_rtt_p95_ms) if status else None,
            "query_rtt_p99_ms": float(status.query_rtt_p99_ms) if status else None,
            "query_rtt_max_ms": float(status.query_rtt_max_ms) if status else None,
            "alignment_gap_ms": float(status.alignment_gap_ns) / 1e6 if status and status.alignment_gap_ns >= 0 else None,
            "comparison_age_ms": float(status.aligned_comparison_age_ns) / 1e6 if status and status.aligned_comparison_age_ns >= 0 else None,
            "state_transition_count": int(status.state_transition_count) if status else None,
            "reinitialization_requests": int(status.reinitialization_request_sequence) if status else None,
            "fast_lio_corrected_p95_us": float(self.latest_lio.get("p95_corrected_scan_end_to_end_us", 0.0)),
            "fast_lio_max_queue_depth": int(float(self.latest_lio.get("maximum_queue_depth", 0))),
        }
        row.update(self.process_metrics())
        self.rows.append(row)
        if self.clock_ns >= (self.measure_end_ns or 0):
            self.finish()

    def finish(self) -> None:
        def values(key: str) -> list[float]:
            return [float(row[key]) for row in self.rows if row.get(key) is not None]

        def maximum(key: str) -> float | None:
            items = values(key)
            return max(items) if items else None

        measurement_end_ns = min(self.clock_ns, self.measure_end_ns or self.clock_ns)
        status_metrics = self.status_events.finish(measurement_end_ns)
        comparison_ratio = status_metrics["comparison_valid_ratio"]
        monitoring_ratio = status_metrics["monitoring_available_ratio"]
        timeouts = status_metrics["query_timeout_count_delta"]
        generations = status_metrics["query_generation_mismatch_count_delta"]
        supervisor_cpu = values("supervisor_cpu_percent")
        supervisor_rss = values("supervisor_rss_bytes")
        result = {
            "schema_version": 1,
            "mode": self.mode,
            "warmup_s": self.args.warmup_s,
            "measurement_s": self.args.measure_s,
            "sample_count": len(self.rows),
            **status_metrics,
            "supervisor_metrics_applicable": self.mode == "on",
            "comparison_valid_ratio": comparison_ratio,
            "monitoring_available_ratio": monitoring_ratio,
            "query_count": status_metrics["query_sequence_delta"],
            "query_success_count": status_metrics["query_success_count_delta"],
            "query_timeout_count": timeouts,
            "query_failure_count": status_metrics["query_failure_count_delta"],
            "query_generation_mismatch_count": generations,
            "query_rtt_p50_ms": maximum("query_rtt_p50_ms"),
            "query_rtt_p95_ms": maximum("query_rtt_p95_ms"),
            "query_rtt_p99_ms": maximum("query_rtt_p99_ms"),
            "query_rtt_max_ms": maximum("query_rtt_max_ms"),
            "alignment_gap_p99_ms": percentile(values("alignment_gap_ms"), 0.99) if values("alignment_gap_ms") else None,
            "aligned_comparison_age_p99_ms": percentile(values("comparison_age_ms"), 0.99) if values("comparison_age_ms") else None,
            "state_transition_count": status_metrics["state_transition_count_delta"],
            "reinitialization_requests": status_metrics["reinitialization_request_sequence_delta"],
            "supervisor_cpu_mean_percent": statistics.mean(supervisor_cpu) if supervisor_cpu else None,
            "supervisor_cpu_p95_percent": percentile(supervisor_cpu, 0.95) if supervisor_cpu else None,
            "supervisor_cpu_max_percent": max(supervisor_cpu) if supervisor_cpu else None,
            "supervisor_peak_rss_bytes": max(supervisor_rss) if supervisor_rss else None,
            "fast_lio_corrected_p95_us": maximum("fast_lio_corrected_p95_us"),
            "fast_lio_max_queue_depth": maximum("fast_lio_max_queue_depth"),
            "rows": self.rows,
        }
        for role in ("bridge", "fast_lio"):
            cpu = values(f"{role}_cpu_percent")
            rss = values(f"{role}_rss_bytes")
            result[f"{role}_cpu_mean_percent"] = statistics.mean(cpu) if cpu else None
            result[f"{role}_cpu_p95_percent"] = percentile(cpu, 0.95) if cpu else None
            result[f"{role}_cpu_max_percent"] = max(cpu) if cpu else None
            result[f"{role}_peak_rss_bytes"] = max(rss) if rss else None
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

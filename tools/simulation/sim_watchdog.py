#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


@dataclass
class Stream:
    name: str
    expected_hz: float | None
    stale_s: float
    count: int = 0
    first_wall: float | None = None
    last_wall: float | None = None
    last_stamp: float | None = None
    gaps: deque[float] = field(default_factory=lambda: deque(maxlen=5000))
    stamp_regressions: int = 0
    nonfinite_messages: int = 0

    def update(self, wall: float, stamp: float | None, finite: bool = True) -> None:
        self.count += 1
        if self.first_wall is None:
            self.first_wall = wall
        if self.last_wall is not None:
            self.gaps.append(wall - self.last_wall)
        self.last_wall = wall

        if stamp is not None:
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.stamp_regressions += 1
            self.last_stamp = stamp

        if not finite:
            self.nonfinite_messages += 1

    def rate_hz(self) -> float:
        if self.count < 2 or self.first_wall is None or self.last_wall is None:
            return 0.0
        dt = self.last_wall - self.first_wall
        return (self.count - 1) / dt if dt > 0 else 0.0

    def age_s(self, now: float) -> float | None:
        return None if self.last_wall is None else now - self.last_wall

    def summary(self, now: float) -> dict[str, Any]:
        gaps = list(self.gaps)
        return {
            "count": self.count,
            "rate_hz": self.rate_hz(),
            "age_s": self.age_s(now),
            "gap_mean_s": statistics.fmean(gaps) if gaps else None,
            "gap_max_s": max(gaps) if gaps else None,
            "gap_p95_s": percentile(gaps, 95.0) if gaps else None,
            "stamp_regressions": self.stamp_regressions,
            "nonfinite_messages": self.nonfinite_messages,
        }


def percentile(values: list[float], p: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * p / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def stamp_seconds(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def all_finite(values: list[float]) -> bool:
    return all(math.isfinite(v) for v in values)


class Watchdog(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("px4_mid360_sim_watchdog")
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.monotonic()
        self.started_iso = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.stop_requested = False
        self.last_clock: float | None = None
        self.clock_regressions = 0
        self.diagnostic_errors = 0
        self.diagnostic_warnings = 0
        self.last_diagnostics: dict[str, Any] = {}
        self.events_seen: set[str] = set()

        self.streams = {
            "imu": Stream("imu", expected_hz=200.0, stale_s=0.10),
            "lidar": Stream("lidar", expected_hz=10.0, stale_s=0.50),
            "odom": Stream("odom", expected_hz=10.0, stale_s=0.75),
            "registered": Stream("registered", expected_hz=10.0, stale_s=0.75),
            "local_map": Stream("local_map", expected_hz=None, stale_s=30.0),
            "diagnostics": Stream("diagnostics", expected_hz=None, stale_s=5.0),
        }

        self.metrics_file = (self.output / "metrics.csv").open("w", newline="", encoding="utf-8")
        self.metrics = csv.writer(self.metrics_file)
        self.metrics.writerow([
            "wall_iso", "elapsed_s",
            "imu_count", "imu_rate_hz", "imu_age_s", "imu_gap_max_s",
            "lidar_count", "lidar_rate_hz", "lidar_age_s", "lidar_gap_max_s",
            "odom_count", "odom_rate_hz", "odom_age_s", "odom_gap_max_s",
            "registered_count", "registered_rate_hz", "registered_age_s",
            "local_map_count", "local_map_age_s",
            "diagnostic_warnings", "diagnostic_errors",
            "clock_regressions",
        ])
        self.events_file = (self.output / "events.log").open("a", encoding="utf-8")

        qos = 10
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(Imu, "/lidar/imu", self.on_imu, qos)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_lidar, qos)
        self.create_subscription(Odometry, "/lio/odometry_corrected", self.on_odom, qos)
        self.create_subscription(PointCloud2, "/lio/registered_points", self.on_registered, qos)
        self.create_subscription(PointCloud2, "/lio/local_map", self.on_local_map, qos)
        self.create_subscription(DiagnosticArray, "/lio/diagnostics", self.on_diagnostics, qos)

        self.create_timer(1.0, self.sample)
        self.create_timer(2.0, self.evaluate)

    def event(self, severity: str, key: str, text: str) -> None:
        unique = f"{severity}:{key}"
        if unique in self.events_seen:
            return
        self.events_seen.add(unique)
        line = f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} [{severity}] {text}"
        print(line, flush=True)
        self.events_file.write(line + "\n")
        self.events_file.flush()

    def on_clock(self, msg: Clock) -> None:
        current = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.last_clock is not None and current < self.last_clock:
            self.clock_regressions += 1
            self.event("ERROR", "clock_regression", "Simulation clock regressed.")
        self.last_clock = current

    def on_imu(self, msg: Imu) -> None:
        finite = all_finite([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        ])
        self.streams["imu"].update(time.monotonic(), stamp_seconds(msg), finite)

    def on_lidar(self, msg: PointCloud2) -> None:
        finite = msg.width > 0 and msg.point_step > 0 and msg.row_step > 0
        self.streams["lidar"].update(time.monotonic(), stamp_seconds(msg), finite)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        finite = all_finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.streams["odom"].update(time.monotonic(), stamp_seconds(msg), finite)

    def on_registered(self, msg: PointCloud2) -> None:
        finite = msg.width > 0 and msg.point_step > 0
        self.streams["registered"].update(time.monotonic(), stamp_seconds(msg), finite)

    def on_local_map(self, msg: PointCloud2) -> None:
        finite = msg.width > 0 and msg.point_step > 0
        self.streams["local_map"].update(time.monotonic(), stamp_seconds(msg), finite)

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        self.streams["diagnostics"].update(time.monotonic(), stamp_seconds(msg), True)
        snapshot: dict[str, Any] = {}
        for status in msg.status:
            raw_level = status.level

            if isinstance(raw_level, (bytes, bytearray)):
                level = raw_level[0] if raw_level else 0
            else:
                level = int(raw_level)

            if level >= 2:
                self.diagnostic_errors += 1
            elif level == 1:
                self.diagnostic_warnings += 1

            snapshot[status.name] = {
                "level": level,
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": {value.key: value.value for value in status.values},
            }
        self.last_diagnostics = snapshot
        (self.output / "diagnostics_latest.json").write_text(
            json.dumps(snapshot, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    def sample(self) -> None:
        now = time.monotonic()
        s = {name: stream.summary(now) for name, stream in self.streams.items()}
        self.metrics.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            now - self.started_wall,
            s["imu"]["count"], s["imu"]["rate_hz"], s["imu"]["age_s"], s["imu"]["gap_max_s"],
            s["lidar"]["count"], s["lidar"]["rate_hz"], s["lidar"]["age_s"], s["lidar"]["gap_max_s"],
            s["odom"]["count"], s["odom"]["rate_hz"], s["odom"]["age_s"], s["odom"]["gap_max_s"],
            s["registered"]["count"], s["registered"]["rate_hz"], s["registered"]["age_s"],
            s["local_map"]["count"], s["local_map"]["age_s"],
            self.diagnostic_warnings, self.diagnostic_errors,
            self.clock_regressions,
        ])
        self.metrics_file.flush()
        self.write_summary(final=False)

    def evaluate(self) -> None:
        now = time.monotonic()
        elapsed = now - self.started_wall
        if elapsed < 8.0:
            return

        for name, stream in self.streams.items():
            age = stream.age_s(now)
            if stream.count == 0:
                self.event("ERROR", f"{name}_missing", f"No messages received on {name}.")
                continue
            if age is not None and age > stream.stale_s:
                self.event("ERROR", f"{name}_stale", f"{name} stale for {age:.3f} s.")

            rate = stream.rate_hz()
            if stream.expected_hz is not None and stream.count >= 10:
                lower = stream.expected_hz * 0.70
                upper = stream.expected_hz * 1.30
                if rate < lower:
                    self.event(
                        "WARN", f"{name}_rate_low",
                        f"{name} rate {rate:.2f} Hz below {lower:.2f} Hz."
                    )
                if rate > upper:
                    self.event(
                        "WARN", f"{name}_rate_high",
                        f"{name} rate {rate:.2f} Hz above {upper:.2f} Hz."
                    )

            if stream.stamp_regressions:
                self.event(
                    "ERROR", f"{name}_stamp_regression",
                    f"{name} contains timestamp regressions."
                )
            if stream.nonfinite_messages:
                self.event(
                    "ERROR", f"{name}_invalid",
                    f"{name} contains invalid/non-finite messages."
                )

    def write_summary(self, final: bool) -> None:
        now = time.monotonic()
        summary = {
            "started_at": self.started_iso,
            "final": final,
            "elapsed_s": now - self.started_wall,
            "clock_regressions": self.clock_regressions,
            "diagnostic_warnings": self.diagnostic_warnings,
            "diagnostic_errors": self.diagnostic_errors,
            "streams": {
                name: stream.summary(now)
                for name, stream in self.streams.items()
            },
            "last_diagnostics": self.last_diagnostics,
        }
        (self.output / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    def close(self) -> None:
        self.write_summary(final=True)
        self.metrics_file.close()
        self.events_file.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Watchdog(args.output)

    def stop_handler(*_: Any) -> None:
        node.stop_requested = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

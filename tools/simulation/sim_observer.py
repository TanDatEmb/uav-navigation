#!/usr/bin/env python3
"""PX4 MID-360 observability node: callbacks are bounded; analysis is sampled."""
from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
import traceback
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import yaml
import rclpy
from rclpy.serialization import deserialize_message
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from gazebo_probe import GazeboProbe
from observer_core import EventLifecycle, ObserverState, StreamTracker, classify
from pointcloud_probe import MissingXyzFields, PointCloudProbeError, inspect_pointcloud
from process_probe import cpu_percent, host_metrics, process_metrics
from ros_graph_probe import sample_graph
from snapshot_collector import collect_snapshot


def iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def diagnostic_level(raw: Any) -> int:
    return raw[0] if isinstance(raw, (bytes, bytearray)) and raw else (
        0 if isinstance(raw, (bytes, bytearray)) else int(raw))


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    temporary.replace(path)


class CsvSink:
    def __init__(self, path: Path, fields: list[str]):
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=fields, extrasaction="ignore")
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class SimObserver(Node):
    def __init__(self, session: Path, config: dict[str, Any]):
        super().__init__("px4_mid360_sim_observer")
        self.session, self.config = session, config
        self.started = time.monotonic()
        self.stop_requested = False
        self.latest = session/"latest"
        self.metrics = session/"metrics"
        self.latest.mkdir(parents=True, exist_ok=True)
        self.metrics.mkdir(parents=True, exist_ok=True)
        (session/"snapshots").mkdir(exist_ok=True)
        self.event_file = (self.metrics/"events.jsonl").open("a", encoding="utf-8")
        self.state = ObserverState.STARTING
        self.previous_classification: dict[str, Any] = {}
        self.last_event: dict[str, Any] | None = None
        self.last_clock_ns: int | None = None
        self.clock_regressions = 0
        self.pointcloud_samples = 0
        self.pointcloud_scan_count = 0
        self.imu_message_count = 0
        self.clock_message_count = 0
        self.pointcloud_state: dict[str, Any] = {}
        self.diagnostics: dict[str, Any] = {}
        self.raw_diagnostics: dict[str, Any] = {}
        self.host_previous = host_metrics()
        self.process_previous: dict[int, dict[str, Any]] = {}
        self.events = EventLifecycle(float(config["snapshot"]["cooldown_s"]),
                                     float(config["observer"]["event_persist_s"]))
        self.gazebo_probe = GazeboProbe(config["gazebo"]["topics"])
        self.gazebo_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gazebo_probe")
        self.snapshot_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="snapshot")
        self.gazebo_future = None
        self.diagnostic_values: dict[str, str] = {}
        topic_types = {
            "clock": "rosgraph_msgs/msg/Clock", "imu": "sensor_msgs/msg/Imu",
            "lidar": "sensor_msgs/msg/PointCloud2", "odometry": "nav_msgs/msg/Odometry",
            "registered_points": "sensor_msgs/msg/PointCloud2",
            "local_map": "sensor_msgs/msg/PointCloud2",
            "diagnostics": "diagnostic_msgs/msg/DiagnosticArray", "tf": "tf2_msgs/msg/TFMessage",
        }
        self.streams = {
            name: StreamTracker(item["topic"], topic_types[name],
                                bool(item.get("check_header_regression", True)))
            for name, item in config["streams"].items()
        }
        self.stream_csv = CsvSink(self.metrics/"streams.csv", [
            "wall_time", "monotonic_time_s", "name", "message_count",
            "receive_rate_hz_short_window", "receive_rate_hz_long_window", "age_wall",
            "wall_gap_last", "wall_gap_max", "wall_gap_p95", "header_stamp_last",
            "header_stamp_gap_last", "header_stamp_gap_max",
            "header_stamp_regression_count", "zero_stamp_count",
            "publisher_count", "subscriber_count", "message_type"])
        cloud_fields = ["wall_time", "monotonic_time_s"] + list(
            inspect_pointcloud.__annotations__.keys())  # replaced below with extras ignored
        self.cloud_csv = CsvSink(self.metrics/"pointcloud.csv", [
            "wall_time", "monotonic_time_s", "width", "height", "point_step", "row_step",
            "total_points", "sampled_points", "is_dense", "fields", "frame_id", "stamp_ns",
            "finite_xyz_count", "nan_xyz_count", "positive_inf_xyz_count",
            "negative_inf_xyz_count", "finite_ratio", "zero_xyz_count",
            "range_below_min_count", "range_above_max_count", "minimum_finite_range",
            "maximum_finite_range", "mean_finite_range", "density_contract_violation"])
        self.process_csv = CsvSink(self.metrics/"process.csv", [
            "wall_time", "monotonic_time_s", "role", "pid", "ppid", "pgid", "alive",
            "cpu_percent", "rss_kib", "vsz_bytes", "thread_count", "read_bytes",
            "write_bytes", "voluntary_context_switches", "involuntary_context_switches",
            "major_faults", "minor_faults", "command"])
        self.gz_csv = CsvSink(self.metrics/"gazebo.csv", [
            "wall_time", "monotonic_time_s", "raw_scan_sequence", "pointcloud_sequence",
            "imu_sequence", "clock_sequence", "raw_scan_finite_ratio", "raw_scan_inf_count",
            "real_time_factor", "sim_paused"])
        self.sync_csv = CsvSink(self.metrics/"synchronization.csv", [
            "wall_time", "monotonic_time_s", "header_to_clock_offset_ns",
            "imu_to_lidar_timestamp_offset_ns", "accepted_imu_count", "accepted_lidar_count",
            "rejected_imu_count", "rejected_lidar_count", "synchronized_group_count",
            "correction_success_count", "processing_lag_ns", "worker_heartbeat"])
        self.create_subscription(Clock, "/clock", self._safe("clock", self.on_clock_raw), 20, raw=True)
        self.create_subscription(Imu, "/lidar/imu", self._safe("imu", self.on_imu_raw), 200, raw=True)
        self.create_subscription(PointCloud2, "/lidar/points", self._safe("lidar", self.on_lidar_raw), 20, raw=True)
        self.create_subscription(Odometry, "/lio/odometry_corrected", self._safe("odometry", self.on_odom), 20)
        self.create_subscription(PointCloud2, "/lio/registered_points",
                                 self._safe("registered_points", lambda _: self.observe_raw("registered_points")), 10, raw=True)
        self.create_subscription(PointCloud2, "/lio/local_map",
                                 self._safe("local_map", lambda _: self.observe_raw("local_map")), 2, raw=True)
        self.create_subscription(DiagnosticArray, "/lio/diagnostics", self._safe("diagnostics", self.on_diagnostics), 20)
        self.create_subscription(TFMessage, "/tf", self._safe("tf", lambda _: self.observe_raw("tf")), 20, raw=True)
        self.create_timer(1.0/max(float(config["observer"]["sample_hz"]), .1), self.sample)
        self.create_timer(float(config["gazebo"]["sample_period_s"]), self.sample_gazebo)

    def _safe(self, probe: str, callback):
        def wrapped(message):
            try:
                callback(message)
            except Exception as error:
                self.emit("ERROR", "OBSERVER_PROBE_EXCEPTION", "observer",
                          f"{probe}: {type(error).__name__}: {error}",
                          {"probe": probe, "traceback": traceback.format_exc()})
        return wrapped

    def observe(self, name: str, message: Any, stamp: int | None = None):
        self.streams[name].update(time.monotonic(), stamp_ns(message) if stamp is None else stamp)

    def on_clock_raw(self, serialized: bytes):
        self.observe_raw("clock")
        self.clock_message_count += 1
        if self.clock_message_count % 10:
            return
        message = deserialize_message(serialized, Clock)
        value = int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
        tracker = self.streams["clock"]
        if value and self.last_clock_ns is not None and value < self.last_clock_ns:
            self.clock_regressions += 1
            self.emit("ERROR", "CLOCK_REGRESSION", "gazebo", "simulation clock regressed",
                      {"previous_ns": self.last_clock_ns, "current_ns": value})
        if value:
            self.last_clock_ns = value
            tracker.last_stamp_ns = value

    def on_imu_raw(self, serialized: bytes):
        self.observe_raw("imu")
        self.imu_message_count += 1
        if self.imu_message_count % 20:
            return
        message = deserialize_message(serialized, Imu)
        self.update_sampled_stamp("imu", stamp_ns(message))

    def update_sampled_stamp(self, name: str, sampled: int | None):
        if not sampled:
            return
        tracker = self.streams[name]
        if tracker.last_stamp_ns is not None:
            gap = sampled-tracker.last_stamp_ns
            tracker.stamp_gap_last_ns = gap
            if gap < 0:
                tracker.stamp_regressions += 1
            else:
                tracker.stamp_gap_max_ns = max(tracker.stamp_gap_max_ns or 0, gap)
        tracker.last_stamp_ns = sampled

    def observe_raw(self, name: str):
        self.streams[name].update(time.monotonic(), None)

    def on_lidar_raw(self, serialized: bytes):
        self.observe_raw("lidar")
        self.pointcloud_scan_count += 1
        every = int(self.config["pointcloud"]["sample_every_n_scans"])
        if self.pointcloud_scan_count % max(1, every):
            return
        message = deserialize_message(serialized, PointCloud2)
        self.update_sampled_stamp("lidar", stamp_ns(message))
        try:
            quality = inspect_pointcloud(
                message, int(self.config["pointcloud"]["maximum_sampled_points"]),
                float(self.config["pointcloud"]["minimum_range_m"]),
                float(self.config["pointcloud"]["maximum_range_m"])).to_dict()
        except MissingXyzFields as error:
            self.emit("ERROR", error.code, "bridge", str(error), {})
            return
        except PointCloudProbeError as error:
            self.emit("ERROR", error.code, "bridge", str(error), {})
            return
        self.pointcloud_samples += 1
        self.pointcloud_state = quality
        row = {"wall_time": iso_now(), "monotonic_time_s": time.monotonic()}
        row.update(quality)
        row["fields"] = json.dumps(row["fields"], separators=(",", ":"))
        self.cloud_csv.write(row)
        atomic_json(self.latest/"pointcloud_state.json", quality)
        if quality["density_contract_violation"]:
            self.emit("ERROR", "POINTCLOUD_DENSITY_CONTRACT_VIOLATION", "bridge",
                      "is_dense=true but sampled XYZ contains NaN/Inf", quality)

    def on_odom(self, message: Odometry): self.observe("odometry", message)
    def on_diagnostics(self, message: DiagnosticArray):
        self.observe("diagnostics", message)
        merged: dict[str, str] = {}
        raw = {}
        for status in message.status:
            level = diagnostic_level(status.level)
            values = {item.key: item.value for item in status.values}
            merged.update(values)
            raw[status.name] = {"level": level, "message": status.message, "values": values}
        aliases = {
            "accepted_imu_count": "core_accepted_imu_count",
            "accepted_lidar_count": "core_accepted_lidar_count",
            "rejected_imu_count": "imu_drop_count",
            "rejected_lidar_count": "lidar_drop_count",
        }
        self.diagnostic_values.update(merged)
        merged = self.diagnostic_values
        parsed: dict[str, Any] = {}
        for output, source in aliases.items():
            try: parsed[output] = int(merged.get(source, 0))
            except ValueError: parsed[output] = 0
        for name in ("synchronized_group_count", "correction_attempt_count",
                     "correction_success_count", "correction_failure_count",
                     "processing_lag_ns", "worker_heartbeat", "current_imu_queue_depth",
                     "current_lidar_queue_depth", "imu_queue_capacity", "lidar_queue_capacity"):
            try: parsed[name] = int(merged.get(name, 0))
            except ValueError: parsed[name] = 0
        for key, value in merged.items():
            if key.startswith("reject_reason_"):
                try: parsed[key] = int(value)
                except ValueError: pass
        self.diagnostics = parsed
        self.raw_diagnostics.update(raw)
        atomic_json(self.latest/"diagnostics.json", self.raw_diagnostics)

    def emit(self, severity: str, code: str, subsystem: str, message: str,
             evidence: dict[str, Any], snapshot: bool = True):
        event = {"wall_time": iso_now(), "monotonic_time_s": time.monotonic(),
                 "sim_time_s": None if self.last_clock_ns is None else self.last_clock_ns/1e9,
                 "severity": severity, "code": code, "subsystem": subsystem,
                 "message": message, "evidence": evidence, "snapshot_id": None}
        if snapshot and severity in ("ERROR", "FATAL") and bool(int(self.session_config("AUTO_SNAPSHOT", "1"))):
            event["snapshot_id"] = "pending"
            self.snapshot_executor.submit(
                collect_snapshot, self.session, dict(event),
                bool(self.config["snapshot"]["auto_gdb"]))
        self.event_file.write(json.dumps(event, separators=(",", ":"))+"\n")
        self.event_file.flush()
        self.last_event = event
        print(f"{event['wall_time']} [{severity}] {code}: {message}", flush=True)

    def session_config(self, key: str, default: str) -> str:
        return str(self.config.get("runtime", {}).get(key, default))

    def sample_gazebo(self):
        if self.gazebo_future is None:
            self.gazebo_future = self.gazebo_executor.submit(self.gazebo_probe.sample)
            return
        if not self.gazebo_future.done():
            return
        try:
            state = self.gazebo_future.result()
            state.update({"wall_time": iso_now(), "monotonic_time_s": time.monotonic()})
            self.gz_csv.write(state)
            atomic_json(self.latest/"gazebo_state.json", state)
        except Exception as error:
            self.emit("ERROR", "OBSERVER_PROBE_EXCEPTION", "gazebo",
                      f"gazebo probe: {error}", {"traceback": traceback.format_exc()})
        self.gazebo_future = self.gazebo_executor.submit(self.gazebo_probe.sample)

    def sample(self):
        now = time.monotonic()
        graph = sample_graph(self, {name: tracker.topic for name, tracker in self.streams.items()})
        states = {}
        for name, tracker in self.streams.items():
            tracker.publisher_count = graph[name]["publisher_count"]
            tracker.subscriber_count = graph[name]["subscriber_count"]
            state = tracker.state(now)
            states[name] = state
            row = {"wall_time": iso_now(), "monotonic_time_s": now, "name": name}
            row.update(state)
            self.stream_csv.write(row)
        atomic_json(self.latest/"stream_state.json", states)
        host = host_metrics()
        host["cpu_total_percent"] = cpu_percent(host, self.host_previous)
        self.host_previous = host
        host["overloaded"] = host["cpu_total_percent"] >= float(self.config["system"]["warn_cpu_total_percent"])
        atomic_json(self.latest/"process_state.json", {"host": host})
        for role, pidfile in self.pid_files():
            item = process_metrics(int(pidfile.read_text().strip()))
            previous = self.process_previous.get(item["pid"], {})
            delta_ticks = item.get("utime_ticks", 0)+item.get("stime_ticks", 0)-previous.get("utime_ticks", 0)-previous.get("stime_ticks", 0)
            elapsed = now-previous.get("_sample_wall", now)
            item["cpu_percent"] = (
                None if not previous else
                100.0*max(0.0, delta_ticks) /
                max(1.0, float(os.sysconf("SC_CLK_TCK"))*elapsed))
            item["_sample_wall"] = now
            self.process_previous[item["pid"]] = item
            row = {"wall_time": iso_now(), "monotonic_time_s": now, "role": role}
            row.update(item)
            self.process_csv.write(row)
        gz = json.loads((self.latest/"gazebo_state.json").read_text()) if (self.latest/"gazebo_state.json").exists() else {}
        classification = {
            "starting": now-self.started < float(self.config["observer"]["startup_grace_s"]),
            "clock_regression": self.clock_regressions > 0, "streams": states,
            "pointcloud": self.pointcloud_state, "diagnostics": self.diagnostics,
            "gazebo": gz, "system": host,
            "finite_error_threshold": float(self.config["pointcloud"]["error_finite_ratio_below"]),
            "finite_warn_threshold": float(self.config["pointcloud"]["warn_finite_ratio_below"])}
        state, code = classify(classification, self.previous_classification)
        self.state = state
        for event_code, phase in self.events.update(code, now):
            warning_codes = {"POINTCLOUD_FINITE_RATIO_LOW"}
            base_code = event_code.removesuffix("_ENTER").removesuffix("_PERSIST").removesuffix("_RECOVERED")
            severity = "WARN" if phase == "recovered" or base_code in warning_codes else "ERROR"
            self.emit(severity, event_code, state.value.split("_")[0].lower(),
                      f"{code or event_code} {phase}", classification, phase != "recovered")
        self.previous_classification = classification
        sync = {"wall_time": iso_now(), "monotonic_time_s": now,
                "header_to_clock_offset_ns": None,
                "imu_to_lidar_timestamp_offset_ns": None}
        if self.last_clock_ns and states["lidar"]["header_stamp_last"]:
            sync["header_to_clock_offset_ns"] = states["lidar"]["header_stamp_last"]-self.last_clock_ns
        if states["imu"]["header_stamp_last"] and states["lidar"]["header_stamp_last"]:
            sync["imu_to_lidar_timestamp_offset_ns"] = states["imu"]["header_stamp_last"]-states["lidar"]["header_stamp_last"]
        sync.update(self.diagnostics)
        self.sync_csv.write(sync)
        atomic_json(self.latest/"observer_state.json", {
            "state": self.state.value, "last_event": self.last_event,
            "elapsed_s": now-self.started, "pointcloud_samples": self.pointcloud_samples})

    def pid_files(self):
        for path in (self.session/"pids").glob("*.pid"):
            yield path.stem, path

    def close(self):
        self.state = ObserverState.STOPPED
        self.gazebo_executor.shutdown(wait=False, cancel_futures=True)
        self.snapshot_executor.shutdown(wait=False, cancel_futures=True)
        for sink in (self.stream_csv, self.cloud_csv, self.process_csv, self.gz_csv, self.sync_csv):
            sink.close()
        self.event_file.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--sample-hz", type=float)
    parser.add_argument("--pointcloud-sample-every", type=int)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.sample_hz: config["observer"]["sample_hz"] = args.sample_hz
    if args.pointcloud_sample_every: config["pointcloud"]["sample_every_n_scans"] = args.pointcloud_sample_every
    rclpy.init()
    node = SimObserver(args.session.resolve(), config)
    signal.signal(signal.SIGINT, lambda *_: setattr(node, "stop_requested", True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "stop_requested", True))
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

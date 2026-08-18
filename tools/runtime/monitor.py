#!/usr/bin/env python3
"""One ROS monitor for dataset and PX4 runtime sessions.

The monitor deliberately observes product topics and one compact diagnostics
surface.  It never treats a topic name in the graph as evidence that data is
healthy: every stream is measured by samples, timestamps, gaps and validity.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import signal
import time
from typing import Any, Callable


# Gazebo simulation time starts near zero. PX4 can briefly publish wall-clock
# timestamps before its simulation clock is active; those samples must not
# poison the sim health-rate/regression statistics.
SIMULATION_TIMESTAMP_MAX_NS = 1_000_000_000_000_000
PX4_SIMULATION_STREAMS = frozenset({
    "px4_odometry",
    "vehicle_status",
    "local_position",
    "estimator_status_flags",
})


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))]


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int:
    if isinstance(value, (bytes, bytearray, memoryview)):
        return int.from_bytes(bytes(value), byteorder="little", signed=False)
    return int(value)


def _time_ns(value: Any) -> int:
    if value is None:
        return 0
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)
    if hasattr(value, "nanoseconds"):
        return int(value.nanoseconds)
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _message_stamp_ns(message: Any) -> int:
    header = getattr(message, "header", None)
    if header is not None:
        stamp = _time_ns(getattr(header, "stamp", None))
        if stamp:
            return stamp
    for name, scale in (("timestamp_sample", 1_000), ("timestamp", 1_000)):
        value = getattr(message, name, 0)
        if value:
            return int(value) * scale
    clock = getattr(message, "clock", None)
    return _time_ns(clock)


def _recursive_nonfinite(value: Any) -> int:
    if isinstance(value, float):
        return int(not math.isfinite(value))
    if isinstance(value, (list, tuple)):
        return sum(_recursive_nonfinite(item) for item in value)
    if isinstance(value, dict):
        return sum(_recursive_nonfinite(item) for item in value.values())
    return 0


@dataclass
class StreamStats:
    name: str
    topic: str
    expected_hz: float | None = None
    stale_after_s: float = 1.0
    timestamp_upper_bound_ns: int | None = None
    received: int = 0
    first_stamp_ns: int = 0
    last_stamp_ns: int = 0
    last_arrival_ns: int = 0
    intervals_ms: list[float] = field(default_factory=list)
    arrival_times_s: deque[float] = field(default_factory=lambda: deque(maxlen=512))
    window_rates_hz: list[float] = field(default_factory=list)
    timestamp_duplicates: int = 0
    timestamp_regressions: int = 0
    timestamp_epoch_discard_count: int = 0
    stale_events: int = 0
    stale_event_times_ns: list[int] = field(default_factory=list)
    nonfinite_messages: int = 0
    invalid_quaternions: int = 0
    invalid_covariances: int = 0
    frame_ids: set[str] = field(default_factory=set)
    publisher_count: int = 0
    subscriber_count: int = 0
    _stale_reported: bool = False

    def update(
        self,
        stamp_ns: int,
        arrival_ns: int,
        *,
        frame_id: str = "",
        nonfinite: int = 0,
        invalid_quaternion: bool = False,
        invalid_covariance: bool = False,
    ) -> bool:
        if (
            stamp_ns > 0
            and self.timestamp_upper_bound_ns is not None
            and stamp_ns > self.timestamp_upper_bound_ns
        ):
            self.timestamp_epoch_discard_count += 1
            return False
        self.received += 1
        self.last_arrival_ns = arrival_ns
        self.arrival_times_s.append(arrival_ns / 1e9)
        if stamp_ns:
            if self.last_stamp_ns:
                delta_ms = (stamp_ns - self.last_stamp_ns) / 1e6
                if delta_ms == 0:
                    self.timestamp_duplicates += 1
                elif delta_ms < 0:
                    self.timestamp_regressions += 1
                else:
                    self.intervals_ms.append(delta_ms)
            if not self.first_stamp_ns:
                self.first_stamp_ns = stamp_ns
            self.last_stamp_ns = stamp_ns
        if frame_id:
            self.frame_ids.add(frame_id)
        self.nonfinite_messages += int(nonfinite > 0)
        self.invalid_quaternions += int(invalid_quaternion)
        self.invalid_covariances += int(invalid_covariance)
        self._stale_reported = False
        now_s = arrival_ns / 1e9
        if len(self.arrival_times_s) >= 2:
            start = self.arrival_times_s[0]
            elapsed = now_s - start
            if elapsed > 0:
                self.window_rates_hz.append((len(self.arrival_times_s) - 1) / elapsed)
                self.window_rates_hz = self.window_rates_hz[-512:]
        return True

    def check_stale(self, now_ns: int) -> None:
        # A single first sample is not yet an active stream. Startup can
        # deliver that sample before the following burst; rate and timestamp
        # checks still catch a stream that never becomes active, while stale
        # events describe a stream that has already established a cadence.
        if self.received >= 2 and not self._stale_reported:
            age_s = (now_ns - self.last_arrival_ns) / 1e9
            if age_s > self.stale_after_s:
                self.stale_events += 1
                self.stale_event_times_ns.append(now_ns)
                self._stale_reported = True

    def as_dict(self) -> dict[str, Any]:
        elapsed_s = (self.last_stamp_ns - self.first_stamp_ns) / 1e9 if self.first_stamp_ns else 0.0
        mean_rate = (self.received - 1) / elapsed_s if elapsed_s > 0 and self.received > 1 else 0.0
        return {
            "topic": self.topic,
            "expected_hz": self.expected_hz,
            "received": self.received,
            "mean_rate_hz": mean_rate,
            "minimum_window_rate_hz": min(self.window_rates_hz, default=0.0),
            "p95_interval_ms": _percentile(self.intervals_ms, 0.95),
            "maximum_gap_ms": max(self.intervals_ms, default=0.0),
            "stale_event_count": self.stale_events,
            "stale_event_times_ns": self.stale_event_times_ns,
            "timestamp_duplicate_count": self.timestamp_duplicates,
            "timestamp_regression_count": self.timestamp_regressions,
            "timestamp_epoch_discard_count": self.timestamp_epoch_discard_count,
            "nonfinite_message_count": self.nonfinite_messages,
            "invalid_quaternion_count": self.invalid_quaternions,
            "invalid_covariance_count": self.invalid_covariances,
            "frame_ids": sorted(self.frame_ids),
            "publisher_count": self.publisher_count,
            "subscriber_count": self.subscriber_count,
            "last_stamp_ns": self.last_stamp_ns,
            "last_age_s": ((time.time_ns() - self.last_arrival_ns) / 1e9)
            if self.last_arrival_ns
            else None,
        }


def _odom_payload(message: Any) -> dict[str, Any]:
    pose = getattr(message, "pose", None)
    pose = getattr(pose, "pose", pose)
    twist = getattr(message, "twist", None)
    twist = getattr(twist, "twist", twist)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    return {
        "stamp_ns": _message_stamp_ns(message),
        "frame_id": str(getattr(getattr(message, "header", None), "frame_id", "")),
        "child_frame_id": str(getattr(message, "child_frame_id", "")),
        "position": [_finite(getattr(position, name, None)) for name in ("x", "y", "z")],
        "q_xyzw": [_finite(getattr(orientation, name, None)) for name in ("x", "y", "z", "w")],
        "linear_velocity": [_finite(getattr(linear, name, None)) for name in ("x", "y", "z")],
        "angular_velocity": [_finite(getattr(angular, name, None)) for name in ("x", "y", "z")],
    }


def _px4_odom_payload(message: Any) -> dict[str, Any]:
    return {
        "timestamp_us": int(getattr(message, "timestamp", 0)),
        "timestamp_sample_us": int(getattr(message, "timestamp_sample", 0)),
        "pose_frame": int(getattr(message, "pose_frame", 0)),
        "velocity_frame": int(getattr(message, "velocity_frame", 0)),
        "position": [_finite(value) for value in getattr(message, "position", [])],
        "q_wxyz": [_finite(value) for value in getattr(message, "q", [])],
        "velocity": [_finite(value) for value in getattr(message, "velocity", [])],
        "angular_velocity": [_finite(value) for value in getattr(message, "angular_velocity", [])],
        "position_variance": [_finite(value) for value in getattr(message, "position_variance", [])],
        "orientation_variance": [_finite(value) for value in getattr(message, "orientation_variance", [])],
        "velocity_variance": [_finite(value) for value in getattr(message, "velocity_variance", [])],
        "reset_counter": int(getattr(message, "reset_counter", 0)),
        "quality": int(getattr(message, "quality", 0)),
    }


def _pointcloud_payload(message: Any) -> tuple[dict[str, Any], int]:
    payload: dict[str, Any] = {
        "stamp_ns": _message_stamp_ns(message),
        "frame_id": str(getattr(getattr(message, "header", None), "frame_id", "")),
        "width": int(getattr(message, "width", 0)),
        "height": int(getattr(message, "height", 0)),
        "is_dense": bool(getattr(message, "is_dense", False)),
    }
    invalid = 0
    try:
        from sensor_msgs_py import point_cloud2

        samples: list[list[float]] = []
        for index, point in enumerate(
            point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=False)
        ):
            values = [float(value) for value in point]
            if any(not math.isfinite(value) for value in values):
                invalid += 1
            elif index % 16 == 0:
                samples.append(values)
            if index >= 4096:
                break
        payload["sampled_finite_points"] = len(samples)
        payload["sampled_nonfinite_points"] = invalid
    except (ImportError, RuntimeError, TypeError, ValueError):
        payload["point_decode"] = "NOT_AVAILABLE"
    return payload, invalid


def _diagnostic_payload(message: Any) -> dict[str, Any]:
    statuses: list[dict[str, Any]] = []
    values: dict[str, Any] = {}
    for status in getattr(message, "status", []):
        item_values: dict[str, Any] = {}
        for item in getattr(status, "values", []):
            value: Any = item.value
            if value.lower() in {"true", "false"}:
                value = value.lower() == "true"
            else:
                try:
                    value = float(value) if any(c in value for c in ".eE") else int(value)
                except ValueError:
                    pass
            item_values[item.key] = value
            values[item.key] = value
        statuses.append({
            "name": status.name,
            "level": _integer(status.level),
            "message": status.message,
            "values": item_values,
        })
    return {"stamp_ns": _message_stamp_ns(message), "statuses": statuses, "values": values}


@dataclass
class TopicSpec:
    name: str
    topic: str
    message_type: Any
    formatter: Callable[[Any], dict[str, Any]]


class RuntimeMonitor:
    """ROS node used by all runtime workflows."""

    def __init__(self, output: Path, workflow: str, config: dict[str, Any]) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.output = output
        self.workflow = workflow
        self.config = config
        self.output.mkdir(parents=True, exist_ok=True)
        self.samples_path = output / "samples.jsonl"
        self.latest_path = output / "monitor.json"
        self._sample_stream = self.samples_path.open("a", encoding="utf-8")
        self._rclpy = rclpy
        self.node = Node("uav_navigation_runtime_monitor")
        depth = int(config.get("runtime", {}).get("monitor_queue_depth", 100))
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        thresholds = config.get("runtime", {}).get("thresholds", {})
        self.streams: dict[str, StreamStats] = {}
        self.latest: dict[str, dict[str, Any]] = {}
        self.diagnostics: dict[str, Any] = {"values": {}, "statuses": [], "state": "STARTUP"}
        self.blocked_topics: dict[str, str] = {}
        self.specs = self._specs()
        for spec in self.specs:
            stream_config = config.get("runtime", {}).get("streams", {}).get(spec.name, {})
            self.streams[spec.name] = StreamStats(
                spec.name,
                spec.topic,
                stream_config.get("expected_hz"),
                float(stream_config.get("stale_after_s", thresholds.get("stale_after_s", 1.0))),
                timestamp_upper_bound_ns=(
                    SIMULATION_TIMESTAMP_MAX_NS
                    if self.workflow == "sim" and spec.name in PX4_SIMULATION_STREAMS
                    else None
                ),
            )
            try:
                self.node.create_subscription(
                    spec.message_type,
                    spec.topic,
                    self._callback(spec),
                    qos,
                )
            except (ImportError, AttributeError, RuntimeError, TypeError) as error:
                self.blocked_topics[spec.topic] = str(error)
        self._timer = self.node.create_timer(0.2, self._tick)
        self._tick()

    def _specs(self) -> list[TopicSpec]:
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2

        specs = [
            TopicSpec("imu", str(self.config["fast_lio"]["ros__parameters"]["input"]["imu_topic"]), Imu, lambda m: {
                "stamp_ns": _message_stamp_ns(m),
                "frame_id": str(m.header.frame_id),
                "angular_velocity": [_finite(m.angular_velocity.x), _finite(m.angular_velocity.y), _finite(m.angular_velocity.z)],
                "linear_acceleration": [_finite(m.linear_acceleration.x), _finite(m.linear_acceleration.y), _finite(m.linear_acceleration.z)],
            }),
            TopicSpec("lidar", str(self.config["fast_lio"]["ros__parameters"]["input"]["lidar_topic"]), PointCloud2, lambda m: _pointcloud_payload(m)[0]),
            TopicSpec("corrected_odometry", "/lio/odometry_corrected", Odometry, _odom_payload),
            TopicSpec("propagated_odometry", "/lio/odometry_propagated", Odometry, _odom_payload),
            TopicSpec("diagnostics", "/lio/diagnostics", DiagnosticArray, _diagnostic_payload),
            TopicSpec("mapping_diagnostics", "/navigation_mapping/diagnostics", DiagnosticArray, _diagnostic_payload),
            TopicSpec("planning_diagnostics", "/navigation_planning/diagnostics", DiagnosticArray, _diagnostic_payload),
        ]
        if self.workflow != "dataset":
            # Gazebo's OdometryPublisher is the independent simulator truth.
            # It is ENU/FLU and must remain a separate stream; comparing an
            # estimate to a bridge output derived from that same estimate
            # would only prove that the bridge copied its own numbers.
            specs.append(TopicSpec(
                "ground_truth_odometry", "/sim/ground_truth/odometry", Odometry, _odom_payload
            ))
        try:
            from tf2_msgs.msg import TFMessage
            specs.append(TopicSpec("tf", "/tf", TFMessage, lambda m: {"transform_count": len(m.transforms)}))
            specs.append(TopicSpec("tf_static", "/tf_static", TFMessage, lambda m: {"transform_count": len(m.transforms)}))
        except ImportError:
            self.blocked_topics["/tf"] = "tf2_msgs unavailable"
        if self.workflow != "dataset":
            try:
                from px4_msgs.msg import (
                    EstimatorInnovations,
                    EstimatorStatus,
                    EstimatorStatusFlags,
                    VehicleAttitude,
                    VehicleLocalPosition,
                    VehicleOdometry,
                    VehicleStatus,
                )
                specs.extend([
                    TopicSpec("external_odometry", "/fmu/in/vehicle_visual_odometry", VehicleOdometry, _px4_odom_payload),
                    TopicSpec("px4_odometry", "/fmu/out/vehicle_odometry", VehicleOdometry, _px4_odom_payload),
                    TopicSpec("vehicle_status", "/fmu/out/vehicle_status_v1", VehicleStatus, lambda m: {
                        "timestamp_us": int(m.timestamp), "arming_state": int(m.arming_state),
                        "nav_state": int(m.nav_state), "failsafe": bool(m.failsafe),
                    }),
                    TopicSpec("local_position", "/fmu/out/vehicle_local_position_v1", VehicleLocalPosition, lambda m: {
                        "timestamp_us": int(m.timestamp), "timestamp_sample_us": int(m.timestamp_sample),
                        "x_ned_m": _finite(m.x), "y_ned_m": _finite(m.y), "z_ned_m": _finite(m.z),
                        "vx_ned_m_s": _finite(m.vx), "vy_ned_m_s": _finite(m.vy), "vz_ned_m_s": _finite(m.vz),
                        "heading_ned_rad": _finite(m.heading), "xy_valid": bool(m.xy_valid),
                        "z_valid": bool(m.z_valid), "v_xy_valid": bool(m.v_xy_valid), "v_z_valid": bool(m.v_z_valid),
                        "dead_reckoning": bool(m.dead_reckoning),
                    }),
                    TopicSpec("vehicle_attitude", "/fmu/out/vehicle_attitude", VehicleAttitude, lambda m: {
                        "timestamp_us": int(m.timestamp), "q_wxyz": [_finite(v) for v in m.q],
                    }),
                    TopicSpec("estimator_status", "/fmu/out/estimator_status", EstimatorStatus, lambda m: {
                        "timestamp_us": int(m.timestamp), "control_mode_flags": int(m.control_mode_flags),
                        "filter_fault_flags": int(m.filter_fault_flags), "control_status_flags": int(m.control_mode_flags),
                        "reset_count_pos_ne": int(m.reset_count_pos_ne), "reset_count_vel_ne": int(m.reset_count_vel_ne),
                        "reset_count_quat": int(m.reset_count_quat),
                    }),
                    TopicSpec("estimator_status_flags", "/fmu/out/estimator_status_flags", EstimatorStatusFlags, lambda m: {
                        # PX4 calls these control-status flags.  They are useful
                        # telemetry but not proof that a particular EV sample
                        # fused, so the report labels them as observations only.
                        "timestamp_us": int(m.timestamp), "cs_ev_pos": bool(m.cs_ev_pos),
                        "cs_ev_vel": bool(m.cs_ev_vel), "cs_ev_yaw": bool(m.cs_ev_yaw),
                        "cs_inertial_dead_reckoning": bool(m.cs_inertial_dead_reckoning),
                        "fs_bad_hdg": bool(m.fs_bad_hdg), "reject_hor_pos": bool(m.reject_hor_pos),
                        "reject_hor_vel": bool(m.reject_hor_vel), "reject_yaw": bool(m.reject_yaw),
                    }),
                    TopicSpec("estimator_innovations", "/fmu/out/estimator_innovations", EstimatorInnovations, lambda m: {
                        "timestamp_us": int(m.timestamp), "timestamp_sample_us": int(m.timestamp_sample),
                        "gps_hpos": [_finite(v) for v in m.gps_hpos], "gps_vpos": _finite(m.gps_vpos),
                        "ev_hpos": [_finite(v) for v in m.ev_hpos], "ev_vpos": _finite(m.ev_vpos),
                        "ev_vel": [_finite(v) for v in m.ev_vel], "heading": _finite(m.heading),
                    }),
                ])
            except ImportError as error:
                self.blocked_topics["/fmu/out/estimator_status_flags"] = f"px4_msgs unavailable: {error}"
        return specs

    def _callback(self, spec: TopicSpec) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            arrival_ns = time.time_ns()
            try:
                if spec.name == "lidar":
                    payload, _ = _pointcloud_payload(message)
                    point_invalid = 0
                else:
                    payload = spec.formatter(message)
                    point_invalid = 0
            except (AttributeError, RuntimeError, TypeError, ValueError) as error:
                payload = {"decode_error": str(error), "stamp_ns": _message_stamp_ns(message)}
                point_invalid = 1
            stamp_ns = int(payload.get("stamp_ns", _message_stamp_ns(message)))
            frame_id = str(payload.get("frame_id", ""))
            nonfinite = _recursive_nonfinite(payload) + point_invalid
            q = payload.get("q_xyzw") or payload.get("q_wxyz")
            invalid_q = False
            if q:
                values = [value for value in q if value is not None]
                invalid_q = len(values) != 4 or abs(math.sqrt(sum(value * value for value in values)) - 1.0) > 0.05
            covariance_values = payload.get("position_variance") or payload.get("covariance")
            invalid_covariance = bool(covariance_values) and _recursive_nonfinite(covariance_values) > 0
            stats = self.streams[spec.name]
            accepted_by_monitor = stats.update(
                stamp_ns,
                arrival_ns,
                frame_id=frame_id,
                nonfinite=nonfinite,
                invalid_quaternion=invalid_q,
                invalid_covariance=invalid_covariance,
            )
            if accepted_by_monitor:
                self.latest[spec.name] = payload
            if spec.name == "diagnostics":
                self._update_diagnostic_state(payload)
            sample = {
                "kind": "sample",
                "stream": spec.name,
                "arrival_wall_ns": arrival_ns,
                "timestamp_ns": stamp_ns,
                "payload": payload,
                "accepted_by_monitor": accepted_by_monitor,
            }
            self._sample_stream.write(json.dumps(sample, sort_keys=True, allow_nan=False) + "\n")

        return callback

    def _update_diagnostic_state(self, payload: dict[str, Any]) -> None:
        estimator_values: dict[str, Any] | None = None
        for status in payload.get("statuses", []):
            if str(status.get("name", "")).endswith("/estimator"):
                candidate = status.get("values", {})
                if isinstance(candidate, dict):
                    estimator_values = candidate
                break
        if estimator_values is None:
            return
        values = estimator_values
        state = values.get("state", values.get("status", self.diagnostics.get("state", "STARTUP")))
        self.diagnostics["state"] = str(state)
        self.diagnostics["navigation_valid"] = bool(values.get("navigation_valid", False))
        self.diagnostics["last_failure_code"] = str(values.get("last_failure_code", values.get("last_update_failure_class", "NONE")))
        self.diagnostics["last_failure_reason"] = str(values.get("last_failure_reason", payload.get("message", "")))
        self.diagnostics["values"] = values

    def _tick(self) -> None:
        now_ns = time.time_ns()
        for stats in self.streams.values():
            stats.check_stale(now_ns)
            try:
                stats.publisher_count = len(self.node.get_publishers_info_by_topic(stats.topic))
                stats.subscriber_count = len(self.node.get_subscriptions_info_by_topic(stats.topic))
            except (AttributeError, RuntimeError):
                pass
        snapshot = self.snapshot()
        temporary = self.latest_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(self.latest_path)

    def snapshot(self) -> dict[str, Any]:
        return {
            "workflow": self.workflow,
            "updated_at": time.time(),
            "streams": {name: stats.as_dict() for name, stats in self.streams.items()},
            "latest": self.latest,
            "diagnostics": self.diagnostics,
            "blocked_topics": self.blocked_topics,
        }

    def close(self) -> None:
        self._tick()
        self._sample_stream.close()
        self.node.destroy_node()
        self._rclpy.try_shutdown()


def run_monitor(output: Path, workflow: str, config_path: Path) -> int:
    import yaml
    import rclpy

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"runtime config must be a mapping: {config_path}")
    common_path = config_path.parent / "common.yaml"
    if common_path.is_file():
        common = yaml.safe_load(common_path.read_text(encoding="utf-8"))
        if isinstance(common, dict):
            config["runtime"] = dict(common.get("runtime", {}))
            config["runtime"].update(config.get("runtime_overrides", {}))
    rclpy.init(args=[])
    monitor = RuntimeMonitor(output, workflow, config)
    stopping = False

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal stopping
        stopping = True
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(monitor.node, timeout_sec=0.2)
    finally:
        monitor.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workflow", choices=("dataset", "sim"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    return run_monitor(args.output, args.workflow, args.config)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Canonical, headless P0.8 SITL qualification orchestrator.

This is the only P0.8 executable that launches a SITL qualification.  The
module deliberately keeps its ROS-independent contracts importable without a
ROS environment so the lifecycle, ownership, QoS, accounting, and artifact
tests can run in ``make test-tools``.
"""

from __future__ import annotations

import argparse
import csv
import datetime as datetime_module
from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import shutil
import socket
import subprocess
import sys
import time
from typing import Any, Callable, Iterable, TextIO

TOOLS = Path(__file__).resolve().parent
WORKSPACE = TOOLS.parents[1]
sys.path.insert(0, str(TOOLS))

from p0_8_probe_metrics import StatusEventAccumulator  # noqa: E402

try:  # ROS is deliberately optional for the ROS-independent test surface.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from navigation_interfaces.msg import OdometrySupervisorStatus
    from px4_msgs.msg import VehicleOdometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    try:
        from rclpy.subscription import SubscriptionEventCallbacks
    except ImportError:  # pragma: no cover - API variation across ROS distros.
        SubscriptionEventCallbacks = None  # type: ignore[assignment,misc]
    ROS_IMPORT_ERROR: Exception | None = None
except ImportError as error:  # pragma: no cover - exercised on non-ROS hosts.
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    ROS_IMPORT_ERROR = error


PRODUCT_BASE_SHA = "9494791092ed17047fbc288d133804b98486057d"
PX4_REQUIRED_SHA = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
PX4_MSGS_REQUIRED_SHA = "86d8239e962f6939e05c3737784f60c02fa884db"
RAW_ODOMETRY_RE = re.compile(r"^/fmu/out/vehicle_odometry(?:_v\d+)?$")
RAW_ODOMETRY_TYPE = "px4_msgs/msg/VehicleOdometry"
GAZEBO_CLOCK_TOPIC = "/world/px4_lio_smoke/clock"
FROZEN_PRODUCT_PATHS = (
    "src/navigation_estimator/fast_lio_core",
    "src/navigation_estimator/fast_lio_ros",
    "src/px4_interface/px4_odometry_bridge",
    "src/odometry_supervisor",
    "src/navigation_interfaces",
)


class RunMode(str, Enum):
    STARTUP = "startup"
    SMOKE_ON = "smoke-on"
    SITL_OFF = "sitl-off"
    SITL_ON = "sitl-on"
    MEMORY_ON = "memory-on"


class Stage(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    XRCE_AGENT_STARTING = "XRCE_AGENT_STARTING"
    XRCE_AGENT_READY = "XRCE_AGENT_READY"
    PX4_GAZEBO_STARTING = "PX4_GAZEBO_STARTING"
    GAZEBO_CLOCK_READY = "GAZEBO_CLOCK_READY"
    ROS_BRIDGE_STARTING = "ROS_BRIDGE_STARTING"
    ROS_CLOCK_READY = "ROS_CLOCK_READY"
    XRCE_SESSION_READY = "XRCE_SESSION_READY"
    RAW_PX4_ODOMETRY_READY = "RAW_PX4_ODOMETRY_READY"
    PX4_INGRESS_STARTING = "PX4_INGRESS_STARTING"
    PX4_INGRESS_READY = "PX4_INGRESS_READY"
    SENSORS_READY = "SENSORS_READY"
    FAST_LIO_STARTING = "FAST_LIO_STARTING"
    FAST_LIO_READY = "FAST_LIO_READY"
    PRIOR_ACCEPTED = "PRIOR_ACCEPTED"
    SUPERVISOR_STARTING = "SUPERVISOR_STARTING"
    SUPERVISOR_READY = "SUPERVISOR_READY"
    WARMUP = "WARMUP"
    MEASURING = "MEASURING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    CLEANUP = "CLEANUP"


class FailureCode(str, Enum):
    RESOURCE_CONFLICT = "RESOURCE_CONFLICT"
    PRODUCT_FREEZE_VIOLATION = "PRODUCT_FREEZE_VIOLATION"
    PROVENANCE_INVALID = "PROVENANCE_INVALID"
    XRCE_PORT_NOT_BOUND = "XRCE_PORT_NOT_BOUND"
    XRCE_SESSION_NOT_ESTABLISHED = "XRCE_SESSION_NOT_ESTABLISHED"
    PX4_GAZEBO_EXITED = "PX4_GAZEBO_EXITED"
    GAZEBO_CLOCK_TOPIC_MISSING = "GAZEBO_CLOCK_TOPIC_MISSING"
    ROS_BRIDGE_EXITED = "ROS_BRIDGE_EXITED"
    ROS_CLOCK_PUBLISHER_MISSING = "ROS_CLOCK_PUBLISHER_MISSING"
    ROS_CLOCK_SAMPLE_MISSING = "ROS_CLOCK_SAMPLE_MISSING"
    ROS_CLOCK_NOT_ADVANCING = "ROS_CLOCK_NOT_ADVANCING"
    RAW_ODOMETRY_TOPIC_MISSING = "RAW_ODOMETRY_TOPIC_MISSING"
    RAW_ODOMETRY_SAMPLE_MISSING = "RAW_ODOMETRY_SAMPLE_MISSING"
    QOS_INCOMPATIBLE = "QOS_INCOMPATIBLE"
    PX4_INGRESS_EXITED = "PX4_INGRESS_EXITED"
    PX4_INGRESS_REJECTING = "PX4_INGRESS_REJECTING"
    PX4_OUTPUT_MISSING = "PX4_OUTPUT_MISSING"
    SENSOR_TOPIC_MISSING = "SENSOR_TOPIC_MISSING"
    SENSOR_SAMPLE_MISSING = "SENSOR_SAMPLE_MISSING"
    FAST_LIO_EXITED = "FAST_LIO_EXITED"
    FAST_LIO_OUTPUT_MISSING = "FAST_LIO_OUTPUT_MISSING"
    FAST_LIO_NOT_TRACKING = "FAST_LIO_NOT_TRACKING"
    INITIAL_PRIOR_NOT_ACCEPTED = "INITIAL_PRIOR_NOT_ACCEPTED"
    INITIAL_PRIOR_FALLBACK_USED = "INITIAL_PRIOR_FALLBACK_USED"
    SUPERVISOR_EXITED = "SUPERVISOR_EXITED"
    SUPERVISOR_STATUS_MISSING = "SUPERVISOR_STATUS_MISSING"
    SUPERVISOR_DIAGNOSTICS_MISSING = "SUPERVISOR_DIAGNOSTICS_MISSING"
    MEASUREMENT_CONTRACT_FAILED = "MEASUREMENT_CONTRACT_FAILED"
    CLEANUP_INCOMPLETE = "CLEANUP_INCOMPLETE"


@dataclass(frozen=True)
class ModePolicy:
    mode: RunMode
    supervisor_enabled: bool
    warmup_sim_s: float | None
    measurement_sim_s: float | None
    stop_after: Stage | None = None


MODE_POLICIES: dict[RunMode, ModePolicy] = {
    RunMode.STARTUP: ModePolicy(RunMode.STARTUP, False, None, None, Stage.PX4_INGRESS_READY),
    RunMode.SMOKE_ON: ModePolicy(RunMode.SMOKE_ON, True, 10.0, 20.0),
    RunMode.SITL_OFF: ModePolicy(RunMode.SITL_OFF, False, 30.0, 120.0),
    RunMode.SITL_ON: ModePolicy(RunMode.SITL_ON, True, 30.0, 120.0),
    RunMode.MEMORY_ON: ModePolicy(RunMode.MEMORY_ON, True, 120.0, 1200.0),
}


@dataclass(frozen=True)
class StagePolicy:
    timeout_s: float
    timeout_failure: FailureCode


# One table owns every wall-clock deadline.  Simulation time is never used for
# a watchdog or cleanup deadline.
STAGE_POLICIES: dict[Stage, StagePolicy] = {
    Stage.PREFLIGHT: StagePolicy(30.0, FailureCode.PROVENANCE_INVALID),
    Stage.XRCE_AGENT_STARTING: StagePolicy(20.0, FailureCode.XRCE_PORT_NOT_BOUND),
    Stage.XRCE_AGENT_READY: StagePolicy(20.0, FailureCode.XRCE_PORT_NOT_BOUND),
    Stage.PX4_GAZEBO_STARTING: StagePolicy(90.0, FailureCode.PX4_GAZEBO_EXITED),
    Stage.GAZEBO_CLOCK_READY: StagePolicy(60.0, FailureCode.GAZEBO_CLOCK_TOPIC_MISSING),
    Stage.ROS_BRIDGE_STARTING: StagePolicy(30.0, FailureCode.ROS_BRIDGE_EXITED),
    Stage.ROS_CLOCK_READY: StagePolicy(30.0, FailureCode.ROS_CLOCK_NOT_ADVANCING),
    Stage.XRCE_SESSION_READY: StagePolicy(60.0, FailureCode.XRCE_SESSION_NOT_ESTABLISHED),
    Stage.RAW_PX4_ODOMETRY_READY: StagePolicy(45.0, FailureCode.RAW_ODOMETRY_SAMPLE_MISSING),
    Stage.PX4_INGRESS_STARTING: StagePolicy(30.0, FailureCode.PX4_INGRESS_EXITED),
    Stage.PX4_INGRESS_READY: StagePolicy(45.0, FailureCode.PX4_OUTPUT_MISSING),
    Stage.SENSORS_READY: StagePolicy(60.0, FailureCode.SENSOR_SAMPLE_MISSING),
    Stage.FAST_LIO_STARTING: StagePolicy(30.0, FailureCode.FAST_LIO_EXITED),
    Stage.FAST_LIO_READY: StagePolicy(120.0, FailureCode.FAST_LIO_OUTPUT_MISSING),
    Stage.PRIOR_ACCEPTED: StagePolicy(45.0, FailureCode.INITIAL_PRIOR_NOT_ACCEPTED),
    Stage.SUPERVISOR_STARTING: StagePolicy(30.0, FailureCode.SUPERVISOR_EXITED),
    Stage.SUPERVISOR_READY: StagePolicy(45.0, FailureCode.SUPERVISOR_STATUS_MISSING),
    Stage.WARMUP: StagePolicy(300.0, FailureCode.ROS_CLOCK_NOT_ADVANCING),
    Stage.MEASURING: StagePolicy(1500.0, FailureCode.ROS_CLOCK_NOT_ADVANCING),
    Stage.CLEANUP: StagePolicy(30.0, FailureCode.CLEANUP_INCOMPLETE),
}


SEQUENTIAL_STAGES = tuple(stage for stage in Stage if stage not in {Stage.FAILED, Stage.CLEANUP})


class QualificationFailure(RuntimeError):
    def __init__(self, code: FailureCode, summary: str, stage: Stage | None = None) -> None:
        super().__init__(summary)
        self.code = code
        self.summary = summary
        self.stage = stage


class StageMachine:
    """Reject lifecycle jumps so startup ordering is executable contract."""

    def __init__(self) -> None:
        self.current: Stage | None = None

    def transition(self, target: Stage) -> None:
        if self.current is None:
            if target is not Stage.PREFLIGHT:
                raise ValueError(f"illegal initial stage: {target.value}")
        elif target is Stage.FAILED:
            pass
        elif target is Stage.CLEANUP:
            if self.current not in {Stage.COMPLETE, Stage.FAILED}:
                raise ValueError(f"cleanup from {self.current.value} is illegal")
        elif self.current is Stage.FAILED:
            raise ValueError("no stage may follow FAILED except CLEANUP")
        else:
            try:
                expected = SEQUENTIAL_STAGES[SEQUENTIAL_STAGES.index(self.current) + 1]
            except (ValueError, IndexError) as error:
                raise ValueError(f"no legal successor for {self.current.value}") from error
            bypasses = {
                (Stage.PX4_INGRESS_READY, Stage.COMPLETE),
                (Stage.PRIOR_ACCEPTED, Stage.WARMUP),
            }
            if target is not expected and (self.current, target) not in bypasses:
                raise ValueError(f"expected {expected.value}, received {target.value}")
        self.current = target


def atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, default=str)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def file_sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def command_output(args: list[str], cwd: Path | None = None, timeout: float = 10.0) -> str:
    try:
        result = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                                timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return result.stdout.strip()


def command_succeeds(args: list[str], cwd: Path | None = None, timeout: float = 10.0) -> bool:
    try:
        return subprocess.run(args, cwd=cwd, capture_output=True, timeout=timeout,
                              check=False).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def stamp_ns(message: Any) -> int:
    value = getattr(message, "clock", None)
    if value is None:
        value = getattr(message, "header", None)
    if value is not None and hasattr(value, "stamp"):
        value = value.stamp
    if value is not None and hasattr(value, "sec"):
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)
    if hasattr(message, "timestamp"):
        return int(message.timestamp) * 1000
    return 0


def parse_bool(value: str | None) -> bool | None:
    if value is None:
        return None
    if value.lower() in {"true", "1", "yes"}:
        return True
    if value.lower() in {"false", "0", "no"}:
        return False
    return None


def parse_int(value: str | None) -> int | None:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def int_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


def parse_float(value: str | None) -> float | None:
    try:
        return float(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def qos_depth_value(value: Any) -> int | None:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


@dataclass
class TopicObservation:
    logical_name: str
    resolved_topic: str
    message_type: str
    requested_qos: dict[str, Any]
    publisher_count: int = 0
    first_wall_monotonic_ns: int | None = None
    last_wall_monotonic_ns: int | None = None
    first_ros_stamp_ns: int | None = None
    last_ros_stamp_ns: int | None = None
    message_count: int = 0
    longest_wall_gap_ns: int = 0
    longest_ros_stamp_gap_ns: int = 0
    latest_payload_summary: dict[str, Any] | None = None
    publisher_endpoint_qos: list[dict[str, Any]] = field(default_factory=list)
    qos_incompatible_event_count: int = 0

    def observe(self, message: Any, summary: dict[str, Any] | None = None,
                wall_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if wall_ns is None else wall_ns
        ros_ns = stamp_ns(message)
        if self.first_wall_monotonic_ns is None:
            self.first_wall_monotonic_ns = now_ns
        if self.last_wall_monotonic_ns is not None:
            self.longest_wall_gap_ns = max(
                self.longest_wall_gap_ns, now_ns - self.last_wall_monotonic_ns)
        if ros_ns > 0:
            if self.first_ros_stamp_ns is None:
                self.first_ros_stamp_ns = ros_ns
            if self.last_ros_stamp_ns is not None:
                self.longest_ros_stamp_gap_ns = max(
                    self.longest_ros_stamp_gap_ns, ros_ns - self.last_ros_stamp_ns)
            self.last_ros_stamp_ns = ros_ns
        self.last_wall_monotonic_ns = now_ns
        self.message_count += 1
        self.latest_payload_summary = summary or {}

    def as_dict(self) -> dict[str, Any]:
        return {
            "logical_name": self.logical_name,
            "resolved_topic": self.resolved_topic,
            "message_type": self.message_type,
            "publisher_count": self.publisher_count,
            "first_wall_monotonic_ns": self.first_wall_monotonic_ns,
            "last_wall_monotonic_ns": self.last_wall_monotonic_ns,
            "first_ros_stamp_ns": self.first_ros_stamp_ns,
            "last_ros_stamp_ns": self.last_ros_stamp_ns,
            "message_count": self.message_count,
            "longest_wall_gap_ns": self.longest_wall_gap_ns,
            "longest_ros_stamp_gap_ns": self.longest_ros_stamp_gap_ns,
            "latest_payload_summary": self.latest_payload_summary,
            "requested_qos": self.requested_qos,
            "publisher_endpoint_qos": self.publisher_endpoint_qos,
            "qos_incompatible_event_count": self.qos_incompatible_event_count,
        }


def classify_topic_observation(observation: TopicObservation) -> FailureCode | None:
    """Classify readiness without collapsing no-publisher/no-sample/QoS cases."""
    if observation.qos_incompatible_event_count:
        return FailureCode.QOS_INCOMPATIBLE
    if observation.publisher_count <= 0:
        return (FailureCode.RAW_ODOMETRY_TOPIC_MISSING
                if observation.logical_name == "raw_px4_odometry"
                else FailureCode.SENSOR_TOPIC_MISSING)
    if observation.message_count <= 0:
        return (FailureCode.RAW_ODOMETRY_SAMPLE_MISSING
                if observation.logical_name == "raw_px4_odometry"
                else FailureCode.SENSOR_SAMPLE_MISSING)
    return None


def resolve_raw_candidates(candidates: Iterable[str]) -> tuple[list[str], str | None]:
    """Resolve versioned PX4 odometry topics deterministically."""
    valid = sorted(
        (topic for topic in candidates if RAW_ODOMETRY_RE.fullmatch(topic)),
        key=lambda topic: (0 if topic == "/fmu/out/vehicle_odometry" else 1,
                           int(topic.rsplit("_v", 1)[1]) if "_v" in topic else 0,
                           topic),
    )
    return valid, valid[0] if valid else None


class DiagnosticsCache:
    """Persistent DiagnosticStatus snapshots keyed by status name."""

    def __init__(self) -> None:
        self.by_status_name: dict[str, dict[str, Any]] = {}

    def update(self, message: Any) -> None:
        array_stamp = stamp_ns(message)
        for status in getattr(message, "status", ()):  # status may be absent in a malformed msg.
            self.by_status_name[status.name] = {
                "name": status.name,
                "level": int_value(status.level),
                "message": status.message,
                "stamp_ns": array_stamp,
                "values": {item.key: item.value for item in status.values},
            }

    def snapshot(self, name: str) -> dict[str, Any] | None:
        value = self.by_status_name.get(name)
        return None if value is None else json.loads(json.dumps(value))

    def values(self, name: str) -> dict[str, str]:
        snapshot = self.by_status_name.get(name)
        return {} if snapshot is None else dict(snapshot["values"])

    def as_dict(self) -> dict[str, Any]:
        return json.loads(json.dumps(self.by_status_name))


@dataclass(frozen=True)
class TopicSpec:
    logical_name: str
    topic: str
    message_type: str
    requested_qos: dict[str, Any]


def qos_dict(history: str, depth: int, reliability: str, durability: str) -> dict[str, Any]:
    return {
        "history": history,
        "depth": depth,
        "reliability": reliability,
        "durability": durability,
    }


RAW_QOS = qos_dict("KEEP_LAST", 10, "BEST_EFFORT", "VOLATILE")
PX4_DIAGNOSTICS_QOS = qos_dict("KEEP_LAST", 1, "RELIABLE", "TRANSIENT_LOCAL")
SENSOR_QOS = qos_dict("KEEP_LAST", 100, "BEST_EFFORT", "VOLATILE")
LIO_QOS = qos_dict("KEEP_LAST", 10, "RELIABLE", "VOLATILE")
SUPERVISOR_QOS = qos_dict("KEEP_LAST", 10, "RELIABLE", "VOLATILE")
CLOCK_QOS = qos_dict("KEEP_LAST", 10, "BEST_EFFORT", "VOLATILE")


class RosReadinessMonitor(Node):  # type: ignore[misc]
    """One persistent ROS participant for discovery, readiness, and measurement."""

    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover - guarded by orchestrator preflight.
            raise RuntimeError(f"ROS unavailable: {ROS_IMPORT_ERROR}")
        super().__init__(
            "p0_8_canonical_sitl_orchestrator",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, False)],
        )
        self.ledger: dict[str, TopicObservation] = {}
        self.diagnostics = DiagnosticsCache()
        self.clock_values: list[int] = []
        self.latest_clock_ns = 0
        self.last_clock_ns = 0
        self.raw_candidates: list[str] = []
        self.raw_selected_topic: str | None = None
        self.supervisor_status: Any | None = None
        self.supervisor_status_count = 0
        self.status_accumulator: StatusEventAccumulator | None = None
        self.measurement_end_ns: int | None = None
        self.measurement_baseline_diagnostics: dict[str, Any] = {}
        self._qualification_subscriptions: dict[str, Any] = {}
        self._create_static_subscriptions()

    @staticmethod
    def _qos(profile: dict[str, Any]) -> Any:
        values = {
            "history": HistoryPolicy.KEEP_LAST,
            "reliability": ReliabilityPolicy.BEST_EFFORT,
            "durability": DurabilityPolicy.VOLATILE,
        }
        if profile["reliability"] == "RELIABLE":
            values["reliability"] = ReliabilityPolicy.RELIABLE
        if profile["durability"] == "TRANSIENT_LOCAL":
            values["durability"] = DurabilityPolicy.TRANSIENT_LOCAL
        return QoSProfile(depth=profile["depth"], **values)

    def _add_observation(self, spec: TopicSpec) -> TopicObservation:
        observation = TopicObservation(
            spec.logical_name, spec.topic, spec.message_type, dict(spec.requested_qos))
        self.ledger[spec.logical_name] = observation
        return observation

    def _subscribe(self, logical: str, message_type: Any, spec: TopicSpec,
                   callback: Callable[[Any], None]) -> None:
        observation = self._add_observation(spec)
        kwargs: dict[str, Any] = {}
        if SubscriptionEventCallbacks is not None:
            kwargs["event_callbacks"] = SubscriptionEventCallbacks(
                incompatible_qos=lambda _event: setattr(
                    observation, "qos_incompatible_event_count",
                    observation.qos_incompatible_event_count + 1))
        try:
            subscription = self.create_subscription(
                message_type, spec.topic, callback, self._qos(spec.requested_qos), **kwargs)
        except TypeError:  # pragma: no cover - compatibility with older rclpy signatures.
            subscription = self.create_subscription(
                message_type, spec.topic, callback, self._qos(spec.requested_qos))
        self._qualification_subscriptions[logical] = subscription

    def _create_static_subscriptions(self) -> None:
        self._subscribe("clock", Clock,
                        TopicSpec("clock", "/clock", "rosgraph_msgs/msg/Clock", CLOCK_QOS),
                        self._on_clock)
        self._subscribe(
            "px4_odometry_ros", Odometry,
            TopicSpec("px4_odometry_ros", "/px4/odometry_ros", "nav_msgs/msg/Odometry", LIO_QOS),
            lambda message: self._observe("px4_odometry_ros", message, self._odom_summary(message)))
        self._subscribe(
            "px4_diagnostics", DiagnosticArray,
            TopicSpec("px4_diagnostics", "/px4/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", PX4_DIAGNOSTICS_QOS),
            self._on_px4_diagnostics)
        self._subscribe(
            "imu", Imu,
            TopicSpec("imu", "/lidar/imu", "sensor_msgs/msg/Imu", SENSOR_QOS),
            lambda message: self._observe("imu", message, {"stamp_ns": stamp_ns(message)}))
        self._subscribe(
            "lidar", PointCloud2,
            TopicSpec("lidar", "/lidar/points", "sensor_msgs/msg/PointCloud2", SENSOR_QOS),
            lambda message: self._observe("lidar", message, {
                "stamp_ns": stamp_ns(message), "width": int(message.width),
                "height": int(message.height), "point_step": int(message.point_step)}))
        self._subscribe(
            "lio_corrected", Odometry,
            TopicSpec("lio_corrected", "/lio/odometry_corrected", "nav_msgs/msg/Odometry", LIO_QOS),
            lambda message: self._observe("lio_corrected", message, self._odom_summary(message)))
        self._subscribe(
            "lio_propagated", Odometry,
            TopicSpec("lio_propagated", "/lio/odometry_propagated", "nav_msgs/msg/Odometry", LIO_QOS),
            lambda message: self._observe("lio_propagated", message, self._odom_summary(message)))
        self._subscribe(
            "lio_diagnostics", DiagnosticArray,
            TopicSpec("lio_diagnostics", "/lio/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", LIO_QOS),
            self._on_lio_diagnostics)
        self._subscribe(
            "supervisor_status", OdometrySupervisorStatus,
            TopicSpec("supervisor_status", "/navigation/odometry_supervisor/status", "navigation_interfaces/msg/OdometrySupervisorStatus", SUPERVISOR_QOS),
            self._on_supervisor_status)
        self._subscribe(
            "supervisor_diagnostics", DiagnosticArray,
            TopicSpec("supervisor_diagnostics", "/navigation/odometry_supervisor/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", SUPERVISOR_QOS),
            self._on_supervisor_diagnostics)

    @staticmethod
    def _odom_summary(message: Any) -> dict[str, Any]:
        return {
            "stamp_ns": stamp_ns(message),
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
        }

    def _observe(self, logical: str, message: Any, summary: dict[str, Any] | None = None) -> None:
        if logical in self.ledger:
            self.ledger[logical].observe(message, summary)

    def _on_clock(self, message: Any) -> None:
        value = stamp_ns(message)
        if value <= 0:
            return
        self._observe("clock", message, {"clock_ns": value})
        self.last_clock_ns = self.latest_clock_ns
        self.latest_clock_ns = value
        self.clock_values.append(value)
        self.clock_values = self.clock_values[-64:]

    def _on_px4_diagnostics(self, message: Any) -> None:
        self._observe("px4_diagnostics", message, {"status_names": [s.name for s in message.status]})
        self.diagnostics.update(message)

    def _on_lio_diagnostics(self, message: Any) -> None:
        self._observe("lio_diagnostics", message, {"status_names": [s.name for s in message.status]})
        self.diagnostics.update(message)

    def _on_supervisor_diagnostics(self, message: Any) -> None:
        self._observe("supervisor_diagnostics", message, {"status_names": [s.name for s in message.status]})
        self.diagnostics.update(message)
        if self.status_accumulator is None:
            return
        event_ns = stamp_ns(message) or self.latest_clock_ns
        values = self.diagnostics.values("odometry_supervisor")
        heading = parse_bool(values.get("heading_observable"))
        if heading is not None:
            self.status_accumulator.record_heading_event(event_ns, heading)
        pending = parse_int(values.get("pending_query_epoch_ns"))
        if pending is not None:
            self.status_accumulator.record_pending_query(
                event_ns, pending, parse_int(values.get("pending_query_age_ns")))

    def _on_supervisor_status(self, message: Any) -> None:
        self._observe("supervisor_status", message, {
            "health": int_value(message.health), "reason": str(message.reason),
            "comparison_valid": bool(message.comparison_valid),
        })
        self.supervisor_status = message
        self.supervisor_status_count += 1
        if self.status_accumulator is not None:
            self.status_accumulator.record(message, stamp_ns(message) or self.latest_clock_ns)

    def discover_raw_topic(self) -> list[str]:
        graph = dict(self.get_topic_names_and_types())
        self.raw_candidates, _ = resolve_raw_candidates(
            topic for topic, types in graph.items()
            if RAW_ODOMETRY_TYPE in types)
        return list(self.raw_candidates)

    def subscribe_raw_topic(self) -> str | None:
        candidates = self.discover_raw_topic()
        if not candidates:
            return None
        selected = candidates[0]
        if self.raw_selected_topic == selected:
            return selected
        spec = TopicSpec("raw_px4_odometry", selected, RAW_ODOMETRY_TYPE, RAW_QOS)
        self._subscribe(
            "raw_px4_odometry", VehicleOdometry, spec,
            lambda message: self._observe("raw_px4_odometry", message, {
                "timestamp_us": int(message.timestamp),
                "timestamp_sample_us": int(message.timestamp_sample),
                "pose_frame": int(message.pose_frame),
                "velocity_frame": int(message.velocity_frame),
            }))
        self.raw_selected_topic = selected
        return selected

    def refresh_topic_graph(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        for observation in self.ledger.values():
            if observation.resolved_topic not in graph:
                observation.publisher_count = 0
                continue
            observation.publisher_count = int(self.count_publishers(observation.resolved_topic))
            infos = self.get_publishers_info_by_topic(observation.resolved_topic)
            endpoint_qos: list[dict[str, Any]] = []
            for info in infos:
                profile = getattr(info, "qos_profile", None)
                endpoint_qos.append({
                    "node_name": getattr(info, "node_name", ""),
                    "node_namespace": getattr(info, "node_namespace", ""),
                    "reliability": str(getattr(profile, "reliability", "")),
                    "durability": str(getattr(profile, "durability", "")),
                    "history": str(getattr(profile, "history", "")),
                    "depth": qos_depth_value(getattr(profile, "depth", 0)),
                })
            observation.publisher_endpoint_qos = endpoint_qos

    def topic(self, logical: str) -> TopicObservation | None:
        return self.ledger.get(logical)

    def topic_ready(self, logical: str, minimum_messages: int = 1) -> bool:
        observation = self.ledger.get(logical)
        return observation is not None and observation.publisher_count > 0 and \
            observation.message_count >= minimum_messages

    def ros_clock_advancing(self) -> bool:
        return any(after > before for before, after in zip(self.clock_values, self.clock_values[1:]))

    def diagnostics_values(self, status_name: str) -> dict[str, str]:
        return self.diagnostics.values(status_name)

    def start_measurement(self, duration_s: float) -> None:
        self.measurement_end_ns = self.latest_clock_ns + int(duration_s * 1_000_000_000)
        self.status_accumulator = StatusEventAccumulator()
        self.status_accumulator.start(self.latest_clock_ns, self.measurement_end_ns,
                                      self.supervisor_status)
        self.measurement_baseline_diagnostics = {
            name: self.diagnostics.snapshot(name)
            for name in ("fast_lio/estimator", "fast_lio/transport",
                         "fast_lio/propagated_odometry", "odometry_supervisor")
        }

    def as_topics(self) -> dict[str, Any]:
        return {
            logical: observation.as_dict() for logical, observation in self.ledger.items()
        } | {
            "raw_candidates": list(self.raw_candidates),
            "raw_selected_topic": self.raw_selected_topic,
        }


@dataclass
class ProcessSpec:
    role: str
    argv: list[str]
    cwd: Path
    environment: dict[str, str]
    log_name: str


@dataclass
class ProcessRecord:
    role: str
    pid: int
    pgid: int
    argv: list[str]
    cwd: str
    selected_environment: dict[str, str]
    log_path: str
    start_wall_monotonic_ns: int
    process: subprocess.Popen[str] | None = field(repr=False, default=None)
    start_create_time: float | None = None
    stop_wall_monotonic_ns: int | None = None
    exit_code: int | None = None
    signal_sequence: list[str] = field(default_factory=list)
    cleanup_verified: bool | None = None
    owned_identities: dict[int, float] = field(default_factory=dict, repr=False)

    def as_dict(self) -> dict[str, Any]:
        return {
            "role": self.role,
            "pid": self.pid,
            "pgid": self.pgid,
            "argv": self.argv,
            "cwd": self.cwd,
            "selected_environment": self.selected_environment,
            "log_path": self.log_path,
            "start_wall_monotonic_ns": self.start_wall_monotonic_ns,
            "stop_wall_monotonic_ns": self.stop_wall_monotonic_ns,
            "exit_code": self.exit_code,
            "signal_sequence": list(self.signal_sequence),
            "cleanup_verified": self.cleanup_verified,
        }


class ProcessRegistry:
    """Own processes by isolated PGID and never kill outside that registry."""

    def __init__(self, artifact_dir: Path) -> None:
        self.artifact_dir = artifact_dir
        self.records: dict[str, ProcessRecord] = {}
        self._logs: dict[str, TextIO] = {}

    def launch(self, spec: ProcessSpec) -> ProcessRecord:
        if spec.role in self.records:
            raise RuntimeError(f"duplicate process role: {spec.role}")
        log_path = self.artifact_dir / "logs" / spec.log_name
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stream = log_path.open("w", encoding="utf-8")
        environment = dict(os.environ)
        environment.update(spec.environment)
        try:
            process = subprocess.Popen(
                spec.argv,
                cwd=spec.cwd,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                text=True,
            )
        except BaseException:
            stream.close()
            raise
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            pgid = process.pid
        create_time: float | None = None
        try:
            import psutil
            create_time = float(psutil.Process(process.pid).create_time())
        except Exception:
            pass
        record = ProcessRecord(
            spec.role, process.pid, pgid, list(spec.argv), str(spec.cwd),
            {key: spec.environment[key] for key in sorted(spec.environment)},
            str(log_path), time.monotonic_ns(), process, create_time)
        if create_time is not None:
            record.owned_identities[record.pid] = create_time
        self.records[spec.role] = record
        self._logs[spec.role] = stream
        return record

    def alive(self, role: str) -> bool:
        record = self.records.get(role)
        return record is not None and record.process is not None and record.process.poll() is None

    def update(self) -> None:
        for record in self.records.values():
            for pid in self.group_members(record.role):
                try:
                    import psutil
                    record.owned_identities.setdefault(pid, float(psutil.Process(pid).create_time()))
                except Exception:
                    continue
            if record.process is not None:
                code = record.process.poll()
                if code is not None:
                    record.exit_code = int(code)
                    if record.stop_wall_monotonic_ns is None:
                        record.stop_wall_monotonic_ns = time.monotonic_ns()

    def _identity_matches(self, record: ProcessRecord) -> bool:
        if record.process is None:
            return False
        try:
            import psutil
            if os.getpgid(record.pid) != record.pgid:
                return False
            if record.start_create_time is None:
                return record.process.poll() is None
            return abs(float(psutil.Process(record.pid).create_time()) - record.start_create_time) < 1e-6
        except Exception:
            return False

    def owned_group_members(self, role: str) -> list[int]:
        record = self.records.get(role)
        if record is None:
            return []
        self.update()
        try:
            import psutil
        except ImportError:
            return [record.pid] if self.alive(role) else []
        members: list[int] = []
        for pid in self.group_members(role):
            expected = record.owned_identities.get(pid)
            if expected is None:
                continue
            try:
                if abs(float(psutil.Process(pid).create_time()) - expected) < 1e-6:
                    members.append(pid)
            except Exception:
                continue
        return members

    def group_members(self, role: str) -> list[int]:
        record = self.records.get(role)
        if record is None:
            return []
        try:
            import psutil
        except ImportError:
            return [record.pid] if self.alive(role) else []
        result: list[int] = []
        for process in psutil.process_iter(["pid"]):
            try:
                if os.getpgid(int(process.pid)) == record.pgid:
                    result.append(int(process.pid))
            except (OSError, psutil.Error):
                continue
        return sorted(set(result))

    def _signal(self, record: ProcessRecord, sig: signal.Signals) -> None:
        # If the session leader identity is gone, fail closed.  A reused PGID
        # is never sufficient authority to signal an unrelated process group.
        if not self._identity_matches(record) and not self.owned_group_members(record.role):
            return
        try:
            os.killpg(record.pgid, sig)
            record.signal_sequence.append(sig.name)
        except ProcessLookupError:
            pass
        except PermissionError:
            record.signal_sequence.append(f"{sig.name}:permission_denied")

    def _wait(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.update()
            if all(not self.owned_group_members(role) for role in self.records):
                return
            time.sleep(0.05)

    def cleanup(self, timeout_s: float) -> dict[str, Any]:
        # Reverse dependency order prevents downstream nodes from keeping
        # infrastructure alive during escalation.
        ordered = list(reversed(tuple(self.records)))
        for role in ordered:
            self._signal(self.records[role], signal.SIGINT)
        self._wait(min(8.0, timeout_s / 3.0))
        for role in ordered:
            if self.owned_group_members(role):
                self._signal(self.records[role], signal.SIGTERM)
        self._wait(min(8.0, timeout_s / 3.0))
        for role in ordered:
            if self.owned_group_members(role):
                self._signal(self.records[role], signal.SIGKILL)
        self._wait(max(0.1, timeout_s / 3.0))
        self.update()
        orphan_roles: list[str] = []
        orphan_pgids: list[int] = []
        for role, record in self.records.items():
            members = self.owned_group_members(role)
            record.cleanup_verified = not members
            if members:
                orphan_roles.append(role)
                orphan_pgids.append(record.pgid)
            stream = self._logs.get(role)
            if stream is not None:
                stream.flush()
                stream.close()
        return {
            "complete": not orphan_roles,
            "orphan_roles": orphan_roles,
            "orphan_pgids": orphan_pgids,
            "owned_roles": list(self.records),
        }

    def as_dict(self) -> dict[str, Any]:
        self.update()
        return {role: record.as_dict() for role, record in self.records.items()}


class ResourceSampler:
    """Aggregate every process in each owned PGID at one wall-second cadence."""

    FIELDS = (
        "wall_monotonic_ns", "sim_time_ns", "role", "state", "pgid",
        "process_count", "pids", "cpu_percent", "rss_bytes", "uss_bytes", "thread_count",
    )

    def __init__(self, path: Path, registry: ProcessRegistry) -> None:
        self.path = path
        self.registry = registry
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.last_sample_ns: int | None = None
        self.rows: list[dict[str, Any]] = []

    def sample(self, sim_time_ns: int, force: bool = False) -> None:
        now_ns = time.monotonic_ns()
        if not force and self.last_sample_ns is not None and now_ns - self.last_sample_ns < 1_000_000_000:
            return
        self.last_sample_ns = now_ns
        try:
            import psutil
        except ImportError:
            rows = [{"wall_monotonic_ns": now_ns, "sim_time_ns": sim_time_ns,
                     "role": role, "state": "unavailable", "pgid": record.pgid,
                     "process_count": None, "pids": "", "cpu_percent": None,
                     "rss_bytes": None, "uss_bytes": None, "thread_count": None}
                    for role, record in self.registry.records.items()]
        else:
            rows = []
            for role, record in self.registry.records.items():
                pids = self.registry.owned_group_members(role)
                cpu = rss = uss = threads = 0
                uss_seen = False
                alive = 0
                for pid in pids:
                    try:
                        process = psutil.Process(pid)
                        if process.status() == psutil.STATUS_ZOMBIE:
                            continue
                        alive += 1
                        cpu += float(process.cpu_percent(None))
                        rss += int(process.memory_info().rss)
                        threads += int(process.num_threads())
                        try:
                            uss += int(process.memory_full_info().uss)
                            uss_seen = True
                        except (AttributeError, psutil.Error):
                            pass
                    except (OSError, psutil.Error):
                        continue
                rows.append({
                    "wall_monotonic_ns": now_ns, "sim_time_ns": sim_time_ns,
                    "role": role, "state": "alive" if alive else "dead",
                    "pgid": record.pgid, "process_count": alive,
                    "pids": ",".join(str(pid) for pid in pids),
                    "cpu_percent": cpu if alive else None,
                    "rss_bytes": rss if alive else None,
                    "uss_bytes": uss if alive and uss_seen else None,
                    "thread_count": threads if alive else None,
                })
        for row in rows:
            self.writer.writerow(row)
            self.rows.append(row)
        self.stream.flush()

    def close(self) -> None:
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()


class ArtifactWriter:
    def __init__(self, output: Path, initial: dict[str, Any]) -> None:
        if output.exists():
            raise QualificationFailure(FailureCode.RESOURCE_CONFLICT,
                                        f"artifact output already exists: {output}")
        output.mkdir(parents=True)
        (output / "logs").mkdir()
        self.output = output
        self.events_path = output / "events.jsonl"
        self.events = self.events_path.open("w", encoding="utf-8")
        self.run = dict(initial)
        # Keep the complete canonical file set even when PREFLIGHT fails.
        atomic_write_json(output / "processes.json", {})
        atomic_write_json(output / "topics.json", {})
        atomic_write_json(output / "measurement.json", {})
        atomic_write_json(output / "acceptance.json", {})
        with (output / "resources.csv").open("w", encoding="utf-8") as stream:
            stream.write("wall_monotonic_ns,sim_time_ns,role,state,pgid,process_count,pids,cpu_percent,rss_bytes,uss_bytes,thread_count\n")
        atomic_write_json(output / "run.json", self.run)

    def event(self, event: dict[str, Any]) -> None:
        self.events.write(json.dumps(event, sort_keys=True, default=str) + "\n")
        self.events.flush()
        os.fsync(self.events.fileno())

    def update(self, **fields: Any) -> None:
        self.run.update(fields)
        atomic_write_json(self.output / "run.json", self.run)

    def close(self) -> None:
        self.events.flush()
        os.fsync(self.events.fileno())
        self.events.close()


def _process_command_line(pid: int) -> str:
    try:
        return " ".join(Path(f"/proc/{pid}/cmdline").read_text(errors="replace").split("\0"))
    except OSError:
        return ""


def occupied_udp_port(port: int) -> list[dict[str, Any]]:
    owners: list[dict[str, Any]] = []
    try:
        import psutil
        for connection in psutil.net_connections(kind="udp"):
            if connection.laddr and connection.laddr.port == port:
                owners.append({"pid": connection.pid, "command": _process_command_line(connection.pid) if connection.pid else ""})
    except (ImportError, OSError):
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("0.0.0.0", port))
        finally:
            probe.close()
    except OSError:
        if not owners:
            owners.append({"pid": None, "command": "owner unavailable"})
    return owners


def product_paths_unchanged(workspace: Path, base_sha: str = PRODUCT_BASE_SHA) -> bool:
    result = subprocess.run(
        ["git", "diff", "--quiet", base_sha, "--", *FROZEN_PRODUCT_PATHS],
        cwd=workspace, check=False, capture_output=True,
    )
    return result.returncode == 0


class Preflight:
    def __init__(self, workspace: Path, px4_dir: Path, allow_dirty: bool) -> None:
        self.workspace = workspace
        self.px4_dir = px4_dir
        self.allow_dirty = allow_dirty

    def run(self) -> dict[str, Any]:
        if not self.workspace.is_dir() or not (self.workspace / "install/setup.bash").is_file():
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        "workspace or install/setup.bash is missing", Stage.PREFLIGHT)
        if not command_succeeds(["git", "cat-file", "-e", f"{PRODUCT_BASE_SHA}^{{commit}}"], self.workspace):
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        f"required product base is unavailable: {PRODUCT_BASE_SHA}", Stage.PREFLIGHT)
        frozen = product_paths_unchanged(self.workspace)
        if not frozen:
            raise QualificationFailure(FailureCode.PRODUCT_FREEZE_VIOLATION,
                                        "product paths differ from the required base", Stage.PREFLIGHT)
        git_sha = command_output(["git", "rev-parse", "HEAD"], self.workspace)
        git_status = command_output(["git", "status", "--porcelain"], self.workspace)
        if git_status and not self.allow_dirty:
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        "acceptance run requires a clean workspace", Stage.PREFLIGHT)
        if not self.px4_dir.is_dir():
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        f"PX4 source does not exist: {self.px4_dir}", Stage.PREFLIGHT)
        px4_sha = command_output(["git", "-C", str(self.px4_dir), "rev-parse", "HEAD"])
        px4_status = command_output(["git", "-C", str(self.px4_dir), "status", "--porcelain"])
        if px4_sha != PX4_REQUIRED_SHA or px4_status:
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        "PX4 SHA or clean-state requirement failed", Stage.PREFLIGHT)
        msgs = self.workspace / "src/external/px4_msgs"
        msgs_sha = command_output(["git", "-C", str(msgs), "rev-parse", "HEAD"])
        msgs_status = command_output(["git", "-C", str(msgs), "status", "--porcelain"])
        if msgs_sha != PX4_MSGS_REQUIRED_SHA or msgs_status:
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        "px4_msgs SHA or clean-state requirement failed", Stage.PREFLIGHT)
        required_commands = ["python3", "ros2", "gz", "MicroXRCEAgent"]
        missing = [name for name in required_commands if shutil.which(name) is None]
        if missing:
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        f"required commands are missing: {', '.join(missing)}", Stage.PREFLIGHT)
        required_paths = (
            self.workspace / "tools/simulation/run_px4_mid360.sh",
            self.workspace / "src/uav_simulation/bridge/px4_mid360_bridge.yaml",
            self.workspace / "src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml",
            self.px4_dir / "build/px4_sitl_default/bin/px4",
        )
        missing_paths = [str(path) for path in required_paths if not path.exists()]
        if missing_paths:
            raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                        f"required paths are missing: {missing_paths}", Stage.PREFLIGHT)
        owners = occupied_udp_port(8888)
        if owners:
            raise QualificationFailure(FailureCode.RESOURCE_CONFLICT,
                                        f"UDP 8888 is occupied: {owners}", Stage.PREFLIGHT)
        conflict_patterns = {
            "xrce_agent": "MicroXRCEAgent",
            "px4_gazebo": "px4_sitl_default",
            "gazebo": "gz sim",
            "ros_gz_bridge": "ros_gz_bridge",
            "px4_odometry_bridge": "px4_odometry_bridge_node",
            "fast_lio": "fast_lio_node",
            "supervisor": "odometry_supervisor_node",
        }
        conflicts: list[dict[str, Any]] = []
        try:
            import psutil
            for process in psutil.process_iter(["pid", "cmdline"]):
                if int(process.pid) == os.getpid():
                    continue
                command = " ".join(process.info.get("cmdline") or [])
                for role, pattern in conflict_patterns.items():
                    if pattern in command:
                        conflicts.append({"role": role, "pid": int(process.pid), "command": command})
        except (ImportError, OSError):
            pass
        if conflicts:
            raise QualificationFailure(FailureCode.RESOURCE_CONFLICT,
                                        f"external SITL process conflict: {conflicts}", Stage.PREFLIGHT)
        return {
            "workspace": str(self.workspace),
            "product_base_sha": PRODUCT_BASE_SHA,
            "qualification_harness_sha": git_sha,
            "workspace_dirty": bool(git_status),
            "product_paths_unchanged_from_base": frozen,
            "acceptance_eligible": not bool(git_status),
            "px4": {"path": str(self.px4_dir), "sha": px4_sha, "dirty": bool(px4_status)},
            "px4_msgs": {"sha": msgs_sha, "dirty": bool(msgs_status)},
            "config_hashes": {
                "bridge": file_sha256(self.workspace / "src/uav_simulation/bridge/px4_mid360_bridge.yaml"),
                "fast_lio": file_sha256(self.workspace / "src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml"),
            },
            "host": {"hostname": platform.node(), "platform": platform.platform(),
                     "python": sys.version, "environment": {
                         key: os.environ[key] for key in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID")
                         if key in os.environ}},
        }


class SitlOrchestrator:
    def __init__(self, workspace: Path, px4_dir: Path, output: Path,
                 policy: ModePolicy, allow_dirty: bool = False) -> None:
        self.workspace = workspace.resolve()
        self.px4_dir = px4_dir.resolve()
        self.output = output.resolve()
        self.policy = policy
        self.allow_dirty = allow_dirty
        self.machine = StageMachine()
        self.registry = ProcessRegistry(self.output)
        self.artifact: ArtifactWriter | None = None
        self.monitor: RosReadinessMonitor | None = None
        self.resources: ResourceSampler | None = None
        self.provenance: dict[str, Any] = {}
        self.stage_records: list[dict[str, Any]] = []
        self.current_stage_record: dict[str, Any] | None = None
        self.primary_failure: dict[str, Any] | None = None
        self.cleanup_result: dict[str, Any] = {"complete": False}
        self.measurement_result: dict[str, Any] | None = None
        self.measurement_rows: list[dict[str, Any]] = []
        self.final_topics: dict[str, Any] = {}
        self.ros_initialized = False

    def _persist(self) -> None:
        if self.artifact is None:
            return
        self.artifact.update(
            stage_timeline=self.stage_records,
            processes=self.registry.as_dict(),
            topics=self.monitor.as_topics() if self.monitor else {},
            measurement=self.measurement_result,
            failure=self.primary_failure,
            cleanup=self.cleanup_result,
        )

    def _begin(self, stage: Stage) -> None:
        self.machine.transition(stage)
        now = time.monotonic_ns()
        sim = self.monitor.latest_clock_ns if self.monitor and self.monitor.latest_clock_ns else None
        record = {
            "stage": stage.value,
            "entered_wall_monotonic_ns": now,
            "exited_wall_monotonic_ns": None,
            "duration_wall_s": None,
            "entered_sim_time_ns": sim,
            "exited_sim_time_ns": None,
            "timeout_s": STAGE_POLICIES.get(stage, StagePolicy(0, FailureCode.PROVENANCE_INVALID)).timeout_s,
            "readiness_evidence": {}, "result": "RUNNING", "failure_code": None,
        }
        self.stage_records.append(record)
        self.current_stage_record = record
        if self.artifact:
            self.artifact.event({"event": "stage_enter", "stage": stage.value,
                                 "wall_monotonic_ns": now, "sim_time_ns": sim})
            self._persist()

    def _end(self, result: str = "PASS", evidence: dict[str, Any] | None = None,
             failure: FailureCode | None = None) -> None:
        if self.current_stage_record is None:
            return
        now = time.monotonic_ns()
        record = self.current_stage_record
        record["exited_wall_monotonic_ns"] = now
        record["duration_wall_s"] = (now - record["entered_wall_monotonic_ns"]) / 1e9
        record["exited_sim_time_ns"] = self.monitor.latest_clock_ns if self.monitor and self.monitor.latest_clock_ns else None
        record["readiness_evidence"] = evidence or {}
        record["result"] = result
        record["failure_code"] = failure.value if failure else None
        if self.artifact:
            self.artifact.event({"event": "stage_exit", "stage": record["stage"],
                                 "wall_monotonic_ns": now, "sim_time_ns": record["exited_sim_time_ns"],
                                 "result": result, "failure_code": failure.value if failure else None})
            self._persist()
        self.current_stage_record = None

    def _fail(self, failure: QualificationFailure) -> None:
        if self.current_stage_record is not None:
            self._end("FAIL", self._failure_evidence(failure), failure.code)
        try:
            self.machine.transition(Stage.FAILED)
        except ValueError:
            pass
        self.primary_failure = {
            "code": failure.code.value,
            "stage": (failure.stage or self.machine.current).value if (failure.stage or self.machine.current) else None,
            "summary": failure.summary,
            "wall_timestamp_ns": time.time_ns(),
            "sim_timestamp_ns": self.monitor.latest_clock_ns if self.monitor else None,
            "process_states": self.registry.as_dict(),
            "topic_observations": self.monitor.as_topics() if self.monitor else {},
            "latest_relevant_diagnostics": self._relevant_diagnostics(),
            "log_tails": self._log_tails(),
        }
        self._persist()

    def _failure_evidence(self, failure: QualificationFailure) -> dict[str, Any]:
        return {"summary": failure.summary, "code": failure.code.value}

    def _relevant_diagnostics(self) -> dict[str, Any]:
        if self.monitor is None:
            return {}
        names = ("px4_odometry_bridge", "fast_lio/estimator", "fast_lio/transport",
                 "fast_lio/propagated_odometry", "odometry_supervisor")
        return {name: self.monitor.diagnostics.snapshot(name) for name in names}

    def _log_tails(self, limit: int = 80) -> dict[str, list[str]]:
        result: dict[str, list[str]] = {}
        for role, record in self.registry.records.items():
            try:
                lines = Path(record.log_path).read_text(encoding="utf-8", errors="replace").splitlines()
                result[role] = lines[-limit:]
            except OSError:
                result[role] = []
        return result

    def _spin_once(self) -> None:
        if self.monitor is not None:
            self.monitor.refresh_topic_graph()
            rclpy.spin_once(self.monitor, timeout_sec=0.05)  # type: ignore[union-attr]
        self.registry.update()
        if self.resources is not None:
            self.resources.sample(self.monitor.latest_clock_ns if self.monitor else 0)

    def _wait(self, stage: Stage, predicate: Callable[[], bool],
              evidence: Callable[[], dict[str, Any]], process_roles: Iterable[str] = ()) -> None:
        self._begin(stage)
        policy = STAGE_POLICIES[stage]
        deadline = time.monotonic() + policy.timeout_s
        roles = tuple(process_roles)
        while time.monotonic() < deadline:
            self._spin_once()
            for role in roles:
                if not self.registry.alive(role):
                    code = {
                        "xrce_agent": FailureCode.XRCE_PORT_NOT_BOUND,
                        "px4_gazebo": FailureCode.PX4_GAZEBO_EXITED,
                        "ros_gz_bridge": FailureCode.ROS_BRIDGE_EXITED,
                        "px4_odometry_bridge": FailureCode.PX4_INGRESS_EXITED,
                        "fast_lio": FailureCode.FAST_LIO_EXITED,
                        "supervisor": FailureCode.SUPERVISOR_EXITED,
                    }.get(role, policy.timeout_failure)
                    raise QualificationFailure(code, f"owned process exited: {role}", stage)
            if predicate():
                values = evidence()
                self._end("PASS", values)
                return
        values = evidence()
        if stage is Stage.RAW_PX4_ODOMETRY_READY:
            if not self.monitor.raw_candidates:
                raise QualificationFailure(FailureCode.RAW_ODOMETRY_TOPIC_MISSING,
                                            f"raw VehicleOdometry topic was not discovered: {values}", stage)
            raw_observation = self.monitor.topic("raw_px4_odometry")
            if raw_observation is not None:
                classification = classify_topic_observation(raw_observation)
                if classification is not None:
                    raise QualificationFailure(classification,
                                                f"raw VehicleOdometry readiness failed: {values}", stage)
        if stage is Stage.SENSORS_READY and self.monitor:
            for logical in ("imu", "lidar"):
                observation = self.monitor.topic(logical)
                if observation is not None:
                    classification = classify_topic_observation(observation)
                    if classification is not None:
                        raise QualificationFailure(classification,
                                                    f"sensor readiness failed for {logical}: {values}", stage)
        if stage is Stage.PX4_INGRESS_READY and self.monitor:
            bridge_values = self.monitor.diagnostics_values("px4_odometry_bridge")
            if bridge_values and bridge_values.get("state") not in {"running", "stabilizing"}:
                raise QualificationFailure(FailureCode.PX4_INGRESS_REJECTING,
                                            f"PX4 ingress diagnostics are not running: {values}", stage)
        if (self.monitor and self.monitor.topic("clock") and
                self.monitor.topic("clock").publisher_count <= 0 and
                stage in {Stage.ROS_CLOCK_READY, Stage.WARMUP, Stage.MEASURING}):
            raise QualificationFailure(FailureCode.ROS_CLOCK_PUBLISHER_MISSING,
                                        f"/clock publisher missing during {stage.value}: {values}", stage)
        if stage is Stage.ROS_CLOCK_READY and self.monitor:
            clock = self.monitor.topic("clock")
            if clock and clock.publisher_count > 0 and clock.message_count == 0:
                raise QualificationFailure(FailureCode.ROS_CLOCK_SAMPLE_MISSING,
                                            f"/clock publisher has emitted no sample: {values}", stage)
        raise QualificationFailure(policy.timeout_failure,
                                    f"stage timeout at {stage.value}: {values}", stage)

    def _launch(self, role: str, argv: list[str], environment: dict[str, str] | None = None) -> None:
        self.registry.launch(ProcessSpec(role, argv, self.workspace, environment or {}, f"{role}.log"))

    def _port_bound_by_agent(self) -> bool:
        record = self.registry.records.get("xrce_agent")
        if record is None or not self.registry.alive("xrce_agent"):
            return False
        try:
            import psutil
            for connection in psutil.net_connections(kind="udp"):
                if connection.laddr and connection.laddr.port == 8888 and connection.pid:
                    try:
                        if os.getpgid(connection.pid) == record.pgid:
                            return True
                    except OSError:
                        continue
        except (ImportError, OSError):
            return False
        return False

    def _gazebo_clock_present(self) -> bool:
        topics = command_output(["gz", "topic", "-l"], timeout=5.0).splitlines()
        return GAZEBO_CLOCK_TOPIC in topics

    def _session_marker(self) -> bool:
        record = self.registry.records.get("xrce_agent")
        if record is None:
            return False
        try:
            text = Path(record.log_path).read_text(encoding="utf-8", errors="replace").lower()
        except OSError:
            return False
        return any(marker in text for marker in (
            "session established", "client connected", "create session", "session created"))

    def _bridge_ready(self) -> bool:
        values = self.monitor.diagnostics_values("px4_odometry_bridge") if self.monitor else {}
        return (
            self.registry.alive("px4_odometry_bridge") and
            values.get("state") == "running" and
            parse_bool(values.get("output_valid")) is True and
            parse_bool(values.get("continuity_valid")) is True and
            parse_int(values.get("last_valid_sample_time_ns")) not in (None, 0) and
            self.monitor is not None and self.monitor.topic_ready("px4_odometry_ros") and
            (self.monitor.topic("px4_odometry_ros").last_ros_stamp_ns or 0) > 0 and
            values.get("output_frame") == "odom" and values.get("output_child_frame") == "base_link"
        )

    def _fast_lio_ready(self) -> bool:
        values = self.monitor.diagnostics_values("fast_lio/estimator") if self.monitor else {}
        return (
            self.registry.alive("fast_lio") and values.get("status") in {"TRACKING", "Tracking"} and
            parse_bool(values.get("navigation_valid")) is True and
            parse_bool(values.get("corrected_estimate_valid")) is True and
            self.monitor is not None and self.monitor.topic_ready("lio_corrected") and
            self.monitor.topic_ready("lio_propagated") and
            (self.monitor.topic("lio_corrected").last_ros_stamp_ns or 0) > 0 and
            (self.monitor.topic("lio_propagated").last_ros_stamp_ns or 0) > 0 and
            self.monitor.topic("lio_corrected").latest_payload_summary is not None and
            self.monitor.topic("lio_propagated").latest_payload_summary is not None and
            self.monitor.topic("lio_corrected").latest_payload_summary.get("frame_id") == "odom" and
            self.monitor.topic("lio_corrected").latest_payload_summary.get("child_frame_id") == "base_link" and
            self.monitor.topic("lio_propagated").latest_payload_summary.get("frame_id") == "odom" and
            self.monitor.topic("lio_propagated").latest_payload_summary.get("child_frame_id") == "base_link"
        )

    def _prior_ready(self) -> bool:
        values = self.monitor.diagnostics_values("fast_lio/estimator") if self.monitor else {}
        if parse_bool(values.get("initial_prior_fallback_applied")) is True:
            raise QualificationFailure(FailureCode.INITIAL_PRIOR_FALLBACK_USED,
                                        "FAST-LIO initial prior fallback was applied", Stage.PRIOR_ACCEPTED)
        return (
            values.get("initial_prior_status") in {"applied", "closed"} and
            values.get("initial_prior_source") == "topic" and
            parse_bool(values.get("initial_prior_applied")) is True and
            parse_bool(values.get("initial_prior_fallback_applied")) is False and
            parse_int(values.get("initial_prior_zero_fallback_count")) == 0 and
            values.get("initial_prior_reason") == "TOPIC_PRIOR_ACCEPTED"
        )

    def _supervisor_ready(self) -> bool:
        values = self.monitor.diagnostics_values("odometry_supervisor") if self.monitor else {}
        return (
            self.registry.alive("supervisor") and
            self.monitor is not None and self.monitor.supervisor_status is not None and
            self.monitor.supervisor_status_count > 0 and bool(values) and
            int_value(self.monitor.supervisor_status.health) != 0
        )

    def _measurement_summary(self) -> dict[str, Any]:
        assert self.monitor is not None
        accumulator = self.monitor.status_accumulator
        result: dict[str, Any] = {
            "schema_version": 2,
            "mode": self.policy.mode.value,
            "warmup_s": self.policy.warmup_sim_s,
            "measurement_s": self.policy.measurement_sim_s,
            "supervisor_metrics_applicable": self.policy.supervisor_enabled,
            "baseline_diagnostics": {
                **self.monitor.measurement_baseline_diagnostics,
            },
            "final_diagnostics": self._relevant_diagnostics(),
            "topic_summary": self.monitor.as_topics(),
            "resource_sample_count": len(self.resources.rows) if self.resources else 0,
        }
        if accumulator is None:
            result.update({"status_event_count": None, "acceptance": {"measurement": False}})
            return result
        result.update(accumulator.finish(self.monitor.latest_clock_ns))
        estimator = self.monitor.diagnostics_values("fast_lio/estimator")
        transport = self.monitor.diagnostics_values("fast_lio/transport")
        propagated = self.monitor.diagnostics_values("fast_lio/propagated_odometry")
        metrics = {
            "status": estimator.get("status"),
            "navigation_valid": parse_bool(estimator.get("navigation_valid")),
            "corrected_estimate_valid": parse_bool(estimator.get("corrected_estimate_valid")),
            "p95_corrected_scan_end_to_end_us": parse_int(transport.get("p95_corrected_scan_end_to_end_us")),
            "maximum_queue_depth": parse_int(transport.get("maximum_queue_depth")),
            "processing_lag_exceeded": parse_bool(transport.get("processing_lag_exceeded")),
            "load_shedding_count": parse_int(propagated.get("load_shedding_count")),
            "imu_drop_count": parse_int(transport.get("imu_drop_count")),
            "lidar_drop_count": parse_int(transport.get("lidar_drop_count")),
            "overflow_detected": parse_bool(transport.get("overflow_detected")),
            "invalid_timestamp_rejected_count": parse_int(transport.get("invalid_timestamp_rejected_count")),
            "current_input_queue_depth": parse_int(transport.get("current_input_queue_depth")),
            "current_imu_queue_depth": parse_int(transport.get("current_imu_queue_depth")),
            "current_lidar_queue_depth": parse_int(transport.get("current_lidar_queue_depth")),
        }
        result["fast_lio_metrics"] = metrics
        if self.policy.mode is RunMode.MEMORY_ON and self.monitor.status_accumulator.start_ns is not None:
            start_ns = self.monitor.status_accumulator.start_ns
            end_ns = self.monitor.status_accumulator.end_ns or self.monitor.latest_clock_ns
            memory_rows = [row for row in (self.resources.rows if self.resources else [])
                           if start_ns <= int(row.get("sim_time_ns") or 0) <= end_ns]
            try:
                result["memory"] = robust_rss_growth(memory_rows)
            except ValueError as error:
                result["memory"] = {"pass": False, "error": str(error)}
        checks = {
            "status_events_present": result["status_event_count"] > 0,
            "healthy_histogram": result.get("health_histogram", {}) == {"1": result["status_event_count"]},
            "comparison_valid": result.get("comparison_valid_ratio") is not None and result["comparison_valid_ratio"] >= 0.99,
            "monitoring_available": result.get("monitoring_available_ratio") is not None and result["monitoring_available_ratio"] >= 0.99,
            "query_timeout_delta_zero": result.get("query_timeout_count_delta") == 0,
            "query_generation_mismatch_delta_zero": result.get("query_generation_mismatch_count_delta") == 0,
            "state_transition_delta_zero": result.get("state_transition_count_delta") == 0,
            "reinitialization_delta_zero": result.get("reinitialization_request_sequence_delta") == 0,
            "heading_observable": result.get("heading_observable_ratio") == 1.0,
            "outstanding_query_bound": result.get("maximum_outstanding_queries", 2) <= 1,
        }
        if not self.policy.supervisor_enabled:
            result.update({
                "comparison_valid_ratio": None,
                "monitoring_available_ratio": None,
                "query_timeout_count_delta": None,
                "query_generation_mismatch_count_delta": None,
                "supervisor_metrics_applicable": False,
            })
            checks.clear()
        for key in ("navigation_valid", "corrected_estimate_valid", "p95_corrected_scan_end_to_end_us",
                    "maximum_queue_depth", "processing_lag_exceeded", "load_shedding_count",
                    "imu_drop_count", "lidar_drop_count", "overflow_detected",
                    "invalid_timestamp_rejected_count"):
            value = metrics[key]
            checks[f"fast_lio_{key}"] = value is not None and (
                value is True if key in {"navigation_valid", "corrected_estimate_valid"} else
                (value is False if key == "processing_lag_exceeded" else
                 (value is False if key == "overflow_detected" else value == 0 if key in {
                     "load_shedding_count", "imu_drop_count", "lidar_drop_count",
                     "invalid_timestamp_rejected_count"} else value is not None)))
        result["acceptance"] = checks
        if self.policy.mode is RunMode.MEMORY_ON:
            checks["memory_rss_growth"] = result.get("memory", {}).get("pass", False)
        result["pass"] = all(checks.values())
        return result

    def _run_sequence(self) -> None:
        assert self.artifact is not None
        assert self.monitor is not None
        self._launch("xrce_agent", ["MicroXRCEAgent", "udp4", "-p", "8888"])
        self._wait(Stage.XRCE_AGENT_STARTING,
                   lambda: self.registry.alive("xrce_agent"),
                   lambda: {"alive": self.registry.alive("xrce_agent")}, ("xrce_agent",))
        self._wait(Stage.XRCE_AGENT_READY, self._port_bound_by_agent,
                   lambda: {"udp_8888_owned": self._port_bound_by_agent()}, ("xrce_agent",))
        self._launch("px4_gazebo", [str(self.workspace / "tools/simulation/run_px4_mid360.sh")], {
            "PX4_DIR": str(self.px4_dir), "PX4_REQUIRED_GIT_SHA": PX4_REQUIRED_SHA,
            "GZ_GUI": "0", "SESSION_DIR": str(self.output),
        })
        self._wait(Stage.PX4_GAZEBO_STARTING, lambda: self.registry.alive("px4_gazebo"),
                   lambda: {"alive": self.registry.alive("px4_gazebo")}, ("px4_gazebo",))
        self._wait(Stage.GAZEBO_CLOCK_READY, self._gazebo_clock_present,
                   lambda: {"topic": GAZEBO_CLOCK_TOPIC, "present": self._gazebo_clock_present()}, ("px4_gazebo",))
        bridge_config = self.workspace / "src/uav_simulation/bridge/px4_mid360_bridge.yaml"
        self._launch("ros_gz_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                                       "-r", "__node:=px4_mid360_bridge", "-p", f"config_file:={bridge_config}",
                                       "-p", "use_sim_time:=false"])
        self._wait(Stage.ROS_BRIDGE_STARTING, lambda: self.registry.alive("ros_gz_bridge"),
                   lambda: {"alive": self.registry.alive("ros_gz_bridge"), "use_sim_time": False}, ("ros_gz_bridge",))
        self._wait(Stage.ROS_CLOCK_READY,
                   lambda: self.monitor.topic_ready("clock") and self.monitor.ros_clock_advancing(),
                   lambda: {"publisher_count": self.monitor.topic("clock").publisher_count,
                             "samples": self.monitor.topic("clock").message_count,
                             "advancing": self.monitor.ros_clock_advancing()}, ("ros_gz_bridge",))
        self._wait(Stage.XRCE_SESSION_READY, self._session_marker,
                   lambda: {"agent_log_marker": self._session_marker()}, ("xrce_agent", "px4_gazebo"))
        self._wait(Stage.RAW_PX4_ODOMETRY_READY,
                   lambda: bool(self.monitor.subscribe_raw_topic()) and self.monitor.topic_ready("raw_px4_odometry"),
                   lambda: {"candidates": self.monitor.raw_candidates,
                             "selected": self.monitor.raw_selected_topic,
                             "observation": self.monitor.topic("raw_px4_odometry").as_dict() if self.monitor.topic("raw_px4_odometry") else None},
                   ("xrce_agent", "px4_gazebo", "ros_gz_bridge"))
        self._launch("px4_odometry_bridge", ["ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node",
                                               "--ros-args", "-p", "use_sim_time:=true", "-p", "simulation_clock:=true"])
        self._wait(Stage.PX4_INGRESS_STARTING, lambda: self.registry.alive("px4_odometry_bridge"),
                   lambda: {"alive": self.registry.alive("px4_odometry_bridge")}, ("px4_odometry_bridge",))
        self._wait(Stage.PX4_INGRESS_READY, self._bridge_ready,
                   lambda: {"diagnostics": self.monitor.diagnostics.snapshot("px4_odometry_bridge"),
                             "output": self.monitor.topic("px4_odometry_ros").as_dict()},
                   ("px4_odometry_bridge", "ros_gz_bridge", "px4_gazebo"))
        if self.policy.stop_after is Stage.PX4_INGRESS_READY:
            return
        self._wait(Stage.SENSORS_READY,
                   lambda: self.monitor.topic_ready("imu", 2) and self.monitor.topic_ready("lidar", 1) and
                   (self.monitor.topic("imu").last_ros_stamp_ns or 0) > (self.monitor.topic("imu").first_ros_stamp_ns or 0) and
                   (self.monitor.topic("lidar").last_ros_stamp_ns or 0) >= (self.monitor.topic("lidar").first_ros_stamp_ns or 0),
                   lambda: {name: self.monitor.topic(name).as_dict() for name in ("imu", "lidar")},
                   ("ros_gz_bridge", "px4_gazebo", "px4_odometry_bridge"))
        lio_config = self.workspace / "src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml"
        self._launch("fast_lio", ["ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
                                   f"config_file:={lio_config}", "use_sim_time:=true",
                                   "livox_mount_xyz:=0 0 0.28", "livox_mount_rpy:=0 0 0"])
        self._wait(Stage.FAST_LIO_STARTING, lambda: self.registry.alive("fast_lio"),
                   lambda: {"alive": self.registry.alive("fast_lio")}, ("fast_lio",))
        self._wait(Stage.FAST_LIO_READY, self._fast_lio_ready,
                   lambda: {"diagnostics": self.monitor.diagnostics.snapshot("fast_lio/estimator"),
                             "corrected": self.monitor.topic("lio_corrected").as_dict(),
                             "propagated": self.monitor.topic("lio_propagated").as_dict()},
                   ("fast_lio", "px4_odometry_bridge", "ros_gz_bridge", "px4_gazebo"))
        self._wait(Stage.PRIOR_ACCEPTED, self._prior_ready,
                   lambda: {"diagnostics": self.monitor.diagnostics.snapshot("fast_lio/estimator")}, ("fast_lio",))
        if self.policy.supervisor_enabled:
            self._launch("supervisor", ["ros2", "launch", "odometry_supervisor", "odometry_supervisor.launch.py",
                                         "use_sim_time:=true"])
            self._wait(Stage.SUPERVISOR_STARTING, lambda: self.registry.alive("supervisor"),
                       lambda: {"alive": self.registry.alive("supervisor")}, ("supervisor",))
            self._wait(Stage.SUPERVISOR_READY, self._supervisor_ready,
                       lambda: {"status_count": self.monitor.supervisor_status_count,
                                 "diagnostics": self.monitor.diagnostics.snapshot("odometry_supervisor")},
                       ("supervisor", "fast_lio", "px4_odometry_bridge"))
        self._warmup_end_ns = self.monitor.latest_clock_ns + int((self.policy.warmup_sim_s or 0.0) * 1_000_000_000)
        self._wait(Stage.WARMUP,
                   lambda: self.monitor.latest_clock_ns >= self._warmup_end_ns,
                   lambda: {"clock_ns": self.monitor.latest_clock_ns, "target_ns": self._warmup_end_ns},
                   ("fast_lio", "px4_odometry_bridge", "ros_gz_bridge", "px4_gazebo") +
                   (("supervisor",) if self.policy.supervisor_enabled else ()))
        self.monitor.start_measurement(self.policy.measurement_sim_s or 0.0)
        self._wait(Stage.MEASURING,
                   lambda: self.monitor.latest_clock_ns >= (self.monitor.measurement_end_ns or 0),
                   lambda: {"clock_ns": self.monitor.latest_clock_ns,
                             "target_ns": self.monitor.measurement_end_ns},
                   ("fast_lio", "px4_odometry_bridge", "ros_gz_bridge", "px4_gazebo") +
                   (("supervisor",) if self.policy.supervisor_enabled else ()))
        self.measurement_result = self._measurement_summary()
        atomic_write_json(self.output / "measurement.json", self.measurement_result)
        atomic_write_json(self.output / "acceptance.json", self.measurement_result.get("acceptance", {}))
        if not self.measurement_result.get("pass", False):
            raise QualificationFailure(FailureCode.MEASUREMENT_CONTRACT_FAILED,
                                        f"measurement contract failed: {self.measurement_result.get('acceptance')}", Stage.MEASURING)

    def run(self) -> int:
        initial = {
            "schema_version": 3,
            "run_id": self.output.name,
            "mode": self.policy.mode.value,
            "outcome": "RUNNING",
            "acceptance_eligible": False,
            "product_base_sha": PRODUCT_BASE_SHA,
            "qualification_harness_sha": None,
            "product_paths_unchanged_from_base": None,
            "workspace": str(self.workspace),
            "failure": None,
            "stage_timeline": [],
            "processes": {}, "topics": {}, "measurement": None,
            "cleanup": {"complete": False},
            "artifact_paths": {
                "run": str(self.output / "run.json"),
                "events": str(self.output / "events.jsonl"),
                "processes": str(self.output / "processes.json"),
                "topics": str(self.output / "topics.json"),
                "resources": str(self.output / "resources.csv"),
                "measurement": str(self.output / "measurement.json"),
                "acceptance": str(self.output / "acceptance.json"),
                "logs": str(self.output / "logs"),
            },
        }
        try:
            self.artifact = ArtifactWriter(self.output, initial)
        except QualificationFailure as failure:
            return 2
        try:
            self._begin(Stage.PREFLIGHT)
            self.provenance = Preflight(self.workspace, self.px4_dir, self.allow_dirty).run()
            self._end("PASS", self.provenance)
            self.artifact.update(**self.provenance)
            self.resources = ResourceSampler(self.output / "resources.csv", self.registry)
            if rclpy is None:
                raise QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                            f"ROS imports unavailable: {ROS_IMPORT_ERROR}", Stage.PREFLIGHT)
            rclpy.init(args=[])  # type: ignore[union-attr]
            self.ros_initialized = True
            self.monitor = RosReadinessMonitor()
            self._run_sequence_with_boundary()
            self._begin(Stage.COMPLETE)
            self._end("PASS", {"measurement_pass": self.measurement_result.get("pass") if self.measurement_result else None})
        except QualificationFailure as failure:
            self._fail(failure)
        except KeyboardInterrupt:
            self._fail(QualificationFailure(FailureCode.MEASUREMENT_CONTRACT_FAILED,
                                             "qualification interrupted", self.machine.current))
        except Exception as error:  # artifact must survive unexpected harness defects.
            self._fail(QualificationFailure(FailureCode.PROVENANCE_INVALID,
                                             f"orchestrator exception: {type(error).__name__}: {error}", self.machine.current))
        finally:
            if self.monitor is not None:
                # Destroying the participant before process cleanup prevents a
                # late callback from changing final evidence.
                self.final_topics = self.monitor.as_topics()
                self.monitor.destroy_node()
                try:
                    rclpy.shutdown()  # type: ignore[union-attr]
                except Exception as error:
                    if "already called" not in str(error):
                        raise
                self.monitor = None
            elif self.ros_initialized:
                try:
                    rclpy.shutdown()  # type: ignore[union-attr]
                except Exception as error:
                    if "already called" not in str(error):
                        raise
            try:
                self.machine.transition(Stage.CLEANUP)
            except ValueError:
                self.machine.current = Stage.CLEANUP
            cleanup_entered_ns = time.monotonic_ns()
            cleanup_sim_ns = self.monitor.latest_clock_ns if self.monitor else None
            cleanup_record = {
                "stage": Stage.CLEANUP.value,
                "entered_wall_monotonic_ns": cleanup_entered_ns,
                "timeout_s": STAGE_POLICIES[Stage.CLEANUP].timeout_s,
            }
            self.cleanup_result = self.registry.cleanup(STAGE_POLICIES[Stage.CLEANUP].timeout_s)
            cleanup_exited_ns = time.monotonic_ns()
            cleanup_record.update({
                "exited_wall_monotonic_ns": cleanup_exited_ns,
                "duration_wall_s": (cleanup_exited_ns - cleanup_entered_ns) / 1e9,
                "entered_sim_time_ns": cleanup_sim_ns, "exited_sim_time_ns": cleanup_sim_ns,
                "readiness_evidence": self.cleanup_result,
                "result": "PASS" if self.cleanup_result["complete"] else "FAIL",
                "failure_code": None if self.cleanup_result["complete"] else FailureCode.CLEANUP_INCOMPLETE.value,
            })
            self.stage_records.append(cleanup_record)
            if not self.cleanup_result["complete"] and self.primary_failure is None:
                self.primary_failure = {
                    "code": FailureCode.CLEANUP_INCOMPLETE.value,
                    "stage": Stage.CLEANUP.value,
                    "summary": "owned process groups remain after escalation",
                    "wall_timestamp_ns": time.time_ns(), "sim_timestamp_ns": None,
                    "process_states": self.registry.as_dict(), "topic_observations": {},
                    "latest_relevant_diagnostics": {}, "log_tails": self._log_tails(),
                }
            if self.resources is not None:
                self.resources.sample(0, force=True)
                self.resources.close()
            atomic_write_json(self.output / "processes.json", self.registry.as_dict())
            atomic_write_json(self.output / "topics.json", self.final_topics)
            outcome = "PASS" if self.primary_failure is None and self.cleanup_result["complete"] else "FAIL"
            self.artifact.update(outcome=outcome, acceptance_eligible=bool(self.provenance.get("acceptance_eligible", False)) and outcome == "PASS",
                                 cleanup=self.cleanup_result, stage_timeline=self.stage_records,
                                 processes=self.registry.as_dict(), topics=self.final_topics, failure=self.primary_failure,
                                 measurement=self.measurement_result)
            self.artifact.close()
        return 0 if self.primary_failure is None and self.cleanup_result.get("complete", False) else 2

    def _run_sequence_with_boundary(self) -> None:
        """Run sequence while deriving warm-up boundary from ROS clock."""
        # _run_sequence reaches WARMUP only after ROS_CLOCK_READY.  Its first
        # invocation has no boundary; use a small closure around the method's
        # existing sequence by setting the boundary after the clock is first
        # observed in the pre-launch participant.
        self._warmup_end_ns = 0
        self._run_sequence()


def robust_rss_growth(rows: Iterable[dict[str, Any]], role: str = "supervisor",
                      window_fraction: float = 0.10, minimum_samples: int = 3) -> dict[str, Any]:
    values = [int(row["rss_bytes"]) for row in rows
              if row.get("role") == role and row.get("rss_bytes") not in (None, "", 0)]
    count = max(minimum_samples, int(len(values) * window_fraction))
    if len(values) < count * 2:
        raise ValueError("insufficient resource samples for first/last RSS windows")
    first = values[:count]
    last = values[-count:]
    median = lambda items: sorted(items)[len(items) // 2] if len(items) % 2 else (sorted(items)[len(items) // 2 - 1] + sorted(items)[len(items) // 2]) / 2
    first_median = median(first)
    last_median = median(last)
    return {
        "sample_count": len(values), "window_sample_count": count,
        "first_window_rss_median": first_median, "last_window_rss_median": last_median,
        "rss_growth_ratio": (last_median / first_median - 1.0) if first_median else None,
        "peak_rss_bytes": max(values), "first_window_values": first, "last_window_values": last,
        "pass": bool(first_median) and (last_median / first_median - 1.0) < 0.10,
    }


def run_series(args: argparse.Namespace) -> int:
    if args.mode != RunMode.SMOKE_ON.value or args.repeat < 2:
        raise SystemExit("--repeat is supported for smoke-on and must be at least 2")
    root = args.output.resolve()
    if root.exists():
        raise SystemExit(f"series output already exists: {root}")
    root.mkdir(parents=True)
    children: list[dict[str, Any]] = []
    streak = 0
    first_failure: dict[str, Any] | None = None
    for index in range(1, args.repeat + 1):
        child = root / f"run-{index:02d}"
        code = SitlOrchestrator(args.workspace, args.px4_dir, child,
                                MODE_POLICIES[RunMode.SMOKE_ON], args.allow_dirty).run()
        payload = json.loads((child / "run.json").read_text(encoding="utf-8"))
        passed = code == 0 and payload.get("outcome") == "PASS"
        streak = streak + 1 if passed else 0
        if not passed and first_failure is None:
            first_failure = {"index": index, "run": str(child), "failure": payload.get("failure")}
        children.append({"index": index, "run": str(child), "outcome": payload.get("outcome"), "passed": passed})
    result = {"schema_version": 1, "mode": RunMode.SMOKE_ON.value, "ordered_child_runs": children,
              "consecutive_streak": streak, "first_failure": first_failure,
              "gate_pass": args.repeat >= 3 and streak >= 3}
    atomic_write_json(root / "series.json", result)
    return 0 if result["gate_pass"] else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=[mode.value for mode in RunMode])
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--px4-dir", type=Path,
                        default=Path.home() / "Dev/Autopilot-p0.7-v1.17")
    parser.add_argument("--output", type=Path,
                        default=None, help="fresh canonical artifact directory")
    parser.add_argument("--allow-dirty", action="store_true",
                        help="debug only; records acceptance_eligible=false")
    parser.add_argument("--repeat", type=int, default=1,
                        help="repeat smoke-on in this entrypoint and write a series artifact")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.workspace = args.workspace.resolve()
    args.px4_dir = args.px4_dir.resolve()
    if args.output is None:
        stamp = datetime_module.datetime.now(datetime_module.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = args.workspace / ".artifacts" / "verification" / "p0.8-sitl" / f"{args.mode}-{stamp}"
    if args.repeat > 1:
        return run_series(args)
    return SitlOrchestrator(args.workspace, args.px4_dir, args.output,
                            MODE_POLICIES[RunMode(args.mode)], args.allow_dirty).run()


if __name__ == "__main__":
    raise SystemExit(main())

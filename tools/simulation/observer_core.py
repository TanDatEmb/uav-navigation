#!/usr/bin/env python3
"""ROS-independent stream metrics, event lifecycle, and fault classifier."""
from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * p / 100
    low, high = math.floor(index), math.ceil(index)
    return values[low] if low == high else values[low] * (high-index) + values[high] * (index-low)


@dataclass
class StreamTracker:
    topic: str
    message_type: str
    check_stamp: bool = True
    arrivals: deque[float] = field(default_factory=lambda: deque(maxlen=10000))
    gaps: deque[float] = field(default_factory=lambda: deque(maxlen=10000))
    count: int = 0
    first_wall: float | None = None
    last_wall: float | None = None
    last_stamp_ns: int | None = None
    stamp_gap_last_ns: int | None = None
    stamp_gap_max_ns: int | None = None
    stamp_regressions: int = 0
    zero_stamp_count: int = 0
    stale_events: int = 0
    publisher_count: int = 0
    subscriber_count: int = 0

    def update(self, wall: float, stamp_ns: int | None) -> None:
        self.count += 1
        self.first_wall = wall if self.first_wall is None else self.first_wall
        if self.last_wall is not None:
            self.gaps.append(max(0.0, wall - self.last_wall))
        self.last_wall = wall
        self.arrivals.append(wall)
        if stamp_ns is None:
            return
        if stamp_ns == 0:
            self.zero_stamp_count += 1
            return
        if self.check_stamp and self.last_stamp_ns is not None:
            gap = stamp_ns - self.last_stamp_ns
            self.stamp_gap_last_ns = gap
            if gap < 0:
                self.stamp_regressions += 1
            else:
                self.stamp_gap_max_ns = max(self.stamp_gap_max_ns or 0, gap)
        self.last_stamp_ns = stamp_ns

    def rate(self, now: float, window: float) -> float:
        samples = [value for value in self.arrivals if value >= now-window]
        if len(samples) < 2:
            return 0.0
        return (len(samples)-1) / max(samples[-1]-samples[0], 1e-9)

    def state(self, now: float) -> dict[str, Any]:
        return {
            "topic": self.topic, "message_type": self.message_type, "message_count": self.count,
            "first_receive_wall_time": self.first_wall, "last_receive_wall_time": self.last_wall,
            "receive_rate_hz_short_window": self.rate(now, 5.0),
            "receive_rate_hz_long_window": self.rate(now, 30.0),
            "wall_gap_last": self.gaps[-1] if self.gaps else None,
            "wall_gap_max": max(self.gaps) if self.gaps else None,
            "wall_gap_p95": percentile(list(self.gaps), 95),
            "header_stamp_last": self.last_stamp_ns,
            "header_stamp_gap_last": self.stamp_gap_last_ns,
            "header_stamp_gap_max": self.stamp_gap_max_ns,
            "header_stamp_regression_count": self.stamp_regressions,
            "zero_stamp_count": self.zero_stamp_count,
            "age_wall": None if self.last_wall is None else now-self.last_wall,
            "publisher_count": self.publisher_count, "subscriber_count": self.subscriber_count,
            "stale_events": self.stale_events,
        }


class ObserverState(str, Enum):
    STARTING = "STARTING"
    WAITING_FOR_GAZEBO = "WAITING_FOR_GAZEBO"
    WAITING_FOR_BRIDGE = "WAITING_FOR_BRIDGE"
    WAITING_FOR_FAST_LIO = "WAITING_FOR_FAST_LIO"
    INITIALIZING = "INITIALIZING"
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    LIDAR_SOURCE_STALL = "LIDAR_SOURCE_STALL"
    GAZEBO_POINT_CONVERSION_STALL = "GAZEBO_POINT_CONVERSION_STALL"
    BRIDGE_STALL = "BRIDGE_STALL"
    INVALID_POINTCLOUD = "INVALID_POINTCLOUD"
    FAST_LIO_INPUT_REJECTION = "FAST_LIO_INPUT_REJECTION"
    FAST_LIO_PROCESSING_STALL = "FAST_LIO_PROCESSING_STALL"
    OUTPUT_STALL = "OUTPUT_STALL"
    CLOCK_FAULT = "CLOCK_FAULT"
    SYSTEM_OVERLOAD = "SYSTEM_OVERLOAD"
    SHUTTING_DOWN = "SHUTTING_DOWN"
    STOPPED = "STOPPED"


def progressed(current: dict[str, Any], previous: dict[str, Any], key: str) -> bool:
    return float(current.get(key, 0) or 0) > float(previous.get(key, 0) or 0)


def classify(current: dict[str, Any], previous: dict[str, Any]) -> tuple[ObserverState, str | None]:
    gz, old_gz = current.get("gazebo", {}), previous.get("gazebo", {})
    streams, old_streams = current.get("streams", {}), previous.get("streams", {})
    diag, old_diag = current.get("diagnostics", {}), previous.get("diagnostics", {})
    system = current.get("system", {})
    if current.get("clock_regression"):
        return ObserverState.CLOCK_FAULT, "CLOCK_FAULT"
    if system.get("overloaded"):
        return ObserverState.SYSTEM_OVERLOAD, "SYSTEM_OVERLOAD"
    if current.get("starting", False):
        return ObserverState.STARTING, None
    clock = progressed(gz, old_gz, "clock_sequence")
    imu = progressed(streams.get("imu", {}), old_streams.get("imu", {}), "message_count")
    raw = progressed(gz, old_gz, "raw_scan_sequence")
    gz_cloud = progressed(gz, old_gz, "pointcloud_sequence")
    ros_cloud = progressed(streams.get("lidar", {}), old_streams.get("lidar", {}), "message_count")
    if clock and imu and not raw:
        return ObserverState.LIDAR_SOURCE_STALL, "LIDAR_SOURCE_STALL"
    if raw and not gz_cloud:
        return ObserverState.GAZEBO_POINT_CONVERSION_STALL, "GAZEBO_POINT_CONVERSION_STALL"
    if gz_cloud and not ros_cloud:
        return ObserverState.BRIDGE_STALL, "BRIDGE_STALL"
    quality = current.get("pointcloud", {})
    lidar_state = streams.get("lidar", {})
    cloud_current = bool(lidar_state.get("message_count", 0)) and (
        lidar_state.get("age_wall") is None or lidar_state.get("age_wall", 999) < 0.5)
    if cloud_current and quality.get("finite_ratio") is not None and quality["finite_ratio"] < float(current.get("finite_error_threshold", .1)):
        return ObserverState.INVALID_POINTCLOUD, "INVALID_POINTCLOUD"
    if cloud_current and quality.get("finite_ratio") is not None and quality["finite_ratio"] < float(current.get("finite_warn_threshold", .5)):
        return ObserverState.DEGRADED, "POINTCLOUD_FINITE_RATIO_LOW"
    accepted = progressed(diag, old_diag, "accepted_lidar_count")
    rejected = progressed(diag, old_diag, "rejected_lidar_count")
    if ros_cloud and not accepted and rejected:
        return ObserverState.FAST_LIO_INPUT_REJECTION, "FAST_LIO_INPUT_REJECTION"
    accepted_imu = progressed(diag, old_diag, "accepted_imu_count")
    sync = progressed(diag, old_diag, "synchronized_group_count")
    output = progressed(streams.get("odometry", {}), old_streams.get("odometry", {}), "message_count")
    correction = progressed(diag, old_diag, "correction_success_count")
    if accepted and accepted_imu and not sync:
        return ObserverState.FAST_LIO_PROCESSING_STALL, "FAST_LIO_PROCESSING_STALL"
    if correction and not output:
        return ObserverState.OUTPUT_STALL, "OUTPUT_STALL"
    return ObserverState.HEALTHY, None


class EventLifecycle:
    def __init__(self, cooldown_s: float = 30.0, persist_s: float = 10.0):
        self.cooldown_s, self.persist_s = cooldown_s, persist_s
        self.active: dict[str, dict[str, float]] = {}

    def update(self, code: str | None, now: float) -> list[tuple[str, str]]:
        emitted: list[tuple[str, str]] = []
        for active_code in list(self.active):
            if active_code != code:
                emitted.append((active_code + "_RECOVERED", "recovered"))
                del self.active[active_code]
        if code is None:
            return emitted
        record = self.active.get(code)
        if record is None:
            self.active[code] = {"entered": now, "last": now}
            emitted.append((code + "_ENTER", "entered"))
        elif now-record["last"] >= self.cooldown_s and now-record["entered"] >= self.persist_s:
            record["last"] = now
            emitted.append((code + "_PERSIST", "persists"))
        return emitted

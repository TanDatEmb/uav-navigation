"""Offline diagnostic analysis of SITL propagated-state transport witnesses.

This is evidence tooling, never an admission or flight policy. Producer-to-
callback includes DDS and executor scheduling; it is not pure DDS latency.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sqlite3
import struct
from typing import Any


DISPOSITIONS = {
    1: "ACCEPTED", 2: "INVALID_MESSAGE", 3: "HEALTH_NOT_READY",
    4: "HEALTH_INVALID", 5: "LOCALIZATION_EPOCH_MISMATCH",
    6: "SEQUENCE_NON_INCREASING", 7: "SOURCE_TIMESTAMP_INVALID",
    8: "SOURCE_TIMESTAMP_NON_INCREASING",
}


def _quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def _summary(values: list[float]) -> dict[str, float | int | None]:
    return {
        "n": len(values), "p50": _quantile(values, 0.50),
        "p95": _quantile(values, 0.95), "p99": _quantile(values, 0.99),
        "max": max(values) if values else None,
    }


def classify_gap(
    *, source_period_ms: float, accepted_gap_ms: float,
    producer_gap_ms: float | None, publish_to_callback_ms: float | None,
    mutex_wait_ms: float | None, rejected_between: int,
    clock_gap_ms: float | None, clock_source_delta_ms: float | None,
) -> str:
    """Classify an investigation tail without changing the 200 ms lease."""
    if (clock_gap_ms is not None and clock_source_delta_ms is not None and
            clock_gap_ms > 50.0 and
            clock_source_delta_ms / clock_gap_ms < 0.1):
        return "SIM_TIME_STALL_OR_SLOWDOWN"
    if rejected_between:
        return "ADAPTER_SEMANTIC_REJECTION"
    if mutex_wait_ms is not None and mutex_wait_ms > 100.0:
        return "ADAPTER_MUTEX_CONTENTION"
    if (producer_gap_ms is not None and producer_gap_ms > 100.0 and
            producer_gap_ms >= accepted_gap_ms * 0.75):
        return "PRODUCER_PUBLICATION_GAP"
    if publish_to_callback_ms is not None and publish_to_callback_ms > 100.0:
        return "DDS_OR_EXECUTOR_DELIVERY_DELAY"
    # A callback itself may be delayed even if its individual publish-to-
    # callback value is unavailable. Do not guess a layer from only a gap.
    _ = source_period_ms
    return "UNKNOWN"


def _clock_event_for_interval(
    events: list[dict[str, Any]], before_source: int, after_source: int,
    before_observer_wall: int, after_observer_wall: int,
) -> dict[str, Any] | None:
    # The odometry source stamp may lag the clock by one tick. Correlate
    # independent observer intervals first, then require nearby ROS time.
    matches = [event for event in events if
               int(event.get("arrival_wall_ns") or 0) >= before_observer_wall and
               int(event.get("previous_arrival_wall_ns") or 0) <= after_observer_wall and
               int(event.get("previous_source_stamp_ns") or 0) <= after_source + 100_000_000 and
               int(event.get("source_stamp_ns") or 0) >= before_source - 100_000_000]
    return max(matches, key=lambda event: float(event.get("gap_ms") or 0.0),
               default=None)


def _clock_events_from_rosbag(session: Path) -> list[dict[str, Any]]:
    """Recover independent /clock recorder gaps from a SQLite rosbag.

    The bag timestamp is recorder arrival, not proof of Gazebo production
    timing. Clock's CDR payload is a fixed sec/nanosec pair after its header.
    This is diagnostic fallback for sessions captured before the monitor's
    sub-stale-gap witness was added.
    """
    bag = session / "rosbag" / "rosbag_0.db3"
    if not bag.exists():
        return []
    events: list[dict[str, Any]] = []
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as database:
        topic = database.execute(
            "SELECT id FROM topics WHERE name = '/clock' AND type = 'rosgraph_msgs/msg/Clock'"
        ).fetchone()
        if topic is None:
            return []
        previous: tuple[int, int] | None = None
        for arrival_ns, payload in database.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic[0],),
        ):
            if len(payload) != 12 or payload[:2] not in (b"\x00\x01", b"\x00\x00"):
                continue
            endian = "<" if payload[:2] == b"\x00\x01" else ">"
            seconds, nanoseconds = struct.unpack_from(f"{endian}iI", payload, 4)
            source_ns = seconds * 1_000_000_000 + nanoseconds
            if previous is not None:
                prior_arrival, prior_source = previous
                gap_ms = (arrival_ns - prior_arrival) / 1e6
                if gap_ms > 50.0:
                    events.append({
                        "previous_arrival_wall_ns": prior_arrival,
                        "arrival_wall_ns": arrival_ns,
                        "gap_ms": gap_ms,
                        "previous_source_stamp_ns": prior_source,
                        "source_stamp_ns": source_ns,
                        "source": "rosbag_recorder_arrival",
                    })
            previous = (arrival_ns, source_ns)
    return events


def analyze_session(session: Path) -> dict[str, Any]:
    producer: dict[tuple[int, int], dict[str, Any]] = {}
    ingress: list[dict[str, Any]] = []
    callback_counts: Counter[str] = Counter()
    with (session / "samples.jsonl").open(encoding="utf-8") as rows:
        for line in rows:
            row = json.loads(line)
            stream = row.get("stream")
            if stream not in {"odometry_producer_trace", "odometry_adapter_ingress_trace"}:
                continue
            value = row.get("payload") or {}
            key = (int(value.get("localization_epoch") or 0),
                   int(value.get("sequence") or 0))
            if stream == "odometry_producer_trace":
                if key in producer:
                    raise ValueError(f"duplicate producer identity {key}")
                producer[key] = value
            else:
                value["_observer_arrival_wall_ns"] = int(row.get("arrival_wall_ns") or 0)
                ingress.append(value)
                callback_counts[DISPOSITIONS.get(int(value.get("disposition") or 0),
                                                 "UNKNOWN_DISPOSITION")] += 1
    monitor = json.loads((session / "monitor.json").read_text(encoding="utf-8"))
    clock_stream = monitor.get("streams", {}).get("simulation_clock", {})
    clock_events = clock_stream.get("diagnostic_gap_events")
    clock_event_source = "monitor_callback_arrival"
    if clock_events is None:
        clock_events = _clock_events_from_rosbag(session)
        clock_event_source = "rosbag_recorder_arrival"
    accepted = sorted(
        (row for row in ingress if int(row.get("disposition") or 0) == 1),
        key=lambda row: int(row.get("accepted_receive_steady_ns") or 0),
    )
    metrics: dict[str, list[float]] = {name: [] for name in (
        "source_period_ms", "producer_publish_interval_ms",
        "worker_ready_to_publish_ms", "publisher_enter_to_publish_ms",
        "publisher_publish_to_exit_ms",
        "publish_to_callback_ms", "mutex_wait_ms", "callback_to_accept_ms",
        "accepted_receive_gap_ms",
    )}
    ordered_producer = sorted(
        producer.values(), key=lambda row: int(row["publisher_publish_call_steady_ns"]))
    for row in ordered_producer:
        ready = int(row.get("worker_estimate_ready_steady_ns") or 0)
        entered = int(row.get("publisher_enter_steady_ns") or 0)
        publish_call = int(row.get("publisher_publish_call_steady_ns") or 0)
        exited = int(row.get("publisher_exit_steady_ns") or 0)
        if publish_call >= ready > 0:
            metrics["worker_ready_to_publish_ms"].append((publish_call - ready) / 1e6)
        if publish_call >= entered > 0:
            metrics["publisher_enter_to_publish_ms"].append((publish_call - entered) / 1e6)
        if exited >= publish_call > 0:
            metrics["publisher_publish_to_exit_ms"].append((exited - publish_call) / 1e6)
    for left, right in zip(ordered_producer, ordered_producer[1:]):
        if left.get("localization_epoch") != right.get("localization_epoch"):
            continue
        metrics["producer_publish_interval_ms"].append(
            (int(right["publisher_publish_call_steady_ns"]) -
             int(left["publisher_publish_call_steady_ns"])) / 1e6)
    for row in accepted:
        callback = int(row.get("callback_enter_steady_ns") or 0)
        acquired = int(row.get("lock_acquired_steady_ns") or 0)
        requested = int(row.get("lock_requested_steady_ns") or 0)
        received = int(row.get("accepted_receive_steady_ns") or 0)
        if acquired >= requested > 0:
            metrics["mutex_wait_ms"].append((acquired - requested) / 1e6)
        if received >= callback > 0:
            metrics["callback_to_accept_ms"].append((received - callback) / 1e6)
        key = (int(row.get("localization_epoch") or 0),
               int(row.get("sequence") or 0))
        published = producer.get(key)
        if published is not None:
            publish_call = int(published.get("publisher_publish_call_steady_ns") or 0)
            if callback >= publish_call > 0:
                metrics["publish_to_callback_ms"].append((callback - publish_call) / 1e6)
    tails: list[dict[str, Any]] = []
    for before, after in zip(accepted, accepted[1:]):
        if before.get("localization_epoch") != after.get("localization_epoch"):
            continue
        source_period = (int(after["source_stamp_ros_ns"]) -
                         int(before["source_stamp_ros_ns"])) / 1e6
        gap = (int(after["accepted_receive_steady_ns"]) -
               int(before["accepted_receive_steady_ns"])) / 1e6
        metrics["source_period_ms"].append(source_period)
        metrics["accepted_receive_gap_ms"].append(gap)
        if gap <= 100.0:
            continue
        left = producer.get((int(before["localization_epoch"]), int(before["sequence"])))
        right = producer.get((int(after["localization_epoch"]), int(after["sequence"])))
        producer_gap = None if not (left and right) else (
            int(right["publisher_publish_call_steady_ns"]) -
            int(left["publisher_publish_call_steady_ns"])) / 1e6
        callback = int(after["callback_enter_steady_ns"])
        publish_call = int(right["publisher_publish_call_steady_ns"]) if right else 0
        delivery = (callback - publish_call) / 1e6 if callback >= publish_call > 0 else None
        requested = int(after.get("lock_requested_steady_ns") or 0)
        acquired = int(after.get("lock_acquired_steady_ns") or 0)
        mutex = (acquired - requested) / 1e6 if acquired >= requested > 0 else None
        between = sum(1 for row in ingress if
                      int(before["accepted_receive_steady_ns"]) <
                      int(row.get("callback_enter_steady_ns") or 0) <
                      int(after["accepted_receive_steady_ns"]) and
                      int(row.get("disposition") or 0) != 1)
        event = _clock_event_for_interval(clock_events,
                                          int(before["source_stamp_ros_ns"]),
                                          int(after["source_stamp_ros_ns"]),
                                          int(before.get("_observer_arrival_wall_ns") or 0),
                                          int(after.get("_observer_arrival_wall_ns") or 0))
        clock_gap = float(event["gap_ms"]) if event else None
        clock_source_delta = ((int(event["source_stamp_ns"]) -
                               int(event["previous_source_stamp_ns"])) / 1e6
                              if event else None)
        tails.append({
            "before_sequence": int(before["sequence"]),
            "after_sequence": int(after["sequence"]),
            "source_period_ms": source_period,
            "accepted_receive_gap_ms": gap,
            "producer_publish_gap_ms": producer_gap,
            "publish_to_callback_ms": delivery,
            "mutex_wait_ms": mutex,
            "rejected_between": between,
            "clock_arrival_gap_ms": clock_gap,
            "clock_source_delta_ms": clock_source_delta,
            "clock_event_source": clock_event_source if event else None,
            "class": classify_gap(
                source_period_ms=source_period, accepted_gap_ms=gap,
                producer_gap_ms=producer_gap, publish_to_callback_ms=delivery,
                mutex_wait_ms=mutex, rejected_between=between,
                clock_gap_ms=clock_gap, clock_source_delta_ms=clock_source_delta),
        })
    accepted_gaps = metrics["accepted_receive_gap_ms"]
    matched_accepted_count = sum(
        (int(row.get("localization_epoch") or 0), int(row.get("sequence") or 0)) in producer
        for row in accepted)
    return {
        "session": str(session), "producer_count": len(producer),
        "callback_count": len(ingress), "accepted_count": len(accepted),
        "rejected_count": len(ingress) - len(accepted),
        "dispositions": dict(callback_counts),
        "matched_accepted_count": matched_accepted_count,
        "accepted_without_producer_trace_count": len(accepted) - matched_accepted_count,
        "trace_complete_for_accepted": bool(accepted) and matched_accepted_count == len(accepted),
        "metrics_ms": {name: _summary(values) for name, values in metrics.items()},
        "maximum_accepted_receive_gap_ms": max(accepted_gaps) if accepted_gaps else None,
        "remaining_margin_to_200_ms": (200.0 - max(accepted_gaps)) if accepted_gaps else None,
        "maximum_observed_clock_arrival_gap_ms": (
            clock_stream.get("maximum_observed_arrival_gap_ms")
            if clock_stream.get("maximum_observed_arrival_gap_ms") is not None
            else max((float(event["gap_ms"]) for event in clock_events), default=None)
        ),
        "tails_over_100_ms": tails,
        "unknown_tail_count": sum(tail["class"] == "UNKNOWN" for tail in tails),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_session(args.session), indent=2, sort_keys=True))

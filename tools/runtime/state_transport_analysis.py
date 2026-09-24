"""Offline diagnostic analysis of SITL propagated-state transport witnesses.

This is evidence tooling, never an admission or flight policy. Producer-to-
callback includes DDS and executor scheduling; it is not pure DDS latency.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
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
            clock_gap_ms > 100.0 and
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
) -> dict[str, Any] | None:
    matches = [event for event in events
               if before_source <= int(event.get("previous_source_stamp_ns") or 0)
               and int(event.get("source_stamp_ns") or 0) <= after_source]
    return max(matches, key=lambda event: float(event.get("gap_ms") or 0.0),
               default=None)


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
                ingress.append(value)
                callback_counts[DISPOSITIONS.get(int(value.get("disposition") or 0),
                                                 "UNKNOWN_DISPOSITION")] += 1
    monitor = json.loads((session / "monitor.json").read_text(encoding="utf-8"))
    clock_events = (monitor.get("streams", {}).get("simulation_clock", {})
                    .get("arrival_gap_events", []))
    accepted = sorted(
        (row for row in ingress if int(row.get("disposition") or 0) == 1),
        key=lambda row: int(row.get("accepted_receive_steady_ns") or 0),
    )
    metrics: dict[str, list[float]] = {name: [] for name in (
        "source_period_ms", "producer_publish_interval_ms",
        "publish_to_callback_ms", "mutex_wait_ms", "callback_to_accept_ms",
        "accepted_receive_gap_ms",
    )}
    ordered_producer = sorted(
        producer.values(), key=lambda row: int(row["publisher_publish_call_steady_ns"]))
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
                                          int(after["source_stamp_ros_ns"]))
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
        "tails_over_100_ms": tails,
        "unknown_tail_count": sum(tail["class"] == "UNKNOWN" for tail in tails),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_session(args.session), indent=2, sort_keys=True))

#!/usr/bin/env python3
"""Generate machine-readable and human-readable evidence reports."""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any

from observer_core import percentile


def rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def number(value: Any, default=0.0) -> float:
    try: return float(value)
    except (TypeError, ValueError): return default


def generate(session: Path) -> dict[str, Any]:
    stream_rows = rows(session/"metrics/streams.csv")
    cloud_rows = rows(session/"metrics/pointcloud.csv")
    process_rows = rows(session/"metrics/process.csv")
    gz_rows = rows(session/"metrics/gazebo.csv")
    events = []
    event_path = session/"metrics/events.jsonl"
    if event_path.exists():
        for line in event_path.read_text().splitlines():
            try: events.append(json.loads(line))
            except json.JSONDecodeError: pass
    snapshot_names = [path.name for path in (session/"snapshots").glob("*") if path.is_dir()]
    for event in events:
        if event.get("snapshot_id") == "pending":
            matches = [name for name in snapshot_names if event.get("code", "") in name]
            event["snapshot_id"] = matches[-1] if matches else "capture_incomplete"
    streams = {}
    for name in sorted({row["name"] for row in stream_rows}):
        selected = [row for row in stream_rows if row["name"] == name]
        last = selected[-1]
        streams[name] = {
            "count": int(number(last["message_count"])),
            "mean_rate_hz": number(last["receive_rate_hz_long_window"]),
            "minimum_short_window_rate_hz": min(
                (number(row["receive_rate_hz_short_window"]) for row in selected
                 if number(row["receive_rate_hz_short_window"]) > 0), default=0.0),
            "maximum_gap_s": max(number(row["wall_gap_max"]) for row in selected),
            "p95_gap_s": number(last["wall_gap_p95"]),
            "timestamp_regressions": int(number(last["header_stamp_regression_count"])),
            "stale_events": int(number(last.get("stale_events"))),
        }
    ratios = [number(row["finite_ratio"]) for row in cloud_rows]
    pointcloud = {
        "sampled_scans": len(cloud_rows), "minimum_finite_ratio": min(ratios) if ratios else None,
        "median_finite_ratio": statistics.median(ratios) if ratios else None,
        "p5_finite_ratio": percentile(ratios, 5),
        "nan_count": sum(int(number(row["nan_xyz_count"])) for row in cloud_rows),
        "positive_inf_count": sum(int(number(row["positive_inf_xyz_count"])) for row in cloud_rows),
        "negative_inf_count": sum(int(number(row["negative_inf_xyz_count"])) for row in cloud_rows),
        "is_dense_violations": sum(row["density_contract_violation"].lower() == "true" for row in cloud_rows),
    }
    error_events = [event for event in events if event.get("severity") in ("ERROR", "FATAL")]
    entered = [event for event in error_events if event.get("code", "").endswith("_ENTER")]
    root = entered[0]["code"].removesuffix("_ENTER") if entered else "UNRESOLVED"
    downstream = [event["code"] for event in entered[1:]]
    duration = 0.0
    if stream_rows:
        duration = max(number(row["monotonic_time_s"]) for row in stream_rows)-min(number(row["monotonic_time_s"]) for row in stream_rows)
    observer_processes = [row for row in process_rows if row.get("role") == "observer"]
    summary = {
        "schema_version": 1, "session": str(session.resolve()),
        "overall_result": "FAIL" if error_events else ("WARN" if events else "PASS"),
        "flight_duration_s": duration,
        "first_fault_time": error_events[0]["wall_time"] if error_events else None,
        "root_cause_classification": root, "downstream_effects": downstream,
        "recovered": any(event.get("code") == root+"_RECOVERED" for event in events),
        "streams": streams, "pointcloud": pointcloud,
        "fast_lio": json.loads((session/"latest/diagnostics.json").read_text()) if (session/"latest/diagnostics.json").exists() else {},
        "system": {
            "observer_cpu_peak": max((number(row["cpu_percent"]) for row in observer_processes), default=None),
            "observer_rss_peak_mb": max((number(row["rss_kib"])/1024 for row in observer_processes), default=None),
            "rtf_minimum": min((number(row["real_time_factor"], 99) for row in gz_rows if row.get("real_time_factor")), default=None),
            "process_crashes": sum(row.get("alive") == "False" for row in process_rows),
            "orphan_processes": None,
        },
        "events": events,
        "root_cause_reasoning": reasoning(root, pointcloud, streams),
    }
    (session/"summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True)+"\n")
    return summary


def reasoning(root: str, pointcloud: dict[str, Any], streams: dict[str, Any]) -> str:
    if root == "INVALID_POINTCLOUD":
        return (f"ROS LiDAR progressed, but sampled finite ratio reached "
                f"{pointcloud['minimum_finite_ratio']}; subsequent faults are downstream.")
    if root == "LIDAR_SOURCE_STALL":
        return "Gazebo clock and IMU progressed while the native raw scan did not."
    if root == "BRIDGE_STALL":
        return "Gazebo point-cloud progression continued while ROS /lidar/points did not."
    if root == "FAST_LIO_INPUT_REJECTION":
        return "ROS LiDAR progressed while accepted scans stopped and rejection counters increased."
    if root == "FAST_LIO_PROCESSING_STALL":
        return "Accepted IMU/LiDAR progressed while synchronization and corrected output stopped."
    return ("Root cause unresolved. A classified ENTER event was not recorded; inspect missing "
            "Gazebo progression, pointcloud quality, and FAST-LIO counter fields.")


def render(summary: dict[str, Any], path: Path) -> None:
    lines = ["# PX4 MID-360 Flight Observability Report", "", "## Executive summary", "",
             f"- Overall result: **{summary['overall_result']}**",
             f"- Flight duration: {summary['flight_duration_s']:.1f} s",
             f"- First fault: {summary['first_fault_time'] or 'none'}",
             f"- Root-cause classification: `{summary['root_cause_classification']}`",
             f"- Recovered: {summary['recovered']}", "",
             "## Root cause reasoning", "", summary["root_cause_reasoning"], "",
             "## Stream statistics", "",
             "| Stream | Count | Mean Hz | Min short Hz | Max gap s | p95 gap s | Regressions |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for name, item in summary["streams"].items():
        lines.append(f"| {name} | {item['count']} | {item['mean_rate_hz']:.2f} | "
                     f"{item['minimum_short_window_rate_hz']:.2f} | {item['maximum_gap_s']:.3f} | "
                     f"{item['p95_gap_s']:.3f} | {item['timestamp_regressions']} |")
    pc = summary["pointcloud"]
    lines += ["", "## Point cloud quality", "",
              f"- Sampled scans: {pc['sampled_scans']}",
              f"- Finite ratio min/median/p5: {pc['minimum_finite_ratio']} / {pc['median_finite_ratio']} / {pc['p5_finite_ratio']}",
              f"- NaN/+Inf/-Inf: {pc['nan_count']} / {pc['positive_inf_count']} / {pc['negative_inf_count']}",
              f"- is_dense violations: {pc['is_dense_violations']}", "",
              "## System", "", f"```json\n{json.dumps(summary['system'], indent=2)}\n```", "",
              "## Timeline", ""]
    if summary["events"]:
        for event in summary["events"]:
            lines.append(f"- {event['wall_time']} `{event['code']}` — {event['message']}")
    else:
        lines.append("- No anomaly events recorded.")
    path.write_text("\n".join(lines)+"\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True, type=Path)
    args = parser.parse_args()
    summary = generate(args.session.resolve())
    render(summary, args.session/"REPORT.md")
    print(args.session/"REPORT.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

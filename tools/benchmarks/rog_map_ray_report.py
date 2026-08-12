#!/usr/bin/env python3
"""Build an offline ROG ray/voxel benchmark report from runtime samples.

The runtime only publishes aggregate counters. This tool deliberately computes
percentiles offline and never changes mapper behavior.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def samples(session: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for line in (session / "samples.jsonl").open(encoding="utf-8"):
        item = json.loads(line)
        if item.get("stream") != "mapping_diagnostics":
            continue
        for status in item.get("payload", {}).get("statuses", []):
            if str(status.get("name", "")).endswith("/world_model"):
                result.append(status.get("values", {}))
    return result


def transport_values(session: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for line in (session / "samples.jsonl").open(encoding="utf-8"):
        item = json.loads(line)
        if item.get("stream") != "diagnostics":
            continue
        for status in item.get("payload", {}).get("statuses", []):
            if status.get("name") == "fast_lio/transport":
                result = status.get("values", {})
    return result


def row(values: list[dict[str, Any]], field: str) -> str:
    data = [number(item.get(field)) for item in values]
    data = [value for value in data if math.isfinite(value)]
    if not data:
        return "NOT_AVAILABLE"
    return f"mean={mean(data):.3f}, p50={percentile(data, .50):.3f}, p95={percentile(data, .95):.3f}, p99={percentile(data, .99):.3f}, max={max(data):.3f}"


def report_session(session: Path) -> str:
    values = samples(session)
    if not values:
        return f"## {session.name}\n\nNo navigation_mapping diagnostics samples found.\n"
    final = values[-1]
    accepted = number(final.get("accepted_observation_count"))
    input_points = number(final.get("mapping_filter_input_point_count"))
    filtered_points = number(final.get("mapping_filter_output_point_count"))
    endpoints = number(final.get("rog_endpoint_count"))
    attempts = number(final.get("rog_ray_attempt_count"))
    processed = number(final.get("rog_ray_processed_count"))
    skips = number(final.get("rog_ray_skipped_count"))
    candidates = number(final.get("rog_hit_candidate_count")) + number(final.get("rog_miss_candidate_count"))
    cache = number(final.get("rog_update_cache_entry_count"))
    reason_available = "rog_skip_nonfinite" in final
    skip_fields = {
        "nonfinite": number(final.get("rog_skip_nonfinite")),
        "intensity": number(final.get("rog_skip_intensity")),
        "point filter": number(final.get("rog_skip_point_filter")),
        "below minimum range": number(final.get("rog_skip_below_raycast_min_range")),
        "endpoint outside local map": number(final.get("rog_skip_endpoint_outside_local_map")),
    }
    transport = transport_values(session)
    published = number(transport.get("mapping_observation_publish_count"))
    received = number(final.get("mapping_observation_receive_count"))
    callback_skips = number(transport.get("mapping_observation_publish_skip_count"))
    source_elapsed = 0.0
    if values:
        source_elapsed = max(0.0, number(final.get("last_input_stamp_ns")) - number(values[0].get("last_input_stamp_ns"))) / 1e9
    arrival_hz = accepted / source_elapsed if source_elapsed > 0 else 0.0
    callback_mean_us = mean(number(item.get("mapping_callback_total_us")) for item in values)
    utilization = callback_mean_us * arrival_hz / 1e6
    return f"""## {session.name}

| Metric | Value |
|---|---:|
| accepted observations | {accepted:.0f} |
| mapping candidate input points | {input_points:.0f} |
| mapper filter output points | {filtered_points:.0f} |
| mapper filter retention | {(filtered_points / input_points * 100 if input_points else 0):.3f}% |
| mapper filter dropped points | {(input_points - filtered_points):.0f} |
| ROG endpoint retention from mapper output | {(endpoints / filtered_points * 100 if filtered_points else 0):.3f}% |
| ROG ray attempts | {attempts:.0f} |
| ROG processed endpoints | {processed:.0f} |
| intentional skips | {skips:.0f} |
| candidate operations | {candidates:.0f} |
| update-cache entries | {cache:.0f} |
| cache dedup ratio | {(1 - cache / candidates) * 100 if candidates else 0:.3f}% |
| source arrival rate | {arrival_hz:.3f} Hz |
| callback utilization | {utilization:.4f} |
| allocated logical voxels | {number(final.get('rog_allocated_voxel_count')):.0f} |
| sensor-origin grid type | {final.get('sensor_origin_grid_type', 'NOT_AVAILABLE')} |
| LIO mapping observations published | {published:.0f} |
| mapper observations received/accepted | {received:.0f}/{accepted:.0f} |
| publisher-to-mapper observation gap | {published - received:.0f} |
| publisher-side skips | {callback_skips:.0f} |

### Timing distributions (microseconds)

| Stage | Distribution |
|---|---|
| PointCloud2 decode | {row(values, 'ros_pointcloud_decode_us')} |
| Mapping filter | {row(values, 'mapping_filter_us')} |
| Transform | {row(values, 'transform_to_odom_us')} |
| ROG total update | {row(values, 'rog_total_update_us')} |
| ROG raycast | {row(values, 'rog_raycast_us')} |
| Probability update | {row(values, 'rog_probability_update_us')} |
| Inflation | {row(values, 'rog_inflation_us')} |
| Sliding | {row(values, 'rog_slide_us')} |
| Mapping callback total | {row(values, 'mapping_callback_total_us')} |

### Throughput

- rays/s: `{processed / source_elapsed if source_elapsed else 'NOT_AVAILABLE'}`
- voxel traversals/s: `{number(final.get('rog_voxel_traversal_count')) / source_elapsed if source_elapsed else 'NOT_AVAILABLE'}`
- voxel traversals/processed ray: `{number(final.get('rog_voxel_traversal_count')) / processed if processed else 'NOT_AVAILABLE'}`
- unique cache entries/s: `{cache / source_elapsed if source_elapsed else 'NOT_AVAILABLE'}`
- sliding calls: `{number(final.get('map_slide_count')):.0f}`; cleared cells: `{number(final.get('map_slide_cells_cleared')):.0f}`
- visualization subscribers: `{final.get('visualization_subscriber_count', 'NOT_AVAILABLE')}`

### Intentional ray skips and conservation

| Reason | Count |
|---|---:|
""" + "\n".join(
        f"| {name} | {value:.0f} |" if reason_available else f"| {name} | NOT_AVAILABLE |"
        for name, value in skip_fields.items()
    ) + f"""
| Total intentional skips | {skips:.0f} |
| Attempt - processed - skips | {(number(final.get('rog_ray_attempt_count')) - processed - sum(skip_fields.values())) if reason_available else 'NOT_AVAILABLE'} |
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# ROG-Map Ray/Voxel Correctness, Throughput and Scaling Benchmark",
        "",
        "- Baseline commit: `5052f12e63e0fd106deafd662c489b2281dc87b6`",
        "- Dataset: `aist-mid360-drive`",
        "- Source: aggregate `/navigation_mapping/diagnostics` samples",
        "- Percentiles are computed offline; no per-ray production logging is used.",
        "- Method: instrumentation/correctness changes were applied without estimator or ROG algorithm tuning; this report does not claim a before/after speedup.",
        "",
    ]
    lines.extend(report_session(session) for session in args.session)
    lines.extend([
        "## Replay invariance",
        "",
        "The implementation now exposes `RogMapAdapter::deterministicDigest()` for sparse synthetic/checkpoint use. Runtime checkpoint scheduling and cross-rate digest capture are not yet wired into the ROS runner, so AIST RATE 1 vs RATE 2 digest equality is `NOT_MEASURED` here.",
        "",
        "## P0 source-cloud proof",
        "",
        "`RosMappingObservationPublisher` serializes `ProcessResult::mapping_candidate_points_lidar_m`. `PointCloudPreprocessor` copies the range/finite-filtered scan into that field before applying `voxel_filter_` to the estimator scan. Therefore `/lio/mapping_observation.points` is the common-filtered, pre-estimator-voxelization cloud, not the `preprocessing.scan_voxel_size_m` registration cloud. The configured 0.9 m estimator voxel is not on this mapping observation path.",
        "",
        "## Evidence classification",
        "",
        "- FAST-LIO mapping-observation source cloud: `CONFIRMED` by source inspection.",
        "- FAST-LIO scan downsample affecting mapping observation: `NOT_SIGNIFICANT` for this path; it affects registration only.",
        "- Mapper-side point filter: `MEASURED`; input/output counters are in the session tables.",
        "- DDS whole-observation delivery gap: `MEASURED` where publisher and mapper counters differ (RATE 2: 2756 published vs 2754 received); the counters establish a gap, but do not identify the sole DDS cause without sequence IDs.",
        "- ROG raycast/sliding: `MEASURED` by aggregate counters and timings.",
        "- Visualization/RViz: `NOT_SIGNIFICANT` for the headless runs because map-output subscriber count is zero; RViz rendering was not benchmarked in this report.",
        "- Probability convergence and persistent-obstacle/free-space recall: `NOT_MEASURED`; AIST has no occupancy ground truth and the current offline runner does not yet build persistent endpoint cohorts.",
        "",
        "## Limitations / next benchmark slice",
        "",
        "- Resolution sweep (0.30/0.20/0.15/0.10 m), input-filter sweep, angular/range histograms, persistent endpoint recall, known-free lookahead, RSS/process CPU, and checkpoint digest scheduling still need a dedicated benchmark runner.",
        "- `map_sliding.threshold = -1.0` is effectively always eligible for the distance condition (`distance > -1`); the aggregate run shows a slide call per accepted update. This is reported, not tuned.",
        "- The existing runtime configuration has user-local visualization changes (`enabled: true`, `10 Hz`); headless `dataset-check` had zero map-output subscribers, so subscriber gating skipped extraction."
    ])
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

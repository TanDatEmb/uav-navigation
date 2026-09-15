#!/usr/bin/env python3
"""Small A/B companion for recorder overhead and bounded-loss evidence.

The direct arm models the old callback-owned JSONL write.  The bounded arm
measures the same producer-side work while serialization and file I/O run in
the EvidenceWriter owner thread.  This is a benchmark artifact, not flight
qualification evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import tempfile
import time
from typing import Any

from evidence_writer import EvidenceWriter


def _percentile(values: list[int], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return float(ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))])


def _stats(values: list[int]) -> dict[str, Any]:
    return {
        "count": len(values),
        "mean_us": statistics.fmean(values) / 1000.0 if values else None,
        "p50_us": (_percentile(values, 0.50) or 0.0) / 1000.0 if values else None,
        "p95_us": (_percentile(values, 0.95) or 0.0) / 1000.0 if values else None,
        "maximum_us": (max(values) / 1000.0) if values else None,
    }


def _record(index: int, payload_bytes: int) -> dict[str, Any]:
    return {
        "kind": "benchmark_sample",
        "index": index,
        "stamp_ns": 1_000_000_000 + index * 10_000_000,
        "payload": "x" * payload_bytes,
    }


def run(count: int, payload_bytes: int, output: Path) -> dict[str, Any]:
    if count <= 0 or payload_bytes < 0:
        raise ValueError("count must be positive and payload_bytes must be non-negative")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="evidence-benchmark-") as directory:
        root = Path(directory)
        direct_path = root / "direct.jsonl"
        writer_path = root / "bounded.jsonl"

        direct_samples: list[int] = []
        with direct_path.open("w", encoding="utf-8") as stream:
            for index in range(count):
                record = _record(index, payload_bytes)
                started = time.perf_counter_ns()
                stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
                direct_samples.append(time.perf_counter_ns() - started)

        writer = EvidenceWriter(writer_path, capacity=max(256, min(count, 4096)), batch_size=128)
        producer_samples: list[int] = []
        accepted = 0
        for index in range(count):
            record = _record(index, payload_bytes)
            started = time.perf_counter_ns()
            if writer.enqueue(record):
                accepted += 1
            producer_samples.append(time.perf_counter_ns() - started)
        writer_stats = writer.close().as_dict()

    direct = _stats(direct_samples)
    bounded_producer = _stats(producer_samples)
    direct_p95 = direct["p95_us"]
    bounded_p95 = bounded_producer["p95_us"]
    return {
        "schema_version": 1,
        "benchmark": "recorder_overhead_ab",
        "qualification_role": "diagnostic_only",
        "count": count,
        "payload_bytes": payload_bytes,
        "arms": {
            "direct_callback_jsonl": direct,
            "bounded_producer_enqueue": bounded_producer,
        },
        "producer_accepted_count": accepted,
        "writer": writer_stats,
        "producer_p95_over_direct_p95_us": (
            bounded_p95 - direct_p95
            if direct_p95 is not None and bounded_p95 is not None else None
        ),
        "interpretation": (
            "Compare producer-side enqueue overhead and writer loss separately; "
            "a dropped record makes the capture incomplete."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=2000)
    parser.add_argument("--payload-bytes", type=int, default=256)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    artifact = run(args.count, args.payload_bytes, args.output)
    args.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(artifact, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

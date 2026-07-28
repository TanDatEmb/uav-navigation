#!/usr/bin/env python3
"""Analyze measurements from the actual vendored ikd-Tree benchmark only."""

import argparse
import json
import math
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("measurement", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-query-scaling-exponent", type=float, default=0.95)
    args = parser.parse_args()
    try:
        result = json.loads(args.measurement.read_text())
        measurements = result["measurements"]
    except (OSError, KeyError, json.JSONDecodeError) as error:
        print(f"invalid benchmark JSON: {error}", file=sys.stderr)
        return 2
    failures = []
    if result.get("backend") != "upstream_ikd_tree" or len(measurements) < 3:
        failures.append("result is not a three-size upstream ikd-Tree measurement")
    ordered = sorted(measurements, key=lambda item: item["point_count"])
    exponents = []
    for lower, upper in zip(ordered, ordered[1:]):
        if lower["point_count"] <= 0 or upper["point_count"] <= lower["point_count"]:
            failures.append("point counts are not strictly increasing")
            continue
        if lower["query_us"] <= 0 or upper["query_us"] <= 0:
            failures.append("query timings must be positive")
            continue
        exponent = math.log(upper["query_us"] / lower["query_us"]) / math.log(
            upper["point_count"] / lower["point_count"]
        )
        exponents.append(exponent)
    if any(value > args.max_query_scaling_exponent for value in exponents):
        failures.append("empirical query scaling is not sublinear under the configured gate")
    report = {
        "schema_version": 1,
        "measurement": str(args.measurement),
        "backend_verified": result.get("backend") == "upstream_ikd_tree",
        "query_scaling_exponents": exponents,
        "max_query_scaling_exponent": args.max_query_scaling_exponent,
        "passed": not failures,
        "failures": failures,
        "interpretation": "Empirical timing evidence only; not a complexity proof.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

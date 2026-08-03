#!/usr/bin/env python3
"""Focused P0.8 dataset and host qualification runner.

The runner deliberately owns only P0.8 evidence. It validates provenance before
starting a run, uses a fresh artifact directory, and keeps raw runtime output
outside git. ROS setup is supplied by the calling shell so baseline worktrees
can use their own install overlay.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import statistics
import shutil
import subprocess
import sys
import time
from typing import Any

TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
from data import bag_topic_counts, data_home, dataset_context  # noqa: E402

from p0_8_provenance import make_provenance, write_json  # noqa: E402


def load_summary(path: Path) -> dict[str, Any]:
    summary_path = path / "summary.json"
    if not summary_path.is_file():
        raise RuntimeError(f"missing replay summary: {summary_path}")
    return json.loads(summary_path.read_text(encoding="utf-8"))


def correctness(summary: dict[str, Any]) -> dict[str, Any]:
    diagnostics = summary.get("diagnostics", {})
    propagated = diagnostics.get("propagated_odometry", {})
    checks = {
        "received_imu": diagnostics.get("received_imu_count") == 55_435,
        "processed_imu": diagnostics.get("processed_imu_count") == 55_435,
        "received_lidar": diagnostics.get("received_lidar_count") == 2_772,
        "processed_lidar": diagnostics.get("processed_lidar_count") == 2_772,
        "drops": diagnostics.get("imu_drop_count", 0) == 0 and diagnostics.get("lidar_drop_count", 0) == 0,
        "overflow": not diagnostics.get("overflow_detected", False),
        "invalid_timestamps": diagnostics.get("invalid_timestamp_rejected_count", 0) == 0,
        "final_queues": all(diagnostics.get(key, 0) == 0 for key in (
            "current_input_queue_depth", "current_imu_queue_depth", "current_lidar_queue_depth")),
        "estimator_exit": summary.get("estimator_returncode") == 0,
        "replay_exit": summary.get("replay_returncode") == 0,
        "no_nonfinite": all(diagnostics.get(key, 0) == 0 for key in (
            "reject_reason_nonfinite_xyz", "output_pose_covariance_nonfinite_count",
            "output_twist_covariance_nonfinite_count")),
        "load_shedding": propagated.get("load_shedding_count", 0) == 0,
        "processing_lag": not diagnostics.get("processing_lag_exceeded", False),
        "propagated_output": propagated.get("publication_count", 0) > 0 or not propagated.get("enabled", False),
    }
    return {"pass": all(checks.values()), "checks": checks}


def command_for_replay(workspace: Path, context: dict[str, Any], output: Path,
                       rate: float, timeout: float, readiness: float,
                       drain: float) -> list[str]:
    return [
        sys.executable,
        str(workspace / "tools/runtime/ros_replay.py"),
        "run",
        "--bag", str(context["bag"]),
        "--config", str(context["config"]),
        "--output", str(output),
        "--imu-topic", str(context["input"]["imu_topic"]),
        "--lidar-topic", str(context["input"]["lidar_topic"]),
        "--rate", str(rate),
        "--replay-timeout", str(timeout),
        "--readiness-timeout", str(readiness),
        "--drain-timeout", str(drain),
    ]


def dataset_run(args: argparse.Namespace) -> int:
    workspace = args.workspace.resolve()
    context = dataset_context(args.dataset, data_home())
    # Resolve the configuration in the worktree under test, not the runner's
    # checkout. The canonical content must be identical for a code-only A/B.
    context["config"] = (workspace / "src/navigation_estimator/fast_lio_ros/config/mid360_aist_replay.yaml").resolve()
    provenance = make_provenance(
        workspace,
        config=context["config"],
        dataset=args.dataset,
        rate=args.rate,
        expected_sha=args.expected_git_sha,
        binary_relative=args.binary_relative,
    )
    counts = bag_topic_counts(context)
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"artifact directory already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = command_for_replay(
        workspace, context, output, args.rate, args.replay_timeout,
        args.readiness_timeout, args.drain_timeout,
    )
    time_file = output.parent / f"{output.name}.time.txt"
    stdout_path = output.parent / f"{output.name}.runner.stdout.log"
    stderr_path = output.parent / f"{output.name}.runner.stderr.log"
    sampler_path = output.parent / f"{output.name}.process.csv"
    started = time.time_ns()
    process = None
    sampler = None
    returncode = 1
    try:
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
            "w", encoding="utf-8"
        ) as stderr:
            process = subprocess.Popen(
                ["/usr/bin/time", "-v", "-o", str(time_file), *command],
                cwd=workspace,
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
            sampler = subprocess.Popen([
                sys.executable, str(Path(__file__).with_name("sample_process.py")),
                "--pid", str(process.pid), "--output", str(sampler_path),
                "--interval", "1.0",
            ])
            returncode = process.wait()
    finally:
        if sampler is not None:
            sampler.wait(timeout=30)
        output.mkdir(parents=True, exist_ok=True)
        if sampler_path.is_file():
            shutil.move(str(sampler_path), str(output / "process.csv"))
        metadata = {
            "schema_version": 2,
            "qualification": "p0.8-performance",
            "started_at_ns": started,
            "finished_at_ns": time.time_ns(),
            "command": command,
            "runner_returncode": returncode,
            "provenance": provenance,
            "input_counts": counts,
            "artifact": str(output),
        }
        write_json(output / "metadata.json", metadata)
        run_payload = {
            "schema_version": 2,
            "action": "dataset-run",
            "dataset": args.dataset,
            "rate": args.rate,
            "provenance": provenance,
            "input_counts": counts,
            "runner_returncode": returncode,
            "summary": str(output / "summary.json"),
            "correctness": correctness(load_summary(output)) if (output / "summary.json").is_file() else {"pass": False},
        }
        write_json(output / "run.json", run_payload)
    return returncode


def metric_value(diagnostics: dict[str, Any], field: str) -> float:
    value = diagnostics.get(field)
    if value is None:
        raise RuntimeError(f"metric not instrumented: {field}")
    return float(value)


def collect_runs(root: Path, role: str) -> list[dict[str, Any]]:
    runs = []
    for path in sorted((root / role).glob("run-*")):
        if not path.is_dir():
            continue
        summary = load_summary(path)
        run = json.loads((path / "run.json").read_text(encoding="utf-8"))
        runs.append({"path": str(path), "summary": summary, "run": run})
    return runs


def statistics_for(runs: list[dict[str, Any]], metric: str) -> dict[str, Any]:
    values = [metric_value(item["summary"]["diagnostics"], metric)
              for item in runs]
    return {
        "count": len(values),
        "values": values,
        "median": statistics.median(values),
        "minimum": min(values),
        "maximum": max(values),
        "mean": statistics.mean(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "coefficient_of_variation": (
            statistics.stdev(values) / statistics.mean(values)
            if len(values) > 1 and statistics.mean(values) else 0.0
        ),
    }


def dataset_compare(args: argparse.Namespace) -> int:
    root = args.root.resolve()
    baseline = collect_runs(root, "baseline")
    candidate = collect_runs(root, "candidate")
    if len(baseline) < 3 or len(candidate) < 3:
        raise RuntimeError("comparison requires at least three runs per side")
    metrics = [
        "pipeline_push_lidar_p95_us", "pipeline_push_lidar_p99_us",
        "corrected_scan_end_to_end_p95_us", "corrected_scan_end_to_end_p99_us",
        "registration_update_p95_us", "registration_update_p99_us",
        "result_processing_p95_us", "result_processing_p99_us",
        "maximum_queue_depth", "worker_busy_ratio",
    ]
    comparison: dict[str, Any] = {"schema_version": 1, "metrics": {}, "runs": {"baseline": baseline, "candidate": candidate}}
    rows = []
    for metric in metrics:
        if metric == "maximum_queue_depth":
            values = {
                "baseline": [float(x["summary"]["diagnostics"][metric]) for x in baseline],
                "candidate": [float(x["summary"]["diagnostics"][metric]) for x in candidate],
            }
            stats = {side: {"count": len(items), "values": items, "median": statistics.median(items)} for side, items in values.items()}
        elif metric == "worker_busy_ratio":
            values = {
                "baseline": [float(x["summary"]["diagnostics"][metric]) for x in baseline],
                "candidate": [float(x["summary"]["diagnostics"][metric]) for x in candidate],
            }
            stats = {side: {"count": len(items), "values": items, "median": statistics.median(items)} for side, items in values.items()}
        else:
            stats = {"baseline": statistics_for(baseline, metric), "candidate": statistics_for(candidate, metric)}
        b = stats["baseline"]["median"]
        c = stats["candidate"]["median"]
        comparison["metrics"][metric] = {
            "baseline": stats["baseline"], "candidate": stats["candidate"],
            "absolute_delta": c - b,
            "relative_delta": (c / b - 1.0) if b else None,
        }
        rows.append([metric, b, c, c - b, (c / b - 1.0) if b else ""])
    write_json(root / "comparison.json", comparison)
    with (root / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "baseline_median", "candidate_median", "absolute_delta", "relative_delta"])
        writer.writerows(rows)
    print(root / "comparison.json")
    return 0


def host(args: argparse.Namespace) -> int:
    write_json(args.output, {"schema_version": 1, "host": make_provenance(
        args.workspace.resolve(),
        config=(args.workspace / "src/navigation_estimator/fast_lio_ros/config/mid360_aist_replay.yaml").resolve(),
        expected_sha=args.expected_git_sha,
    )["host"]})
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    sub = result.add_subparsers(dest="command", required=True)
    run = sub.add_parser("dataset-run")
    run.add_argument("--workspace", type=Path, required=True)
    run.add_argument("--expected-git-sha", required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--dataset", default="aist-mid360-drive")
    run.add_argument("--rate", type=float, default=1.0)
    run.add_argument("--binary-relative")
    run.add_argument("--replay-timeout", type=float, default=900.0)
    run.add_argument("--readiness-timeout", type=float, default=30.0)
    run.add_argument("--drain-timeout", type=float, default=120.0)
    compare = sub.add_parser("dataset-compare")
    compare.add_argument("--root", type=Path, required=True)
    stress = sub.add_parser("dataset-stress")
    stress.add_argument("--workspace", type=Path, required=True)
    stress.add_argument("--expected-git-sha", required=True)
    stress.add_argument("--output", type=Path, required=True)
    stress.add_argument("--rate", type=float, required=True)
    stress.add_argument("--dataset", default="aist-mid360-drive")
    stress.add_argument("--binary-relative")
    stress.add_argument("--replay-timeout", type=float, default=900.0)
    stress.add_argument("--readiness-timeout", type=float, default=30.0)
    stress.add_argument("--drain-timeout", type=float, default=120.0)
    host_parser = sub.add_parser("host")
    host_parser.add_argument("--workspace", type=Path, required=True)
    host_parser.add_argument("--expected-git-sha", required=True)
    host_parser.add_argument("--output", type=Path, required=True)
    # SITL and memory commands are intentionally explicit placeholders until
    # the session protocol has a measured-clock probe attached.
    for name in ("sitl-ab", "memory-run", "report"):
        sub.add_parser(name)
    return result


def main() -> int:
    args = parser().parse_args()
    if args.command in {"dataset-run", "dataset-stress"}:
        return dataset_run(args)
    if args.command == "dataset-compare":
        return dataset_compare(args)
    if args.command == "host":
        return host(args)
    raise SystemExit(f"{args.command} requires a dedicated measured protocol")


if __name__ == "__main__":
    raise SystemExit(main())

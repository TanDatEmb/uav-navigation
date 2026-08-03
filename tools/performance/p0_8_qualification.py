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
            "base_code_sha": args.base_code_sha,
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
            "base_code_sha": args.base_code_sha,
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


def require_valid_dataset_runs(runs: list[dict[str, Any]], side: str) -> None:
    for item in runs:
        run = item["run"]
        if not run.get("correctness", {}).get("pass", False):
            raise RuntimeError(f"{side} run failed correctness: {item['path']}")
        provenance = run.get("provenance", {})
        if provenance.get("git", {}).get("dirty") or any(
            item.get("dirty", False) for item in provenance.get("submodules", {}).values()
        ):
            raise RuntimeError(f"{side} run is not clean: {item['path']}")
        if not provenance.get("git", {}).get("full_sha") or not provenance.get("build", {}).get("binary_sha256"):
            raise RuntimeError(f"{side} run lacks complete provenance: {item['path']}")


def host_key(run: dict[str, Any]) -> tuple[Any, ...]:
    host = run["run"].get("provenance", {}).get("host", {})
    cpu_lines = tuple(line.strip() for line in host.get("cpu", "").splitlines()
                      if line.startswith(("Architecture:", "CPU(s):", "Model name:",
                                           "Thread(s) per core:", "Core(s) per socket:",
                                           "Socket(s):")))
    memory_lines = tuple(" ".join(line.strip().split()[:2])
                         for line in host.get("memory", "").splitlines()
                         if line.strip().startswith(("Mem:", "Swap:")))
    power_lines = tuple(line.strip() for line in host.get("power_profile", "").splitlines()
                        if "energy performance preference:" in line)
    return (
        host.get("kernel"),
        cpu_lines,
        memory_lines[:1],
        power_lines,
    )


def coefficient(values: list[float]) -> float:
    mean = statistics.mean(values) if values else 0.0
    return statistics.stdev(values) / mean if len(values) > 1 and mean else 0.0


def percentile_values(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, int(quantile * len(ordered))))]


def collect_runs(root: Path, role: str) -> list[dict[str, Any]]:
    runs = []
    for path in sorted((root / role).glob("run-*")):
        if not path.is_dir():
            continue
        summary = load_summary(path)
        run = json.loads((path / "run.json").read_text(encoding="utf-8"))
        item = {"path": str(path), "summary": summary, "run": run}
        item["resources"] = resource_snapshot(path)
        runs.append(item)
    return runs


def elapsed_seconds(value: str) -> float:
    parts = value.strip().split(":")
    if len(parts) == 2:
        return float(parts[0]) * 60.0 + float(parts[1])
    if len(parts) == 3:
        return float(parts[0]) * 3600.0 + float(parts[1]) * 60.0 + float(parts[2])
    return float(value)


def resource_snapshot(path: Path) -> dict[str, Any]:
    time_path = path.parent / f"{path.name}.time.txt"
    if not time_path.is_file():
        raise RuntimeError(f"missing /usr/bin/time artifact: {time_path}")
    labels = {
        "Elapsed (wall clock) time": "wall_time_s",
        "User time (seconds)": "user_cpu_s",
        "System time (seconds)": "system_cpu_s",
        "Maximum resident set size (kbytes)": "peak_rss_bytes",
        "Voluntary context switches": "voluntary_context_switches",
        "Involuntary context switches": "involuntary_context_switches",
    }
    values: dict[str, Any] = {}
    for raw_line in time_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        for label, key in labels.items():
            prefix = f"{label}:"
            if key == "wall_time_s" and line.startswith(label):
                raw = line.split("): ", 1)[-1].strip()
                values[key] = elapsed_seconds(raw)
            elif line.startswith(prefix):
                raw = line[len(prefix):].strip()
                values[key] = elapsed_seconds(raw) if key == "wall_time_s" else float(raw)
    missing = set(labels.values()) - set(values)
    if missing:
        raise RuntimeError(f"incomplete resource artifact {time_path}: {sorted(missing)}")
    values["peak_rss_bytes"] *= 1024.0
    process_path = path / "process.csv"
    if not process_path.is_file():
        raise RuntimeError(f"missing process sampler artifact: {process_path}")
    with process_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    cpu = [float(row["cpu_percent_one_logical_core"]) for row in rows]
    rss = [float(row["rss_bytes"]) for row in rows]
    values.update({
        "sampler_count": len(rows),
        "cpu_percent_mean": statistics.mean(cpu) if cpu else 0.0,
        "cpu_percent_p95": percentile_values(cpu, 0.95),
        "cpu_percent_max": max(cpu, default=0.0),
        "sampler_peak_rss_bytes": max(rss, default=0.0),
        "cpu_seconds": values["user_cpu_s"] + values["system_cpu_s"],
    })
    return values


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
    require_valid_dataset_runs(baseline, "baseline")
    require_valid_dataset_runs(candidate, "candidate")
    config_hashes = {
        side: {item["run"].get("provenance", {}).get("configuration", {}).get("sha256")
               for item in runs}
        for side, runs in (("baseline", baseline), ("candidate", candidate))
    }
    if any(len(values) != 1 or None in values for values in config_hashes.values()):
        raise RuntimeError(f"configuration changed within A/B side: {config_hashes}")
    if config_hashes["baseline"] != config_hashes["candidate"]:
        raise RuntimeError("baseline and candidate configurations differ")
    host_conditions_valid = len({host_key(item) for item in baseline + candidate}) == 1
    metrics = [
        "p95_pipeline_push_lidar_us", "p99_pipeline_push_lidar_us",
        "p95_corrected_scan_end_to_end_us", "p99_corrected_scan_end_to_end_us",
        "p95_registration_update_us", "p99_registration_update_us",
        "p95_result_processing_us", "p99_result_processing_us",
        "maximum_queue_depth", "worker_busy_ratio",
        "wall_time_s", "peak_rss_bytes", "cpu_seconds",
    ]
    comparison: dict[str, Any] = {
        "schema_version": 2,
        "metrics": {},
        "runs": {"baseline": baseline, "candidate": candidate},
        "host_conditions_valid": host_conditions_valid,
        "configuration_sha256": next(iter(config_hashes["candidate"])),
        "policy": {
            "p95_corrected_scan_end_to_end_us": {"relative_max": 0.10, "absolute_max_us": 2000.0},
            "p95_registration_update_us": {"relative_max": 0.10, "absolute_max_us": 2000.0},
            "p95_pipeline_push_lidar_us": {"relative_max": 0.15, "absolute_max_us": 500.0},
            "wall_time_s": {"relative_max": 0.10},
            "peak_rss_bytes": {"relative_max": 0.15},
            "maximum_queue_depth": {"relative_max": 0.25},
            "worker_busy_ratio": {"absolute_max": 0.10},
        },
    }
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
        elif metric in {"wall_time_s", "peak_rss_bytes", "cpu_seconds"}:
            values = {side: [float(x["resources"][metric]) for x in runs]
                      for side, runs in (("baseline", baseline), ("candidate", candidate))}
            stats = {side: {
                "count": len(items), "values": items, "median": statistics.median(items),
                "minimum": min(items), "maximum": max(items),
                "coefficient_of_variation": coefficient(items),
            } for side, items in values.items()}
        else:
            stats = {"baseline": statistics_for(baseline, metric), "candidate": statistics_for(candidate, metric)}
        b = stats["baseline"]["median"]
        c = stats["candidate"]["median"]
        comparison["metrics"][metric] = {
            "baseline": stats["baseline"], "candidate": stats["candidate"],
            "absolute_delta": c - b,
            "relative_delta": (c / b - 1.0) if b else None,
            "baseline_range": [min(stats["baseline"]["values"]), max(stats["baseline"]["values"])],
            "candidate_range": [min(stats["candidate"]["values"]), max(stats["candidate"]["values"])],
            "baseline_coefficient_of_variation": coefficient(stats["baseline"]["values"]),
            "candidate_coefficient_of_variation": coefficient(stats["candidate"]["values"]),
        }
        rows.append([metric, b, c, c - b, (c / b - 1.0) if b else ""])
    comparison["acceptance"] = {
        "host_conditions": host_conditions_valid,
        "correctness": True,
        "queue_median": comparison["metrics"]["maximum_queue_depth"]["relative_delta"] <= 0.25,
        "queue_worst_case": max(comparison["metrics"]["maximum_queue_depth"]["candidate"]["values"])
        <= 1.5 * max(comparison["metrics"]["maximum_queue_depth"]["baseline"]["values"]),
        "worker_busy_ratio": comparison["metrics"]["worker_busy_ratio"]["absolute_delta"] <= 0.10,
    }
    for metric, policy in comparison["policy"].items():
        if metric not in comparison["metrics"]:
            continue
        result = comparison["metrics"][metric]
        relative_ok = policy.get("relative_max") is None or result["relative_delta"] <= policy["relative_max"]
        absolute_ok = policy.get("absolute_max_us") is None or result["absolute_delta"] <= policy["absolute_max_us"]
        comparison["acceptance"][metric] = relative_ok or absolute_ok
    comparison["pass"] = all(comparison["acceptance"].values())
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


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"missing artifact: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def sitl_ab(args: argparse.Namespace) -> int:
    root = args.root.resolve()
    sides = {side: sorted((root / side).glob("run-*.json")) for side in ("off", "on")}
    if any(len(paths) < args.min_runs for paths in sides.values()):
        raise RuntimeError("SITL A/B requires three valid runs per side")
    required = {
        "comparison_valid_ratio", "monitoring_available_ratio", "query_timeout_count",
        "query_generation_mismatch_count", "query_rtt_p95_ms", "query_rtt_p99_ms",
        "alignment_gap_p99_ms", "aligned_comparison_age_p99_ms", "supervisor_cpu_p95_percent",
        "supervisor_peak_rss_bytes", "fast_lio_corrected_p95_us", "fast_lio_max_queue_depth",
    }
    loaded = {side: [_load_json(path) for path in paths[:args.min_runs]] for side, paths in sides.items()}
    missing = sorted(required - set(loaded["off"][0]) - set(loaded["on"][0]))
    if missing:
        raise RuntimeError(f"SITL metrics not instrumented: {', '.join(missing)}")
    def median(side: str, key: str) -> float:
        return statistics.median(float(item[key]) for item in loaded[side])
    result = {"schema_version": 1, "warmup_s": args.warmup_s, "measurement_s": args.measure_s,
              "runs": {side: loaded[side] for side in loaded}, "metrics": {}}
    for key in sorted(required):
        result["metrics"][key] = {side: [float(item[key]) for item in loaded[side]] for side in loaded}
        result["metrics"][key]["median_off"] = median("off", key)
        result["metrics"][key]["median_on"] = median("on", key)
    result["acceptance"] = {
        "off_and_on_correct": all(
            item["comparison_valid_ratio"] >= 0.99 and item["monitoring_available_ratio"] >= 0.99 and
            item["query_timeout_count"] == 0 and item["query_generation_mismatch_count"] == 0 and
            item["query_rtt_p95_ms"] < 50 and item["query_rtt_p99_ms"] < 100 and
            item["alignment_gap_p99_ms"] <= 50 and item["aligned_comparison_age_p99_ms"] <= 150 and
            item["supervisor_cpu_p95_percent"] < 15 and item["supervisor_peak_rss_bytes"] < 150 * 1024 * 1024
            for side in loaded for item in loaded[side]
        ),
        "no_new_lio_overhead": median("on", "fast_lio_corrected_p95_us")
        <= 1.05 * median("off", "fast_lio_corrected_p95_us") and
        median("on", "fast_lio_max_queue_depth") <= 1.20 * median("off", "fast_lio_max_queue_depth"),
    }
    result["pass"] = all(result["acceptance"].values())
    write_json(args.output.resolve(), result)
    return 0 if result["pass"] else 2


def memory_run(args: argparse.Namespace) -> int:
    rows = []
    with args.input.resolve().open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("memory input is empty")
    sim_times = [float(row["sim_time_s"]) for row in rows]
    if max(sim_times) < args.minimum_duration_s:
        raise RuntimeError("memory run is shorter than the required 20 simulated minutes")
    stable = [row for row in rows if float(row["sim_time_s"]) >= args.warmup_s]
    rss = [int(row["supervisor_rss_bytes"]) for row in stable]
    first = rss[0]
    result = {
        "schema_version": 1, "input": str(args.input.resolve()), "sample_count": len(rows),
        "duration_s": max(sim_times), "warmup_s": args.warmup_s,
        "rss_first_after_warmup": first, "rss_max_after_warmup": max(rss),
        "rss_growth_ratio": (max(rss) / first - 1.0) if first else None,
        "max_outstanding_queries": max(int(row["outstanding_queries"]) for row in rows),
        "state_transition_count": max(int(row["state_transition_count"]) for row in rows),
    }
    result["acceptance"] = {
        "rss_growth": result["rss_growth_ratio"] is not None and result["rss_growth_ratio"] < 0.10,
        "outstanding_queries": result["max_outstanding_queries"] <= 1,
    }
    result["pass"] = all(result["acceptance"].values())
    write_json(args.output.resolve(), result)
    return 0 if result["pass"] else 2


def report(args: argparse.Namespace) -> int:
    sections = {}
    for name, path in (("dataset", args.dataset), ("sitl", args.sitl), ("memory", args.memory)):
        if path is not None:
            sections[name] = _load_json(path.resolve())
    if args.stress:
        sections["stress"] = {
            str(path.resolve()): _load_json(path.resolve()) for path in args.stress
        }
    if not sections:
        raise RuntimeError("report requires at least one measured qualification artifact")
    result = {"schema_version": 1, "qualification": "p0.8-performance", "sections": sections}
    result["pass"] = all(section.get("pass", False) for section in sections.values())
    write_json(args.output.resolve(), result)
    return 0 if result["pass"] else 2


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
    run.add_argument("--base-code-sha", default=None)
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
    stress.add_argument("--base-code-sha", default=None)
    stress.add_argument("--replay-timeout", type=float, default=900.0)
    stress.add_argument("--readiness-timeout", type=float, default=30.0)
    stress.add_argument("--drain-timeout", type=float, default=120.0)
    host_parser = sub.add_parser("host")
    host_parser.add_argument("--workspace", type=Path, required=True)
    host_parser.add_argument("--expected-git-sha", required=True)
    host_parser.add_argument("--output", type=Path, required=True)
    sitl = sub.add_parser("sitl-ab")
    sitl.add_argument("--root", type=Path, required=True)
    sitl.add_argument("--output", type=Path, required=True)
    sitl.add_argument("--min-runs", type=int, default=3)
    sitl.add_argument("--warmup-s", type=float, default=30.0)
    sitl.add_argument("--measure-s", type=float, default=120.0)
    memory = sub.add_parser("memory-run")
    memory.add_argument("--input", type=Path, required=True)
    memory.add_argument("--output", type=Path, required=True)
    memory.add_argument("--warmup-s", type=float, default=120.0)
    memory.add_argument("--minimum-duration-s", type=float, default=1200.0)
    report_parser = sub.add_parser("report")
    report_parser.add_argument("--dataset", type=Path)
    report_parser.add_argument("--sitl", type=Path)
    report_parser.add_argument("--memory", type=Path)
    report_parser.add_argument("--stress", type=Path, action="append")
    report_parser.add_argument("--output", type=Path, required=True)
    return result


def main() -> int:
    args = parser().parse_args()
    if args.command in {"dataset-run", "dataset-stress"}:
        return dataset_run(args)
    if args.command == "dataset-compare":
        return dataset_compare(args)
    if args.command == "host":
        return host(args)
    if args.command == "sitl-ab":
        return sitl_ab(args)
    if args.command == "memory-run":
        return memory_run(args)
    if args.command == "report":
        return report(args)
    raise SystemExit(f"unsupported command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())

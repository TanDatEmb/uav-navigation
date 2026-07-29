#!/usr/bin/env python3
"""Repeat an offline dataset run and preserve crash-forensics artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Sequence

import yaml

ROOT = Path(__file__).resolve().parents[2]


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    paths = [path] if path.is_file() else sorted(
        item for item in path.rglob("*") if item.is_file()
    )
    for item in paths:
        if path.is_dir():
            digest.update(item.relative_to(path).as_posix().encode("utf-8"))
            digest.update(b"\0")
        with item.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def git_sha() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, text=True,
        capture_output=True,
    ).stdout.strip()


def enable_core_dumps() -> None:
    resource.setrlimit(resource.RLIMIT_CORE, (resource.RLIM_INFINITY,
                                               resource.RLIM_INFINITY))


def resident_set_bytes(pid: int) -> int | None:
    try:
        for line in Path(f"/proc/{pid}/status").read_text(
            encoding="utf-8"
        ).splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError):
        pass
    return None


def monitor_process(process: subprocess.Popen[Any], result: dict[str, Any]) -> None:
    peak = 0
    thread_ids: set[int] = set()
    while process.poll() is None:
        observed = resident_set_bytes(process.pid)
        if observed is not None:
            peak = max(peak, observed)
        try:
            thread_ids.update(
                int(path.name) for path in Path(f"/proc/{process.pid}/task").iterdir()
            )
        except (OSError, ValueError):
            pass
        time.sleep(0.05)
    result["peak_rss_bytes"] = peak
    result["thread_ids"] = sorted(thread_ids)


def core_metadata(directory: Path) -> list[dict[str, Any]]:
    result = []
    for path in sorted(directory.glob("core*")):
        if not path.is_file():
            continue
        stat = path.stat()
        result.append({
            "path": str(path),
            "size_bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
            "sha256": sha256_path(path),
        })
    return result


def gdb_backtrace(command: Sequence[str], directory: Path, core: Path | None) -> dict[str, Any]:
    gdb = shutil.which("gdb")
    output = directory / "backtrace.txt"
    if gdb is None:
        output.write_text("gdb is not installed\n", encoding="utf-8")
        return {"attempted": False, "reason": "gdb is not installed"}
    if core is not None and Path(command[0]).is_file():
        invocation = [
            gdb, "--batch", "-ex", "set pagination off", "-ex", "info threads",
            "-ex", "thread apply all bt full", "-ex", "info registers",
            command[0], str(core),
        ]
        kind = "core"
    else:
        invocation = [
            gdb, "--batch", "-ex", "set pagination off", "-ex", "run",
            "-ex", "info threads", "-ex", "thread apply all bt full",
            "-ex", "info registers", "--args", *command,
        ]
        kind = "crash-rerun"
    completed = subprocess.run(
        invocation, cwd=directory, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, preexec_fn=enable_core_dumps,
    )
    output.write_text(completed.stdout, encoding="utf-8")
    return {"attempted": True, "kind": kind, "returncode": completed.returncode}


def run_iteration(
    iteration: int,
    directory: Path,
    command: list[str],
    common: dict[str, Any],
    diagnose: bool = True,
) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=False)
    runner_output = directory / "runner_output"
    runner_output.mkdir()
    actual_command = [
        str(runner_output) if token == "{output}" else token for token in command
    ]
    started_ns = time.time_ns()
    run = {
        **common,
        "schema_version": 1,
        "iteration": iteration,
        "command": actual_command,
        "started_wall_time_ns": started_ns,
        "core_limit": "unlimited",
    }
    atomic_json(directory / "run.json", run)
    process_observations: dict[str, Any] = {}
    with (directory / "stdout.log").open("w", encoding="utf-8") as stdout, (
        directory / "stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(
            actual_command, cwd=directory, stdout=stdout, stderr=stderr,
            preexec_fn=enable_core_dumps,
        )
        run["process_id"] = process.pid
        atomic_json(directory / "run.json", run)
        monitor = threading.Thread(
            target=monitor_process, args=(process, process_observations)
        )
        monitor.start()
        returncode = process.wait()
        monitor.join()
    elapsed_ns = time.time_ns() - started_ns
    signal_number = -returncode if returncode < 0 else None
    cores = core_metadata(directory)
    crashed = signal_number is not None
    backtrace = None
    if diagnose and crashed:
        diagnostic_output = directory / "gdb_runner_output"
        diagnostic_command = [
            str(diagnostic_output) if token == str(runner_output) else token
            for token in actual_command
        ]
        backtrace = gdb_backtrace(
            diagnostic_command, directory,
            Path(cores[0]["path"]) if cores else None,
        )
    else:
        (directory / "backtrace.txt").write_text(
            "not requested: process did not terminate from a signal\n"
            if not crashed else "disabled by --no-gdb\n",
            encoding="utf-8",
        )
    estimator_state = extract_estimator_state(runner_output)
    summary = {
        "schema_version": 1,
        "iteration": iteration,
        "mode": common["mode"],
        "returncode": returncode,
        "signal": signal_number,
        "signal_name": (
            signal.Signals(signal_number).name
            if signal_number in signal.Signals._value2member_map_ else None
        ),
        "crashed": crashed,
        "runtime_ns": elapsed_ns,
        "peak_rss_bytes": process_observations.get("peak_rss_bytes", 0),
        "thread_ids": process_observations.get("thread_ids", []),
        **estimator_state,
        "core_files": cores,
        "backtrace": backtrace,
    }
    atomic_json(directory / "summary.json", summary)
    atomic_json(directory / "core_metadata.json", {"core_files": cores})
    return summary


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def extract_estimator_state(output: Path) -> dict[str, Any]:
    runner = read_json(output / "summary.json")
    last_record = None
    last_map_before = runner.get("map_size_before_insert")
    last_map_after = runner.get("map_size_after_insert")
    crop_state: dict[str, Any] = {}
    diagnostics = output / "diagnostics.csv"
    try:
        lines = diagnostics.read_text(encoding="utf-8").splitlines()
        if len(lines) > 1:
            header = lines[0].split(",")
            row = lines[-1].split(",")
            values = dict(zip(header, row))
            last_record = int(values["record_index"])
            crop_state = {
                "crop_performed": values.get("crop_performed") == "1",
                "crop_removed_count": int(values.get("crop_removed_count", 0)),
                "map_size_after_maintenance": int(
                    values.get("map_size_after_maintenance", 0)
                ),
            }
    except (OSError, ValueError):
        pass
    return {
        "last_input_record": last_record,
        "last_lidar_scan": runner.get("raw_dataset_lidar_count"),
        "last_successful_correction": runner.get("successful_correction_count"),
        "map_size_before_insert": last_map_before,
        "map_size_after_insert": last_map_after,
        "crop_prune_state": crop_state,
        "ikd_tree_rebuild_state": "synchronous-idle",
    }


def data_home() -> Path:
    configured = os.environ.get("UAV_NAV_DATA_HOME")
    base = Path(
        os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share")
    )
    return Path(configured).expanduser() if configured else base / "uav-nav"


def resolve_dataset(dataset: str) -> tuple[Path, Path]:
    candidate = Path(dataset).expanduser()
    if candidate.exists():
        if not candidate.is_dir():
            raise ValueError("dataset path must be a ROS bag directory")
        raise ValueError("--config is required when DATASET is a path")

    legacy = ROOT / "data" / dataset / "dataset.yaml"
    if legacy.is_file():
        manifest = yaml.safe_load(legacy.read_text(encoding="utf-8"))
        return legacy.parent / manifest["bag"], ROOT / manifest["config"]["path"]

    catalog_path = ROOT / "datasets" / "catalog" / f"{dataset}.yaml"
    if not catalog_path.is_file():
        raise ValueError(f"unknown dataset id: {dataset}")
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    external = data_home() / "datasets" / dataset / "lio"
    config = ROOT / "src/navigation_estimator/fast_lio_ros/config/data" / f"{dataset}.yaml"
    if external.is_dir():
        return external, config

    archive_name = str(catalog.get("archive", {}).get("filename", ""))
    identifying_token = Path(archive_name).stem.removeprefix("rosbag2_")
    for metadata in sorted((ROOT / "data").glob("*/bag/metadata.yaml")):
        if identifying_token and identifying_token in metadata.read_text(encoding="utf-8"):
            return metadata.parent, config
    raise ValueError(
        f"{dataset} is not prepared; run data-get/data-prepare or pass a bag path"
    )


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--dataset")
    result.add_argument("--bag", type=Path)
    result.add_argument("--config", type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument(
        "--max-lidar", type=int, default=int(os.environ.get("MAX_LIDAR", "0"))
    )
    result.add_argument(
        "--repeat", type=int, default=int(os.environ.get("REPEAT", "1"))
    )
    result.add_argument("--mode", default="production-synchronous")
    result.add_argument(
        "--runner",
        nargs="+",
        help="command template; use {output} for its output directory",
    )
    result.add_argument("--no-gdb", action="store_true")
    return result


def main() -> int:
    args = parser().parse_args()
    if args.dataset:
        try:
            bag, config = resolve_dataset(args.dataset)
        except ValueError as error:
            parser().error(str(error))
    elif args.bag and args.config:
        bag, config = args.bag, args.config
    else:
        parser().error("use --dataset or both --bag and --config")
    bag = bag.expanduser().resolve()
    config = config.expanduser().resolve()
    dataset_name = args.dataset or bag.name
    output = (
        args.output.expanduser().resolve()
        if args.output
        else ROOT / ".artifacts" / "runtime" / dataset_name / git_sha()
    )
    if not bag.exists() or not config.is_file():
        parser().error("bag and config must exist")
    if args.repeat <= 0 or args.max_lidar < 0:
        parser().error("repeat must be positive and max-lidar nonnegative")
    output.mkdir(parents=True, exist_ok=False)
    command = args.runner or [
        "ros2", "run", "fast_lio_tools", "lio_offline",
        str(bag), str(config), "{output}",
    ]
    if args.max_lidar:
        command.append(str(args.max_lidar))
    common = {
        "git_sha": git_sha(),
        "mode": args.mode,
        "rebuild_mode": "production-synchronous",
        "dataset_path": str(bag),
        "dataset_sha256": sha256_path(bag),
        "config_path": str(config),
        "config_sha256": sha256_path(config),
        "max_lidar": args.max_lidar or None,
        "repeat": args.repeat,
    }
    summaries = []
    for index in range(1, args.repeat + 1):
        summaries.append(run_iteration(
            index, output / f"iteration-{index:04d}", command, common,
            diagnose=not args.no_gdb,
        ))
    aggregate = {
        **common,
        "schema_version": 1,
        "iterations": summaries,
        "crash_count": sum(bool(item["crashed"]) for item in summaries),
        "failure_count": sum(item["returncode"] != 0 for item in summaries),
    }
    atomic_json(output / "summary.json", aggregate)
    print(output)
    return 1 if aggregate["failure_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

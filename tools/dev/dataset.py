#!/usr/bin/env python3
"""Manifest-driven developer workflow for estimator datasets."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[2]
REQUIRED_OUTPUTS = (
    "run.json",
    "summary.json",
    "diagnostics.csv",
    "trajectory.csv",
    "corrections.csv",
    "local_map.pcd",
    "stdout.log",
    "stderr.log",
)


class DatasetError(RuntimeError):
    pass


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise DatasetError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(value, dict):
        raise DatasetError(f"{path} must contain a YAML mapping")
    return value


def resolve_dataset(value: str, data_root: Path) -> tuple[Path, dict[str, Any]]:
    candidate = Path(value).expanduser()
    if not candidate.is_absolute() and not candidate.exists():
        candidate = data_root / candidate
    directory = candidate.resolve()
    manifest_path = directory / "dataset.yaml"
    if not manifest_path.is_file():
        raise DatasetError(f"dataset manifest not found: {manifest_path}")
    manifest = load_yaml(manifest_path)
    for key in ("name", "bag", "config", "input"):
        if key not in manifest:
            raise DatasetError(f"dataset manifest missing required field: {key}")
    return directory, manifest


def resolve_paths(directory: Path, manifest: dict[str, Any]) -> tuple[Path, Path]:
    bag = (directory / str(manifest["bag"])).resolve()
    config_value = manifest["config"]
    if not isinstance(config_value, dict) or not config_value.get("path"):
        raise DatasetError("dataset config.path is required")
    config = Path(str(config_value["path"])).expanduser()
    if not config.is_absolute():
        config = (ROOT / config).resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise DatasetError(f"invalid rosbag2 directory: {bag}")
    if not config.is_file():
        raise DatasetError(f"estimator config not found: {config}")
    return bag, config


def validate_topics(bag: Path, manifest: dict[str, Any]) -> dict[str, int]:
    metadata = load_yaml(bag / "metadata.yaml")
    info = metadata.get("rosbag2_bagfile_information", {})
    entries = info.get("topics_with_message_count", [])
    topics: dict[str, tuple[str, int]] = {}
    for entry in entries:
        topic = entry.get("topic_metadata", {})
        topics[str(topic.get("name"))] = (
            str(topic.get("type")),
            int(entry.get("message_count", 0)),
        )
    input_config = manifest["input"]
    expected_type = {
        "pointcloud2": "sensor_msgs/msg/PointCloud2",
        "livox_custom": "livox_ros_driver2/msg/CustomMsg",
    }.get(str(input_config.get("lidar_message_type")))
    if expected_type is None:
        raise DatasetError("input.lidar_message_type must be pointcloud2 or livox_custom")
    required = {
        str(input_config.get("lidar_topic")): expected_type,
        str(input_config.get("imu_topic")): "sensor_msgs/msg/Imu",
    }
    counts: dict[str, int] = {}
    for topic, message_type in required.items():
        actual = topics.get(topic)
        if actual is None:
            raise DatasetError(f"required topic absent from bag: {topic}")
        if actual[0] != message_type:
            raise DatasetError(
                f"topic {topic} has type {actual[0]}, expected {message_type}"
            )
        counts[topic] = actual[1]
    expected = manifest.get("expected") or {}
    for key, topic in (
        ("lidar_count", str(input_config.get("lidar_topic"))),
        ("imu_count", str(input_config.get("imu_topic"))),
    ):
        wanted = expected.get(key)
        if wanted is not None and counts[topic] != int(wanted):
            raise DatasetError(
                f"{key} is {counts[topic]}, expected {int(wanted)}"
            )
    return counts


def git_short_sha() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def new_run_directory(name: str, action: str) -> Path:
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = ROOT / ".artifacts" / "datasets" / name / f"{git_short_sha()}-{action}-{run_id}"
    output.mkdir(parents=True, exist_ok=False)
    return output


def write_manifest(
    output: Path,
    action: str,
    directory: Path,
    bag: Path,
    config: Path,
    manifest: dict[str, Any],
    counts: dict[str, int],
) -> None:
    payload = {
        "schema_version": 1,
        "action": action,
        "dataset": str(manifest["name"]),
        "dataset_directory": str(directory),
        "bag_directory": str(bag),
        "config_path": str(config),
        "git_short_sha": git_short_sha(),
        "input": manifest["input"],
        "observed_counts": counts,
    }
    (output / "run.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def inspect_dataset(args: argparse.Namespace) -> int:
    directory, manifest = resolve_dataset(args.dataset, args.data_root)
    bag, config = resolve_paths(directory, manifest)
    counts = validate_topics(bag, manifest)
    output = new_run_directory(str(manifest["name"]), "inspect")
    write_manifest(output, "inspect", directory, bag, config, manifest, counts)
    print(json.dumps({"dataset": manifest["name"], "counts": counts, "output": str(output)}))
    return 0


def run_offline(args: argparse.Namespace, smoke: bool) -> int:
    directory, manifest = resolve_dataset(args.dataset, args.data_root)
    bag, config = resolve_paths(directory, manifest)
    counts = validate_topics(bag, manifest)
    action = "smoke" if smoke else "run"
    output = new_run_directory(str(manifest["name"]), action)
    write_manifest(output, action, directory, bag, config, manifest, counts)
    command = [
        "ros2", "run", "fast_lio_tools", "lio_offline",
        str(bag), str(config), str(output),
    ]
    if smoke:
        command.append(str(args.max_lidar))
    with (output / "stdout.log").open("w", encoding="utf-8") as stdout, (
        output / "stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr)
    if result.returncode:
        raise DatasetError(f"offline runner failed with exit code {result.returncode}: {output}")
    missing = [name for name in REQUIRED_OUTPUTS if not (output / name).is_file()]
    if missing:
        raise DatasetError(f"runner omitted required outputs: {', '.join(missing)}")
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    gates = {
        "adapter rejection count": summary.get("core_rejected_lidar_count") == 0,
        "invalid timestamp rejection count": summary.get(
            "invalid_timestamp_rejected_count"
        ) == 0,
        "accepted LiDAR count": int(summary.get("core_accepted_lidar_count", 0)) > 0,
        "correction output": int(summary.get("successful_correction_count", 0)) > 0,
        "per-point deskew": bool(summary.get("deskew_applied")),
    }
    failed = [name for name, passed in gates.items() if not passed]
    if failed:
        raise DatasetError(f"dataset acceptance gates failed: {', '.join(failed)}")
    for csv_name in ("trajectory.csv", "corrections.csv", "diagnostics.csv"):
        with (output / csv_name).open(encoding="utf-8", newline="") as stream:
            for row in csv.reader(stream):
                if any(
                    value.strip().lower()
                    in {"nan", "+nan", "-nan", "inf", "+inf", "-inf", "infinity"}
                    for value in row
                ):
                    raise DatasetError(f"non-finite value found in {csv_name}")
    print(output)
    return 0


def run_ros(args: argparse.Namespace) -> int:
    directory, manifest = resolve_dataset(args.dataset, args.data_root)
    bag, config = resolve_paths(directory, manifest)
    counts = validate_topics(bag, manifest)
    output = new_run_directory(str(manifest["name"]), "ros")
    write_manifest(output, "ros", directory, bag, config, manifest, counts)
    replay_command = [
        "ros2", "bag", "play", str(bag), "--rate", str(args.rate),
    ]
    with (output / "stdout.log").open("w", encoding="utf-8") as stdout, (
        output / "stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        node = subprocess.Popen(
            [
                "ros2", "run", "fast_lio_ros", "fast_lio_node",
                "--ros-args", "--params-file", str(config),
            ],
            cwd=ROOT,
            stdout=stdout,
            stderr=stderr,
        )
        try:
            time.sleep(1.0)
            if node.poll() is not None:
                raise DatasetError(
                    f"estimator node exited before replay with code {node.returncode}"
                )
            replay = subprocess.run(
                replay_command, cwd=ROOT, stdout=stdout, stderr=stderr
            )
        finally:
            if node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    node.terminate()
                    node.wait(timeout=5)
    if replay.returncode:
        raise DatasetError(
            f"ROS replay failed with exit code {replay.returncode}: {output}"
        )
    if node.returncode not in (0, -signal.SIGINT, 130):
        raise DatasetError(
            f"estimator node failed with exit code {node.returncode}: {output}"
        )
    (output / "summary.json").write_text(
        json.dumps(
            {
                "dataset": manifest["name"],
                "rate": args.rate,
                "replay_returncode": replay.returncode,
                "estimator_returncode": node.returncode,
                "note": "Detailed transport and estimator counters are published on /diagnostics.",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(output)
    return 0


def view_dataset(args: argparse.Namespace) -> int:
    directory, manifest = resolve_dataset(args.dataset, args.data_root)
    base = ROOT / ".artifacts" / "datasets" / str(manifest["name"])
    maps = sorted(base.glob("*/local_map.pcd"), key=os.path.getmtime)
    if not maps:
        raise DatasetError(f"no local registration map found under {base}")
    viewer = shutil.which("pcl_viewer")
    if viewer is None:
        raise DatasetError("pcl_viewer is not installed")
    return subprocess.run([viewer, str(maps[-1])], cwd=directory).returncode


def clean(paths: tuple[str, ...]) -> int:
    for relative in paths:
        target = (ROOT / relative).resolve()
        if target.parent != ROOT and target != ROOT / ".artifacts":
            raise DatasetError(f"refusing unsafe clean target: {target}")
        if target.is_dir():
            shutil.rmtree(target)
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--data-root", type=Path, default=ROOT / "data")
    sub = result.add_subparsers(dest="action", required=True)
    sub.add_parser("help")
    sub.add_parser("clean")
    sub.add_parser("clean-artifacts")
    for name in ("inspect", "view"):
        child = sub.add_parser(name)
        child.add_argument("--dataset", required=True)
    for name in ("smoke", "run"):
        child = sub.add_parser(name)
        child.add_argument("--dataset", required=True)
        child.add_argument("--max-lidar", type=int, default=20)
    child = sub.add_parser("ros")
    child.add_argument("--dataset", required=True)
    child.add_argument("--rate", type=float, default=1.0)
    return result


def main() -> int:
    args = parser().parse_args()
    args.data_root = args.data_root.expanduser().resolve()
    if args.action == "help":
        print(
            """UAV navigation developer targets:
  make build | test | check | clean | clean-artifacts
  make data-info DATASET=aist-mid360-drive
  make data-smoke DATASET=/absolute/or/relative/dataset
  make data-run DATASET=aist-mid360-drive
  make data-replay DATASET=aist-mid360-drive RATE=1.0
  make data-view DATASET=aist-mid360-drive"""
        )
        return 0
    if args.action == "clean":
        return clean(("build", "install", "log"))
    if args.action == "clean-artifacts":
        return clean((".artifacts",))
    if args.action == "inspect":
        return inspect_dataset(args)
    if args.action == "smoke":
        return run_offline(args, True)
    if args.action == "run":
        return run_offline(args, False)
    if args.action == "ros":
        return run_ros(args)
    if args.action == "view":
        return view_dataset(args)
    raise DatasetError(f"unsupported action: {args.action}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DatasetError, OSError, subprocess.SubprocessError) as error:
        print(f"dataset workflow error: {error}", file=sys.stderr)
        raise SystemExit(1)

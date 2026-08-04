#!/usr/bin/env python3
"""Validate the prepared dataset consumed by ``make dataset-check``.

Dataset acquisition and preparation are intentionally outside the runtime
acceptance workflow.  This module only owns the catalog contract and the
read-only checks needed before replay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "datasets" / "catalog"
RAW_SUFFIXES = {".bag", ".mcap", ".db3"}
GENERATED_LIMIT = 10 * 1024 * 1024
PREPARED_SCHEMA_VERSION = 2
CANONICAL_LIDAR_FRAME = "livox_frame"
CANONICAL_IMU_FRAME = "livox_imu_frame"


class DataError(RuntimeError):
    """A dataset registry or prepared-bag contract error."""


def data_home() -> Path:
    configured = os.environ.get("UAV_NAV_DATA_HOME")
    if configured:
        return Path(configured).expanduser().resolve()
    base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share"))
    return (base / "uav-nav").resolve()


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise DataError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(value, dict):
        raise DataError(f"{path} must contain a YAML mapping")
    return value


def validate_entry(path: Path, item: dict[str, Any]) -> None:
    required = {
        "id", "source", "publisher", "landing_page", "license", "size_bytes",
        "download", "archive", "bag", "input", "ground_truth", "cases",
    }
    missing = sorted(required - item.keys())
    if missing:
        raise DataError(f"{path}: missing fields: {', '.join(missing)}")
    dataset_id = item["id"]
    if not isinstance(dataset_id, str) or not dataset_id or any(
        char not in "abcdefghijklmnopqrstuvwxyz0123456789-" for char in dataset_id
    ):
        raise DataError(f"{path}: invalid lowercase kebab-case id")
    download = item["download"]
    if not isinstance(download, dict) or download.get("mode") not in {"auto", "manual", "local"}:
        raise DataError(f"{path}: download.mode must be auto, manual, or local")
    checksum = download.get("checksum")
    if checksum:
        algorithm = checksum.get("algorithm")
        value = str(checksum.get("value", "")).lower()
        lengths = {"md5": 32, "sha256": 64}
        if algorithm not in lengths or len(value) != lengths[algorithm]:
            raise DataError(f"{path}: malformed checksum")
        try:
            int(value, 16)
        except ValueError as error:
            raise DataError(f"{path}: checksum is not hexadecimal") from error


def entries() -> dict[str, tuple[Path, dict[str, Any]]]:
    result: dict[str, tuple[Path, dict[str, Any]]] = {}
    for path in sorted(CATALOG.glob("*.yaml")):
        item = load_yaml(path)
        validate_entry(path, item)
        dataset_id = str(item["id"])
        if dataset_id in result:
            raise DataError(f"duplicate dataset id: {dataset_id}")
        result[dataset_id] = (path, item)
    return result


def lookup(dataset_id: str) -> dict[str, Any]:
    found = entries().get(dataset_id)
    if found is None:
        raise DataError(f"unknown dataset id: {dataset_id}")
    return found[1]


def digest(path: Path, algorithm: str) -> str:
    value = hashlib.new(algorithm)
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def blob_path(home: Path, entry: dict[str, Any]) -> Path:
    checksum = entry["download"].get("checksum")
    if not isinstance(checksum, dict):
        raise DataError(f"{entry['id']} has no content checksum")
    return home / "archives" / f"{checksum['algorithm']}-{str(checksum['value']).lower()}"


def verify_blob(path: Path, checksum: dict[str, str]) -> None:
    observed = digest(path, checksum["algorithm"])
    expected = checksum["value"].lower()
    if observed != expected:
        raise DataError(f"checksum mismatch for {path}: expected {expected}, observed {observed}")


def normalize_message_frame(message: object, canonical_frame: str) -> str:
    header = getattr(message, "header", None)
    if header is None or not hasattr(header, "frame_id"):
        raise DataError("selected ROS message has no header.frame_id")
    source_frame = str(header.frame_id)
    if not source_frame:
        raise DataError("selected ROS message has an empty header.frame_id")
    header.frame_id = canonical_frame
    return source_frame


def prepared_status_schema(status: dict[str, Any], *, path: Path | None = None) -> None:
    observed = status.get("schema_version")
    if observed != PREPARED_SCHEMA_VERSION:
        location = f" in {path}" if path is not None else ""
        raise DataError(
            f"prepared dataset schema{location} is {observed!r}; expected {PREPARED_SCHEMA_VERSION}"
        )


def config_path(dataset_id: str) -> Path:
    if dataset_id != "aist-mid360-drive":
        raise DataError(f"no canonical dataset config is defined for {dataset_id}")
    return ROOT / "config/runtime/dataset.yaml"


def dataset_context(value: str, home: Path) -> dict[str, Any]:
    candidate = Path(value).expanduser()
    if candidate.exists():
        directory = candidate.resolve()
        manifest_path = directory / "dataset.yaml"
        if manifest_path.is_file():
            manifest = load_yaml(manifest_path)
            try:
                bag = (directory / str(manifest["bag"])).resolve()
                dataset_id = str(manifest["name"])
                input_config = dict(manifest["input"])
                configured = Path(str(manifest["config"]["path"])).expanduser()
            except (KeyError, TypeError, ValueError) as error:
                raise DataError(f"invalid dataset manifest: {manifest_path}") from error
            config = configured if configured.is_absolute() else ROOT / configured
        elif (directory / "metadata.yaml").is_file():
            entry = lookup("aist-mid360-drive")
            dataset_id = str(entry["id"])
            bag = directory
            input_config = dict(entry["input"])
            config = config_path(dataset_id)
        else:
            raise DataError(f"dataset path needs dataset.yaml or metadata.yaml: {directory}")
        return {
            "id": dataset_id,
            "directory": directory,
            "bag": bag,
            "config": config.resolve(),
            "input": input_config,
        }

    entry = lookup(value)
    prepared = home / "datasets" / value
    status_path = prepared / "status.json"
    bag = prepared / "lio"
    if not status_path.is_file() or not bag.is_dir():
        raise DataError(f"{value} is not prepared; provide a prepared dataset under {home}")
    prepared_status_schema(load_yaml(status_path), path=status_path)
    return {
        "id": value,
        "directory": prepared,
        "bag": bag,
        "config": config_path(value),
        "input": dict(entry["input"]),
    }


def bag_topic_counts(context: dict[str, Any]) -> dict[str, int]:
    bag = Path(context["bag"])
    config = Path(context["config"])
    if not (bag / "metadata.yaml").is_file():
        raise DataError(f"invalid ROS 2 bag: {bag}")
    if not config.is_file():
        raise DataError(f"estimator config not found: {config}")
    metadata = load_yaml(bag / "metadata.yaml")
    topic_entries = metadata.get("rosbag2_bagfile_information", {}).get(
        "topics_with_message_count", []
    )
    observed = {
        str(item.get("topic_metadata", {}).get("name")): (
            str(item.get("topic_metadata", {}).get("type")),
            int(item.get("message_count", 0)),
        )
        for item in topic_entries
    }
    input_config = context["input"]
    lidar_type = {
        "pointcloud2": "sensor_msgs/msg/PointCloud2",
        "livox_custom": "livox_ros_driver2/msg/CustomMsg",
    }.get(str(input_config.get("lidar_message_type")))
    required = {
        str(input_config["lidar_topic"]): lidar_type,
        str(input_config["imu_topic"]): "sensor_msgs/msg/Imu",
    }
    counts: dict[str, int] = {}
    for topic, expected_type in required.items():
        actual = observed.get(topic)
        if actual is None or actual[0] != expected_type or actual[1] <= 0:
            raise DataError(
                f"invalid required topic {topic}: observed={actual}, expected_type={expected_type}"
            )
        counts[topic] = actual[1]
    if set(observed) != set(required):
        raise DataError(f"prepared bag is not LIO-only: {sorted(observed)}")
    return counts


def check_tracked_blobs() -> list[str]:
    result = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, check=True, capture_output=True)
    violations: list[str] = []
    for raw in result.stdout.split(b"\0"):
        if not raw:
            continue
        relative = Path(os.fsdecode(raw))
        path = ROOT / relative
        if relative.suffix.lower() in RAW_SUFFIXES:
            violations.append(f"tracked raw dataset: {relative}")
        if path.is_file() and path.stat().st_size > GENERATED_LIMIT:
            violations.append(f"tracked generated/large file: {relative}")
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("list", "check"))
    parser.add_argument("--dataset")
    parser.add_argument("--data-home", type=Path, default=data_home())
    args = parser.parse_args()
    home = args.data_home.expanduser().resolve()
    if args.action == "list":
        for dataset_id, (_, item) in entries().items():
            ready = (home / "datasets" / dataset_id / "status.json").is_file()
            print(f"{dataset_id}\t{item['download']['mode']}\t{'ready' if ready else 'absent'}")
        return 0
    entries()
    violations = check_tracked_blobs()
    if violations:
        raise DataError("; ".join(violations))
    if not args.dataset:
        print("dataset catalog and tracked-blob guard: OK")
        return 0
    entry = lookup(args.dataset)
    archive = blob_path(home, entry)
    checksum = entry["download"].get("checksum")
    if isinstance(checksum, dict) and archive.is_file():
        verify_blob(archive, checksum)
    context = dataset_context(args.dataset, home)
    counts = bag_topic_counts(context)
    status = load_yaml(Path(context["directory"]) / "status.json")
    prepared_status_schema(status, path=Path(context["directory"]) / "status.json")
    expected = status.get("selected_message_counts", {})
    if expected and counts != expected:
        raise DataError(f"prepared counts differ from provenance: {counts} != {expected}")
    print(json.dumps({
        "dataset": args.dataset,
        "topics": counts,
        "config": str(context["config"]),
        "tracked_blob_guard": "ok",
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DataError, OSError, subprocess.SubprocessError) as error:
        print(f"data workflow error: {error}", file=os.sys.stderr)
        raise SystemExit(1)

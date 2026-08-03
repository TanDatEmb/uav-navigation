#!/usr/bin/env python3
"""Single entrypoint for FAST-LIO dataset acquisition, validation, and runs."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
import zipfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/performance"))
from p0_8_provenance import make_provenance  # noqa: E402

CATALOG = ROOT / "datasets" / "catalog"
RAW_SUFFIXES = {".bag", ".mcap", ".db3"}
GENERATED_LIMIT = 10 * 1024 * 1024
REQUIRED_OFFLINE_OUTPUTS = (
    "run.json",
    "summary.json",
    "diagnostics.csv",
    "trajectory.csv",
    "corrections.csv",
    "local_map.pcd",
    "stdout.log",
    "stderr.log",
)
PREPARED_SCHEMA_VERSION = 2
CANONICAL_LIDAR_FRAME = "livox_frame"
CANONICAL_IMU_FRAME = "livox_imu_frame"


class DataError(RuntimeError):
    pass


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def data_home() -> Path:
    configured = os.environ.get("UAV_NAV_DATA_HOME")
    if configured:
        return Path(configured).expanduser().resolve()
    base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share"))
    return (base / "uav-nav").resolve()


def load_yaml(path: Path) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise DataError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(value, dict):
        raise DataError(f"{path} must contain a YAML mapping")
    return value


def entries() -> dict[str, tuple[Path, dict]]:
    result: dict[str, tuple[Path, dict]] = {}
    for path in sorted(CATALOG.glob("*.yaml")):
        item = load_yaml(path)
        validate_entry(path, item)
        dataset_id = str(item["id"])
        if dataset_id in result:
            raise DataError(f"duplicate dataset id: {dataset_id}")
        result[dataset_id] = (path, item)
    return result


def validate_entry(path: Path, item: dict) -> None:
    required = {
        "id", "source", "publisher", "landing_page", "license", "size_bytes",
        "download", "archive", "bag", "input", "ground_truth", "cases",
    }
    missing = sorted(required - item.keys())
    if missing:
        raise DataError(f"{path}: missing fields: {', '.join(missing)}")
    dataset_id = item["id"]
    if not isinstance(dataset_id, str) or not dataset_id or any(
        c not in "abcdefghijklmnopqrstuvwxyz0123456789-" for c in dataset_id
    ):
        raise DataError(f"{path}: invalid lowercase kebab-case id")
    download = item["download"]
    if not isinstance(download, dict) or download.get("mode") not in {
        "auto", "manual", "local"
    }:
        raise DataError(f"{path}: download.mode must be auto, manual, or local")
    checksum = download.get("checksum")
    if download["mode"] == "auto" and (
        not download.get("urls") or not isinstance(checksum, dict)
    ):
        raise DataError(f"{path}: automatic download needs URL and checksum")
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


def lookup(dataset_id: str) -> dict:
    found = entries().get(dataset_id)
    if found is None:
        raise DataError(f"unknown dataset id: {dataset_id}")
    return found[1]


def layout(home: Path) -> None:
    for name in ("archives", "datasets", "tmp"):
        (home / name).mkdir(parents=True, exist_ok=True)


def blob_path(home: Path, entry: dict) -> Path:
    checksum = entry["download"]["checksum"]
    return home / "archives" / f"{checksum['algorithm']}-{checksum['value'].lower()}"


def digest(path: Path, algorithm: str) -> str:
    value = hashlib.new(algorithm)
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def verify_blob(path: Path, checksum: dict) -> None:
    observed = digest(path, checksum["algorithm"])
    expected = checksum["value"].lower()
    if observed != expected:
        raise DataError(
            f"checksum mismatch for {path}: expected {expected}, observed {observed}"
        )


def download(entry: dict, home: Path) -> Path:
    mode = entry["download"]["mode"]
    if mode != "auto":
        raise DataError(
            f"{entry['id']} is {mode}; obtain it from {entry['landing_page']} and "
            "place the verified source in the external data home"
        )
    layout(home)
    destination = blob_path(home, entry)
    checksum = entry["download"]["checksum"]
    if destination.is_file():
        verify_blob(destination, checksum)
        return destination
    expected_size = entry.get("size_bytes")
    if expected_size is not None:
        free = shutil.disk_usage(home).free
        required = int(expected_size) + 64 * 1024 * 1024
        if free < required:
            raise DataError(f"insufficient disk space: need {required}, have {free}")
    partial = destination.with_suffix(".part")
    for url in entry["download"]["urls"]:
        offset = partial.stat().st_size if partial.exists() else 0
        request = urllib.request.Request(
            url, headers={"Range": f"bytes={offset}-"} if offset else {}
        )
        try:
            with urllib.request.urlopen(request, timeout=300) as response:
                status = getattr(response, "status", 200)
                if offset and status != 206:
                    partial.unlink()
                    offset = 0
                mode_flag = "ab" if offset else "wb"
                with partial.open(mode_flag) as output:
                    shutil.copyfileobj(response, output, 4 * 1024 * 1024)
            break
        except (OSError, urllib.error.URLError) as error:
            last_error = error
    else:
        raise DataError(f"all download URLs failed: {last_error}")
    if expected_size is not None and partial.stat().st_size != int(expected_size):
        raise DataError(
            f"download size mismatch: expected {expected_size}, "
            f"observed {partial.stat().st_size}"
        )
    verify_blob(partial, checksum)
    os.replace(partial, destination)
    return destination


def safe_extract_zip(blob: Path, target: Path) -> None:
    with zipfile.ZipFile(blob) as archive:
        root = target.resolve()
        for member in archive.infolist():
            destination = (target / member.filename).resolve()
            if destination != root and root not in destination.parents:
                raise DataError(f"unsafe archive member: {member.filename}")
        archive.extractall(target)


def tree_digest(directory: Path) -> str:
    value = hashlib.sha256()
    for path in sorted(p for p in directory.rglob("*") if p.is_file()):
        value.update(path.relative_to(directory).as_posix().encode())
        value.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                value.update(chunk)
    return value.hexdigest()


def tree_size(directory: Path) -> int:
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())


def normalize_message_frame(message: object, canonical_frame: str) -> str:
    header = getattr(message, "header", None)
    if header is None or not hasattr(header, "frame_id"):
        raise DataError("selected ROS message has no header.frame_id")
    source_frame = str(header.frame_id)
    if not source_frame:
        raise DataError("selected ROS message has an empty header.frame_id")
    header.frame_id = canonical_frame
    return source_frame


def prepared_status_schema(status: dict, *, path: Path | None = None) -> None:
    observed = status.get("schema_version")
    if observed != PREPARED_SCHEMA_VERSION:
        location = f" in {path}" if path is not None else ""
        raise DataError(
            f"prepared dataset schema{location} is {observed!r}; "
            f"expected {PREPARED_SCHEMA_VERSION}; run make data-fetch to rebuild"
        )


def _ros_message_class(type_name: str):
    if type_name == "sensor_msgs/msg/PointCloud2":
        from sensor_msgs.msg import PointCloud2

        return PointCloud2
    if type_name == "sensor_msgs/msg/Imu":
        from sensor_msgs.msg import Imu

        return Imu
    raise DataError(
        "frame normalization supports only sensor_msgs/msg/PointCloud2 and "
        f"sensor_msgs/msg/Imu, not {type_name}"
    )


def filter_ros2_bag(
    source: Path,
    destination: Path,
    topics: list[str],
    canonical_frames: dict[str, str],
) -> tuple[dict[str, int], dict[str, dict]]:
    try:
        import rosbag2_py
    except ImportError as error:
        raise DataError(
            "ROS 2 Python bag support is required; source /opt/ros/jazzy/setup.bash"
        ) from error
    try:
        from rclpy.serialization import deserialize_message, serialize_message
    except ImportError as error:
        raise DataError(
            "ROS 2 Python serialization support is required; source /opt/ros/jazzy/setup.bash"
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {
        metadata.name: metadata for metadata in reader.get_all_topics_and_types()
    }
    missing = [topic for topic in topics if topic not in available]
    if missing:
        raise DataError(f"source bag is missing selected topics: {missing}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(destination), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in topics:
        writer.create_topic(available[topic])
    counts = {topic: 0 for topic in topics}
    frame_stats = {
        topic: {
            "canonical_frame": canonical_frames[topic],
            "source_frame_ids": {},
            "normalized_message_count": 0,
        }
        for topic in topics
    }
    selected = set(topics)
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic in selected:
            message_type = _ros_message_class(available[topic].type)
            message = deserialize_message(serialized, message_type)
            source_frame = normalize_message_frame(message, canonical_frames[topic])
            source_counts = frame_stats[topic]["source_frame_ids"]
            source_counts[source_frame] = source_counts.get(source_frame, 0) + 1
            frame_stats[topic]["normalized_message_count"] += 1
            writer.write(topic, serialize_message(message), timestamp)
            counts[topic] += 1
    del writer
    if any(count == 0 for count in counts.values()):
        raise DataError(f"selected topic contains no messages: {counts}")
    return counts, frame_stats


def prepare(entry: dict, home: Path, keep_archive: bool) -> Path:
    layout(home)
    blob = download(entry, home)
    destination = home / "datasets" / entry["id"]
    if (destination / "status.json").is_file():
        status = load_yaml(destination / "status.json")
        if status.get("schema_version") == PREPARED_SCHEMA_VERSION:
            return destination
    with tempfile.TemporaryDirectory(dir=home / "tmp") as temporary:
        stage = Path(temporary) / entry["id"]
        source = stage / "source"
        source.mkdir(parents=True)
        archive_format = entry["archive"].get("format")
        if archive_format == "zip":
            safe_extract_zip(blob, source)
        else:
            shutil.copy2(blob, source / (entry["archive"].get("filename") or "source"))
        metadata_files = list(source.rglob("metadata.yaml"))
        if len(metadata_files) != 1:
            raise DataError(
                "prepare currently requires exactly one ROS 2 bag metadata.yaml; "
                "multi-sensor/ROS 1 filtering remains manual"
            )
        source_bag = metadata_files[0].parent
        lio = stage / "lio"
        selected_topics = [
            entry["input"]["lidar_topic"], entry["input"]["imu_topic"]
        ]
        canonical_frames = {
            entry["input"]["lidar_topic"]: CANONICAL_LIDAR_FRAME,
            entry["input"]["imu_topic"]: CANONICAL_IMU_FRAME,
        }
        selected_counts, frame_stats = filter_ros2_bag(
            source_bag, lio, selected_topics, canonical_frames
        )
        provenance = {
            "schema_version": PREPARED_SCHEMA_VERSION,
            "dataset": entry["id"],
            "source_archive": {
                "path": str(blob),
                "checksum": entry["download"]["checksum"],
            },
            "source_bag_metadata": str(metadata_files[0].relative_to(stage)),
            "selected_topics": selected_topics,
            "selected_message_counts": selected_counts,
            "conversion_tool": "tools/data.py",
            "normalization_tool_version": "tools/data.py/prepared-schema-2",
            "frame_normalization": {
                "policy": "rewrite only header.frame_id; preserve topic, type, "
                           "message header timestamp, bag timestamp, and payload",
                "canonical_frames": canonical_frames,
                "modified_fields": [
                    "sensor_msgs/msg/PointCloud2.header.frame_id",
                    "sensor_msgs/msg/Imu.header.frame_id",
                ],
                "topics": frame_stats,
            },
            "derived_bag_sha256": tree_digest(lio),
            "derived_bag_size_bytes": tree_size(lio),
            "note": "Derived ROS 2 bag contains only the selected LiDAR and IMU "
                    "topics.",
        }
        (stage / "status.json").write_text(
            json.dumps(provenance, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        config = config_path(entry["id"])
        workflow_manifest = {
            "name": entry["id"],
            "bag": "lio",
            "config": {"path": str(config)},
            "input": entry["input"],
            "expected": {"lidar_count": None, "imu_count": None},
        }
        (stage / "dataset.yaml").write_text(
            yaml.safe_dump(workflow_manifest, sort_keys=False),
            encoding="utf-8",
        )
        destination.parent.mkdir(parents=True, exist_ok=True)
        backup_root = None
        backup = None
        if destination.exists():
            backup_root = Path(
                tempfile.mkdtemp(
                    dir=destination.parent, prefix=f".{entry['id']}.old-"
                )
            )
            backup = backup_root / destination.name
            os.replace(destination, backup)
        try:
            os.replace(stage, destination)
        except OSError:
            if backup is not None:
                os.replace(backup, destination)
            raise
        else:
            if backup is not None:
                shutil.rmtree(backup)
                backup_root.rmdir()
    if not keep_archive:
        verify_blob(blob, entry["download"]["checksum"])
        blob.unlink()
    return destination


def check_tracked_blobs() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "-z"], cwd=ROOT, check=True, capture_output=True
    )
    violations: list[str] = []
    for raw in result.stdout.split(b"\0"):
        if not raw:
            continue
        relative = Path(os.fsdecode(raw))
        path = ROOT / relative
        suffix = path.suffix.lower()
        if suffix in RAW_SUFFIXES:
            violations.append(f"tracked raw dataset: {relative}")
        if suffix == ".pcd" and path.is_file() and path.stat().st_size > GENERATED_LIMIT:
            violations.append(f"tracked large PCD: {relative}")
        if path.is_file() and path.stat().st_size > GENERATED_LIMIT:
            allowed = relative.parts[:2] == ("test", "fixtures")
            if not allowed:
                violations.append(f"tracked generated/large file: {relative}")
    return violations


def config_path(dataset_id: str) -> Path:
    if dataset_id != "aist-mid360-drive":
        raise DataError(f"no canonical M1 config is defined for {dataset_id}")
    return ROOT / "src/navigation_estimator/fast_lio_ros/config/mid360_aist_replay.yaml"


def dataset_context(value: str, home: Path) -> dict:
    candidate = Path(value).expanduser()
    if candidate.exists():
        directory = candidate.resolve()
        manifest_path = directory / "dataset.yaml"
        if manifest_path.is_file():
            manifest = load_yaml(manifest_path)
            bag = (directory / str(manifest["bag"])).resolve()
            dataset_id = str(manifest["name"])
            input_config = manifest["input"]
            configured = Path(str(manifest["config"]["path"])).expanduser()
            config = configured if configured.is_absolute() else (ROOT / configured)
        elif (directory / "metadata.yaml").is_file():
            entry = lookup("aist-mid360-drive")
            dataset_id = entry["id"]
            bag = directory
            input_config = entry["input"]
            config = config_path(dataset_id)
        else:
            raise DataError(
                f"dataset path needs dataset.yaml or rosbag metadata.yaml: {directory}"
            )
        return {
            "id": dataset_id,
            "directory": directory,
            "bag": bag.resolve(),
            "config": config.resolve(),
            "input": input_config,
        }

    entry = lookup(value)
    prepared = home / "datasets" / value
    bag = prepared / "lio"
    if not (prepared / "status.json").is_file() or not bag.is_dir():
        raise DataError(f"{value} is not prepared; run make data-fetch DATASET={value}")
    prepared_status_schema(
        load_yaml(prepared / "status.json"), path=prepared / "status.json"
    )
    return {
        "id": value,
        "directory": prepared,
        "bag": bag,
        "config": config_path(value),
        "input": entry["input"],
    }


def bag_topic_counts(context: dict) -> dict[str, int]:
    bag = context["bag"]
    config = context["config"]
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
    counts = {}
    for topic, expected_type in required.items():
        actual = observed.get(topic)
        if actual is None or actual[0] != expected_type or actual[1] <= 0:
            raise DataError(
                f"invalid required topic {topic}: observed={actual}, "
                f"expected_type={expected_type}"
            )
        counts[topic] = actual[1]
    if set(observed) != set(required):
        raise DataError(f"derived bag is not LIO-only: {sorted(observed)}")
    return counts


def git_short_sha() -> str:
    return subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, check=True,
        text=True, capture_output=True,
    ).stdout.strip()


def new_run_directory(
    dataset_id: str, action: str, *, create: bool = True
) -> Path:
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = (
        ROOT / ".artifacts/datasets" / dataset_id
        / f"{git_short_sha()}-{action}-{stamp}"
    )
    if create:
        output.mkdir(parents=True, exist_ok=False)
    return output


def write_run_json(
    output: Path, action: str, context: dict, counts: dict[str, int],
    provenance: dict | None = None, **extra: object,
) -> None:
    payload = {
        "schema_version": 1,
        "action": action,
        "dataset": context["id"],
        "dataset_directory": str(context["directory"]),
        "bag_directory": str(context["bag"]),
        "config_path": str(context["config"]),
        "git_short_sha": git_short_sha(),
        "input": context["input"],
        "observed_counts": counts,
    }
    if provenance is not None:
        payload["provenance"] = provenance
    payload.update(extra)
    atomic_json(output / "run.json", payload)


def reject_non_finite_csv(output: Path) -> None:
    invalid = {"nan", "+nan", "-nan", "inf", "+inf", "-inf", "infinity"}
    for name in ("trajectory.csv", "corrections.csv", "diagnostics.csv"):
        with (output / name).open(encoding="utf-8", newline="") as stream:
            if any(value.strip().lower() in invalid for row in csv.reader(stream)
                   for value in row):
                raise DataError(f"non-finite value found in {name}")


def run_offline(args: argparse.Namespace, home: Path, smoke: bool) -> int:
    context = dataset_context(args.dataset, home)
    counts = bag_topic_counts(context)
    action = "smoke" if smoke else "run"
    output = new_run_directory(context["id"], action)
    write_run_json(output, action, context, counts)
    command = [
        "ros2", "run", "fast_lio_tools", "lio_offline",
        str(context["bag"]), str(context["config"]), str(output),
    ]
    if smoke:
        command.append(str(args.max_lidar))
    with (output / "stdout.log").open("w", encoding="utf-8") as stdout, (
        output / "stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr)
    if result.returncode:
        raise DataError(f"offline runner failed with {result.returncode}: {output}")
    missing = [name for name in REQUIRED_OFFLINE_OUTPUTS
               if not (output / name).is_file()]
    if missing:
        raise DataError(f"offline runner omitted: {', '.join(missing)}")
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    failures = []
    if int(summary.get("core_rejected_lidar_count", 0)) != 0:
        failures.append("LiDAR adapter rejection")
    if int(summary.get("invalid_timestamp_rejected_count", 0)) != 0:
        failures.append("invalid timestamp")
    if int(summary.get("successful_correction_count", 0)) <= 0:
        failures.append("no corrected output")
    if not bool(summary.get("deskew_applied")):
        failures.append("deskew not applied")
    if failures:
        raise DataError(f"offline gates failed: {', '.join(failures)}")
    reject_non_finite_csv(output)
    print(output)
    return 0


def run_replay(args: argparse.Namespace, home: Path) -> int:
    context = dataset_context(args.dataset, home)
    counts = bag_topic_counts(context)
    provenance = make_provenance(
        ROOT,
        config=context["config"],
        dataset=context["id"],
        rate=args.rate,
    )
    output = new_run_directory(
        context["id"], f"replay-{args.rate}x", create=False
    )
    output.mkdir(parents=True, exist_ok=False)
    write_run_json(
        output, "replay", context, counts, provenance,
        status="started", replay_returncode=None, estimator_returncode=None,
    )
    command = [
        sys.executable, str(ROOT / "tools/runtime/ros_replay.py"), "run",
        "--bag", str(context["bag"]), "--config", str(context["config"]),
        "--output", str(output),
        "--imu-topic", str(context["input"]["imu_topic"]),
        "--lidar-topic", str(context["input"]["lidar_topic"]),
        "--rate", str(args.rate),
        "--allow-existing-output",
    ]
    if args.enable_rviz:
        command.extend([
            "--enable-rviz",
            "--rviz-config",
            str(ROOT / "src/navigation_estimator/fast_lio_ros/rviz/fast_lio.rviz"),
        ])
    command.extend([
        "--replay-timeout", str(args.replay_timeout),
        "--readiness-timeout", str(args.readiness_timeout),
        "--drain-timeout", str(args.drain_timeout),
    ])
    result = subprocess.run(command, cwd=ROOT)
    write_run_json(
        output, "replay", context, counts, provenance,
        status="passed" if result.returncode == 0 else "failed",
        wrapper_returncode=result.returncode,
    )
    return result.returncode


def view_latest(dataset_id: str) -> int:
    maps = sorted(
        (ROOT / ".artifacts/datasets" / dataset_id).glob("*/local_map.pcd"),
        key=os.path.getmtime,
    )
    if not maps:
        raise DataError(f"no local_map.pcd artifact for {dataset_id}")
    viewer = shutil.which("pcl_viewer")
    if viewer is None:
        raise DataError("pcl_viewer is not installed")
    return subprocess.run([viewer, str(maps[-1])], cwd=ROOT).returncode


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def distribution(rows: list[dict[str, str]], key: str) -> dict:
    values = [float(row[key]) for row in rows if row.get(key) not in (None, "")]
    return {
        "sample_count": len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def _flag(row: dict[str, str], key: str) -> bool:
    return row.get(key, "").strip().lower() in {"1", "true", "yes"}


def build_data_report(
    dataset_id: str, artifact: Path, summary: dict,
    rows: list[dict[str, str]],
) -> dict:
    synchronized = [row for row in rows if _flag(row, "synchronized")]
    attempted = [row for row in rows if _flag(row, "correction_attempted")]
    succeeded = [row for row in attempted if _flag(row, "correction_succeeded")]
    failed = [row for row in attempted if not _flag(row, "correction_succeeded")]
    map_updates = [row for row in rows if _flag(row, "map_update_performed")]
    deskewed = [row for row in rows if _flag(row, "deskew_applied")]
    processing_rows = [
        row for row in rows
        if float(row.get("total_processing_us") or 0) > 0
    ]
    residual_rows = [
        row for row in attempted
        if int(row.get("accepted_residuals") or 0) > 0
        and math.isfinite(float(row.get("residual_rms") or "nan"))
    ]
    reasons: dict[str, int] = {}
    for row in rows:
        reason = (row.get("reason") or "").strip() or "SUCCESS"
        reasons[reason] = reasons.get(reason, 0) + 1

    expected = {
        "synchronized": int(summary.get("synchronized_group_count", 0)),
        "attempted": int(summary.get("correction_attempt_count", 0)),
        "succeeded": int(summary.get("successful_correction_count", 0)),
        "failed": int(summary.get("failed_correction_count", 0)),
    }
    observed = {
        "synchronized": len(synchronized),
        "attempted": len(attempted),
        "succeeded": len(succeeded),
        "failed": len(failed),
    }
    errors = []
    if observed["succeeded"] + observed["failed"] != observed["attempted"]:
        errors.append("succeeded + failed != correction attempted")
    if observed["attempted"] > observed["synchronized"]:
        errors.append("correction attempted exceeds synchronized groups")
    for key in expected:
        if observed[key] != expected[key]:
            errors.append(
                f"{key} rows {observed[key]} != summary {expected[key]}"
            )
    if errors:
        raise DataError("data-report invariant failure: " + "; ".join(errors))

    warnings = []
    wall_runtime_us = float(summary.get("wall_runtime_us", 0))
    processing_total_us = sum(
        float(row.get("total_processing_us") or 0) for row in processing_rows
    )
    if wall_runtime_us > 0 and processing_total_us > wall_runtime_us * 1.25:
        warnings.append(
            "sum(total_processing_us) materially exceeds wall runtime"
        )
    duration = float(summary.get("dataset_duration_seconds", 0))
    report = {
        "dataset": dataset_id,
        "artifact": str(artifact),
        "dataset_duration_s": duration,
        "wall_runtime_s": wall_runtime_us / 1e6,
        "realtime_factor": duration / max(wall_runtime_us / 1e6, 1e-9),
        "imu": {
            "accepted": summary.get("core_accepted_imu_count"),
            "rejected": summary.get("core_rejected_imu_count"),
        },
        "lidar": {
            "raw": summary.get("raw_dataset_lidar_count"),
            "buffer_accepted": summary.get("core_accepted_lidar_count"),
            "rejected": summary.get("core_rejected_lidar_count"),
        },
        "synchronization": {
            "synchronized_groups": observed["synchronized"],
            "overlap_rejected": summary.get("overlap_rejected_count"),
            "missing_bracket_rejected": summary.get(
                "missing_bracket_rejected_count"
            ),
            "invalid_timestamp_rejected": summary.get(
                "invalid_timestamp_rejected_count"
            ),
            "ratio": summary.get("synchronization_ratio"),
        },
        "corrections": observed | {
            "effective_output_rate_hz": summary.get(
                "effective_corrected_output_rate_hz"
            )
        },
        "deskew_count": len(deskewed),
        "rejection_reason_histogram": dict(sorted(reasons.items())),
        "residual_rms": distribution(residual_rows, "residual_rms"),
        "ikfom_iterations": distribution(attempted, "iterations"),
        "total_processing_us": distribution(
            processing_rows, "total_processing_us"
        ),
        "ikfom_update_us": distribution(attempted, "ikfom_update_us"),
        "map_update_us": distribution(map_updates, "map_insert_crop_us"),
        "map_maintenance_us": distribution(
            map_updates, "map_maintenance_us"
        ),
        "map_points": {
            "final": summary.get("map_point_count"),
            "max": max(
                (int(row["map_points"]) for row in rows if row.get("map_points")),
                default=None,
            ),
        },
        "consistency": {
            "status": "warning" if warnings else "ok",
            "warnings": warnings,
            "processing_total_us": processing_total_us,
            "wall_runtime_us": wall_runtime_us,
        },
    }
    return report


def report_latest(dataset_id: str) -> int:
    base = ROOT / ".artifacts/datasets" / dataset_id
    summaries = sorted(
        (
            path for path in base.glob("*/summary.json")
            if (path.parent / "diagnostics.csv").is_file()
        ),
        key=os.path.getmtime,
    )
    if not summaries:
        raise DataError(f"no offline run summary found for {dataset_id}")
    summary_path = summaries[-1]
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    diagnostics_csv = summary_path.parent / "diagnostics.csv"
    with diagnostics_csv.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    report = build_data_report(
        dataset_id, summary_path.parent, summary, rows
    )
    replay_summaries = sorted(
        (
            path for path in base.glob("*/summary.json")
            if not (path.parent / "diagnostics.csv").is_file()
            and "replay-" in path.parent.name
        ),
        key=os.path.getmtime,
    )
    if replay_summaries:
        replay = json.loads(replay_summaries[-1].read_text(encoding="utf-8"))
        runtime = replay.get("diagnostics", {})
        report["replay"] = {
            "artifact": str(replay_summaries[-1].parent),
            "rate": replay.get("rate"),
            "queue_depth_final": runtime.get("current_input_queue_depth"),
            "queue_depth_max": runtime.get("maximum_queue_depth"),
            "processing_lag_ns": runtime.get("processing_lag_ns"),
            "imu_drop_count": runtime.get("imu_drop_count"),
            "lidar_drop_count": runtime.get("lidar_drop_count"),
        }
    atomic_json(summary_path.parent / "report.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def run_data_test(args: argparse.Namespace, home: Path) -> int:
    commands = [
        ("check",),
        ("smoke", "--max-lidar", str(args.max_lidar)),
        ("run",),
        ("replay", "--rate", "1.0"),
        ("report",),
    ]
    for command in commands:
        completed = subprocess.run(
            [
                sys.executable, str(Path(__file__).resolve()),
                "--data-home", str(home), command[0],
                "--dataset", args.dataset, *command[1:],
            ],
            cwd=ROOT,
        )
        if completed.returncode:
            return completed.returncode
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--data-home", type=Path, default=data_home())
    sub = result.add_subparsers(dest="action", required=True)
    sub.add_parser("list")
    sub.add_parser("clean")
    sub.add_parser("clean-artifacts")
    for action in ("fetch", "check"):
        child = sub.add_parser(action)
        child.add_argument("--dataset", required=(action == "fetch"))
    for action in ("view", "run", "replay", "smoke", "report", "test"):
        child = sub.add_parser(action)
        child.add_argument("--dataset", required=True)
        if action in ("smoke", "test"):
            child.add_argument("--max-lidar", type=int, default=20)
        if action == "replay":
            child.add_argument("--rate", type=float, default=1.0)
            child.add_argument("--enable-rviz", action="store_true")
            child.add_argument("--replay-timeout", type=float, default=900.0)
            child.add_argument("--readiness-timeout", type=float, default=30.0)
            child.add_argument("--drain-timeout", type=float, default=120.0)
    return result


def main() -> int:
    args = parser().parse_args()
    home = args.data_home.expanduser().resolve()
    if args.action == "list":
        for dataset_id, (_, item) in entries().items():
            installed = (home / "datasets" / dataset_id / "status.json").is_file()
            print(f"{dataset_id}\t{item['download']['mode']}\t"
                  f"{'ready' if installed else 'absent'}")
        return 0
    if args.action in {"clean", "clean-artifacts"}:
        names = (
            ("build", "install", "log")
            if args.action == "clean" else (".artifacts",)
        )
        for name in names:
            target = ROOT / name
            if target.is_dir():
                shutil.rmtree(target)
        return 0
    if args.action == "fetch":
        print(prepare(lookup(args.dataset), home, True))
        return 0
    if args.action == "check":
        entries()
        violations = check_tracked_blobs()
        if violations:
            raise DataError("; ".join(violations))
        if not args.dataset:
            print("dataset catalog and tracked-blob guard: OK")
            return 0
        entry = lookup(args.dataset)
        path = blob_path(home, entry)
        if not path.is_file():
            raise DataError(f"source archive missing: run data-fetch for {args.dataset}")
        verify_blob(path, entry["download"]["checksum"])
        context = dataset_context(args.dataset, home)
        counts = bag_topic_counts(context)
        status = load_yaml(context["directory"] / "status.json")
        prepared_status_schema(status, path=context["directory"] / "status.json")
        expected = status.get("selected_message_counts", {})
        if counts != expected:
            raise DataError(f"prepared counts differ from provenance: {counts} != {expected}")
        print(json.dumps({
            "dataset": args.dataset,
            "checksum": "verified",
            "topics": counts,
            "config": str(context["config"]),
            "tracked_blob_guard": "ok",
        }, sort_keys=True))
        return 0
    if args.action == "smoke":
        return run_offline(args, home, True)
    if args.action == "run":
        return run_offline(args, home, False)
    if args.action == "replay":
        return run_replay(args, home)
    if args.action == "view":
        return view_latest(dataset_context(args.dataset, home)["id"])
    if args.action == "report":
        return report_latest(dataset_context(args.dataset, home)["id"])
    if args.action == "test":
        return run_data_test(args, home)
    raise DataError(f"unsupported action: {args.action}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DataError, OSError, subprocess.SubprocessError) as error:
        print(f"data workflow error: {error}", file=sys.stderr)
        raise SystemExit(1)

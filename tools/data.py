#!/usr/bin/env python3
"""External dataset registry, content cache, and legacy dataset workflow."""

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
CATALOG = ROOT / "datasets" / "catalog"
RAW_SUFFIXES = {".bag", ".mcap", ".db3"}
GENERATED_LIMIT = 10 * 1024 * 1024
MATRIX_SETS = {
    "core": (
        "aist-mid360-drive",
        "local-mid360-static01",
        "local-mid360-yaw01",
    ),
    "runtime": (
        "aist-mid360-drive",
        "m3dgr-mid360-dynamic01",
    ),
    "map": (
        "m3dgr-mid360-dynamic01",
        "m3dgr-mid360-corridor02",
        "local-mid360-square01",
    ),
    "optional-uav": (
        "tiers-mid360-updown01",
        "tiers-mid360-square01",
    ),
}


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


def filter_ros2_bag(source: Path, destination: Path, topics: list[str]) -> dict[str, int]:
    try:
        import rosbag2_py
    except ImportError as error:
        raise DataError(
            "ROS 2 Python bag support is required; source /opt/ros/jazzy/setup.bash"
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
    selected = set(topics)
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic in selected:
            writer.write(topic, serialized, timestamp)
            counts[topic] += 1
    del writer
    if any(count == 0 for count in counts.values()):
        raise DataError(f"selected topic contains no messages: {counts}")
    return counts


def prepare(entry: dict, home: Path, keep_archive: bool) -> Path:
    layout(home)
    blob = download(entry, home)
    destination = home / "datasets" / entry["id"]
    if (destination / "status.json").is_file():
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
        selected_counts = filter_ros2_bag(source_bag, lio, selected_topics)
        provenance = {
            "schema_version": 1,
            "dataset": entry["id"],
            "source_archive": {
                "path": str(blob),
                "checksum": entry["download"]["checksum"],
            },
            "source_bag_metadata": str(metadata_files[0].relative_to(stage)),
            "selected_topics": selected_topics,
            "selected_message_counts": selected_counts,
            "conversion_tool": "tools/data.py",
            "derived_bag_sha256": tree_digest(lio),
            "derived_bag_size_bytes": tree_size(lio),
            "note": "Derived ROS 2 bag contains only the selected LiDAR and IMU "
                    "topics.",
        }
        (stage / "status.json").write_text(
            json.dumps(provenance, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        config = (
            ROOT / "src/navigation_estimator/fast_lio_ros/config/data"
            / f"{entry['id']}.yaml"
        )
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
        os.replace(stage, destination)
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


def workflow_dataset(value: str, home: Path) -> str:
    candidate = Path(value).expanduser()
    if candidate.exists():
        return str(candidate)
    prepared = home / "datasets" / value
    if (prepared / "dataset.yaml").is_file():
        return str(prepared)
    legacy = ROOT / "data" / value
    if (legacy / "dataset.yaml").is_file():
        return str(legacy)
    if value == "aist-mid360-drive":
        local = ROOT / "data" / "mid360_17_01"
        if (local / "dataset.yaml").is_file():
            return str(local)
    raise DataError(f"{value} is not prepared in the external data home")


def matrix_datasets(name: str) -> tuple[str, ...]:
    if name == "all":
        ordered: list[str] = []
        for group in ("core", "runtime", "map"):
            for dataset_id in MATRIX_SETS[group]:
                if dataset_id not in ordered:
                    ordered.append(dataset_id)
        return tuple(ordered)
    if name not in MATRIX_SETS:
        raise DataError(f"unknown matrix set: {name}")
    return MATRIX_SETS[name]


def matrix_actions(repeat: int, margin: bool) -> tuple[tuple[str, ...], ...]:
    if repeat <= 0:
        raise DataError("matrix repeat must be positive")
    actions: list[tuple[str, ...]] = [
        ("info",),
        ("smoke", "--max-lidar", "20"),
        ("run",),
    ]
    actions.extend(("run",) for _ in range(repeat))
    actions.extend(
        [
            ("replay", "--rate", "0.5"),
            ("replay", "--rate", "1.0"),
        ]
    )
    if margin:
        actions.append(("replay", "--rate", "1.2"))
    return tuple(actions)


def read_trajectory(path: Path) -> list[tuple[int, tuple[float, ...]]]:
    samples = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            samples.append((
                int(row["time_ns"]),
                tuple(float(row[key]) for key in (
                    "x", "y", "z", "qx", "qy", "qz", "qw"
                )),
            ))
    return samples


def quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = value
    norm_squared = x * x + y * y + z * z + w * w
    return (-x / norm_squared, -y / norm_squared,
            -z / norm_squared, w / norm_squared)


def rotation_angle(value: tuple[float, float, float, float]) -> float:
    norm = math.sqrt(sum(component * component for component in value))
    return 2.0 * math.acos(min(1.0, max(-1.0, abs(value[3] / norm))))


def trajectory_metrics(
    estimate: list[tuple[int, tuple[float, ...]]],
    truth: list[tuple[int, tuple[float, ...]]],
    maximum_time_delta_ns: int,
) -> dict:
    if not estimate or not truth:
        raise DataError("trajectory metrics require non-empty trajectories")
    truth_by_time = sorted(truth)
    matched = []
    truth_index = 0
    for estimate_sample in sorted(estimate):
        while (
            truth_index + 1 < len(truth_by_time)
            and abs(truth_by_time[truth_index + 1][0] - estimate_sample[0])
            <= abs(truth_by_time[truth_index][0] - estimate_sample[0])
        ):
            truth_index += 1
        truth_sample = truth_by_time[truth_index]
        if abs(truth_sample[0] - estimate_sample[0]) <= maximum_time_delta_ns:
            matched.append((estimate_sample, truth_sample))
    if not matched:
        raise DataError("no estimate/ground-truth timestamps matched")
    squared_position_errors = []
    rpe_translation_squared = []
    rpe_rotation_squared = []
    for (_, estimate_value), (_, truth_value) in matched:
        squared_position_errors.append(sum(
            (estimate_value[index] - truth_value[index]) ** 2
            for index in range(3)
        ))
    for index in range(1, len(matched)):
        previous_estimate = matched[index - 1][0][1]
        current_estimate = matched[index][0][1]
        previous_truth = matched[index - 1][1][1]
        current_truth = matched[index][1][1]
        estimate_delta = tuple(
            current_estimate[axis] - previous_estimate[axis]
            for axis in range(3)
        )
        truth_delta = tuple(
            current_truth[axis] - previous_truth[axis] for axis in range(3)
        )
        rpe_translation_squared.append(sum(
            (estimate_delta[axis] - truth_delta[axis]) ** 2
            for axis in range(3)
        ))
        estimate_rotation = quaternion_multiply(
            quaternion_inverse(tuple(previous_estimate[3:7])),
            tuple(current_estimate[3:7]),
        )
        truth_rotation = quaternion_multiply(
            quaternion_inverse(tuple(previous_truth[3:7])),
            tuple(current_truth[3:7]),
        )
        rotation_error = quaternion_multiply(
            quaternion_inverse(truth_rotation), estimate_rotation
        )
        rpe_rotation_squared.append(rotation_angle(rotation_error) ** 2)
    rms = lambda values: (
        math.sqrt(sum(values) / len(values)) if values else None
    )
    return {
        "ate_translation_rmse_m": rms(squared_position_errors),
        "rpe_translation_rmse_m": rms(rpe_translation_squared),
        "rpe_rotation_rmse_rad": rms(rpe_rotation_squared),
        "trajectory_coverage": len(matched) / len(estimate),
        "matched_pose_count": len(matched),
        "estimate_pose_count": len(estimate),
        "thresholds_applied": False,
    }


def ground_truth_metrics(
    entry: dict, dataset_path: Path, run_output: Path
) -> dict:
    ground_truth = entry.get("ground_truth") or {}
    if not ground_truth.get("available"):
        return {"available": False}
    relative = ground_truth.get("path")
    if not relative:
        return {
            "available": True,
            "computed": False,
            "reason": "catalog has no prepared ground_truth.path",
        }
    if ground_truth.get("alignment") != "same_frame":
        return {
            "available": True,
            "computed": False,
            "reason": "ground-truth frame alignment is not verified",
        }
    metrics = trajectory_metrics(
        read_trajectory(run_output / "trajectory.csv"),
        read_trajectory(dataset_path / str(relative)),
        int(ground_truth.get("maximum_time_delta_ns", 20_000_000)),
    )
    return {"available": True, "computed": True, **metrics}


def run_matrix(args: argparse.Namespace, home: Path) -> int:
    dataset_ids = matrix_datasets(args.case)
    actions = matrix_actions(args.repeat, args.margin)
    if args.plan_only:
        for dataset_id in dataset_ids:
            print(dataset_id)
            for action in actions:
                print("  " + " ".join(action))
        return 0
    sha = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, text=True,
        capture_output=True,
    ).stdout.strip()
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = ROOT / ".artifacts" / "data-matrix" / args.case / f"{sha}-{stamp}"
    output.mkdir(parents=True, exist_ok=False)
    results = []
    for dataset_id in dataset_ids:
        try:
            dataset_path = Path(workflow_dataset(dataset_id, home))
        except DataError as error:
            results.append({
                "dataset": dataset_id,
                "status": "unavailable",
                "error": str(error),
                "actions": [],
            })
            continue
        action_results = []
        full_run_output: Path | None = None
        for action in actions:
            command = [
                sys.executable, str(Path(__file__).resolve()),
                "--data-home", str(home), action[0],
                "--dataset", dataset_id, *action[1:],
            ]
            completed = subprocess.run(
                command, cwd=ROOT, text=True, capture_output=True
            )
            if completed.stdout:
                print(completed.stdout, end="")
            if completed.stderr:
                print(completed.stderr, end="", file=sys.stderr)
            output_path = None
            for line in reversed(completed.stdout.splitlines()):
                candidate = Path(line.strip())
                if candidate.is_dir():
                    output_path = str(candidate)
                    break
            action_result = {
                "action": list(action),
                "returncode": completed.returncode,
                "output": output_path,
            }
            action_results.append(action_result)
            if action == ("run",) and full_run_output is None and output_path:
                full_run_output = Path(output_path)
            if completed.returncode != 0:
                break
        entry = lookup(dataset_id)
        metrics = (
            ground_truth_metrics(entry, dataset_path, full_run_output)
            if full_run_output is not None
            else {"available": bool(
                (entry.get("ground_truth") or {}).get("available")
            ), "computed": False, "reason": "full offline run unavailable"}
        )
        results.append({
            "dataset": dataset_id,
            "status": (
                "passed"
                if len(action_results) == len(actions)
                and all(item["returncode"] == 0 for item in action_results)
                else "failed"
            ),
            "actions": action_results,
            "ground_truth_metrics": metrics,
        })
        atomic_json(output / "summary.json", {
            "schema_version": 1,
            "set": args.case,
            "git_sha": sha,
            "datasets": results,
        })
    failures = [item for item in results if item["status"] != "passed"]
    atomic_json(output / "summary.json", {
        "schema_version": 1,
        "set": args.case,
        "git_sha": sha,
        "datasets": results,
        "failure_count": len(failures),
    })
    print(output)
    return 1 if failures else 0


def call_legacy(action: str, args: argparse.Namespace, home: Path) -> int:
    dataset_path = Path(workflow_dataset(args.dataset, home))
    if action == "replay":
        manifest = load_yaml(dataset_path / "dataset.yaml")
        bag = (dataset_path / str(manifest["bag"])).resolve()
        config = Path(str(manifest["config"]["path"])).expanduser()
        if not config.is_absolute():
            config = (ROOT / config).resolve()
        sha = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, check=True,
            text=True, capture_output=True,
        ).stdout.strip()
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = (
            ROOT / ".artifacts" / "datasets" / str(manifest["name"])
            / f"{sha}-replay-{args.rate}x-{stamp}"
        )
        input_config = manifest["input"]
        command = [
            sys.executable, str(ROOT / "tools/runtime/ros_replay.py"), "run",
            "--bag", str(bag), "--config", str(config),
            "--output", str(output),
            "--imu-topic", str(input_config["imu_topic"]),
            "--lidar-topic", str(input_config["lidar_topic"]),
            "--rate", str(args.rate),
        ]
        return subprocess.run(command, cwd=ROOT).returncode
    aliases = {"info": "inspect", "replay": "ros"}
    command = [
        sys.executable, str(ROOT / "tools" / "dev" / "dataset.py"),
        aliases.get(action, action), "--dataset",
        str(dataset_path),
    ]
    if action == "smoke":
        command += ["--max-lidar", str(args.max_lidar)]
    return subprocess.run(command, cwd=ROOT).returncode


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--data-home", type=Path, default=data_home())
    sub = result.add_subparsers(dest="action", required=True)
    sub.add_parser("list")
    for action in ("get", "check", "prepare", "rm"):
        child = sub.add_parser(action)
        child.add_argument("--dataset")
        if action == "prepare":
            child.add_argument("--keep-archive", choices=("0", "1"), default="1")
    matrix = sub.add_parser("matrix")
    matrix.add_argument("--case", required=True)
    matrix.add_argument("--repeat", type=int, default=3)
    matrix.add_argument("--margin", action="store_true")
    matrix.add_argument("--plan-only", action="store_true")
    for action in ("info", "view", "run", "replay", "smoke"):
        child = sub.add_parser(action)
        child.add_argument("--dataset", required=True)
        if action == "smoke":
            child.add_argument("--max-lidar", type=int, default=20)
        if action == "replay":
            child.add_argument("--rate", type=float, default=1.0)
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
    if args.action == "check":
        entries()
        violations = check_tracked_blobs()
        if args.dataset:
            entry = lookup(args.dataset)
            path = blob_path(home, entry) if entry["download"].get("checksum") else None
            if path and path.exists():
                verify_blob(path, entry["download"]["checksum"])
        if violations:
            raise DataError("; ".join(violations))
        print("dataset catalog and tracked-blob guard: OK")
        return 0
    if args.action == "matrix":
        return run_matrix(args, home)
    if args.action in {"info", "view", "run", "replay", "smoke"}:
        return call_legacy(args.action, args, home)
    if not args.dataset:
        raise DataError(f"{args.action} requires --dataset")
    entry = lookup(args.dataset)
    if args.action == "get":
        print(download(entry, home))
    elif args.action == "prepare":
        print(prepare(entry, home, args.keep_archive == "1"))
    elif args.action == "rm":
        target = home / "datasets" / entry["id"]
        if target.exists():
            shutil.rmtree(target)
        print(target)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DataError, OSError, subprocess.SubprocessError) as error:
        print(f"data workflow error: {error}", file=sys.stderr)
        raise SystemExit(1)

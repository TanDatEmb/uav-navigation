#!/usr/bin/env python3
"""External dataset registry, content cache, and legacy dataset workflow."""

from __future__ import annotations

import argparse
import hashlib
import json
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


class DataError(RuntimeError):
    pass


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
        shutil.copytree(source_bag, lio)
        provenance = {
            "schema_version": 1,
            "dataset": entry["id"],
            "source_archive": {
                "path": str(blob),
                "checksum": entry["download"]["checksum"],
            },
            "source_bag_metadata": str(metadata_files[0].relative_to(stage)),
            "selected_topics": [
                entry["input"]["lidar_topic"], entry["input"]["imu_topic"]
            ],
            "conversion_tool": "tools/data.py",
            "derived_bag_sha256": tree_digest(lio),
            "derived_bag_size_bytes": tree_size(lio),
            "note": "Source bag contains the requested LIO topics; copied as a "
                    "derived external bag. Topic filtering is required for "
                    "multi-sensor sources.",
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


def call_legacy(action: str, args: argparse.Namespace, home: Path) -> int:
    aliases = {"info": "inspect", "replay": "ros"}
    command = [
        sys.executable, str(ROOT / "tools" / "dev" / "dataset.py"),
        aliases.get(action, action), "--dataset",
        workflow_dataset(args.dataset, home),
    ]
    if action == "smoke":
        command += ["--max-lidar", str(args.max_lidar)]
    if action == "replay":
        command += ["--rate", str(args.rate)]
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
    matrix.add_argument("--case")
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
        for dataset_id, (_, item) in entries().items():
            if not args.case or args.case in item["cases"]:
                print(f"{dataset_id}\t{','.join(item['cases'])}")
        return 0
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

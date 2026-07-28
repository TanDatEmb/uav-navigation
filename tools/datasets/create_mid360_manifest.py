#!/usr/bin/env python3
"""Create a SHA-256 manifest for an already recorded real Mid-360 bag."""

import argparse
import hashlib
import json
from pathlib import Path
import sys


TOPICS = [
    {"name": "/livox/lidar", "type": "livox_ros_driver2/msg/CustomMsg"},
    {"name": "/livox/imu", "type": "sensor_msgs/msg/Imu"},
    {"name": "/tf", "type": "tf2_msgs/msg/TFMessage"},
    {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage"},
]


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--source-kind", choices=["physical_capture", "official_download"], required=True)
    parser.add_argument("--driver-version", default="not-recorded")
    parser.add_argument("--canonical-url")
    parser.add_argument("--license", default="capture owner must specify redistribution terms")
    args = parser.parse_args()
    root = args.dataset.resolve()
    if not root.is_dir() or not (root / "metadata.yaml").is_file():
        print("dataset must be an existing rosbag2 directory containing metadata.yaml", file=sys.stderr)
        return 2
    files = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            files.append({"path": str(path.relative_to(root)), "bytes": path.stat().st_size, "sha256": digest(path)})
    if not files:
        print("dataset contains no files to hash", file=sys.stderr)
        return 2
    source = {"kind": args.source_kind, "driver_version": args.driver_version, "license": args.license}
    if args.canonical_url:
        source["canonical_url"] = args.canonical_url
    manifest = {"schema_version": 1, "dataset_id": args.dataset_id, "not_synthetic": True,
                "source": source, "topics": TOPICS, "files": files}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

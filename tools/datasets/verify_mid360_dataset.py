#!/usr/bin/env python3
"""Verify a real Mid-360 manifest, its bytes, and optionally rosbag metadata."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


REQUIRED_TOPICS = {
    "/livox/lidar": "livox_ros_driver2/msg/CustomMsg",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--skip-bag-inspection",
        action="store_true",
        help="hash/provenance check only; this cannot establish that files are a valid rosbag",
    )
    args = parser.parse_args()
    root = args.dataset.resolve()
    try:
        manifest = json.loads((root / "manifest.json").read_text())
    except (OSError, json.JSONDecodeError) as error:
        print(f"cannot read manifest.json: {error}", file=sys.stderr)
        return 2
    failures = []
    if manifest.get("schema_version") != 1 or manifest.get("not_synthetic") is not True:
        failures.append("manifest is not schema 1 real-data evidence")
    if manifest.get("source", {}).get("kind") not in {"physical_capture", "official_download"}:
        failures.append("manifest source kind is not an allowed real-data origin")
    declared_topics = {entry.get("name"): entry.get("type") for entry in manifest.get("topics", [])}
    for topic, message_type in REQUIRED_TOPICS.items():
        if declared_topics.get(topic) != message_type:
            failures.append(f"manifest does not declare {topic} as {message_type}")
    for entry in manifest.get("files", []):
        path = root / entry.get("path", "")
        if not path.is_file():
            failures.append(f"missing file: {entry.get('path')}")
            continue
        if path.stat().st_size != entry.get("bytes"):
            failures.append(f"byte count mismatch: {entry.get('path')}")
        if digest(path) != entry.get("sha256"):
            failures.append(f"SHA-256 mismatch: {entry.get('path')}")
    if not manifest.get("files"):
        failures.append("manifest contains no hashed files")
    if not args.skip_bag_inspection:
        if not (root / "metadata.yaml").is_file():
            failures.append("metadata.yaml is missing")
        else:
            result = subprocess.run(["ros2", "bag", "info", str(root)], text=True, capture_output=True)
            inspected = result.stdout + result.stderr
            if result.returncode:
                failures.append("ros2 bag info failed")
            for topic, message_type in REQUIRED_TOPICS.items():
                if topic not in inspected or message_type not in inspected:
                    failures.append(f"bag inspection cannot confirm {topic} / {message_type}")
    if failures:
        print("Mid-360 dataset verification FAILED:", *failures, sep="\n- ", file=sys.stderr)
        return 1
    print("Mid-360 dataset verification PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

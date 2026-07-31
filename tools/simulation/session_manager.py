#!/usr/bin/env python3
"""Create, resolve, and prune isolated PX4 MID-360 sessions safely."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import time
from pathlib import Path


def command(args: list[str], cwd: Path | None = None) -> str:
    try:
        return subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                              timeout=10, check=False).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def create(root: Path, workspace: Path, px4: Path, arguments: dict) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    session = root/f"px4-mid360-{stamp}"
    suffix = 1
    while session.exists():
        session = root/f"px4-mid360-{stamp}-{suffix:02d}"
        suffix += 1
    for name in ("metrics", "snapshots", "logs", "latest", "pids"):
        (session/name).mkdir(parents=True)
    config = workspace/"tools/simulation/config/px4_mid360_observer.yaml"
    config_hash = hashlib.sha256(config.read_bytes()).hexdigest()
    branch = command(["git", "branch", "--show-current"], workspace)
    dirty = bool(command(["git", "status", "--porcelain"], workspace))
    data = {
        "schema_version": 1, "session_id": session.name,
        "start_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "start_monotonic_time": time.monotonic(), "hostname": platform.node(),
        "kernel": platform.release(), "ubuntu_version": command(["lsb_release", "-ds"]),
        "ROS_DISTRO": os.environ.get("ROS_DISTRO", ""), "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", ""),
        "gazebo_version": command(["gz", "sim", "--version"]),
        "px4_git_revision": command(["git", "rev-parse", "HEAD"], px4),
        "uav_navigation_git_revision": command(["git", "rev-parse", "HEAD"], workspace),
        "branch": branch, "dirty": dirty, "config_sha256": config_hash,
        "launch_arguments": arguments, "PX4_DIR": str(px4), "GZ_GUI": arguments["GZ_GUI"],
    }
    (session/"session.json").write_text(json.dumps(data, indent=2)+"\n")
    (session/"environment.txt").write_text("\n".join(
        f"{key}={value}" for key, value in sorted(os.environ.items())
        if not any(word in key.upper() for word in ("TOKEN", "SECRET", "PASSWORD", "KEY")))+"\n")
    (session/"git_state.txt").write_text(
        command(["git", "status", "--short", "--branch"], workspace)+"\n")
    (session/"process_tree_start.txt").write_text(
        command(["ps", "-e", "-o", "pid,ppid,pgid,args", "--forest"])+"\n")
    latest = root/"latest"
    temporary = root/".latest.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(session.name)
    temporary.replace(latest)
    return session


def prune(root: Path, keep: int) -> list[Path]:
    resolved = root.resolve()
    if resolved == Path("/") or root.name != "simulation":
        raise ValueError(f"refusing unsafe session root: {root}")
    sessions = sorted((path for path in root.glob("px4-mid360-*")
                       if path.is_dir() and path.parent.resolve() == resolved),
                      key=lambda path: path.stat().st_mtime, reverse=True)
    removed = sessions[max(0, keep):]
    for path in removed:
        shutil.rmtree(path)
    return removed


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    new = sub.add_parser("create")
    new.add_argument("--root", type=Path, required=True)
    new.add_argument("--workspace", type=Path, required=True)
    new.add_argument("--px4-dir", type=Path, required=True)
    new.add_argument("--gz-gui", required=True)
    new.add_argument("--arguments-json", default="{}")
    clean = sub.add_parser("clean")
    clean.add_argument("--root", type=Path, required=True)
    clean.add_argument("--keep", type=int, default=10)
    args = parser.parse_args()
    if args.command == "create":
        values = json.loads(args.arguments_json)
        values["GZ_GUI"] = args.gz_gui
        print(create(args.root.resolve(), args.workspace.resolve(), args.px4_dir.resolve(), values))
    else:
        for removed in prune(args.root, args.keep):
            print(removed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

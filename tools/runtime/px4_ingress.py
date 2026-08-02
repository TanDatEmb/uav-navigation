#!/usr/bin/env python3
"""Build and validate the PX4 v1.17 ingress package in this workspace."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
PX4_SHA = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
PX4_DEFAULT_DIR = Path.home() / "Dev" / "Autopilot-p0.7-v1.17"
PX4_CURRENT_DIR = Path.home() / "Dev" / "Autopilot"


def run(command: list[str], cwd: Path = ROOT) -> int:
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, cwd=cwd, check=False).returncode


def verify_messages() -> int:
    return run([sys.executable, str(ROOT / "tools/runtime/px4_deps.py"), "verify"])


def colcon(target: str) -> int:
    if verify_messages() != 0:
        return 2
    workers = os.environ.get("PARALLEL_WORKERS", "1")
    if target == "build":
        os.environ["MAKEFLAGS"] = f"-j{os.environ.get('MAKE_JOBS', '1')}"
        return run([
            "colcon", "--log-base", str(ROOT / "log"), "build",
            "--base-paths", "src", "--build-base", str(ROOT / "build"),
            "--install-base", str(ROOT / "install"), "--symlink-install",
            "--parallel-workers", workers, "--executor", "sequential",
            "--event-handlers", "console_direct+",
            "--packages-up-to", "px4_odometry_bridge",
        ])
    if target == "test":
        return run([
            "colcon", "--log-base", str(ROOT / "log"), "test",
            "--base-paths", "src", "--build-base", str(ROOT / "build"),
            "--install-base", str(ROOT / "install"),
            "--packages-select", "px4_odometry_bridge",
            "--event-handlers", "console_direct+",
        ])
    if target == "check":
        return run(["colcon", "test-result", "--test-result-base", str(ROOT / "build"), "--verbose"])
    raise ValueError(target)


def px4_worktree() -> Path | None:
    requested = os.environ.get("PX4_DIR")
    path = Path(requested).expanduser().resolve() if requested else PX4_DEFAULT_DIR
    if not path.exists():
        if not PX4_CURRENT_DIR.is_dir():
            print(f"BLOCKED: canonical PX4 source is unavailable: {PX4_CURRENT_DIR}", file=sys.stderr)
            return None
        if run(["git", "-C", str(PX4_CURRENT_DIR), "cat-file", "-e",
                f"{PX4_SHA}^{{commit}}"]) != 0:
            print(f"BLOCKED: PX4 commit {PX4_SHA} is unavailable", file=sys.stderr)
            return None
        if run(["git", "-C", str(PX4_CURRENT_DIR), "worktree", "add", "--detach", str(path), PX4_SHA]) != 0:
            return None
    actual = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], text=True,
                            stdout=subprocess.PIPE, check=False).stdout.strip()
    if actual != PX4_SHA:
        print(f"BLOCKED: PX4_DIR={path} is at {actual}, expected {PX4_SHA}", file=sys.stderr)
        return None
    dirty = subprocess.run(["git", "-C", str(path), "status", "--porcelain"], text=True,
                           stdout=subprocess.PIPE, check=False).stdout.strip()
    if dirty:
        print(f"BLOCKED: PX4_DIR={path} is dirty", file=sys.stderr)
        return None
    return path


def sitl() -> int:
    path = px4_worktree()
    if path is None:
        return 2
    environment = os.environ.copy()
    environment["PX4_DIR"] = str(path)
    environment["GZ_GUI"] = "0"
    environment["ENABLE_RVIZ"] = "0"
    print("+ make sim-px4-mid360-headless PX4_DIR=" + str(path), flush=True)
    return subprocess.run(["make", "sim-px4-mid360-headless"], cwd=ROOT,
                          env=environment, check=False).returncode


def smoke() -> int:
    if verify_messages() != 0:
        return 2
    return run([
        "timeout", "15", "ros2", "run", "px4_odometry_bridge",
        "px4_odometry_bridge_node", "--ros-args", "-p", "use_sim_time:=true",
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", choices=("build", "test", "check", "sitl", "smoke"))
    args = parser.parse_args()
    if args.target in {"build", "test", "check"}:
        return colcon(args.target)
    if args.target == "sitl":
        return sitl()
    return smoke()


if __name__ == "__main__":
    raise SystemExit(main())

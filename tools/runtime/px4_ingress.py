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


def verify_submodule() -> int:
    checkout = ROOT / "src/external/px4_msgs"
    staged = subprocess.run(
        ["git", "ls-files", "--stage", "src/external/px4_msgs"],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, check=False,
    ).stdout.strip()
    if not staged.startswith("160000 "):
        print("ERROR: src/external/px4_msgs is not tracked as a Git submodule", file=sys.stderr)
        return 2
    if not checkout.is_dir() or not (checkout / "package.xml").is_file():
        print(
            "ERROR: px4_msgs package not found; initialize submodule with: "
            "git submodule update --init --recursive",
            file=sys.stderr,
        )
        return 2
    actual = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        text=True, stdout=subprocess.PIPE, check=False,
    ).stdout.strip()
    remote = subprocess.run(
        ["git", "-C", str(checkout), "remote", "get-url", "origin"],
        text=True, stdout=subprocess.PIPE, check=False,
    ).stdout.strip()
    dirty = subprocess.run(
        ["git", "-C", str(checkout), "status", "--porcelain"],
        text=True, stdout=subprocess.PIPE, check=False,
    ).stdout.strip()
    print(f"px4_msgs submodule SHA: {actual}")
    print(f"px4_msgs remote: {remote}")
    print(f"px4_msgs working tree: {'dirty' if dirty else 'clean'}")
    if actual != "86d8239e962f6939e05c3737784f60c02fa884db":
        print("ERROR: px4_msgs submodule SHA mismatch", file=sys.stderr)
        return 2
    if remote.rstrip("/").removesuffix(".git") != "https://github.com/PX4/px4_msgs":
        print("ERROR: px4_msgs submodule remote mismatch", file=sys.stderr)
        return 2
    if dirty:
        print("ERROR: px4_msgs submodule working tree is dirty", file=sys.stderr)
        return 2
    print("px4_msgs submodule: verified")
    return 0


def colcon(target: str) -> int:
    if verify_submodule() != 0:
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
    if verify_submodule() != 0:
        return 2
    return run([
        "timeout", "15", "ros2", "run", "px4_odometry_bridge",
        "px4_odometry_bridge_node", "--ros-args", "-p", "use_sim_time:=true",
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "target", choices=("build", "test", "check", "verify-submodule", "sitl", "smoke")
    )
    args = parser.parse_args()
    if args.target in {"build", "test", "check"}:
        return colcon(args.target)
    if args.target == "verify-submodule":
        return verify_submodule()
    if args.target == "sitl":
        return sitl()
    return smoke()


if __name__ == "__main__":
    raise SystemExit(main())

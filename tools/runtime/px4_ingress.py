#!/usr/bin/env python3
"""Run the explicitly opt-in PX4 ingress validation workflow.

The default repository build never resolves or mutates a PX4 checkout. This
helper requires isolated dependency workspaces supplied by the caller and
fails with an actionable BLOCKED result when they are absent.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def run(command: list[str], cwd: Path | None = None) -> int:
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, cwd=cwd, check=False).returncode


def read_lock(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    section = ""
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.endswith(":") and not line.startswith(" "):
            section = line[:-1]
            continue
        if ":" in line and not line.startswith("-"):
            key, value = (part.strip() for part in line.split(":", 1))
            if value and value not in ("{}", "[]"):
                values[f"{section}.{key}"] = value.strip("'\"")
    return values


def require_workspace(name: str) -> Path:
    variable = {"px4_msgs": "PX4_MSGS_WS", "px4_ros2": "PX4_ROS2_WS"}[name]
    value = os.environ.get(variable)
    if not value:
        raise RuntimeError(
            f"BLOCKED: {variable} is not set; prepare an isolated {name} workspace at the locked ref"
        )
    path = Path(value).expanduser().resolve()
    if not (path / "install" / "setup.bash").is_file():
        raise RuntimeError(f"BLOCKED: {variable}={path} has no install/setup.bash")
    return path


def colcon_target(target: str, lock: Path) -> int:
    values = read_lock(lock)
    print("P0.7 compatibility lock:", values.get("canonical.px4.sha", "unknown"))
    msgs = require_workspace("px4_msgs")
    ros2 = require_workspace("px4_ros2")
    env = os.environ.copy()
    setup = f"source /opt/ros/jazzy/setup.bash && source {msgs}/install/setup.bash && source {ros2}/install/setup.bash"
    command = ["bash", "-lc", f"{setup} && colcon build --packages-select navigation_interfaces px4_odometry_bridge"]
    if target == "test":
        command = ["bash", "-lc", f"{setup} && colcon test --packages-select px4_odometry_bridge && colcon test-result --verbose"]
    if target == "sitl":
        command = ["bash", "-lc", f"{setup} && echo 'P0.7 stable SITL requires the locked PX4 v1.17 isolated worktree' && make -C '{ROOT}' sim-px4-mid360-headless"]
    if target == "smoke":
        command = ["bash", "-lc", f"{setup} && ros2 run px4_odometry_bridge px4_odometry_bridge_node --ros-args -p use_sim_time:=true"]
    return run(command, ROOT)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", choices=("build", "test", "sitl", "smoke"))
    parser.add_argument("--lock", type=Path, required=True)
    args = parser.parse_args()
    try:
        return colcon_target(args.target, args.lock.resolve())
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

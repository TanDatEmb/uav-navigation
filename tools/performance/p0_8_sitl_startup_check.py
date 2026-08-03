#!/usr/bin/env python3
"""Fail-closed PX4/Gazebo/XRCE/bridge startup check for P0.8."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


RAW_TOPIC_RE = re.compile(r"^(/fmu/out/vehicle_odometry(?:_v\d+)?)\s+\[px4_msgs/msg/VehicleOdometry\]")


def run(args: list[str], timeout: float = 10.0, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout, check=False)


def stamp_ns(message: Any) -> int:
    stamp = getattr(message, "clock", getattr(message, "header", None))
    if hasattr(stamp, "stamp"):
        stamp = stamp.stamp
    if hasattr(stamp, "sec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return 0


class StartupProbe(Node):
    def __init__(self) -> None:
        super().__init__("p0_8_sitl_startup_probe")
        self.px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_values: list[int] = []
        self.raw_count = 0
        self.output_count = 0
        self.output_stamp_ns = 0
        self.bridge_values: dict[str, str] = {}
        self.create_subscription(Clock, "/clock", lambda message: self.clock_values.append(stamp_ns(message)), 20)
        self.create_subscription(Odometry, "/px4/odometry_ros", self.on_output, 10)
        self.create_subscription(DiagnosticArray, "/px4/diagnostics", self.on_diagnostics, 10)

    def subscribe_raw(self, topic: str) -> None:
        self.create_subscription(VehicleOdometry, topic, self.on_raw, self.px4_qos)

    def on_raw(self, _message: VehicleOdometry) -> None:
        self.raw_count += 1

    def on_output(self, message: Odometry) -> None:
        self.output_count += 1
        self.output_stamp_ns = stamp_ns(message)

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "px4_odometry_bridge":
                self.bridge_values = {item.key: item.value for item in status.values}


def spin_for(node: StartupProbe, seconds: float, predicate) -> bool:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def terminate(process: subprocess.Popen[str] | None, timeout: float = 8.0) -> dict[str, Any]:
    if process is None:
        return {"present": False, "exit_code": None, "clean": True}
    result: dict[str, Any] = {"present": True, "pid": process.pid, "exit_code": None, "clean": False}
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=timeout)
        except (OSError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except OSError:
                    pass
                process.wait(timeout=3.0)
    # The launcher is a shell process and may have left a Gazebo child alive
    # after the shell's EXIT trap ran.  The process was started in its own
    # session, so reap the complete owned process group explicitly.
    for group_signal in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, group_signal)
        except OSError:
            break
        time.sleep(0.2)
    result["exit_code"] = process.returncode
    result["clean"] = process.returncode in (0, -signal.SIGINT, -signal.SIGTERM)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--px4-dir", type=Path, default=Path.home() / "Dev/Autopilot-p0.7-v1.17")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    px4_dir = args.px4_dir.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    logs = output / "logs"
    logs.mkdir(exist_ok=True)
    result: dict[str, Any] = {
        "schema_version": 1,
        "git_sha": run(["git", "rev-parse", "HEAD"], cwd=workspace).stdout.strip(),
        "px4_sha": run(["git", "-C", str(px4_dir), "rev-parse", "HEAD"]).stdout.strip(),
        "px4_msgs_sha": run(["git", "-C", str(workspace / "src/external/px4_msgs"), "rev-parse", "HEAD"]).stdout.strip(),
        "session": str(output),
        "clock": {"topic_present": False, "sample_received": False, "first_ns": 0, "last_ns": 0, "advanced": False},
        "xrce": {"agent_started": False, "client_connected": False},
        "raw_odometry": {"candidate_topics": [], "selected_topic": "", "publisher_count": 0, "sample_received": False},
        "bridge": {"started": False, "alive": False, "use_sim_time": True, "simulation_clock": True},
        "output": {"topic_present": False, "sample_received": False, "sample_stamp_ns": 0},
        "processes": {}, "result": "FAIL", "failure_stage": "", "failure_reason": "",
    }
    processes: dict[str, subprocess.Popen[str]] = {}
    process_meta: dict[str, dict[str, Any]] = {}
    node: StartupProbe | None = None
    try:
        px4_log = (logs / "px4_gazebo.log").open("w", encoding="utf-8")
        processes["px4_gazebo"] = subprocess.Popen(
            ["bash", str(workspace / "tools/simulation/run_px4_mid360.sh")],
            cwd=workspace, env={**os.environ, "PX4_DIR": str(px4_dir), "SESSION_DIR": str(output), "GZ_GUI": "0"},
            stdout=px4_log, stderr=subprocess.STDOUT, start_new_session=True, text=True)
        process_meta["px4_gazebo"] = {
            "command": ["bash", str(workspace / "tools/simulation/run_px4_mid360.sh")],
            "log": str(logs / "px4_gazebo.log"), "started_at_unix": time.time(),
        }
        agent_log = (logs / "xrce_agent.log").open("w", encoding="utf-8")
        agent_command = ["MicroXRCEAgent", "udp4", "-p", "8888"]
        processes["xrce_agent"] = subprocess.Popen(
            agent_command, stdout=agent_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True)
        process_meta["xrce_agent"] = {
            "command": agent_command, "log": str(logs / "xrce_agent.log"),
            "started_at_unix": time.time(),
        }
        result["xrce"]["agent_started"] = processes["xrce_agent"].poll() is None
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            if processes["px4_gazebo"].poll() is not None:
                result["failure_stage"] = "px4_gazebo"
                result["failure_reason"] = "PX4/Gazebo process exited"
                return 2
            topics = run(["gz", "topic", "-l"], timeout=5).stdout.splitlines()
            if "/world/px4_lio_smoke/clock" in topics:
                result["clock"]["topic_present"] = True
                break
            time.sleep(0.5)
        if not result["clock"]["topic_present"]:
            result["failure_stage"] = "clock_topic"
            result["failure_reason"] = "Gazebo clock topic did not appear"
            return 2

        # The Gazebo clock must be bridged before any ROS node using
        # use_sim_time is started.  This is infrastructure for the startup
        # check, not the PX4 odometry bridge under qualification.
        clock_bridge_log = (logs / "clock_bridge.log").open("w", encoding="utf-8")
        processes["clock_bridge"] = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
             "-r", "__node:=px4_mid360_clock_bridge",
             "-p", f"config_file:={workspace / 'src/uav_simulation/bridge/px4_mid360_bridge.yaml'}",
             "-p", "use_sim_time:=false"],
            stdout=clock_bridge_log, stderr=subprocess.STDOUT, start_new_session=True, text=True)
        process_meta["clock_bridge"] = {
            "command": ["ros2", "run", "ros_gz_bridge", "parameter_bridge"],
            "log": str(logs / "clock_bridge.log"), "started_at_unix": time.time(),
        }

        rclpy.init()
        node = StartupProbe()
        if not spin_for(node, 20.0, lambda: len(node.clock_values) >= 2 and node.clock_values[-1] > node.clock_values[0]):
            result["failure_stage"] = "clock_advance"
            result["failure_reason"] = "ROS /clock did not advance"
            return 2
        result["clock"].update({"sample_received": True, "first_ns": node.clock_values[0],
                                "last_ns": node.clock_values[-1], "advanced": True})

        candidates: list[str] = []
        raw_deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < raw_deadline and not candidates:
            graph = dict(node.get_topic_names_and_types())
            candidates = sorted(
                topic for topic, types in graph.items()
                if re.fullmatch(r"/fmu/out/vehicle_odometry(?:_v\d+)?", topic)
                and "px4_msgs/msg/VehicleOdometry" in types)
            if not candidates:
                rclpy.spin_once(node, timeout_sec=0.2)
        result["raw_odometry"]["candidate_topics"] = candidates
        if not candidates:
            result["failure_stage"] = "raw_topic"
            result["failure_reason"] = "no versioned VehicleOdometry topic appeared"
            return 2
        raw_topic = candidates[0]
        result["raw_odometry"]["selected_topic"] = raw_topic
        result["raw_odometry"]["publisher_count"] = node.count_publishers(raw_topic)
        node.subscribe_raw(raw_topic)
        if not spin_for(node, 20.0, lambda: node.raw_count > 0):
            result["failure_stage"] = "raw_sample"
            result["failure_reason"] = "no raw VehicleOdometry sample"
            return 2
        result["raw_odometry"]["sample_received"] = True
        agent_log_text = Path(process_meta["xrce_agent"]["log"]).read_text(encoding="utf-8", errors="replace")
        result["xrce"]["session_established_log"] = "session established" in agent_log_text
        bridge_log = (logs / "bridge.log").open("w", encoding="utf-8")
        processes["bridge"] = subprocess.Popen(
            ["ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node", "--ros-args",
             "-p", "use_sim_time:=true", "-p", "simulation_clock:=true"],
            stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True, text=True)
        process_meta["bridge"] = {
            "command": ["ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node"],
            "log": str(logs / "bridge.log"), "started_at_unix": time.time(),
        }
        result["bridge"]["started"] = True
        if not spin_for(node, 30.0, lambda: node.output_count > 0):
            result["failure_stage"] = "bridge_output"
            result["failure_reason"] = "bridge started but no /px4/odometry_ros sample"
            return 2
        spin_for(node, 1.0, lambda: node.bridge_values.get("state") == "running")
        result["bridge"].update({"alive": processes["bridge"].poll() is None,
                                  "diagnostic_state": node.bridge_values.get("state", ""),
                                  "diagnostic_message": node.bridge_values.get("message", ""),
                                  "timestamp_rejected_count": int(node.bridge_values.get("timestamp_rejected_count", 0)),
                                  "conversion_rejected_count": int(node.bridge_values.get("conversion_rejected_count", 0)),
                                  "reset_suppressed_count": int(node.bridge_values.get("reset_suppressed_count", 0))})
        result["output"].update({"topic_present": True, "sample_received": True,
                                  "sample_stamp_ns": node.output_stamp_ns})
        result["xrce"]["client_connected"] = (
            result["raw_odometry"]["sample_received"] and
            result["xrce"].get("session_established_log", False))
        result["result"] = "PASS"
        return 0
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for role, process in processes.items():
            state = terminate(process)
            state.update(process_meta.get(role, {}), pgid=process.pid)
            result["processes"][role] = state
        output.joinpath("startup-check.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())

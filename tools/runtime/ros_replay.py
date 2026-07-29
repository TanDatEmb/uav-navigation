#!/usr/bin/env python3
"""Drain-aware ROS 2 replay harness for fast_lio runtime acceptance."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any


RECORDED_TOPICS = (
    "/lio/diagnostics",
    "/lio/odometry",
    "/lio/registered_points",
    "/lio/local_map",
    "/tf",
    "/tf_static",
)


def diagnostics_values(message: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for status in message.status:
        if status.name != "fast_lio/transport":
            continue
        level = status.level
        if isinstance(level, (bytes, bytearray)):
            level = level[0]
        result["level"] = int(level)
        result["message"] = status.message
        for item in status.values:
            value: Any = item.value
            if value in ("true", "false"):
                value = value == "true"
            else:
                try:
                    value = float(value) if "." in value else int(value)
                except ValueError:
                    pass
            result[item.key] = value
    return result


def drained(state: dict[str, Any]) -> bool:
    return (
        state.get("current_input_queue_depth") == 0
        and state.get("current_imu_queue_depth") == 0
        and state.get("current_lidar_queue_depth") == 0
        and state.get("received_imu_count") == state.get("processed_imu_count")
        and state.get("received_lidar_count") == state.get("processed_lidar_count")
    )


def acceptance_failures(state: dict[str, Any]) -> list[str]:
    failures = []
    if bool(state.get("overflow_detected")):
        failures.append("input queue overflow detected")
    if bool(state.get("processing_lag_exceeded")):
        failures.append("maximum processing lag exceeded")
    if int(state.get("imu_drop_count", 0)) != 0:
        failures.append("IMU messages dropped")
    if int(state.get("lidar_drop_count", 0)) != 0:
        failures.append("LiDAR messages dropped")
    if not drained(state):
        failures.append("estimator queue did not drain")
    return failures


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def collect(output: Path, state_path: Path) -> int:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray

    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("fast_lio_runtime_collector")

    def callback(message: DiagnosticArray) -> None:
        state = diagnostics_values(message)
        if not state:
            return
        state["collector_wall_time_ns"] = time.time_ns()
        with output.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(state, sort_keys=True) + "\n")
        atomic_json(state_path, state)

    node.create_subscription(DiagnosticArray, "/lio/diagnostics", callback, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def wait_for_subscriber(
    topic: str,
    process: subprocess.Popen[Any],
    timeout: float,
    minimum_count: int = 1,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"estimator exited during readiness with {process.returncode}")
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            text=True,
            capture_output=True,
            timeout=5,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if line.strip().startswith("Subscription count:"):
                    if int(line.rsplit(":", 1)[1]) >= minimum_count:
                        return
        time.sleep(0.2)
    raise RuntimeError(f"subscription readiness timed out for {topic}")


def wait_for_state(
    state_path: Path, process: subprocess.Popen[Any], timeout: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"estimator exited while waiting for diagnostics: {process.returncode}")
        if state_path.is_file():
            return json.loads(state_path.read_text(encoding="utf-8"))
        time.sleep(0.1)
    raise RuntimeError("first transport diagnostics timed out")


def wait_for_drain(
    state_path: Path, process: subprocess.Popen[Any], timeout: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    latest: dict[str, Any] = {}
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"estimator exited before drain with {process.returncode}")
        if state_path.is_file():
            latest = json.loads(state_path.read_text(encoding="utf-8"))
            if drained(latest):
                return latest
        time.sleep(0.2)
    raise RuntimeError(
        "drain timeout: "
        + json.dumps(
            {
                key: latest.get(key)
                for key in (
                    "current_input_queue_depth",
                    "current_imu_queue_depth",
                    "current_lidar_queue_depth",
                    "processing_lag_ns",
                    "received_imu_count",
                    "processed_imu_count",
                    "received_lidar_count",
                    "processed_lidar_count",
                )
            },
            sort_keys=True,
        )
    )


def stop(process: subprocess.Popen[Any], timeout: float = 10.0) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def run(args: argparse.Namespace) -> int:
    args.output.mkdir(parents=True, exist_ok=False)
    state_path = args.output / "diagnostics_state.json"
    logs: list[Any] = []
    processes: list[subprocess.Popen[Any]] = []

    def log(name: str) -> Any:
        stream = (args.output / name).open("w", encoding="utf-8")
        logs.append(stream)
        return stream

    node = subprocess.Popen(
        [
            "ros2", "run", "fast_lio_ros", "fast_lio_node",
            "--ros-args", "--params-file", str(args.config),
        ],
        stdout=log("node.stdout.log"),
        stderr=log("node.stderr.log"),
        start_new_session=True,
    )
    processes.append(node)
    collector = subprocess.Popen(
        [
            sys.executable, str(Path(__file__).resolve()), "collect",
            "--output", str(args.output / "diagnostics.jsonl"),
            "--state", str(state_path),
        ],
        stdout=log("collector.log"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    processes.append(collector)
    recorder: subprocess.Popen[Any] | None = None
    replay: subprocess.Popen[Any] | None = None
    summary: dict[str, Any] = {"rate": args.rate}
    try:
        wait_for_subscriber(args.imu_topic, node, args.readiness_timeout)
        wait_for_subscriber(args.lidar_topic, node, args.readiness_timeout)
        wait_for_state(state_path, node, args.readiness_timeout)
        recorder = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(args.output / "outputs"), *RECORDED_TOPICS],
            stdout=log("record.stdout.log"),
            stderr=log("record.stderr.log"),
            start_new_session=True,
        )
        processes.append(recorder)
        wait_for_subscriber(
            "/lio/diagnostics", recorder, args.readiness_timeout, minimum_count=2
        )
        replay = subprocess.Popen(
            [
                "ros2", "bag", "play", str(args.bag), "--rate", str(args.rate),
                "--delay", "5",
            ],
            stdout=log("replay.stdout.log"),
            stderr=log("replay.stderr.log"),
            start_new_session=True,
        )
        processes.append(replay)
        replay_returncode = replay.wait()
        summary["replay_returncode"] = replay_returncode
        if replay_returncode != 0:
            raise RuntimeError(f"rosbag replay failed with {replay_returncode}")
        final = wait_for_drain(state_path, node, args.drain_timeout)
        # Wait for one more timer publication so the saved final sample is post-drain.
        prior_wall_time = final.get("collector_wall_time_ns", 0)
        deadline = time.monotonic() + min(2.0, args.drain_timeout)
        while time.monotonic() < deadline:
            final = json.loads(state_path.read_text(encoding="utf-8"))
            if final.get("collector_wall_time_ns", 0) > prior_wall_time:
                break
            time.sleep(0.1)
        summary["diagnostics"] = final
        summary["failures"] = acceptance_failures(final)
    except Exception as error:
        summary["failures"] = [str(error)]
    finally:
        for process in reversed(processes):
            stop(process)
        summary["estimator_returncode"] = node.returncode
        atomic_json(args.output / "summary.json", summary)
        for stream in logs:
            stream.close()
    return 1 if summary["failures"] else 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    subparsers = result.add_subparsers(dest="command", required=True)
    collector = subparsers.add_parser("collect")
    collector.add_argument("--output", type=Path, required=True)
    collector.add_argument("--state", type=Path, required=True)
    replay = subparsers.add_parser("run")
    replay.add_argument("--bag", type=Path, required=True)
    replay.add_argument("--config", type=Path, required=True)
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--imu-topic", required=True)
    replay.add_argument("--lidar-topic", required=True)
    replay.add_argument("--rate", type=float, default=1.0)
    replay.add_argument("--readiness-timeout", type=float, default=30.0)
    replay.add_argument("--drain-timeout", type=float, default=120.0)
    return result


if __name__ == "__main__":
    arguments = parser().parse_args()
    if arguments.command == "collect":
        raise SystemExit(collect(arguments.output, arguments.state))
    raise SystemExit(run(arguments))

#!/usr/bin/env python3
"""Drain-aware ROS 2 replay harness for fast_lio runtime acceptance."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
from typing import Any


RECORDED_TOPICS = (
    "/lio/diagnostics",
    "/lio/odometry_corrected",
    "/lio/registered_points",
    "/lio/local_map",
    "/tf",
    "/tf_static",
)


def diagnostic_status_values(status: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
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


def diagnostics_values(message: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for status in message.status:
        if status.name == "fast_lio/transport":
            result.update(diagnostic_status_values(status))
        elif status.name == "fast_lio/propagated_odometry":
            result["propagated_odometry"] = diagnostic_status_values(status)
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
    propagated = state.get("propagated_odometry", {})
    if propagated.get("enabled") is True:
        if propagated.get("timestamp_regression_count", 0) != 0:
            failures.append("propagated timestamp regression detected")
        if propagated.get("duplicate_correction_drop_count", 0) != 0:
            failures.append("propagated duplicate correction detected")
        if propagated.get("queue_overflow_count", 0) != 0:
            failures.append("propagated auxiliary queue overflow detected")
        if propagated.get("load_shedding_count", 0) != 0:
            failures.append("propagated load shedding detected")
        if propagated.get("continuity_reset_count", 0) != 0:
            failures.append("propagated continuity reset detected")
        if propagated.get("requires_reanchor") is True:
            failures.append("propagated continuity requires reanchor")
        if propagated.get("publication_count", 0) <= 0:
            failures.append("propagated odometry was not published")
    return failures


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def process_start_ticks(pid: int) -> int:
    stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    closing_parenthesis = stat.rfind(")")
    if closing_parenthesis < 0:
        raise RuntimeError(f"invalid /proc stat for pid {pid}")
    fields_after_command = stat[closing_parenthesis + 2:].split()
    return int(fields_after_command[19])


def register_process(
    registry_path: Path,
    process: subprocess.Popen[Any],
    role: str,
    command: list[str],
) -> None:
    payload: dict[str, Any] = {"schema_version": 1, "processes": []}
    if registry_path.is_file():
        payload = json.loads(registry_path.read_text(encoding="utf-8"))
    payload["processes"].append({
        "role": role,
        "pid": process.pid,
        "process_group": os.getpgid(process.pid),
        "start_ticks": process_start_ticks(process.pid),
        "command": command,
    })
    atomic_json(registry_path, payload)


def track_process(
    processes: list[subprocess.Popen[Any]],
    registry_path: Path,
    process: subprocess.Popen[Any],
    role: str,
    command: list[str],
) -> None:
    processes.append(process)
    register_process(registry_path, process, role, command)


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
        rclpy.try_shutdown()
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


def process_group_exists(process_group: int) -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        closing_parenthesis = stat.rfind(")")
        if closing_parenthesis < 0:
            continue
        fields = stat[closing_parenthesis + 2:].split()
        if fields[0] != "Z" and int(fields[2]) == process_group:
            return True
    return False


def wait_for_process_group_exit(process_group: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not process_group_exists(process_group):
            return True
        time.sleep(0.05)
    return not process_group_exists(process_group)


def stop(process: subprocess.Popen[Any], timeout: float = 10.0) -> None:
    process_group = process.pid
    escalation = (
        (signal.SIGINT, timeout),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    )
    for stop_signal, wait_timeout in escalation:
        if not process_group_exists(process_group):
            break
        try:
            os.killpg(process_group, stop_signal)
        except ProcessLookupError:
            break
        if wait_for_process_group_exit(process_group, wait_timeout):
            break
    try:
        process.wait(timeout=0.1)
    except subprocess.TimeoutExpired:
        pass
    if process_group_exists(process_group):
        raise RuntimeError(
            f"process group {process_group} survived SIGKILL cleanup"
        )


def wait_for_replay(process: subprocess.Popen[Any], timeout: float) -> int:
    if timeout <= 0.0:
        return process.wait()
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"rosbag replay timed out after {timeout:.1f} seconds"
        ) from error


class ReplayInterrupted(RuntimeError):
    pass


def rviz_command(
    args: argparse.Namespace,
    environment: dict[str, str] | None = None,
) -> list[str] | None:
    if not args.enable_rviz:
        return None
    executable = shutil.which("rviz2")
    if executable is None:
        raise RuntimeError("RViz requested but rviz2 is not installed or not on PATH")
    if args.rviz_config is None or not args.rviz_config.is_file():
        raise RuntimeError(f"RViz config does not exist: {args.rviz_config}")
    display_environment = os.environ if environment is None else environment
    if not (display_environment.get("DISPLAY") or display_environment.get("WAYLAND_DISPLAY")):
        raise RuntimeError(
            "RViz requested in a headless environment; DISPLAY or WAYLAND_DISPLAY is required"
        )
    return [executable, "-d", str(args.rviz_config)]


def run(args: argparse.Namespace) -> int:
    requested_rviz_command = rviz_command(args)
    args.output.mkdir(parents=True, exist_ok=False)
    state_path = args.output / "diagnostics_state.json"
    registry_path = args.output / "process_groups.json"
    logs: list[Any] = []
    processes: list[subprocess.Popen[Any]] = []

    def log(name: str) -> Any:
        stream = (args.output / name).open("w", encoding="utf-8")
        logs.append(stream)
        return stream

    node_command = [
        "ros2", "run", "fast_lio_ros", "fast_lio_node",
        "--ros-args", "--params-file", str(args.config),
    ]
    node = subprocess.Popen(
        node_command,
        stdout=log("node.stdout.log"),
        stderr=log("node.stderr.log"),
        start_new_session=True,
    )
    processes.append(node)
    register_process(registry_path, node, "node", node_command)
    collector_command = [
        sys.executable, str(Path(__file__).resolve()), "collect",
        "--output", str(args.output / "diagnostics.jsonl"),
        "--state", str(state_path),
    ]
    collector = subprocess.Popen(
        collector_command,
        stdout=log("collector.log"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    processes.append(collector)
    register_process(registry_path, collector, "collector", collector_command)
    recorder: subprocess.Popen[Any] | None = None
    replay: subprocess.Popen[Any] | None = None
    rviz: subprocess.Popen[Any] | None = None
    summary: dict[str, Any] = {"rate": args.rate}
    previous_signal_handlers: dict[signal.Signals, Any] = {}

    def interrupt(signum: int, _frame: Any) -> None:
        raise ReplayInterrupted(f"replay interrupted by {signal.Signals(signum).name}")

    for handled_signal in (signal.SIGINT, signal.SIGTERM):
        previous_signal_handlers[handled_signal] = signal.signal(
            handled_signal, interrupt
        )
    try:
        wait_for_subscriber(args.imu_topic, node, args.readiness_timeout)
        wait_for_subscriber(args.lidar_topic, node, args.readiness_timeout)
        wait_for_state(state_path, node, args.readiness_timeout)
        recorder_command = [
            "ros2", "bag", "record", "-o",
            str(args.output / "outputs"), *RECORDED_TOPICS,
        ]
        recorder = subprocess.Popen(
            recorder_command,
            stdout=log("record.stdout.log"),
            stderr=log("record.stderr.log"),
            start_new_session=True,
        )
        processes.append(recorder)
        register_process(registry_path, recorder, "recorder", recorder_command)
        wait_for_subscriber(
            "/lio/diagnostics", recorder, args.readiness_timeout, minimum_count=2
        )
        if requested_rviz_command is not None:
            rviz = subprocess.Popen(
                requested_rviz_command,
                stdout=log("rviz.stdout.log"),
                stderr=log("rviz.stderr.log"),
                start_new_session=True,
            )
            track_process(
                processes, registry_path, rviz, "rviz", requested_rviz_command
            )
            time.sleep(0.5)
            if rviz.poll() is not None:
                raise RuntimeError(
                    f"RViz exited during startup with return code {rviz.returncode}; "
                    f"see {args.output / 'rviz.stderr.log'}"
                )
        replay_command = [
            "ros2", "bag", "play", str(args.bag), "--rate", str(args.rate),
            "--delay", "5",
        ]
        replay = subprocess.Popen(
            replay_command,
            stdout=log("replay.stdout.log"),
            stderr=log("replay.stderr.log"),
            start_new_session=True,
        )
        processes.append(replay)
        register_process(registry_path, replay, "replay", replay_command)
        replay_returncode = wait_for_replay(replay, args.replay_timeout)
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
            try:
                stop(process)
            except Exception as error:
                summary.setdefault("cleanup_failures", []).append(str(error))
                summary.setdefault("failures", []).append(
                    f"process cleanup failed: {error}"
                )
        for handled_signal, previous_handler in previous_signal_handlers.items():
            signal.signal(handled_signal, previous_handler)
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
    replay.add_argument("--replay-timeout", type=float, default=900.0)
    replay.add_argument("--enable-rviz", action="store_true")
    replay.add_argument("--rviz-config", type=Path)
    return result


if __name__ == "__main__":
    arguments = parser().parse_args()
    if arguments.command == "collect":
        raise SystemExit(collect(arguments.output, arguments.state))
    raise SystemExit(run(arguments))

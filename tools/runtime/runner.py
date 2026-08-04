#!/usr/bin/env python3
"""Single entrypoint for dataset, headless SITL and interactive sessions."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from typing import Any, Callable

import yaml

from process_group import Session, resolve_latest
import report


ROOT = Path(__file__).resolve().parents[2]
RUNTIME_CONFIG = ROOT / "config/runtime"
ARTIFACT_ROOT = ROOT / ".artifacts/runtime"
RVIZ_CONFIG = ROOT / "src/navigation_estimator/livox_ros_driver2_interface/config/display_point_cloud_ROS2.rviz"
NO_RVIZ_ENV = {
    "ENABLE_RVIZ": "0",
    "RVIZ_ENABLE": "0",
    "DISABLE_RVIZ": "1",
    "NAVIGATION_NO_RVIZ": "1",
}
RVIZ_ENV = {
    "ENABLE_RVIZ": "1",
    "RVIZ_ENABLE": "1",
    "DISABLE_RVIZ": "0",
    "NAVIGATION_NO_RVIZ": "0",
}


def load_config(name: str) -> dict[str, Any]:
    path = RUNTIME_CONFIG / name
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"runtime config must be a mapping: {path}")
    common = yaml.safe_load((RUNTIME_CONFIG / "common.yaml").read_text(encoding="utf-8"))
    if isinstance(common, dict):
        value["runtime"] = dict(common.get("runtime", {}))
        value["runtime"].update(value.get("runtime_overrides", {}))
    value["_path"] = str(path)
    return value


def _ros_shell(command: list[str], *, enable_rviz: bool = False) -> list[str]:
    environment = RVIZ_ENV if enable_rviz else NO_RVIZ_ENV
    parts = [
        "export " + " ".join(f"{key}={value}" for key, value in environment.items()),
        "source /opt/ros/jazzy/setup.bash",
    ]
    install = ROOT / "install/setup.bash"
    if install.is_file():
        parts.append(f"source {shlex.quote(str(install))}")
    parts.append("exec " + shlex.join(command))
    return ["bash", "-lc", "; ".join(parts)]


def _rviz_command(*, use_sim_time: bool = False) -> list[str]:
    command = [
        "rviz2", "-d", str(RVIZ_CONFIG),
        "--ros-args",
    ]
    if use_sim_time:
        command.extend(["-p", "use_sim_time:=true"])
    command.extend(["-r", "/livox/lidar:=/lidar/points"])
    return command


def _start_rviz(session: Session, *, use_sim_time: bool = False) -> None:
    if not RVIZ_CONFIG.is_file():
        raise FileNotFoundError(f"RViz config does not exist: {RVIZ_CONFIG}")
    session.start(
        "rviz",
        _ros_shell(_rviz_command(use_sim_time=use_sim_time), enable_rviz=True),
        cwd=ROOT,
    )


def _command_exists(name: str) -> bool:
    from shutil import which
    return which(name) is not None


def _run(command: list[str], *, timeout: float = 5.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=timeout, check=False)


def _write_runtime(session: Session, **values: Any) -> None:
    path = session.directory / "runtime.json"
    current: dict[str, Any] = {}
    if path.exists():
        try:
            current = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            current = {}
    current.update(values)
    path.write_text(json.dumps(current, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _ros_params(session: Session, source: Path) -> Path:
    """Keep runner metadata out of the ROS parameter file passed to launch."""
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "fast_lio" not in value:
        raise ValueError(f"runtime ROS config is missing fast_lio: {source}")
    target = session.directory / "fast_lio_params.yaml"
    target.write_text(yaml.safe_dump({"fast_lio": value["fast_lio"]}, sort_keys=False), encoding="utf-8")
    return target


def _monitor_snapshot(session: Session) -> dict[str, Any]:
    try:
        return json.loads((session.directory / "monitor.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _stream_count(session: Session, name: str) -> int:
    return int(_monitor_snapshot(session).get("streams", {}).get(name, {}).get("received", 0))


def _wait_until(session: Session, predicate: Callable[[dict[str, Any]], bool], timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = _monitor_snapshot(session)
        if predicate(snapshot):
            return
        for record in session.records():
            if record.get("role") in {"monitor", "lio", "px4_gazebo", "bridge", "px4_ingress"}:
                # A monitor snapshot is the authoritative readiness signal, but
                # an early process exit must be reported immediately.
                if not Path(f"/proc/{record.get('pid')}").exists() and record.get("role") == "monitor":
                    raise RuntimeError("runtime monitor exited before readiness")
        time.sleep(0.25)
    raise TimeoutError(f"timed out waiting for {description}")


def _wait_process(process: subprocess.Popen[Any], timeout_s: float, description: str) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = process.poll()
        if result is not None:
            return int(result)
        time.sleep(0.25)
    raise TimeoutError(f"timed out waiting for {description}")


def _stop_and_report(session: Session, workflow: str, config_path: Path, *, px4_dir: Path | None = None, observation_complete: bool = False) -> dict[str, Any]:
    cleanup_failures = session.stop()
    if cleanup_failures:
        failures = _load_runtime_failures(session)
        _write_runtime(session, failures=failures + [f"cleanup: {item}" for item in cleanup_failures])
    session.mark_stopped("runner_cleanup")
    return report.build(session.directory, workflow, config_path, ROOT, px4_dir, observation_complete)


def _dataset_context(dataset: str) -> tuple[dict[str, Any], dict[str, int]]:
    sys.path.insert(0, str(ROOT / "tools"))
    import data
    home = data.data_home()
    context = data.dataset_context(dataset, home)
    return context, data.bag_topic_counts(context)


def run_dataset(dataset: str, rate: float, *, enable_rviz: bool = False) -> int:
    if not dataset:
        raise ValueError("DATASET is required")
    if rate <= 0:
        raise ValueError("RATE must be greater than zero")
    config = load_config("dataset.yaml")
    config["runtime"]["dataset"] = dataset
    config["runtime"]["replay_rate"] = rate
    session = Session.create(ARTIFACT_ROOT, "dataset")
    _write_runtime(
        session,
        workflow="dataset",
        dataset=dataset,
        rate=rate,
        rviz=enable_rviz,
        replay_tail_grace_s=float(config["runtime"]["thresholds"].get("replay_tail_grace_s", 0.5)),
        failures=[],
    )
    monitor_process: subprocess.Popen[Any] | None = None
    try:
        context, counts = _dataset_context(dataset)
        _write_runtime(session, dataset_context={"id": context["id"], "bag": str(context["bag"]), "counts": counts})
        ros_config = _ros_params(session, RUNTIME_CONFIG / "dataset.yaml")
        monitor_process = session.start(
            "monitor",
            [sys.executable, str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory), "--workflow", "dataset", "--config", str(RUNTIME_CONFIG / "dataset.yaml")],
            cwd=ROOT,
        )
        lio = session.start(
            "lio",
            _ros_shell([
                "ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
                f"config_file:={ros_config}",
                "use_sim_time:=false", "enable_external_odometry:=false",
                "publish_sensor_frames:=true", "livox_mount_xyz:=0 0 0",
                "livox_mount_rpy:=0 0 0",
            ], enable_rviz=enable_rviz),
            cwd=ROOT,
        )
        if enable_rviz:
            _start_rviz(session)
        timeout_s = float(config["runtime"]["timeouts"]["startup_s"])
        _wait_until(session, lambda snapshot: _stream_count(session, "diagnostics") > 0, timeout_s, "LIO diagnostics")
        replay = session.start(
            "replay",
            _ros_shell([
                "ros2", "bag", "play", str(context["bag"]), "--rate", str(rate),
                "--remap", f"{context['input']['lidar_topic']}:=/lidar/points",
                f"{context['input']['imu_topic']}:=/lidar/imu",
            ], enable_rviz=enable_rviz),
            cwd=ROOT,
        )
        replay_code = _wait_process(replay, float(config["runtime"]["timeouts"]["replay_s"]), "dataset replay")
        _write_runtime(session, replay_returncode=replay_code, replay_finished_wall_ns=time.time_ns())
        if replay_code != 0:
            raise RuntimeError(f"dataset replay exited with {replay_code}")
        _wait_until(
            session,
            lambda snapshot: _stream_count(session, "imu") >= int(counts[context["input"]["imu_topic"]] * float(config["runtime"]["thresholds"].get("minimum_rate_fraction", 0.90)))
            and _stream_count(session, "lidar") >= int(counts[context["input"]["lidar_topic"]] * float(config["runtime"]["thresholds"].get("minimum_rate_fraction", 0.90)))
            and _stream_count(session, "corrected_odometry") > 0
            and _stream_count(session, "propagated_odometry") > 0,
            float(config["runtime"]["timeouts"]["drain_s"]),
            "dataset outputs and queue drain",
        )
        _write_runtime(session, failures=[])
    except Exception as error:
        _write_runtime(session, failures=[str(error)])
    finally:
        result = _stop_and_report(session, "dataset", RUNTIME_CONFIG / "dataset.yaml")
    print(result["verdict"])
    print(session.directory)
    return 0 if result["verdict"] == "PASS" else 1


def _sim_prerequisites(px4_dir: Path) -> list[str]:
    missing: list[str] = []
    for command in ("ros2", "python3", "gz", "MicroXRCEAgent"):
        if not _command_exists(command):
            missing.append(f"missing command: {command}")
    for path in (
        px4_dir / "build/px4_sitl_default/bin/px4",
        px4_dir / "build/px4_sitl_default/rootfs/gz_env.sh",
        ROOT / "src/uav_simulation/models/x500_mid360/model.sdf",
        ROOT / "src/uav_simulation/models/lidar_mid360/model.sdf",
        ROOT / "src/uav_simulation/worlds/px4_lio_smoke.sdf",
        ROOT / "src/uav_simulation/bridge/px4_mid360_bridge.yaml",
        ROOT / "install/setup.bash",
    ):
        if not path.exists():
            missing.append(f"missing path: {path}")
    return missing


def _wait_gazebo(world: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    topic = f"/world/{world}/clock"
    while time.monotonic() < deadline:
        result = _run(["gz", "topic", "-l"], timeout=5.0)
        if result.returncode == 0 and topic in result.stdout.splitlines():
            return
        time.sleep(0.5)
    raise TimeoutError(f"Gazebo clock did not appear: {topic}")


def run_sim(headless: bool) -> int:
    config = load_config("sim.yaml")
    offboard_config = load_config("offboard.yaml")
    px4_dir = Path(os.environ.get("PX4_DIR", str(Path.home() / "Dev/Autopilot"))).expanduser().resolve()
    session = Session.create(ARTIFACT_ROOT, "sim-check" if headless else "sim")
    session.write_state({"workflow": "sim", "headless": headless, "px4_dir": str(px4_dir)})
    _write_runtime(
        session,
        workflow="sim",
        headless=headless,
        rviz=not headless,
        px4_dir=str(px4_dir),
        failures=[],
        startup_complete=False,
    )
    prereq = _sim_prerequisites(px4_dir)
    if prereq:
        _write_runtime(session, failures=prereq)
        result = _stop_and_report(session, "sim", RUNTIME_CONFIG / "sim.yaml", px4_dir=px4_dir)
        print(result["verdict"])
        print(session.directory)
        return 1
    try:
        ros_config = _ros_params(session, RUNTIME_CONFIG / "sim.yaml")
        monitor = session.start(
            "monitor",
            [sys.executable, str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory), "--workflow", "sim", "--config", str(RUNTIME_CONFIG / "sim.yaml")],
            cwd=ROOT,
        )
        session.start(
            "px4_gazebo",
            ["bash", str(ROOT / "tools/simulation/run_px4_mid360.sh")],
            cwd=ROOT,
            env={"PX4_DIR": str(px4_dir), "GZ_GUI": "0" if headless else "1", "SESSION_DIR": str(session.directory)},
        )
        simulation = config.get("simulation", {})
        if isinstance(simulation, dict) and "ros__parameters" in simulation:
            simulation = simulation["ros__parameters"]
        world = str(simulation["world"])
        _wait_gazebo(world, float(config["runtime"]["timeouts"]["startup_s"]))
        session.start("xrce_agent", ["MicroXRCEAgent", "udp4", "-p", "8888"], cwd=ROOT)
        bridge_config = ROOT / "src/uav_simulation/bridge/px4_mid360_bridge.yaml"
        session.start("bridge", _ros_shell([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
            "-r", "__node:=px4_mid360_bridge", "-p", f"config_file:={bridge_config}", "-p", "use_sim_time:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("px4_ingress", _ros_shell([
            "ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node", "--ros-args",
            "-p", "use_sim_time:=true", "-p", "simulation_clock:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("lio", _ros_shell([
            "ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
            f"config_file:={ros_config}", "use_sim_time:=true",
            "enable_external_odometry:=true", "publish_sensor_frames:=true",
            "livox_mount_xyz:=0 0 0.28", "livox_mount_rpy:=0 0 0",
        ], enable_rviz=not headless), cwd=ROOT)
        if not headless:
            _start_rviz(session, use_sim_time=True)
        _wait_until(session, lambda snapshot: _stream_count(session, "imu") > 0 and _stream_count(session, "lidar") > 0, float(config["runtime"]["timeouts"]["startup_s"]), "simulated sensor streams")
        _wait_until(session, lambda snapshot: str(snapshot.get("diagnostics", {}).get("state", "")).upper() == "TRACKING", float(config["runtime"]["timeouts"]["lio_tracking_s"]), "LIO TRACKING")
        _wait_until(session, lambda snapshot: _stream_count(session, "external_odometry") > 0, float(config["runtime"]["timeouts"]["external_odometry_s"]), "PX4 external odometry")
        _write_runtime(session, startup_complete=True)
        if headless:
            scenario = session.start(
                "offboard",
                [sys.executable, str(ROOT / "tools/runtime/offboard_scenario.py"), "--output", str(session.directory / "scenario.json"), "--config", str(RUNTIME_CONFIG / "offboard.yaml")],
                cwd=ROOT,
            )
            scenario_code = _wait_process(scenario, float(offboard_config["scenario"]["wall_timeout_s"]) + 10.0, "offboard scenario")
            scenario_payload = {}
            try:
                scenario_payload = json.loads((session.directory / "scenario.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                scenario_payload = {"failures": ["scenario did not create a report"]}
            if scenario_code != 0:
                scenario_payload.setdefault("failures", []).append(f"offboard scenario exited with {scenario_code}")
            (session.directory / "scenario.json").write_text(json.dumps(scenario_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            time.sleep(float(config["runtime"]["timeouts"]["post_flight_monitor_s"]))
        else:
            print(f"Session: {session.directory}")
            print("Topics: /lidar/imu /lidar/points /lio/odometry_corrected /lio/odometry_propagated /lio/diagnostics /fmu/in/vehicle_visual_odometry")
            print("Manual flight: arm, enter OFFBOARD only with your own controller, and use make status; stop with make stop.")
            while not session.state().get("stopped"):
                monitor_returncode = monitor.poll()
                if monitor_returncode is not None:
                    # make stop stops the monitor before the runner wakes up;
                    # session.stop() may need a few seconds to reap the rest
                    # of the process groups before publishing STOPPED.
                    deadline = time.monotonic() + 5.0
                    while not session.state().get("stopped") and time.monotonic() < deadline:
                        time.sleep(0.2)
                    if not session.state().get("stopped"):
                        raise RuntimeError(f"runtime monitor exited with {monitor_returncode}")
                    break
                time.sleep(0.5)
    except KeyboardInterrupt:
        _write_runtime(session, failures=["session interrupted"])
    except Exception as error:
        _write_runtime(session, failures=[str(error)])
    finally:
        result = _stop_and_report(session, "sim", RUNTIME_CONFIG / "sim.yaml", px4_dir=px4_dir, observation_complete=not headless and not _load_runtime_failures(session))
    print(result["verdict"])
    print(session.directory)
    return 0 if result["verdict"] in {"PASS", "OBSERVATION_COMPLETE"} else 1


def _load_runtime_failures(session: Session) -> list[str]:
    try:
        value = json.loads((session.directory / "runtime.json").read_text(encoding="utf-8"))
        return [str(item) for item in value.get("failures", [])]
    except (OSError, ValueError):
        return []


def status() -> int:
    session_path = resolve_latest(ARTIFACT_ROOT)
    session = Session.from_path(session_path)
    snapshot = _monitor_snapshot(session)
    latest = snapshot.get("latest", {})
    diagnostics = snapshot.get("diagnostics", {})
    state = session.state()
    print(f"Session: {session_path}")
    print(f"PX4: {'running' if any(r.get('role') == 'px4_gazebo' for r in session.live_records()) else 'stopped'}")
    print(f"Gazebo: {'running' if any(r.get('role') == 'px4_gazebo' for r in session.live_records()) else 'stopped'}")
    for label, name in (("LiDAR rate", "lidar"), ("IMU rate", "imu"), ("Corrected odometry rate", "corrected_odometry"), ("Propagated odometry rate", "propagated_odometry"), ("External odometry rate", "external_odometry")):
        row = snapshot.get("streams", {}).get(name, {})
        print(f"{label}: {_number(row.get('mean_rate_hz')):.2f} Hz")
    print(f"LIO state: {diagnostics.get('state', 'NOT_AVAILABLE')}")
    print(f"PX4 estimator state: {latest.get('estimator_status_flags', {})}")
    print(f"External vision fusion state: {latest.get('estimator_status_flags', {}).get('cs_ev_pos', 'NOT_AVAILABLE')}")
    print(f"Offboard state: {latest.get('vehicle_status', {}).get('nav_state', 'NOT_AVAILABLE')}")
    print(f"Drop count: {diagnostics.get('values', {}).get('imu_drop_count', 0)}/{diagnostics.get('values', {}).get('lidar_drop_count', 0)}")
    print(f"Queue depth: {diagnostics.get('values', {}).get('current_queue_depth', diagnostics.get('values', {}).get('current_input_queue_depth', 'NOT_AVAILABLE'))}")
    print(f"Last failure: {diagnostics.get('last_failure_reason', 'NONE')}")
    return 0


def stop() -> int:
    session_path = resolve_latest(ARTIFACT_ROOT)
    session = Session.from_path(session_path)
    workflow = str(session.state().get("workflow", "sim"))
    config_path = RUNTIME_CONFIG / ("dataset.yaml" if workflow == "dataset" else "sim.yaml")
    failures = session.stop()
    if failures:
        _write_runtime(session, failures=[f"cleanup: {item}" for item in failures])
    session.mark_stopped("make stop")
    px4_dir = Path(session.state().get("px4_dir", "")) if session.state().get("px4_dir") else None
    result = report.build(session.directory, workflow, config_path, ROOT, px4_dir, observation_complete=workflow == "sim" and not session.state().get("headless", False))
    print(result["verdict"])
    print(session_path)
    return 0 if result["verdict"] in {"OBSERVATION_COMPLETE", "PASS"} and not failures else 1


def clean() -> int:
    root = ARTIFACT_ROOT.resolve()
    if root == Path("/") or root != ROOT / ".artifacts/runtime":
        raise ValueError(f"refusing unsafe clean root: {root}")
    if root.exists():
        import shutil
        shutil.rmtree(root)
    return 0


def _number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    dataset = sub.add_parser("dataset-check")
    dataset.add_argument("--dataset", required=True)
    dataset.add_argument("--rate", type=float, required=True)
    dataset.add_argument("--rviz", action="store_true", help="launch RViz for this replay")
    sub.add_parser("sim-check")
    sub.add_parser("sim")
    sub.add_parser("status")
    sub.add_parser("stop")
    sub.add_parser("clean")
    args = parser.parse_args()
    enable_rviz = args.command == "sim" or (args.command == "dataset-check" and args.rviz)
    os.environ.update(RVIZ_ENV if enable_rviz else NO_RVIZ_ENV)
    if args.command == "dataset-check":
        return run_dataset(args.dataset, args.rate, enable_rviz=args.rviz)
    if args.command == "sim-check":
        return run_sim(True)
    if args.command == "sim":
        return run_sim(False)
    if args.command == "status":
        return status()
    if args.command == "stop":
        return stop()
    return clean()


if __name__ == "__main__":
    raise SystemExit(main())

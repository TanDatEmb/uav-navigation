#!/usr/bin/env python3
"""Single entrypoint for dataset, headless SITL and interactive sessions."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
from shutil import which
import sys
import time
from typing import Any, Callable

import yaml

from process_group import Session, resolve_latest
import report


ROOT = Path(__file__).resolve().parents[2]
RUNTIME_CONFIG = ROOT / "config/runtime"
ARTIFACT_ROOT = ROOT / ".artifacts/runtime"
RVIZ_CONFIG = ROOT / "src/navigation_bringup/rviz/fast_lio.rviz"
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

ROS_PARAMETER_NODES = (
    "fast_lio",
    "px4_external_odometry_bridge",
    "px4_odometry_bridge",
)

GUI_ENV_REMOVE = {
    "GTK_MODULES",
    "GTK_PATH",
    "GIO_MODULE_DIR",
}
PX4_HEADLESS_COM_RC_IN_MODE = "4"
PX4_INTERACTIVE_COM_RC_IN_MODE = "1"

GENERATED_CLEAN_PATHS = (
    ROOT / ".artifacts",
    ROOT / ".pytest_cache",
    ROOT / "build",
    ROOT / "build-gprof",
    ROOT / "install",
    ROOT / "install-gprof",
    ROOT / "log",
    ROOT / "log-gprof",
    ROOT / "src/mapping/rog_map_vendor/log",
    ROOT / "symlink_install_manifest.txt",
)


def _filtered_xdg_data_dirs(value: str | None) -> str | None:
    if not value:
        return None
    kept = []
    for entry in value.split(":"):
        if not entry:
            continue
        if "/snap/" in entry or entry.endswith("/var/lib/snapd/desktop") or entry == "/var/lib/snapd/desktop":
            continue
        kept.append(entry)
    return ":".join(kept) if kept else None


def _gui_environment(base: dict[str, str], *, rviz: bool = False) -> dict[str, str]:
    environment = dict(base)
    for key in GUI_ENV_REMOVE:
        environment.pop(key, None)
    filtered_data_dirs = _filtered_xdg_data_dirs(environment.get("XDG_DATA_DIRS"))
    if filtered_data_dirs is None:
        environment.pop("XDG_DATA_DIRS", None)
    else:
        environment["XDG_DATA_DIRS"] = filtered_data_dirs
    if rviz:
        environment.update(RVIZ_ENV)
    return environment


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
    environment = _gui_environment(RVIZ_ENV if enable_rviz else NO_RVIZ_ENV, rviz=enable_rviz)
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
    return command


def _start_rviz(session: Session, *, use_sim_time: bool = False) -> None:
    if not RVIZ_CONFIG.is_file():
        raise FileNotFoundError(f"RViz config does not exist: {RVIZ_CONFIG}")
    session.start(
        "rviz",
        _ros_shell(_rviz_command(use_sim_time=use_sim_time), enable_rviz=True),
        cwd=ROOT,
        env=_gui_environment(os.environ, rviz=True),
        env_remove=GUI_ENV_REMOVE,
    )


def _command_exists(name: str) -> bool:
    return which(name) is not None


def _executable_candidates(name: str) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        if not directory:
            continue
        candidate = Path(directory) / name
        try:
            resolved = str(candidate.resolve())
        except OSError:
            continue
        if resolved in seen:
            continue
        if candidate.is_file() and os.access(candidate, os.X_OK):
            seen.add(resolved)
            result.append(str(candidate))
    return result


def _detect_gz_command() -> str | None:
    override = os.environ.get("GZ_COMMAND")
    candidates = [override] if override else []
    candidates.extend(_executable_candidates("gz"))
    if not candidates:
        first = which("gz")
        if first:
            candidates.append(first)
    checked: set[str] = set()
    for candidate in candidates:
        if not candidate:
            continue
        key = str(candidate)
        if key in checked:
            continue
        checked.add(key)
        try:
            probe = subprocess.run(
                [candidate, "sim", "--versions"],
                cwd=ROOT,
                env=_gz_runtime_env(),
                text=True,
                capture_output=True,
                timeout=5.0,
                check=False,
            )
        except OSError:
            continue
        if probe.returncode == 0:
            return candidate
    return None


def _run(command: list[str], *, timeout: float = 5.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=timeout, check=False)


def _gz_runtime_env() -> dict[str, str]:
    environment = os.environ.copy()
    # ROS's gz vendor config can shadow Gazebo Sim and hide `gz sim`.
    environment.pop("GZ_CONFIG_PATH", None)
    return environment


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
    """Write only explicit ROS node parameter blocks, excluding runner metadata."""
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "fast_lio" not in value:
        raise ValueError(f"runtime ROS config is missing fast_lio: {source}")
    node_parameters = {
        name: value[name]
        for name in ROS_PARAMETER_NODES
        if name in value
    }
    target = session.directory / "fast_lio_params.yaml"
    target.write_text(yaml.safe_dump(node_parameters, sort_keys=False), encoding="utf-8")
    return target


def _mapping_params(
    session: Session,
    source: Path,
    *,
    interactive: bool = False,
    frontier_debug: bool = False,
    simulation: bool = False,
) -> Path:
    """Copy the product-owned navigation_mapping parameter block."""
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "navigation_runtime" not in value:
        raise ValueError(f"runtime config is missing navigation_runtime: {source}")
    parameters = value["navigation_runtime"]
    ros_parameters = parameters.setdefault("ros__parameters", {})
    mapping = ros_parameters.setdefault("mapping", {})
    navigation = ros_parameters.setdefault("navigation", {})
    input_parameters = mapping.setdefault("input", {})
    map_parameters = mapping.setdefault("map", {})
    visualization_parameters = mapping.setdefault("visualization", {})
    resolution_override = os.environ.get("MAPPING_RESOLUTION_M")
    input_voxel_override = os.environ.get("MAPPING_INPUT_VOXEL_M")
    if resolution_override is not None:
        map_parameters["resolution_m"] = float(resolution_override)
    if input_voxel_override is not None:
        input_parameters["voxel_size_m"] = float(input_voxel_override)
    # Keep one authoritative mapping YAML. Runtime workflows select whether
    # product-side visualization is part of this invocation. Frontier
    # extraction remains an explicit future/debug capability and is not
    # coupled to opening RViz.
    visualization_parameters["enabled"] = interactive
    visualization_parameters["publish_unknown"] = interactive
    visualization_parameters["publish_frontier"] = frontier_debug
    if simulation:
        simulation_parameters = load_config("sim.yaml").get("navigation", {})
        navigation["collision"] = simulation_parameters.get("collision", {})
    target = session.directory / "navigation_mapping_params.yaml"
    target.write_text(
        yaml.safe_dump({"navigation_runtime": parameters}, sort_keys=False),
        encoding="utf-8",
    )
    _write_runtime(
        session,
        mapping_rog_resolution_m=map_parameters.get("resolution_m"),
        mapping_input_voxel_m=input_parameters.get("voxel_size_m"),
        mapping_interactive=interactive,
    )
    return target


def _mapping_ready(snapshot: dict[str, Any]) -> bool:
    """Require an accepted ROG observation and at least one visualization tick."""
    latest = snapshot.get("latest", {}).get("mapping_diagnostics", {})
    for status in latest.get("statuses", []):
        if not str(status.get("name", "")).endswith("/world_model"):
            continue
        values = status.get("values", {})
        try:
            return (
                int(values.get("accepted_observation_count", 0)) > 0
                and (int(values.get("visualization_publish_count", 0)) > 0
                     or int(values.get("visualization_subscriber_count", 0)) == 0)
                and int(values.get("visualization_exception_count", 0)) == 0
            )
        except (TypeError, ValueError):
            return False
    return False


def _px4_manual_control_mode(headless: bool) -> str:
    return PX4_HEADLESS_COM_RC_IN_MODE if headless else PX4_INTERACTIVE_COM_RC_IN_MODE


def _lidar_to_imu_launch_arguments(config: dict[str, Any]) -> tuple[str, str]:
    """Invert the estimator's ^I T_L extrinsic for the static ^L T_I TF."""
    extrinsic = config["fast_lio"]["ros__parameters"]["extrinsic"]
    translation = tuple(float(value) for value in extrinsic["translation_imu_lidar"])
    quaternion = tuple(float(value) for value in extrinsic["rotation_imu_lidar_xyzw"])
    if len(translation) != 3 or len(quaternion) != 4:
        raise ValueError("estimator extrinsic must contain xyz translation and xyzw rotation")
    if not all(math.isfinite(value) for value in (*translation, *quaternion)):
        raise ValueError("estimator extrinsic must be finite")
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-6:
        raise ValueError("estimator extrinsic quaternion is invalid")
    x, y, z, w = (value / norm for value in quaternion)

    # R_LI = R_IL^T and t_LI = -R_IL^T t_IL.
    rotation_lidar_imu = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)),
        (2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)),
        (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)),
    )
    translation_lidar_imu = tuple(
        -sum(rotation_lidar_imu[row][column] * translation[column] for column in range(3))
        for row in range(3)
    )
    pitch = math.asin(max(-1.0, min(1.0, -rotation_lidar_imu[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation_lidar_imu[2][1], rotation_lidar_imu[2][2])
        yaw = math.atan2(rotation_lidar_imu[1][0], rotation_lidar_imu[0][0])
    else:
        roll = math.atan2(-rotation_lidar_imu[1][2], rotation_lidar_imu[1][1])
        yaw = 0.0
    xyz = " ".join(format(value, ".17g") for value in translation_lidar_imu)
    rpy = " ".join(format(value, ".17g") for value in (roll, pitch, yaw))
    return xyz, rpy


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
    # Bound the measured interval before processes are stopped.  A monitor
    # timer can otherwise report a final stale event after its publishers have
    # intentionally begun shutting down.
    _write_runtime(session, observation_finished_wall_ns=time.time_ns())
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


def run_dataset(
    dataset: str,
    rate: float,
    *,
    enable_rviz: bool = False,
    frontier_debug: bool = False,
) -> int:
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
        mapping_config = _mapping_params(
            session,
            RUNTIME_CONFIG / "mapping.yaml",
            interactive=enable_rviz,
            frontier_debug=frontier_debug,
        )
        lidar_to_imu_xyz, lidar_to_imu_rpy = _lidar_to_imu_launch_arguments(config)
        monitor_process = session.start(
            "monitor",
            [sys.executable, str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory), "--workflow", "dataset", "--config", str(RUNTIME_CONFIG / "dataset.yaml")],
            cwd=ROOT,
        )
        session.start(
            "mapping",
            _ros_shell([
                "ros2", "launch", "navigation_bringup", "navigation_runtime.launch.py",
                f"config_file:={mapping_config}", "use_sim_time:=false",
            ], enable_rviz=enable_rviz),
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
                f"livox_lidar_to_imu_xyz:={lidar_to_imu_xyz}",
                f"livox_lidar_to_imu_rpy:={lidar_to_imu_rpy}",
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
        _wait_until(session, _mapping_ready,
                    float(config["runtime"]["timeouts"]["drain_s"]),
                    "ROG-Map output and visualization")
        _write_runtime(session, failures=[])
    except Exception as error:
        _write_runtime(session, failures=[str(error)])
    finally:
        result = _stop_and_report(session, "dataset", RUNTIME_CONFIG / "dataset.yaml")
    print(result["verdict"])
    print(session.directory)
    return 0 if result["verdict"] == "PASS" else 1


def _sim_prerequisites(px4_dir: Path, gz_command: str | None) -> list[str]:
    missing: list[str] = []
    for command in ("ros2", "python3", "MicroXRCEAgent"):
        if not _command_exists(command):
            missing.append(f"missing command: {command}")
    if not gz_command:
        missing.append("missing Gazebo simulator command: no 'gz' binary with 'sim --versions' support")
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


def _wait_gazebo(world: str, timeout_s: float, gz_command: str) -> None:
    deadline = time.monotonic() + timeout_s
    topic = f"/world/{world}/clock"
    env = _gz_runtime_env()
    while time.monotonic() < deadline:
        result = subprocess.run(
            [gz_command, "topic", "-l"],
            cwd=ROOT,
            env=env,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        if result.returncode == 0 and topic in result.stdout.splitlines():
            return
        time.sleep(0.5)
    raise TimeoutError(f"Gazebo clock did not appear: {topic}")


def run_sim(headless: bool) -> int:
    config = load_config("sim.yaml")
    offboard_config = load_config("offboard.yaml")
    px4_dir = Path(os.environ.get("PX4_DIR", str(Path.home() / "Dev/Autopilot"))).expanduser().resolve()
    gz_command = _detect_gz_command()
    session = Session.create(ARTIFACT_ROOT, "sim-check" if headless else "sim")
    session.write_state({"workflow": "sim", "headless": headless, "px4_dir": str(px4_dir)})
    _write_runtime(
        session,
        workflow="sim",
        headless=headless,
        rviz=not headless,
        px4_dir=str(px4_dir),
        gz_command=gz_command,
        failures=[],
        startup_complete=False,
    )
    prereq = _sim_prerequisites(px4_dir, gz_command)
    if prereq:
        _write_runtime(session, failures=prereq)
        result = _stop_and_report(session, "sim", RUNTIME_CONFIG / "sim.yaml", px4_dir=px4_dir)
        print(result["verdict"])
        print(session.directory)
        return 1
    try:
        ros_config = _ros_params(session, RUNTIME_CONFIG / "sim.yaml")
        mapping_config = _mapping_params(
        session, RUNTIME_CONFIG / "mapping.yaml", interactive=not headless, simulation=True
        )
        lidar_to_imu_xyz, lidar_to_imu_rpy = _lidar_to_imu_launch_arguments(config)
        monitor = session.start(
            "monitor",
            [sys.executable, str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory), "--workflow", "sim", "--config", str(RUNTIME_CONFIG / "sim.yaml")],
            cwd=ROOT,
        )
        session.start(
            "px4_gazebo",
            ["bash", str(ROOT / "tools/simulation/run_px4_mid360.sh")],
            cwd=ROOT,
            env=_gui_environment({
                **os.environ,
                "PX4_DIR": str(px4_dir),
                "GZ_GUI": "0" if headless else "1",
                "SESSION_DIR": str(session.directory),
                "GZ_COMMAND": gz_command or "",
                "PX4_PARAM_COM_RC_IN_MODE": _px4_manual_control_mode(headless),
            }),
            env_remove=GUI_ENV_REMOVE,
        )
        simulation = config.get("simulation", {})
        if isinstance(simulation, dict) and "ros__parameters" in simulation:
            simulation = simulation["ros__parameters"]
        world = str(simulation["world"])
        if not gz_command:
            raise RuntimeError("Gazebo simulator command is unavailable")
        _wait_gazebo(world, float(config["runtime"]["timeouts"]["startup_s"]), gz_command)
        session.start("xrce_agent", ["MicroXRCEAgent", "udp4", "-p", "8888"], cwd=ROOT)
        bridge_config = ROOT / "src/uav_simulation/bridge/px4_mid360_bridge.yaml"
        session.start("bridge", _ros_shell([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
            "-r", "__node:=px4_mid360_bridge", "-p", f"config_file:={bridge_config}", "-p", "use_sim_time:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("px4_ingress", _ros_shell([
            "ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node", "--ros-args",
            "--params-file", str(ros_config), "-p", "use_sim_time:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("mapping", _ros_shell([
            "ros2", "launch", "navigation_bringup", "navigation_runtime.launch.py",
            f"config_file:={mapping_config}", "use_sim_time:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("lio", _ros_shell([
            "ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
            f"config_file:={ros_config}", "use_sim_time:=true",
            "enable_external_odometry:=true", "publish_sensor_frames:=true",
            "livox_mount_xyz:=0 0 0.28", "livox_mount_rpy:=0 0 0",
            f"livox_lidar_to_imu_xyz:={lidar_to_imu_xyz}",
            f"livox_lidar_to_imu_rpy:={lidar_to_imu_rpy}",
        ], enable_rviz=not headless), cwd=ROOT)
        if not headless:
            _start_rviz(session, use_sim_time=True)
        _wait_until(session, lambda snapshot: _stream_count(session, "imu") > 0 and _stream_count(session, "lidar") > 0, float(config["runtime"]["timeouts"]["startup_s"]), "simulated sensor streams")
        _wait_until(session, lambda snapshot: str(snapshot.get("diagnostics", {}).get("state", "")).upper() == "TRACKING", float(config["runtime"]["timeouts"]["lio_tracking_s"]), "LIO TRACKING")
        _wait_until(session, lambda snapshot: _stream_count(session, "external_odometry") > 0, float(config["runtime"]["timeouts"]["external_odometry_s"]), "PX4 external odometry")
        _wait_until(session, _mapping_ready,
                    float(config["runtime"]["timeouts"]["external_odometry_s"]),
                    "ROG-Map output and visualization")
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
        _write_runtime(session, failures=[])
        session.mark_stopped("user interrupt")
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
    print(f"PX4 EV position control flag: {latest.get('estimator_status_flags', {}).get('cs_ev_pos', 'NOT_AVAILABLE')}")
    print(f"Offboard state: {latest.get('vehicle_status', {}).get('nav_state', 'NOT_AVAILABLE')}")
    print(f"Drop count: {diagnostics.get('values', {}).get('imu_drop_count', 0)}/{diagnostics.get('values', {}).get('lidar_drop_count', 0)}")
    print(f"Queue depth: {diagnostics.get('values', {}).get('current_queue_depth', diagnostics.get('values', {}).get('current_input_queue_depth', 'NOT_AVAILABLE'))}")
    print(f"Last failure: {diagnostics.get('last_failure_reason', 'NONE')}")
    return 0


def _runtime_session_paths(root: Path) -> list[Path]:
    """Return all workspace-owned runtime sessions, newest first."""
    if not root.is_dir():
        return []
    return sorted(
        (
            path
            for path in root.iterdir()
            if path.is_dir()
            and not path.is_symlink()
            and (path / "processes.json").is_file()
        ),
        key=lambda path: path.name,
        reverse=True,
    )


def stop() -> int:
    session_path = resolve_latest(ARTIFACT_ROOT)
    session_paths = _runtime_session_paths(ARTIFACT_ROOT)
    if session_path not in session_paths:
        session_paths.append(session_path)
    cleanup_failures: list[str] = []
    for path in session_paths:
        session = Session.from_path(path)
        failures = session.stop()
        if failures:
            existing = _load_runtime_failures(session)
            _write_runtime(
                session,
                failures=existing + [f"cleanup: {item}" for item in failures],
            )
            cleanup_failures.extend(f"{path.name}: {item}" for item in failures)
        session.mark_stopped("make stop")

    session = Session.from_path(session_path)
    workflow = str(session.state().get("workflow", "sim"))
    config_path = RUNTIME_CONFIG / ("dataset.yaml" if workflow == "dataset" else "sim.yaml")
    px4_dir = Path(session.state().get("px4_dir", "")) if session.state().get("px4_dir") else None
    result = report.build(
        session.directory,
        workflow,
        config_path,
        ROOT,
        px4_dir,
        observation_complete=workflow == "sim" and not session.state().get("headless", False),
    )
    if cleanup_failures:
        print("FAIL")
        for failure in cleanup_failures:
            print(f"cleanup: {failure}")
    else:
        print("STOPPED")
        if result["verdict"] not in {"OBSERVATION_COMPLETE", "PASS"}:
            print(f"Runtime report: {result['verdict']} (cleanup succeeded)")
    print(session_path)
    return 0 if not cleanup_failures else 1


def clean() -> int:
    import shutil

    removed: list[Path] = []
    for path in GENERATED_CLEAN_PATHS:
        resolved = path.resolve()
        if resolved == Path("/") or resolved == ROOT.resolve():
            raise ValueError(f"refusing unsafe clean path: {resolved}")
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
            removed.append(path)
        elif path.is_file() or path.is_symlink():
            path.unlink()
            removed.append(path)

    for cache_root in (ROOT / "src", ROOT / "tools"):
        if not cache_root.is_dir():
            continue
        for path in cache_root.rglob("__pycache__"):
            if path.is_dir() and not path.is_symlink():
                shutil.rmtree(path)
                removed.append(path)

    for path in (ROOT / ".vscode").glob("browse.vc.db*"):
        if path.is_file() or path.is_symlink():
            path.unlink()
            removed.append(path)

    for path in removed:
        print(f"removed {path.relative_to(ROOT)}")
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
    dataset.add_argument(
        "--frontier-debug",
        action="store_true",
        help="enable ROG frontier extraction/publication for RViz debugging",
    )
    sub.add_parser("sim-check")
    sub.add_parser("sim")
    sub.add_parser("status")
    sub.add_parser("stop")
    sub.add_parser("clean")
    args = parser.parse_args()
    enable_rviz = args.command == "sim" or (args.command == "dataset-check" and args.rviz)
    os.environ.update(RVIZ_ENV if enable_rviz else NO_RVIZ_ENV)
    if args.command == "dataset-check":
        return run_dataset(
            args.dataset,
            args.rate,
            enable_rviz=args.rviz,
            frontier_debug=args.frontier_debug,
        )
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

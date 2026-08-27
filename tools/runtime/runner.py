#!/usr/bin/env python3
"""Single entrypoint for dataset, headless SITL and interactive sessions."""

from __future__ import annotations

import argparse
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
from shutil import which
import sys
import time
from functools import wraps
from typing import Any, Callable
import xml.etree.ElementTree as ET

import yaml

from process_group import Session, resolve_latest, update_latest
import report
from build_provenance import validate_manifest
from runtime_environment import (
    BuildRuntimeBusyError,
    BuildRuntimeLock,
    CANONICAL_PYTHON,
    canonical_python_error,
    require_canonical_python,
)


ROOT = Path(__file__).resolve().parents[2]
RUNTIME_CONFIG = ROOT / "config/runtime"
DATASET_SHADOW_MISSION = RUNTIME_CONFIG / "missions/recorded_replay.yaml"


def _shared_artifact_root(root: Path = ROOT) -> Path:
    """Return one runtime artifact root shared by every Git worktree."""
    override = os.environ.get("UAV_NAV_ARTIFACT_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "--path-format=absolute", "--git-common-dir"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        common_dir = Path(result.stdout.strip()).resolve()
        if result.returncode == 0 and common_dir.name == ".git":
            return common_dir.parent / ".artifacts/runtime"
    except (OSError, subprocess.SubprocessError):
        pass
    return root / ".artifacts/runtime"


ARTIFACT_ROOT = _shared_artifact_root()
RUNTIME_LOCK_PATH = ARTIFACT_ROOT / ".runtime-sim.lock"
DATASET_RUNTIME_LOCK_PATH = ARTIFACT_ROOT / ".runtime-dataset.lock"
BUILD_RUNTIME_LOCK_PATH = ARTIFACT_ROOT / ".build-runtime.lock"
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

CANONICAL_SCENES = (
    "sanity_open", "structured_obstacle", "long_route", "tunnel", "clutter",
    "planner_negative",
)
TEST_CASES = ("positive", "degenerate", "detour", "no_path")
MOTION_PRESETS = ("nominal", "slow", "fast")

# Keep the complete SITL stack off the default DDS domain and off the PX4
# default XRCE port.  A physical vehicle (or another developer's SITL) on the
# same LAN must not be able to discover /fmu topics or external-mode
# registration requests from this test. Both values remain overridable for
# deliberate externally-managed isolation; the runner allows one workspace-owned
# SITL and one isolated dataset replay at a time.
DEFAULT_ROS_DOMAIN_ID = 42
DEFAULT_DATASET_ROS_DOMAIN_ID = 43
DEFAULT_XRCE_PORT = 8892


class RuntimeBusyError(RuntimeError):
    """Raised when a second workspace runtime would collide with a live one."""


def _active_runtime_sessions(
    root: Path = ARTIFACT_ROOT,
    workflows: set[str] | None = None,
) -> list[str]:
    """Return live workspace-owned sessions, including orphaned children.

    The lock protects normal concurrent runners.  The artifact registry is a
    second line of defence for the failure mode where a runner is SIGKILLed
    but its separately-created child process groups survive.
    """
    if not root.is_dir():
        return []
    active: list[str] = []
    for path in sorted(root.iterdir(), reverse=True):
        if not path.is_dir() or path.is_symlink() or not (path / "processes.json").is_file():
            continue
        try:
            session = Session.from_path(path)
            if workflows is not None and str(session.state().get("workflow", "")) not in workflows:
                continue
            live_roles = sorted({str(record.get("role", "unknown")) for record in session.live_records()})
        except (OSError, ValueError, KeyError, TypeError):
            continue
        if live_roles:
            active.append(f"{path.name} (roles: {', '.join(live_roles)})")
    return active


class RuntimeLock:
    """Hold an advisory repository-wide lock for the full simulation run."""

    def __init__(
        self,
        path: Path | None = None,
        *,
        artifact_root: Path | None = None,
        active_workflows: set[str] | None = None,
    ) -> None:
        self.path = (path or RUNTIME_LOCK_PATH).resolve()
        self.artifact_root = (
            artifact_root
            or (ARTIFACT_ROOT if path is None else self.path.parent)
        ).resolve()
        self.active_workflows = active_workflows
        self._file: Any | None = None

    def _owner(self) -> str:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "owner metadata unavailable"
        if not isinstance(payload, dict):
            return "owner metadata unavailable"
        pid = payload.get("pid", "unknown")
        command = payload.get("command", "unknown command")
        return f"pid={pid}, command={command}"

    def __enter__(self) -> "RuntimeLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            if error.errno not in {errno.EACCES, errno.EAGAIN}:
                self._file.close()
                self._file = None
                raise
            owner = self._owner()
            self._file.close()
            self._file = None
            raise RuntimeBusyError(
                f"another workspace runtime already owns {self.path} ({owner}); "
                "run `make stop` and wait for cleanup before starting another simulation"
            ) from error

        active = _active_runtime_sessions(self.artifact_root, self.active_workflows)
        if active:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
            self._file = None
            raise RuntimeBusyError(
                "workspace runtime process groups are still active: "
                + "; ".join(active)
                + "; run `make stop` before starting another simulation"
            )

        payload = {
            "pid": os.getpid(),
            "command": shlex.join(sys.argv),
            "started_at": time.time(),
            "lock_path": str(self.path),
        }
        self._file.seek(0)
        self._file.truncate()
        self._file.write(json.dumps(payload, sort_keys=True) + "\n")
        self._file.flush()
        os.fsync(self._file.fileno())
        return self

    def __exit__(self, _exception_type: Any, _exception: Any, _traceback: Any) -> None:
        if self._file is None:
            return
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        finally:
            self._file.close()
            self._file = None


class _RunnerSignalGuard:
    """Convert termination signals into the runner's cleanup path."""

    def __init__(self) -> None:
        self._previous: dict[int, Any] = {}

    def _handle(self, signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt(f"received signal {signum}")

    def __enter__(self) -> "_RunnerSignalGuard":
        for signum in (signal.SIGINT, signal.SIGTERM):
            self._previous[signum] = signal.getsignal(signum)
            signal.signal(signum, self._handle)
        return self

    def __exit__(self, _exception_type: Any, _exception: Any, _traceback: Any) -> None:
        for signum, handler in self._previous.items():
            signal.signal(signum, handler)


def _assert_xrce_port_available(port: int) -> None:
    """Fail before spawning Gazebo if the dedicated XRCE UDP port is busy."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError as error:
        raise RuntimeBusyError(
            f"MicroXRCEAgent UDP port {port} is already in use; "
            "run `make stop`, or choose UAV_NAV_XRCE_PORT only for an explicitly isolated setup"
        ) from error
    finally:
        probe.close()

DISPOSABLE_BUILD_VARIANT_SUFFIXES = (
    "gprof",
    "profile",
    "debug",
    "asan",
    "ubsan",
    "tsan",
)

# build/ and install/ are the canonical Release outputs used by `make build`.
# Keep them for incremental use; sanitizer/profiling variants are disposable
# and are cleaned together with their logs.
GENERATED_CLEAN_PATHS = (
    ARTIFACT_ROOT.parent,
    ROOT / ".pytest_cache",
    ROOT / "log",
    ROOT / "src/mapping/rog_map_vendor/log",
    ROOT / "symlink_install_manifest.txt",
) + tuple(
    path
    for suffix in DISPOSABLE_BUILD_VARIANT_SUFFIXES
    for path in (
        ROOT / f"build-{suffix}",
        ROOT / f"install-{suffix}",
        ROOT / f"log-{suffix}",
    )
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
    # Keep ROS/Gazebo logging inside the session artifact directory.  Apart
    # from making each run self-contained, this avoids writing to ~/.ros or
    # ~/.gz when the runner is executed in a read-only/containerized home.
    for key in (
        "ROS_DOMAIN_ID", "PX4_UXRCE_DDS_PORT", "PX4_UXRCE_DDS_NS",
        "ROS_LOG_DIR", "RCUTILS_LOGGING_DIRECTORY", "GZ_LOG_DIR",
    ):
        value = os.environ.get(key)
        if value:
            environment[key] = value
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


def _capture_build_provenance(session: Session, px4_dir: Path | None = None) -> dict[str, Any]:
    evidence = validate_manifest(ROOT, ROOT / "install")
    if px4_dir is not None:
        px4_head = subprocess.run(
            ["git", "-C", str(px4_dir), "rev-parse", "HEAD"],
            text=True, capture_output=True, check=False,
        )
        px4_status = subprocess.run(
            ["git", "-C", str(px4_dir), "status", "--porcelain"],
            text=True, capture_output=True, check=False,
        )
        evidence["external_px4"] = {
            "path": str(px4_dir.resolve()),
            "git_head": px4_head.stdout.strip() if px4_head.returncode == 0 else "",
            "git_dirty": bool(px4_status.stdout.strip()) if px4_status.returncode == 0 else None,
        }
    _write_runtime(session, build_provenance=evidence)
    return evidence


def _resolve_isolation_value(value: int | None, env_name: str, default: int, *, low: int, high: int) -> int:
    """Resolve and validate a process-wide isolation setting.

    The runner intentionally chooses a non-default ROS domain.  This is more
    reliable than relying on a namespace because PX4's current ROS contract
    uses absolute ``/fmu`` topic names.  A unique XRCE UDP port additionally
    prevents two local SITL instances from sharing an agent.
    """
    raw = value if value is not None else os.environ.get(env_name, str(default))
    try:
        resolved = int(raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{env_name} must be an integer") from error
    if not low <= resolved <= high:
        raise ValueError(f"{env_name} must be in [{low}, {high}], got {resolved}")
    return resolved


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


def _mission_planning(source: Path | None) -> dict[str, Any]:
    if source is None:
        return {}
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    mission = value.get("mission", {}) if isinstance(value, dict) else {}
    planning = mission.get("planning", {}) if isinstance(mission, dict) else {}
    if not isinstance(planning, dict):
        raise ValueError("mission.planning must be a mapping")
    allowed_keys = {
        "max_velocity_mps", "max_acceleration_mps2",
        "max_jerk_mps3", "unknown_policy",
    }
    unknown_keys = sorted(set(planning) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"unsupported mission planning fields: {', '.join(unknown_keys)}")
    unknown_policy = planning.get("unknown_policy", "blocked")
    if unknown_policy not in {"blocked", "allow_unknown"}:
        raise ValueError(
            "mission planning unknown_policy must be 'blocked' or 'allow_unknown'"
        )
    result = {}
    result["unknown_policy"] = unknown_policy
    for key in ("max_velocity_mps", "max_acceleration_mps2",
                "max_jerk_mps3"):
        if key in planning:
            number = float(planning[key])
            if not math.isfinite(number) or number <= 0.0:
                raise ValueError(f"mission planning {key} must be finite and positive")
            result[key] = number
    return result


def _resolved_mission_file(
    session: Session,
    source: Path,
    speed_cap_mps: float | None,
) -> Path:
    """Materialize the exact mission contract consumed by every SITL process.

    A speed-cap request is an experiment input, but it must not live only in
    the generated planner backend YAML. Both planner backend and the PX4 External Mode node load
    mission dynamics independently, so passing the original mission alongside
    an overridden planner file silently gave them different velocity
    contracts. The session-owned copy is the single resolved input for the
    planner, controller, scenario and report.
    """
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("mission"), dict):
        raise ValueError(f"mission file must contain a mission mapping: {source}")
    mission = document["mission"]
    planning = mission.get("planning")
    if not isinstance(planning, dict):
        raise ValueError(f"mission file must contain mission.planning: {source}")
    if speed_cap_mps is not None:
        if (
            not isinstance(speed_cap_mps, (int, float))
            or isinstance(speed_cap_mps, bool)
            or not math.isfinite(float(speed_cap_mps))
            or float(speed_cap_mps) <= 0.0
        ):
            raise ValueError("speed_cap_mps must be finite and positive")
        planning["max_velocity_mps"] = float(speed_cap_mps)

    target = session.directory / "resolved_mission.yaml"
    target.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    resolved_planning = _mission_planning(target)
    _write_runtime(
        session,
        mission_contract={
            "source_path": str(source.resolve()),
            "resolved_path": str(target.resolve()),
            "speed_cap_mps": speed_cap_mps,
            "planning": resolved_planning,
        },
    )
    return target


def _collision_obstacles(map_profile: str) -> list[dict[str, Any]]:
    """Ground-truth-only collision geometry for simulator acceptance checks."""
    box = lambda name, center, half_extents: {
        "name": name, "type": "box", "center": center, "half_extents": half_extents,
    }
    cylinder = lambda name, center, radius_m, half_height_m: {
        "name": name, "type": "cylinder", "center": center,
        "radius_m": radius_m, "half_height_m": half_height_m,
    }
    world_name = {
        "smoke": "px4_lio_smoke", "speed": "open", "long_open_slow": "long_open",
        "occlusion": "occlusion", "occlusion_featured": "occlusion",
        "long_open_featured_core_60": "long_open_featured_speed",
        "long_open_featured_core_60_pv": "long_open_featured_speed",
        "long_three_pillars_multiwaypoint": "long_three_pillars_speed",
        "single_pillar_speed": "long_three_pillars_speed",
        "single_pillar_speed_pv": "long_three_pillars_speed",
    }.get(map_profile, map_profile)
    world_path = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
    if world_path.is_file():
        parsed: list[dict[str, Any]] = []
        try:
            root = ET.parse(world_path).getroot()
            for model in root.findall(".//model"):
                name = model.get("name", "")
                pose = (model.findtext("pose") or "0 0 0").split()
                center = [float(value) for value in pose[:3]]
                collision = model.find("./link/collision/geometry")
                if collision is None or len(center) != 3:
                    continue
                box_node = collision.find("box/size")
                cylinder_node = collision.find("cylinder")
                if box_node is not None:
                    size = [float(value) for value in box_node.text.split()]
                    if len(size) == 3:
                        parsed.append({"name": name, "type": "box", "center": center,
                                       "half_extents": [value / 2.0 for value in size]})
                elif cylinder_node is not None:
                    radius = float(cylinder_node.findtext("radius", "0"))
                    length = float(cylinder_node.findtext("length", "0"))
                    parsed.append({"name": name, "type": "cylinder", "center": center,
                                   "radius_m": radius, "half_height_m": length / 2.0})
        except (ET.ParseError, OSError, ValueError):
            parsed = []
        if parsed:
            return parsed
    if map_profile in {"open", "speed", "long_open", "long_open_slow"}:
        if map_profile in {"long_open", "long_open_slow"}:
            return [
                box("long_open_wall_east", [55.0, 0.0, 2.5], [0.125, 8.0, 2.5]),
                box("long_open_wall_north", [25.0, 3.0, 2.5], [30.0, 0.125, 2.5]),
                box("long_open_wall_south", [25.0, -3.0, 2.5], [30.0, 0.125, 2.5]),
            ]
        return [
            box("open_wall_east", [55.0 if map_profile == "long_open" else 10.0, 0.0, 2.5],
                [0.125, 10.0 if map_profile != "long_open" else 8.0, 2.5]),
            box("open_wall_north", [0.0, 10.0 if map_profile != "long_open" else 8.0, 2.5],
                [10.0 if map_profile != "long_open" else 55.0, 0.125, 2.5]),
        ]
    if map_profile == "long_featured":
        return [
            cylinder("long_featured_pillar_01", [9.0, -2.80, 1.0], 0.65, 1.0),
            cylinder("long_featured_pillar_02", [17.5, 2.60, 1.10], 0.75, 1.10),
            cylinder("long_featured_pillar_03", [26.0, -2.60, 1.20], 0.70, 1.20),
            cylinder("long_featured_pillar_04", [34.5, 2.80, 0.90], 0.85, 0.90),
            cylinder("long_featured_pillar_05", [42.5, -2.80, 1.10], 0.70, 1.10),
            cylinder("long_featured_pillar_06", [50.0, 2.60, 1.00], 0.75, 1.00),
            cylinder("long_featured_tree_01", [6.0, -4.0, 1.90], 0.35, 1.90),
            cylinder("long_featured_tree_02", [14.0, 4.0, 1.90], 0.45, 1.90),
            cylinder("long_featured_tree_03", [23.0, -4.5, 1.90], 0.30, 1.90),
            cylinder("long_featured_tree_04", [32.0, 4.5, 1.90], 0.50, 1.90),
            cylinder("long_featured_tree_05", [41.0, -4.0, 1.90], 0.40, 1.90),
            cylinder("long_featured_tree_06", [50.0, 4.0, 1.90], 0.55, 1.90),
            box("long_featured_texture_01", [3.5, 5.0, 1.50], [0.6, 0.25, 1.50]),
            box("long_featured_texture_02", [11.0, -5.2, 1.70], [0.3, 0.6, 1.70]),
            box("long_featured_texture_03", [20.0, 5.5, 1.25], [0.9, 0.225, 1.25]),
            box("long_featured_texture_04", [29.0, -5.0, 1.80], [0.25, 0.75, 1.80]),
            box("long_featured_texture_05", [39.0, 5.2, 1.45], [0.7, 0.3, 1.45]),
            box("long_featured_texture_06", [47.0, -5.5, 1.65], [0.35, 0.8, 1.65]),
        ]
    if map_profile == "pillar":
        return [
            cylinder("pillar_obstacle", [-4.5, 2.5, 2.5], 0.55, 2.5),
            box("wall_east", [7.0, 1.0, 2.5], [0.125, 9.0, 2.5]),
            box("wall_north", [-1.0, 9.0, 2.0], [6.0, 0.125, 2.0]),
        ]
    if map_profile in {"occlusion", "occlusion_featured"}:
        return [
            box("occluder", [-2.0, 2.5, 2.5], [0.175, 2.5, 2.5]),
            box("hidden_obstacle", [-4.3, 2.5, 2.5], [0.45, 0.6, 2.5]),
            box("side_wall", [6.0, 4.0, 2.5], [0.125, 6.0, 2.5]),
            box("feature_mid_east", [4.0, 2.5, 2.5], [0.4, 0.3, 2.5]),
            box("feature_low_west", [-1.0, -5.0, 0.75], [0.6, 0.4, 0.75]),
            box("feature_tall_south", [2.5, -7.5, 2.5], [0.35, 0.35, 2.5]),
            box("feature_corner_east", [8.5, 8.0, 1.6], [1.4, 0.3, 1.6]),
            cylinder("feature_pole_north", [12.0, -6.0, 1.8], 0.35, 1.8),
        ]
    if map_profile == "occlusion_degenerate":
        return [
            box("occluder", [-2.0, 2.5, 2.5], [0.175, 2.5, 2.5]),
            box("hidden_obstacle", [-4.3, 2.5, 2.5], [0.45, 0.6, 2.5]),
            box("side_wall", [6.0, 4.0, 2.5], [0.125, 6.0, 2.5]),
        ]
    if map_profile in {"tunnel_smooth", "tunnel_irregular"}:
        obstacles = [
            box("tunnel_wall_north" if map_profile == "tunnel_smooth" else "wall_north", [20.0, 4.0, 3.0], [21.0, 0.15, 3.0]),
            box("tunnel_wall_south" if map_profile == "tunnel_smooth" else "wall_south", [20.0, -4.0, 3.0], [21.0, 0.15, 3.0]),
            box("tunnel_ceiling" if map_profile == "tunnel_smooth" else "ceiling", [20.0, 0.0, 6.2], [21.0, 4.15, 0.15]),
            box("tunnel_end" if map_profile == "tunnel_smooth" else "end_wall", [41.0, 0.0, 3.0], [0.15, 4.15, 3.0]),
        ]
        if map_profile == "tunnel_irregular":
            obstacles.extend([
                box("alcove_a", [9.0, 3.1, 1.7], [1.7, 1.25, 1.7]),
                box("beam_b", [17.0, -1.3, 4.7], [2.25, 0.175, 0.225]),
                box("corner_c", [28.0, 2.7, 1.4], [0.6, 1.0, 1.4]),
            ])
    return obstacles


def _acceptance_threshold_for_profile(
    map_profile: str, profile_contract: dict[str, Any] | None = None
) -> float:
    """Return the mission-polyline deviation limit for a map profile."""
    if isinstance(profile_contract, dict):
        benchmark = profile_contract.get("benchmark")
        if isinstance(benchmark, dict):
            declared = benchmark.get("max_cross_track_m")
            if (
                isinstance(declared, (int, float))
                and not isinstance(declared, bool)
                and math.isfinite(declared)
                and declared > 0.0
            ):
                return float(declared)
    # Obstacle detours are validated separately by the collision envelope and
    # minimum-clearance gates.  Keep the open-space default strict while
    # allowing the deterministic long_three_pillars detour envelope.
    if map_profile == "long_three_pillars_speed":
        # The certified three-pillar route uses a bounded side detour.  The
        # map profile declares |y| <= 8 m as its acceptance envelope; obstacle
        # collision and clearance remain independent safety gates.
        return 8.0
    if map_profile in {"single_pillar_speed", "single_pillar_speed_pv"}:
        # The route guide's certified side offset is 3.6 m.  Keep a small
        # tracking/map-estimator allowance while the independent |y|<=8 m
        # benchmark envelope remains the hard out-of-map gate.
        return 4.5
    if map_profile == "pillar":
        # The origin-to-detour mission segment geometrically intersects the
        # 0.55 m pillar.  A 0.5 m polyline gate would forbid the lateral
        # displacement required by the independently enforced collision and
        # clearance envelope.  Keep a bounded 1.5 m detour allowance.
        return 1.5
    if map_profile in {"long_open_featured_core_60", "long_open_featured_core_60_pv"}:
        # The long-leg checkpoint measures estimator/controller drift in an
        # open corridor; keep a tighter bound than obstacle detours while
        # allowing the observed sub-metre LIO bias.  The hard map envelope is
        # still enforced independently by the benchmark corridor checks.
        return 0.75
    return 4.5 if map_profile == "long_three_pillars" else 0.5


def _map_registry() -> dict[str, Any]:
    registry = yaml.safe_load((RUNTIME_CONFIG / "map_profiles.yaml").read_text(encoding="utf-8"))
    if not isinstance(registry, dict) or not isinstance(registry.get("profiles"), dict):
        raise ValueError("map_profiles.yaml must contain a profiles mapping")
    profiles = registry["profiles"]
    for name, profile in profiles.items():
        if not isinstance(profile, dict):
            raise ValueError(f"map registry profile must be a mapping: {name}")
        for field in ("world", "mission", "expected_outcome", "collision_truth"):
            if field not in profile:
                raise ValueError(f"map registry profile {name} is missing {field}")
        world = ROOT / "src/uav_simulation/worlds" / f"{profile['world']}.sdf"
        mission = RUNTIME_CONFIG / "missions" / f"{profile['mission']}.yaml"
        if not world.is_file() or not mission.is_file():
            raise ValueError(f"map registry profile {name} references missing assets")
        if len(profile["collision_truth"]) != len(set(profile["collision_truth"])):
            raise ValueError(f"map registry profile {name} has duplicate collision truth names")
        available_names = {item["name"] for item in _collision_obstacles(str(name))}
        missing_truth = set(profile["collision_truth"]) - available_names
        if missing_truth:
            raise ValueError(f"map registry profile {name} has missing collision truth: {sorted(missing_truth)}")
    return profiles


def _scene_registry() -> dict[str, Any]:
    """Return the compact public scene registry used by Make/CI.

    The legacy ``profiles`` mapping remains authoritative for assets and
    collision truth.  Scenes are only a stable user-facing grouping layer, so
    adding a new geometry variant does not add another public Make profile.
    """
    registry = yaml.safe_load((RUNTIME_CONFIG / "map_profiles.yaml").read_text(encoding="utf-8"))
    scenes = registry.get("scenes", {}) if isinstance(registry, dict) else {}
    if not isinstance(scenes, dict) or not scenes:
        raise ValueError("map_profiles.yaml must contain a non-empty scenes mapping")
    profiles = _map_registry()
    known_legacy = set(profiles) | {
        "smoke", "open", "speed", "long_open", "long_open_slow", "long_featured",
        "corridor", "pillar", "no_path", "occlusion",
    }
    for scene, descriptor in scenes.items():
        if not isinstance(descriptor, dict) or not isinstance(descriptor.get("variants"), dict):
            raise ValueError(f"map scene must define variants: {scene}")
        for testcase, profile in descriptor["variants"].items():
            if not isinstance(profile, str) or profile not in known_legacy:
                raise ValueError(f"map scene {scene}/{testcase} references unknown profile {profile}")
    return scenes


def _resolve_scene_profile(
    map_scene: str | None,
    test_case: str,
    motion_preset: str,
    map_profile: str | None,
) -> tuple[str, dict[str, str]]:
    """Resolve canonical scene knobs to one legacy asset profile.

    ``MAP_PROFILE`` remains an escape hatch for old scripts.  When a scene is
    supplied, testcase is preferred and motion preset is used as a fallback
    only when that variant exists (for example long_route/slow).
    """
    if map_profile:
        return map_profile, {
            "scene": map_scene or "legacy",
            "test_case": test_case,
            "motion_preset": motion_preset,
        }
    if not map_scene or map_scene == "smoke":
        return "smoke", {"scene": "smoke", "test_case": test_case, "motion_preset": motion_preset}
    scenes = _scene_registry()
    if map_scene not in scenes:
        raise ValueError(f"unknown canonical map scene: {map_scene}")
    variants = scenes[map_scene]["variants"]
    profile_name = variants.get(test_case)
    if profile_name is None:
        profile_name = variants.get(motion_preset)
    if profile_name is None and test_case == "positive":
        profile_name = variants.get("nominal") or variants.get("positive")
    if profile_name is None:
        available = ", ".join(sorted(variants))
        raise ValueError(f"scene {map_scene} has no variant for testcase={test_case}, motion={motion_preset}; available: {available}")
    return str(profile_name), {
        "scene": map_scene,
        "test_case": test_case,
        "motion_preset": motion_preset,
    }


def _resolve_map_descriptor(session: Session, map_profile: str, map_seed: int) -> tuple[str, dict[str, Any]]:
    canonical = "occlusion_featured" if map_profile == "occlusion" else map_profile
    profile = _map_registry().get(canonical)
    if profile is None:
        # Keep legacy CLI aliases consistent with run_sim and collision-truth
        # resolution.  These profiles intentionally reuse an existing world;
        # attempting to read ``speed.sdf`` (or ``long_open_slow.sdf``) would
        # fail before the simulator starts and make speed evidence impossible.
        world_name = {
            "smoke": "px4_lio_smoke",
            "speed": "open",
            "long_open_slow": "long_open",
            "occlusion_featured": "occlusion",
            "occlusion_degenerate": "occlusion_degenerate",
            "long_open_featured_core_60": "long_open_featured_speed",
            "long_open_featured_core_60_pv": "long_open_featured_speed",
            "single_pillar_speed": "long_three_pillars_speed",
            "single_pillar_speed_pv": "long_three_pillars_speed",
        }.get(map_profile, map_profile)
        world_path = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
        descriptor = {
            "profile": map_profile, "seed": 0, "stochastic": False,
            "world": world_name, "expected_outcome": "mission_complete",
            "world_sha256": hashlib.sha256(world_path.read_bytes()).hexdigest(),
        }
        (session.directory / "resolved_map.sdf").write_bytes(world_path.read_bytes())
        (session.directory / "map_descriptor.json").write_text(
            json.dumps(descriptor, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return world_name, descriptor
    if not isinstance(profile, dict):
        raise ValueError(f"registry entry must be a mapping: {canonical}")
    world_name = str(profile["world"])
    world_path = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
    world_bytes = world_path.read_bytes()
    seed = int(map_seed if profile.get("stochastic", False) else profile.get("seed", 0))
    resolved_bytes = world_bytes
    if profile.get("stochastic", False):
        # The current forest asset is a deterministic seed-11 baseline.  Keep
        # the resolved artifact seed-addressable now; the generator can later
        # replace the static tree poses without changing the runner contract.
        resolved_bytes = b"<!-- map-seed: " + str(seed).encode("ascii") + b" -->\n" + world_bytes
    descriptor = {
        "profile": canonical,
        "requested_profile": map_profile,
        "seed": seed,
        "stochastic": bool(profile.get("stochastic", False)),
        "world": world_name,
        "mission": str(profile.get("mission", canonical)),
        "expected_outcome": str(profile.get("expected_outcome", "mission_complete")),
        "collision_truth": list(profile.get("collision_truth", [])),
        "world_sha256": hashlib.sha256(resolved_bytes).hexdigest(),
    }
    for field in ("route_obstacles", "route_segment_waypoints", "benchmark"):
        if field in profile:
            descriptor[field] = profile[field]
    resolved_world = session.directory / f"resolved_{world_name}.sdf"
    resolved_world.write_bytes(resolved_bytes)
    (session.directory / "map_descriptor.json").write_text(
        json.dumps(descriptor, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return world_name, descriptor


def _external_mode_params(session: Session, source: Path, mission_file: Path | None,
                          speed_cap_mps: float | None = None) -> Path:
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "px4_navigation_external_mode" not in value:
        raise ValueError(f"runtime config is missing px4_navigation_external_mode: {source}")
    # planner backend PVA is the nominal contract.  Limits are injected into planner backend by
    # _mapping_params, never into this transport layer.
    del mission_file, speed_cap_mps
    target = session.directory / "external_mode_params.yaml"
    target.write_text(yaml.safe_dump({"px4_navigation_external_mode": value["px4_navigation_external_mode"]}, sort_keys=False), encoding="utf-8")
    return target


def _external_mode_launch_command(config_file: Path, mission_file: Path | None) -> list[str]:
    command = [
        "ros2", "launch", "navigation_bringup", "px4_external_mode.launch.py",
        f"config_file:={config_file}", "use_sim_time:=true",
    ]
    if mission_file is not None:
        command.append(f"mission_file:={mission_file}")
    return command


def _navigation_runtime_launch_command(
    config_file: Path, mission_file: Path | None
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "navigation_bringup",
        "navigation_runtime.launch.py",
        f"config_file:={config_file}",
        "use_sim_time:=true",
    ]
    if mission_file is not None:
        # The launch file owns a mission_file override whose empty default
        # otherwise replaces the value embedded in config_file. Always pass
        # the session-resolved contract explicitly at this boundary.
        command.append(f"mission_file:={mission_file.resolve()}")
    return command


def _mapping_params(
    session: Session,
    source: Path,
    *,
    mission_file: Path | None = None,
    speed_cap_mps: float | None = None,
) -> Path:
    """Create the only ROS parameter file used by native planner backend navigation."""
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "navigation_runtime_node" not in value:
        raise ValueError(f"runtime config is missing navigation_runtime_node: {source}")
    node_parameters = value["navigation_runtime_node"].setdefault("ros__parameters", {})
    planner_parameters = node_parameters.setdefault("navigation_runtime", {})
    planner = yaml.safe_load(
        (ROOT / "src/runtime/navigation_runtime/config/planner.yaml").read_text(
            encoding="utf-8"
        )
    )
    if not isinstance(planner, dict):
        raise ValueError("planner backend planner config must be a mapping")
    # PVA is the direct control contract, so mission limits must constrain the
    # planner backend optimizer itself.  Applying them here avoids reintroducing an
    # external velocity/acceleration limiter that would distort the planned
    # trajectory after optimization.
    planning = _mission_planning(mission_file) if mission_file is not None else {}
    if mission_file is not None:
        # NavigationRuntimeNode independently loads mission dynamics before
        # constructing planner backend. Passing only the generated planner YAML leaves
        # the controller and planner with different mission identities and
        # makes runtime provenance unable to prove which limits were active.
        planner_parameters["mission_file"] = str(mission_file.resolve())
    # The canonical planner profile owns map evidence production. Do not
    # silently disable sensor-origin raycasting here: BACKUP certification
    # requires KNOWN_FREE evidence, while MAIN unknown-space policy is applied
    # independently below.
    target_speed = speed_cap_mps
    if target_speed is None:
        target_speed = planning.get("max_velocity_mps")
    boundary = planner.setdefault("traj_opt", {}).setdefault("boundary", {})
    product_max_velocity = float(boundary["max_vel"])
    product_max_acceleration = float(boundary["max_acc"])
    product_max_jerk = float(boundary["max_jerk"])
    if target_speed is not None:
        target_speed = float(target_speed)
        if not math.isfinite(target_speed) or target_speed <= 0.0:
            raise ValueError("planner backend target speed must be finite and positive")
        if target_speed > product_max_velocity:
            raise ValueError(
                f"planner backend target speed exceeds X500 limit {product_max_velocity:g} m/s"
            )
        boundary["max_vel"] = target_speed
    if "max_acceleration_mps2" in planning:
        acceleration_limit = float(planning["max_acceleration_mps2"])
        if acceleration_limit > product_max_acceleration:
            raise ValueError(
                "planner backend mission acceleration exceeds X500 limit "
                f"{product_max_acceleration:g} m/s^2"
            )
        boundary["max_acc"] = acceleration_limit
    traj_opt = planner.setdefault("traj_opt", {})
    exp_traj = traj_opt.setdefault("exp_traj", {})
    backup_traj = traj_opt.setdefault("backup_traj", {})
    if "max_jerk_mps3" in planning:
        jerk_limit = float(planning["max_jerk_mps3"])
        if not math.isfinite(jerk_limit) or jerk_limit <= 0.0:
            raise ValueError("planner backend max jerk must be finite and positive")
        if jerk_limit > product_max_jerk:
            raise ValueError(
                f"planner backend mission jerk exceeds X500 limit {product_max_jerk:g} m/s^3"
            )
        boundary["max_jerk"] = jerk_limit
        # Main-trajectory jerk remains an analytic hard gate because its larger
        # optimization problem became unstable with a high-order penalty. The
        # two-piece backup starts from a certified minimum-snap seed and must
        # retain a positive jerk objective while L-BFGS adjusts it.
    planner_target = session.directory / "planner.yaml"
    planner_target.write_text(yaml.safe_dump(planner, sort_keys=False), encoding="utf-8")
    planner_parameters["config_path"] = str(planner_target)
    target = session.directory / "navigation_runtime_params.yaml"
    target.write_text(
        yaml.safe_dump({"navigation_runtime_node": value["navigation_runtime_node"]}, sort_keys=False),
        encoding="utf-8",
    )
    _write_runtime(session, planner_config=str(planner_target), target_speed_mps=target_speed)
    return target

def _mapping_ready(snapshot: dict[str, Any]) -> bool:
    """Require an accepted ROG observation from either runtime architecture."""
    latest = snapshot.get("latest", {}).get("mapping_diagnostics", {})
    for status in latest.get("statuses", []):
        status_name = str(status.get("name", ""))
        if not (status_name.endswith("/world_model") or status_name == "navigation_runtime/planner"):
            continue
        values = status.get("values", {})
        try:
            accepted = int(values.get("accepted_observation_count", 0)) > 0
            if status_name == "navigation_runtime/planner":
                return accepted and int(values.get("processing_exception_count", 0)) == 0
            return accepted and (
                int(values.get("visualization_publish_count", 0)) > 0
                or int(values.get("visualization_subscriber_count", 0)) == 0
            ) and int(values.get("visualization_exception_count", 0)) == 0
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
        live_pgids = {
            int(record["pgid"])
            for record in session.live_records()
            if record.get("pgid") is not None
        }
        for record in session.records():
            if record.get("role") in {
                "monitor", "lio", "px4_gazebo", "bridge", "bridge_lidar",
                "px4_ingress", "mapping",
            }:
                # A monitor snapshot is the authoritative readiness signal, but
                # an early process exit must be reported immediately. Process-
                # group liveness deliberately excludes unreaped zombies.
                if int(record.get("pgid", -1)) not in live_pgids:
                    raise RuntimeError(
                        f"runtime process {record.get('role')} exited before readiness"
                    )
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


def _dataset_replay_command(context: dict[str, Any], rate: float) -> list[str]:
    """Replay recorded sensor time as the authoritative ROS clock.

    Dataset headers retain their acquisition epoch.  Navigation freshness must
    therefore compare them with bag time, never with the host wall clock.
    """
    return [
        "ros2", "bag", "play", str(context["bag"]), "--rate", str(rate), "--clock",
        "--remap", f"{context['input']['lidar_topic']}:=/lidar/points",
        f"{context['input']['imu_topic']}:=/lidar/imu",
    ]


def _expected_dataset_stream_counts(
    context: dict[str, Any], counts: dict[str, int]
) -> dict[str, int]:
    return {
        "imu": counts[context["input"]["imu_topic"]],
        "lidar": counts[context["input"]["lidar_topic"]],
    }


def _dataset_source_duration_ns(context: dict[str, Any]) -> int:
    metadata = yaml.safe_load((Path(context["bag"]) / "metadata.yaml").read_text(
        encoding="utf-8"
    ))
    duration = metadata.get("rosbag2_bagfile_information", {}).get("duration", {}).get(
        "nanoseconds"
    )
    if not isinstance(duration, int) or isinstance(duration, bool) or duration <= 0:
        raise RuntimeError("dataset bag duration is missing or invalid")
    return duration


def _dataset_outputs_drained(session: Session, expected: dict[str, int]) -> bool:
    return (
        _stream_count(session, "imu") == expected["imu"]
        and _stream_count(session, "lidar") == expected["lidar"]
        and _stream_count(session, "corrected_odometry") > 0
        and _stream_count(session, "propagated_odometry") > 0
    )


_SPEED_CERTIFICATION_PROFILES = {
    "long_three_pillars_speed",
    "long_three_pillars_multiwaypoint",
    "long_open_featured_speed",
}


def _required_measured_speed_mps(
    map_profile: str,
    mission_max_velocity_mps: float | None,
    speed_cap_mps: float | None,
) -> float | None:
    """Return the requested cruise target for a long speed benchmark."""
    if map_profile not in _SPEED_CERTIFICATION_PROFILES:
        return None
    requested = speed_cap_mps if speed_cap_mps is not None else mission_max_velocity_mps
    if (
        requested is None
        or not isinstance(requested, (int, float))
        or isinstance(requested, bool)
    ):
        return None
    requested = float(requested)
    return requested if math.isfinite(requested) and requested > 0.0 else None


def _locked_dataset_runner(function: Callable[..., int]) -> Callable[..., int]:
    @wraps(function)
    def wrapped(*args: Any, **kwargs: Any) -> int:
        try:
            # Dataset replay reads the already-built install. Keep builds
            # excluded, but share the install lock with SITL so parallel
            # validation agents do not starve each other before launch.
            with BuildRuntimeLock(ROOT, exclusive=False):
                with RuntimeLock(
                    DATASET_RUNTIME_LOCK_PATH,
                    artifact_root=ARTIFACT_ROOT,
                    active_workflows={"dataset"},
                ):
                    return function(*args, **kwargs)
        except BuildRuntimeBusyError as error:
            print(f"ERROR: {error}", file=sys.stderr)
            return 2
    return wrapped


@_locked_dataset_runner
def run_dataset(
    dataset: str,
    rate: float,
    *,
    enable_rviz: bool = False,
    shadow_planning_goal_distance_m: float = 5.0,
    ros_domain_id: int | None = None,
) -> int:
    if not dataset:
        raise ValueError("DATASET is required")
    if rate <= 0:
        raise ValueError("RATE must be greater than zero")
    if (
        not math.isfinite(shadow_planning_goal_distance_m)
        or shadow_planning_goal_distance_m < 0.0
    ):
        raise ValueError("shadow planning goal distance must be finite and non-negative")
    config = load_config("dataset.yaml")
    config["runtime"]["dataset"] = dataset
    config["runtime"]["replay_rate"] = rate
    isolated_domain = _resolve_isolation_value(
        ros_domain_id,
        "UAV_NAV_DATASET_ROS_DOMAIN_ID",
        DEFAULT_DATASET_ROS_DOMAIN_ID,
        low=0,
        high=232,
    )
    session = Session.create(ARTIFACT_ROOT, "dataset")
    print(f"Session: {session.directory}", flush=True)
    os.environ["ROS_DOMAIN_ID"] = str(isolated_domain)
    os.environ.pop("PX4_UXRCE_DDS_PORT", None)
    os.environ.pop("PX4_UXRCE_DDS_NS", None)
    os.environ["ROS_LOG_DIR"] = str(session.logs)
    os.environ["RCUTILS_LOGGING_DIRECTORY"] = str(session.logs)
    monitor_process: subprocess.Popen[Any] | None = None
    try:
        _write_runtime(
            session,
            workflow="dataset",
            dataset=dataset,
            rate=rate,
            ros_domain_id=isolated_domain,
            dds_isolation="dataset ROS_DOMAIN_ID isolated from SITL",
            rviz=enable_rviz,
            replay_tail_grace_s=float(
                config["runtime"]["thresholds"].get("replay_tail_grace_s", 0.5)
            ),
            dataset_shadow_planning={
                "enabled": shadow_planning_goal_distance_m > 0.0,
                "goal_distance_m": shadow_planning_goal_distance_m,
                "contract": "planner/runtime benchmark only; recorded odometry does not execute commands",
            },
            failures=[],
        )
        _capture_build_provenance(session)
        shadow_mission_file = _resolved_mission_file(
            session, DATASET_SHADOW_MISSION, None
        )
        _write_runtime(
            session,
            dataset_shadow_planning={
                "enabled": shadow_planning_goal_distance_m > 0.0,
                "goal_distance_m": shadow_planning_goal_distance_m,
                "contract": "planner/runtime benchmark only; recorded odometry does not execute commands",
                "mission_file": str(shadow_mission_file),
                "main_unknown_policy": "allow_unknown",
                "backup_unknown_policy": "require_known_free",
            },
        )
        context, counts = _dataset_context(dataset)
        expected_stream_counts = _expected_dataset_stream_counts(context, counts)
        _write_runtime(session, dataset_context={
            "id": context["id"],
            "bag": str(context["bag"]),
            "counts": counts,
            "expected_stream_counts": expected_stream_counts,
            "source_duration_ns": _dataset_source_duration_ns(context),
        })
        ros_config = _ros_params(session, RUNTIME_CONFIG / "dataset.yaml")
        mapping_config = _mapping_params(
            session,
            RUNTIME_CONFIG / "mapping.yaml",
            mission_file=shadow_mission_file,
        )
        lidar_to_imu_xyz, lidar_to_imu_rpy = _lidar_to_imu_launch_arguments(config)
        monitor_process = session.start(
            "monitor",
            _ros_shell([
                str(CANONICAL_PYTHON), str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory),
                "--workflow", "dataset", "--config", str(RUNTIME_CONFIG / "dataset.yaml"),
            ]),
            cwd=ROOT,
        )
        session.start(
            "mapping",
            _ros_shell(
                _navigation_runtime_launch_command(mapping_config, shadow_mission_file),
                enable_rviz=enable_rviz,
            ),
            cwd=ROOT,
        )
        lio = session.start(
            "lio",
            _ros_shell([
                "ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
                f"config_file:={ros_config}",
                "use_sim_time:=true", "enable_external_odometry:=false",
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
        _write_runtime(session, replay_started_wall_ns=time.time_ns())
        replay = session.start(
            "replay",
            _ros_shell(_dataset_replay_command(context, rate), enable_rviz=enable_rviz),
            cwd=ROOT,
        )
        if shadow_planning_goal_distance_m > 0.0:
            shadow_planning = session.start(
                "dataset_shadow_planning",
                _ros_shell([
                    str(CANONICAL_PYTHON),
                    str(ROOT / "tools/runtime/dataset_shadow_planning.py"),
                    "--output", str(session.directory),
                    "--goal-distance-m", str(shadow_planning_goal_distance_m),
                ], enable_rviz=enable_rviz),
                cwd=ROOT,
            )
            shadow_code = _wait_process(
                shadow_planning,
                45.0,
                "dataset bounded shadow planning",
            )
            _write_runtime(session, dataset_shadow_planning_returncode=shadow_code)
            # A planner failure is report evidence, not authority to truncate
            # the recorded-data replay. Finish the source stream so mapping
            # conservation and timing remain directly comparable with the
            # mapping-only baseline; _dataset_report fails closed from the
            # helper result below.
        replay_code = _wait_process(replay, float(config["runtime"]["timeouts"]["replay_s"]), "dataset replay")
        _write_runtime(session, replay_returncode=replay_code, replay_finished_wall_ns=time.time_ns())
        if replay_code != 0:
            raise RuntimeError(f"dataset replay exited with {replay_code}")
        _wait_until(
            session,
            lambda snapshot: _dataset_outputs_drained(session, expected_stream_counts),
            float(config["runtime"]["timeouts"]["drain_s"]),
            "dataset outputs and queue drain",
        )
        _wait_until(session, _mapping_ready,
                    float(config["runtime"]["timeouts"]["drain_s"]),
                    "mapping output and visualization")
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
    python_error = canonical_python_error()
    if python_error:
        missing.append(python_error)
    for command in ("ros2", "MicroXRCEAgent"):
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
        ROOT / "src/uav_simulation/bridge/px4_mid360_control_bridge.yaml",
        ROOT / "src/uav_simulation/bridge/px4_mid360_lidar_bridge.yaml",
        ROOT / "install/setup.bash",
    ):
        if not path.exists():
            missing.append(f"missing path: {path}")
    stale_contract_artifacts = _stale_navigation_contract_artifacts()
    if stale_contract_artifacts:
        missing.append(
            "stale navigation contract generated artifacts: "
            + ", ".join(stale_contract_artifacts[:6])
            + (" ..." if len(stale_contract_artifacts) > 6 else "")
            + "; rebuild the canonical workspace before starting SITL"
        )
    return missing


def _stale_navigation_contract_artifacts() -> list[str]:
    """Detect generated ROS message files not backed by the current contract."""
    stale: list[str] = []
    source_names: set[str] = set()
    for path in (ROOT / "src/contracts/navigation_contracts/msg").glob("*.msg"):
        source_name = path.stem
        source_names.add(source_name.lower())
        source_names.add(re.sub(r"(?<!^)(?=[A-Z])", "_", source_name).lower())
    generated_names: set[str] = set()
    adapter_dir = ROOT / "build/navigation_contracts/rosidl_adapter/navigation_contracts/msg"
    generated_names.update(path.stem.lower() for path in adapter_dir.glob("*.idl"))
    python_dir = ROOT / "install/navigation_contracts/lib"
    for path in python_dir.glob("python*/site-packages/navigation_contracts/msg/_*.py"):
        name = path.stem.lower().lstrip("_")
        if name != "init__":
            generated_names.add(name)
    stale.extend(sorted(generated_names - source_names))
    return stale


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


def _start_gazebo_native_observer(
    session: Session, world: str, gz_command: str
) -> subprocess.Popen[Any]:
    process = session.start(
        "gazebo_native_observer",
        [
            str(CANONICAL_PYTHON),
            str(ROOT / "tools/runtime/gazebo_native_observer.py"),
            "--session",
            str(session.directory),
            "--world",
            world,
            "--gz-command",
            gz_command,
        ],
        cwd=ROOT,
    )
    _write_runtime(
        session,
        gazebo_native_observer={
            "status": "started",
            "world_stats_topic": f"/world/{world}/stats",
            "world_clock_topic": f"/world/{world}/clock",
            "samples": str(session.directory / "gazebo_native_samples.jsonl"),
            "summary": str(session.directory / "gazebo_native_summary.json"),
            "process_period_s": 1.0,
            "verdict_owner": "diagnostic_only",
        },
    )
    return process


def _run_sim_unlocked(
    headless: bool,
    control_interface: str = "offboard",
    *,
    map_profile: str | None = None,
    map_scene: str | None = None,
    test_case: str = "positive",
    motion_preset: str = "nominal",
    map_seed: int = 0,
    ros_domain_id: int | None = None,
    xrce_port: int | None = None,
    auto_scenario: bool = False,
    manual_takeoff: bool = False,
    speed_cap_mps: float | None = None,
    gazebo_native_diagnostic: bool = False,
) -> int:
    if control_interface not in {"offboard", "external_mode"}:
        raise ValueError(f"unsupported control interface: {control_interface}")
    map_profile, scene_descriptor = _resolve_scene_profile(
        map_scene, test_case, motion_preset, map_profile
    )
    world_name = "px4_lio_smoke" if map_profile == "smoke" else {
        "speed": "open",
        "long_open": "long_open",
        "long_open_slow": "long_open",
        "long_featured": "long_featured",
        "long_open_featured_core_60": "long_open_featured_speed",
        "long_open_featured_core_60_pv": "long_open_featured_speed",
        "single_pillar_speed": "long_three_pillars_speed",
        "single_pillar_speed_pv": "long_three_pillars_speed",
        "occlusion_featured": "occlusion",
        "occlusion_degenerate": "occlusion_degenerate",
    }.get(map_profile, map_profile)
    world_path = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
    if not world_path.is_file():
        raise ValueError(f"map profile world does not exist: {world_path}")
    config = load_config("sim.yaml")
    scenario_config_name = "offboard.yaml" if control_interface == "offboard" else "external_mode_scenario.yaml"
    scenario_config = load_config(scenario_config_name)
    scenario_config.setdefault("scenario", {})["map_profile"] = map_profile
    scenario_config["scenario"].update({
        "map_scene": scene_descriptor["scene"],
        "test_case": scene_descriptor["test_case"],
        "motion_preset": scene_descriptor["motion_preset"],
        "manual_takeoff": bool(manual_takeoff),
        "interactive_handover": bool(not headless and auto_scenario),
    })
    if manual_takeoff:
        if headless or control_interface != "external_mode" or not auto_scenario:
            raise ValueError("manual takeoff is supported only by the automatic GUI External Mode workflow")
        scenario_config["scenario"]["activation_timeout_s"] = max(
            float(scenario_config["scenario"].get("activation_timeout_s", 30.0)), 180.0)
        scenario_config["scenario"]["takeoff_timeout_s"] = max(
            float(scenario_config["scenario"].get("takeoff_timeout_s", 45.0)), 180.0)
    mission_file: Path | None = None
    if control_interface == "external_mode" and str(scenario_config["scenario"].get("execution", "single_goal")) == "mission":
        registry_entry = _map_registry().get("occlusion_featured" if map_profile == "occlusion" else map_profile, {})
        mission_name = registry_entry.get("mission", map_profile) if isinstance(registry_entry, dict) else map_profile
        profile_mission = RUNTIME_CONFIG / "missions" / f"{mission_name}.yaml"
        mission_file = profile_mission if map_profile != "smoke" and profile_mission.is_file() else Path(str(scenario_config["scenario"]["mission_file"]))
        if not mission_file.is_absolute():
            mission_file = (ROOT / mission_file).resolve()
        if not mission_file.is_file():
            raise RuntimeError(f"mission file does not exist: {mission_file}")
        scenario = scenario_config["scenario"]
        scenario["mission_file"] = str(mission_file)
        scenario["vehicle_collision_radius_m"] = 0.35
        scenario["collision_obstacles"] = _collision_obstacles(map_profile)
        scenario["map_seed"] = int(map_seed)
        scenario["require_map_observability"] = map_profile == "no_path"
        # A pass-through checkpoint can be inside the acceptance ball at
        # activation (the open SITL baseline starts at [0, 0, 3]).  The
        # controller intentionally advances it without manufacturing a
        # zero-length trajectory, so the harness must accept that explicit
        # prefix skip while still requiring strict order for all published
        # goals.
        try:
            mission_document = yaml.safe_load(mission_file.read_text(encoding="utf-8"))
            mission_waypoints = (
                mission_document.get("mission", {}).get("waypoints", [])
                if isinstance(mission_document, dict) else []
            )
            if not isinstance(mission_waypoints, list) or not mission_waypoints:
                raise ValueError("mission file has no waypoints")
            # The acceptance harness must follow the selected mission artifact,
            # not the scenario template's stale hard-coded waypoint count.
            scenario["mission_waypoint_count"] = len(mission_waypoints)
            first_waypoint = (
                mission_waypoints[0]
            )
            scenario["allow_initial_pass_through_skip"] = (
                isinstance(first_waypoint, dict)
                and str(first_waypoint.get("behavior", "stop")) == "pass_through"
            )
        except (OSError, TypeError, ValueError, IndexError):
            scenario["allow_initial_pass_through_skip"] = False
        scenario["pillar_waypoint_index"] = {
            "open": -1,
            "speed": -1,
            "long_open": -1,
            "long_open_slow": -1,
            "long_featured": -1,
            "long_three_pillars": -1,
            "long_three_pillars_multiwaypoint": -1,
            "long_open_featured_speed": -1,
            # The bounded open-world checkpoint is deliberately obstacle-free
            # in the mission corridor; its gate is LIO/PX4 long-leg stability,
            # not a synthetic single-pillar detour assertion.
            "long_open_featured_core_60": -1,
            "long_open_featured_core_60_pv": -1,
            "single_pillar_speed": 1,
            "single_pillar_speed_pv": 1,
            "corridor": -1,
            "pillar": 3,
            "occlusion": 2,
            "no_path": -1,
        }.get(map_profile, 3)
        planning = _mission_planning(mission_file)
        if "max_velocity_mps" in planning:
            scenario["expected_max_velocity_mps"] = planning["max_velocity_mps"]
        if speed_cap_mps is not None:
            if (
                not isinstance(speed_cap_mps, (int, float))
                or isinstance(speed_cap_mps, bool)
                or not math.isfinite(speed_cap_mps)
                or speed_cap_mps <= 0.0
            ):
                raise ValueError("speed_cap_mps must be finite and positive")
            scenario["expected_max_velocity_mps"] = float(speed_cap_mps)
            # Slow detours can spend most of their time in a lateral retiming
            # segment rather than on the straight leg.  Size the harness
            # timeout from route length and requested cap instead of turning a
            # valid low-speed run into WALL_TIMEOUT.  The factor is a test
            # budget only; it does not change planner limits or acceptance.
            if isinstance(mission_waypoints, list) and len(mission_waypoints) >= 2:
                route_length = 0.0
                for first, second in zip(mission_waypoints, mission_waypoints[1:]):
                    try:
                        a = [float(value) for value in first["position"]]
                        b = [float(value) for value in second["position"]]
                        route_length += math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
                    except (KeyError, TypeError, ValueError):
                        route_length = 0.0
                        break
                if route_length > 0.0:
                    timeout_budget = 120.0 + 3.0 * route_length / max(float(speed_cap_mps), 0.25)
                    scenario["mission_timeout_s"] = max(
                        float(scenario.get("mission_timeout_s", 120.0)), timeout_budget
                    )
        required_speed = _required_measured_speed_mps(
            map_profile, planning.get("max_velocity_mps"), speed_cap_mps
        )
        if required_speed is not None:
            scenario["required_measured_speed_mps"] = required_speed
        if map_profile in {"no_path", "occlusion_degenerate", "tunnel_smooth"}:
            scenario["expected_outcome"] = "fail_closed"
            scenario["mission_timeout_s"] = min(float(scenario.get("mission_timeout_s", 120.0)), 60.0)
        elif map_profile == "corridor":
            scenario["mission_timeout_s"] = max(float(scenario.get("mission_timeout_s", 120.0)), 180.0)
        elif map_profile in {"long_open", "long_open_slow", "long_featured", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "long_open_featured_speed", "long_open_featured_core_60", "long_open_featured_core_60_pv", "single_pillar_speed", "single_pillar_speed_pv"}:
            scenario["mission_timeout_s"] = max(float(scenario.get("mission_timeout_s", 120.0)), 300.0)
        if map_profile in {"pillar", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "single_pillar_speed", "single_pillar_speed_pv"}:
            # This profile has three route obstacles; use the multi-obstacle
            # ground-truth metric instead of the legacy single-pillar check.
            scenario["planned_clearance_check"] = False
            # Cross-track distance is measured against the straight mission
            # polyline.  A central pillar requires a lateral detour, so the
            # generic 0.5 m open-space gate would reject a collision-free
            # route as if it were estimator drift. Collision envelope,
            # minimum clearance, LIO residual, waypoint order, and mission
            # completion remain independent gates.
            scenario.setdefault("acceptance", {})["max_cross_track_p95_m"] = (
                _acceptance_threshold_for_profile(map_profile, registry_entry)
            )
        elif map_profile in {"long_open_featured_core_60", "long_open_featured_core_60_pv"}:
            # The open long-leg checkpoint is nominally straight, but the
            # measured LIO/PX4 handover can contribute sub-metre lateral
            # tracking error. Keep that acceptance allowance explicit in the
            # profile rather than inheriting the generic 0.5 m legacy gate.
            scenario.setdefault("acceptance", {})["max_cross_track_p95_m"] = (
                _acceptance_threshold_for_profile(map_profile)
            )
        elif map_profile == "long_open_featured_speed":
            # This is a high-speed wide-map run, but the world deliberately
            # contains near-side features at +/-6 m for scan matching. A
            # straight-line cross-track gate would reject the valid lateral
            # avoidance path; collision clearance, waypoint order, speed and
            # mission completion remain the authoritative safety gates.
            scenario.setdefault("acceptance", {})["max_cross_track_p95_m"] = 8.0
        elif map_profile in {"tunnel_irregular", "tunnel_smooth", "forest_clutter"}:
            scenario["mission_timeout_s"] = max(float(scenario.get("mission_timeout_s", 120.0)), 240.0)
        if map_profile in {"occlusion", "occlusion_featured"}:
            scenario["pillar_center_enu"] = [-4.3, 2.5, 3.0]
            scenario["pillar_clearance_m"] = 1.0
            scenario["mission_timeout_s"] = max(float(scenario.get("mission_timeout_s", 120.0)), 180.0)
            # This profile evaluates revealed-obstacle behavior and ground
            # truth clearance. Its occluder changes the local frame evidence,
            # so the generic planned-path center-distance assertion is not a
            # valid acceptance metric for this profile.
            scenario["planned_clearance_check"] = False
    px4_dir = Path(os.environ.get("PX4_DIR", str(Path.home() / "Dev/Autopilot"))).expanduser().resolve()
    gz_command = _detect_gz_command()
    workflow = "external-mode" if control_interface == "external_mode" else "sim"
    session_name = (
        "external-mode-check"
        if headless and control_interface == "external_mode"
        else "sim-check"
        if headless
        else "external-mode-gui"
        if control_interface == "external_mode"
        else "sim"
    )
    session = Session.create(ARTIFACT_ROOT, session_name)
    print(f"Session: {session.directory}", flush=True)
    if mission_file is not None:
        mission_file = _resolved_mission_file(session, mission_file, speed_cap_mps)
        scenario_config["scenario"]["mission_file"] = str(mission_file)
    isolated_domain = _resolve_isolation_value(
        ros_domain_id, "UAV_NAV_ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID, low=0, high=232
    )
    isolated_xrce_port = _resolve_isolation_value(
        xrce_port, "UAV_NAV_XRCE_PORT", DEFAULT_XRCE_PORT, low=1024, high=65535
    )
    # PX4's rcS consumes ROS_DOMAIN_ID and PX4_UXRCE_DDS_PORT when it starts
    # uxrce_dds_client.  Set them before any child process is launched so the
    # simulator, agent, bridges, monitor and scenario all share one domain.
    os.environ["ROS_DOMAIN_ID"] = str(isolated_domain)
    os.environ["PX4_UXRCE_DDS_PORT"] = str(isolated_xrce_port)
    # Gazebo Transport is independent of ROS 2 DDS.  Give every benchmark a
    # deterministic private partition as well, otherwise a second SITL (or a
    # physical M40 companion using the default partition) can advertise the
    # same /world topics and make Gazebo fail with a transport socket error.
    os.environ["GZ_PARTITION"] = f"uav_navigation_{isolated_domain}_{isolated_xrce_port}"
    # Do not inherit a namespace from a developer shell: the existing bridge
    # and planner subscribe to absolute /fmu names.  Domain isolation is the
    # safe boundary for this stack.
    os.environ.pop("PX4_UXRCE_DDS_NS", None)
    # ROS 2 and Gazebo otherwise default to per-user log directories.  The
    # session owns a writable, reproducible location and Session.start
    # inherits these variables for monitor, bridge, mapping and simulator
    # processes alike.
    os.environ["ROS_LOG_DIR"] = str(session.logs)
    os.environ["RCUTILS_LOGGING_DIRECTORY"] = str(session.logs)
    os.environ["GZ_LOG_DIR"] = str(session.logs)
    world_name, map_descriptor = _resolve_map_descriptor(session, map_profile, map_seed)
    map_descriptor.update({
        "scene": scene_descriptor["scene"],
        "test_case": scene_descriptor["test_case"],
        "motion_preset": scene_descriptor["motion_preset"],
    })
    scenario_config.setdefault("scenario", {})["route_obstacles"] = list(
        map_descriptor.get("route_obstacles", [])
    )
    scenario_config["scenario"]["route_segment_waypoints"] = list(
        map_descriptor.get("route_segment_waypoints", [])
    )
    (session.directory / "map_descriptor.json").write_text(
        json.dumps(map_descriptor, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    world_path = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
    if not world_path.is_file():
        raise ValueError(f"map profile world does not exist: {world_path}")
    session.write_state({
        "workflow": workflow,
        "headless": headless,
        "auto_scenario": auto_scenario,
        "manual_takeoff": manual_takeoff,
        "px4_dir": str(px4_dir),
        "map_profile": map_profile,
        "map_scene": scene_descriptor["scene"],
        "test_case": scene_descriptor["test_case"],
        "motion_preset": scene_descriptor["motion_preset"],
        "map_seed": map_descriptor.get("seed", 0),
        "map_descriptor": str(session.directory / "map_descriptor.json"),
        "ros_domain_id": isolated_domain,
        "xrce_port": isolated_xrce_port,
        "dds_isolation": (
            "ROS_DOMAIN_ID + dedicated MicroXRCEAgent UDP port + private GZ_PARTITION"
        ),
    })
    scenario_config_path = session.directory / "scenario_config.yaml"
    scenario_config.setdefault("scenario", {})["map_descriptor"] = str(
        session.directory / "map_descriptor.json"
    )
    registry_outcome = str(map_descriptor.get("expected_outcome", ""))
    if registry_outcome:
        scenario_config["scenario"]["expected_outcome"] = (
            "complete" if registry_outcome == "mission_complete" else registry_outcome
        )
    scenario_config_path.write_text(yaml.safe_dump(scenario_config, sort_keys=False), encoding="utf-8")
    _write_runtime(
        session,
        workflow=workflow,
        headless=headless,
        auto_scenario=auto_scenario,
        rviz=not headless,
        px4_dir=str(px4_dir),
        gz_command=gz_command,
        failures=[],
        startup_complete=False,
        map_profile=map_profile,
        map_scene=scene_descriptor["scene"],
        test_case=scene_descriptor["test_case"],
        motion_preset=scene_descriptor["motion_preset"],
        ros_domain_id=isolated_domain,
        xrce_port=isolated_xrce_port,
    )
    prereq = _sim_prerequisites(px4_dir, gz_command)
    try:
        _capture_build_provenance(session, px4_dir)
    except Exception as error:
        prereq.append(f"build provenance: {error}")
    if prereq:
        _write_runtime(session, failures=prereq)
        result = _stop_and_report(session, workflow, RUNTIME_CONFIG / "sim.yaml", px4_dir=px4_dir)
        print(result["verdict"])
        print(session.directory)
        return 1
    try:
        ros_config = _ros_params(session, RUNTIME_CONFIG / "sim.yaml")
        mapping_config = _mapping_params(
            session,
            RUNTIME_CONFIG / "mapping.yaml",
            mission_file=mission_file,
            # The mission file above already owns the resolved speed contract.
            # Do not create a second planner-only source of truth here.
            speed_cap_mps=None if mission_file is not None else speed_cap_mps,
        )
        lidar_to_imu_xyz, lidar_to_imu_rpy = _lidar_to_imu_launch_arguments(config)
        monitor = session.start(
            "monitor",
            _ros_shell([
                str(CANONICAL_PYTHON), str(ROOT / "tools/runtime/monitor.py"), "--output", str(session.directory),
                "--workflow", "sim", "--config", str(RUNTIME_CONFIG / "sim.yaml"),
            ]),
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
                "PX4_GZ_WORLD": world_name,
                # Automated scenarios use the same PX4 input policy in GUI
                # and headless runs; `make sim` remains the manual mode.
                "PX4_PARAM_COM_RC_IN_MODE": _px4_manual_control_mode(
                    headless or (auto_scenario and not manual_takeoff)),
            }),
            env_remove=GUI_ENV_REMOVE,
        )
        simulation = config.get("simulation", {})
        if isinstance(simulation, dict) and "ros__parameters" in simulation:
            simulation = simulation["ros__parameters"]
        world = world_name
        if not gz_command:
            raise RuntimeError("Gazebo simulator command is unavailable")
        _wait_gazebo(world, float(config["runtime"]["timeouts"]["startup_s"]), gz_command)
        if gazebo_native_diagnostic:
            _start_gazebo_native_observer(session, world, gz_command)
        session.start(
            "xrce_agent", ["MicroXRCEAgent", "udp4", "-p", str(isolated_xrce_port)], cwd=ROOT
        )
        control_bridge_source = (
            ROOT / "src/uav_simulation/bridge/px4_mid360_control_bridge.yaml"
        )
        lidar_bridge_source = (
            ROOT / "src/uav_simulation/bridge/px4_mid360_lidar_bridge.yaml"
        )
        control_bridge_config = session.directory / "px4_mid360_control_bridge.yaml"
        lidar_bridge_config = session.directory / "px4_mid360_lidar_bridge.yaml"
        control_bridge_config.write_text(
            control_bridge_source.read_text(encoding="utf-8").replace(
                "/world/px4_lio_smoke/clock", f"/world/{world}/clock"
            ),
            encoding="utf-8",
        )
        lidar_bridge_config.write_text(
            lidar_bridge_source.read_text(encoding="utf-8"), encoding="utf-8"
        )
        session.start("bridge", _ros_shell([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
            "-r", "__node:=px4_mid360_control_bridge",
            "-p", f"config_file:={control_bridge_config}",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("bridge_lidar", _ros_shell([
            "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
            "-r", "__node:=px4_mid360_lidar_bridge",
            "-p", f"config_file:={lidar_bridge_config}",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start("px4_ingress", _ros_shell([
            "ros2", "run", "px4_odometry_bridge", "px4_odometry_bridge_node", "--ros-args",
            "--params-file", str(ros_config), "-p", "use_sim_time:=true",
        ], enable_rviz=not headless), cwd=ROOT)
        session.start(
            "mapping",
            _ros_shell(
                _navigation_runtime_launch_command(mapping_config, mission_file),
                enable_rviz=not headless,
            ),
            cwd=ROOT,
        )
        session.start("lio", _ros_shell([
            "ros2", "launch", "navigation_bringup", "fast_lio.launch.py",
            f"config_file:={ros_config}", "use_sim_time:=true",
            "enable_external_odometry:=true", "publish_sensor_frames:=true",
            "livox_mount_xyz:=0 0 0.28", "livox_mount_rpy:=0 0 0",
            f"livox_lidar_to_imu_xyz:={lidar_to_imu_xyz}",
            f"livox_lidar_to_imu_rpy:={lidar_to_imu_rpy}",
        ], enable_rviz=not headless), cwd=ROOT)
        if control_interface == "external_mode" and not headless and not auto_scenario:
            session.start(
                "external_mode",
                _ros_shell(
                    _external_mode_launch_command(
                        _external_mode_params(session, RUNTIME_CONFIG / "external_mode.yaml", mission_file,
                                              speed_cap_mps),
                        mission_file,
                    ),
                    enable_rviz=True,
                ),
                cwd=ROOT,
            )
        if not headless:
            _start_rviz(session, use_sim_time=True)
        _wait_until(session, lambda snapshot: _stream_count(session, "imu") > 0 and _stream_count(session, "lidar") > 0, float(config["runtime"]["timeouts"]["startup_s"]), "simulated sensor streams")
        _wait_until(session, lambda snapshot: str(snapshot.get("diagnostics", {}).get("state", "")).upper() == "TRACKING", float(config["runtime"]["timeouts"]["lio_tracking_s"]), "LIO TRACKING")
        _wait_until(session, lambda snapshot: _stream_count(session, "external_odometry") > 0, float(config["runtime"]["timeouts"]["external_odometry_s"]), "PX4 external odometry")
        _wait_until(session, _mapping_ready,
                    float(config["runtime"]["timeouts"].get("mapping_ready_s", config["runtime"]["timeouts"]["external_odometry_s"])),
                    "mapping output and visualization")
        _write_runtime(session, startup_complete=True)
        if headless or auto_scenario:
            if auto_scenario:
                mission_kind = (
                    "Manual Takeoff/arm then automatic External Mode mission: "
                    if manual_takeoff else "Automatic External Mode mission: "
                )
                print(
                    mission_kind +
                    f"scene={scene_descriptor['scene']} case={scene_descriptor['test_case']} "
                    f"motion={scene_descriptor['motion_preset']} profile={map_profile}; "
                    "Gazebo GUI and RViz remain enabled."
                )
            scenario_name = "offboard_scenario.py" if control_interface == "offboard" else "external_mode_scenario.py"
            scenario_role = "offboard" if control_interface == "offboard" else "external_mode_scenario"
            scenario = session.start(
                scenario_role,
                _ros_shell([
                    str(CANONICAL_PYTHON), str(ROOT / "tools/runtime" / scenario_name),
                    "--output", str(session.directory / "scenario.json"), "--config", str(scenario_config_path),
                ]),
                cwd=ROOT,
            )
            if control_interface == "external_mode":
                external_mode_args = _external_mode_launch_command(
                    _external_mode_params(session, RUNTIME_CONFIG / "external_mode.yaml", mission_file,
                                          speed_cap_mps),
                    mission_file,
                )
                session.start("external_mode", _ros_shell([
                    *external_mode_args,
                ], enable_rviz=not headless), cwd=ROOT)
            scenario_wait_timeout = (
                math.inf
                if bool(scenario_config["scenario"].get("interactive_handover", False))
                else float(scenario_config["scenario"]["wall_timeout_s"]) + 10.0
            )
            scenario_code = _wait_process(scenario, scenario_wait_timeout, f"{control_interface} scenario")
            scenario_payload = {}
            try:
                scenario_payload = json.loads((session.directory / "scenario.json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                scenario_payload = {"failures": ["scenario did not create a report"]}
            scenario_outcome = str(scenario_payload.get("outcome", ""))
            expected_pause = scenario_code == 2 and scenario_outcome in {
                "ABORTED_OPERATOR", "PAUSED_SAFETY_STOP"
            }
            if scenario_code != 0 and not expected_pause:
                scenario_payload.setdefault("failures", []).append(f"{control_interface} scenario exited with {scenario_code}")
            (session.directory / "scenario.json").write_text(json.dumps(scenario_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            time.sleep(float(config["runtime"]["timeouts"]["post_flight_monitor_s"]))
        else:
            print(f"Session: {session.directory}")
            print("Topics: /lidar/imu /lidar/points /lio/odometry_corrected /lio/odometry_propagated /lio/diagnostics /fmu/in/vehicle_visual_odometry")
            if control_interface == "external_mode":
                print("Manual flight: use QGC/your supervisor to arm and take off, wait until airborne and stable, then select External Mode.")
                print("External Mode reads the selected mission YAML and only publishes navigation setpoints; it does not take off, land, RTL, or disarm.")
            else:
                print("Manual flight: arm, enter OFFBOARD only with your own controller, and use make status; stop with make stop.")
            print("Use make status for the session state and make stop to terminate the GUI session.")
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
        result = _stop_and_report(
            session,
            workflow,
            RUNTIME_CONFIG / "sim.yaml",
            px4_dir=px4_dir,
            observation_complete=not headless and not auto_scenario and not _load_runtime_failures(session),
        )
    print(result["verdict"])
    print(session.directory)
    if workflow == "external-mode":
        external_mode_result = result.get("external_mode", {})
        outcome = str(external_mode_result.get("outcome", ""))
        expected_fail_closed = (
            str(external_mode_result.get("expected_outcome", "complete")) == "fail_closed"
        )
        if outcome in {"ABORTED_OPERATOR", "PAUSED_SAFETY_STOP"} and not (
            outcome == "PAUSED_SAFETY_STOP" and expected_fail_closed
        ):
            return 2
    return 0 if result["verdict"] == "PASS" or result.get("observation_status") == "OBSERVATION_COMPLETE" else 1


def _current_runner_session(existing: set[Path] | None = None) -> Session | None:
    """Find the session created by this runner, including setup-only sessions."""
    existing = existing or set()
    for path in _runtime_session_paths(ARTIFACT_ROOT):
        if path in existing:
            continue
        session = Session.from_path(path)
        state = session.state()
        if state.get("owner_pid") == os.getpid():
            return session
        # Backward-compatible recovery for a session created before owner_pid
        # was added to state.json.
        marker = f"-{os.getpid()}"
        if path.name.endswith(marker) or path.name.rsplit("-", 1)[0].endswith(marker):
            return session
    return None


def _recover_unfinalized_session(
    error: BaseException,
    existing: set[Path] | None = None,
) -> None:
    """Finalize a session when startup fails before the main try/finally."""
    session = _current_runner_session(existing)
    if session is None or (session.directory / "report.json").is_file():
        return
    failures = _load_runtime_failures(session)
    message = str(error).strip() or type(error).__name__
    _write_runtime(session, failures=failures + [f"runner setup: {message}"])
    workflow = str(session.state().get("workflow", "sim"))
    config_path = RUNTIME_CONFIG / ("dataset.yaml" if workflow == "dataset" else "sim.yaml")
    px4_value = session.state().get("px4_dir")
    px4_dir = Path(str(px4_value)) if px4_value else None
    try:
        result = _stop_and_report(
            session,
            workflow,
            config_path,
            px4_dir=px4_dir,
        )
    except Exception as recovery_error:
        print(
            f"failed to finalize aborted session {session.directory}: {recovery_error}",
            file=sys.stderr,
        )
        return
    print(result["verdict"], file=sys.stderr)
    print(session.directory, file=sys.stderr)


def _run_sim_with_runtime_lock(*args: Any, **kwargs: Any) -> int:
    """Run one simulation under a simulation lock and cleanup signal guard."""
    requested_port = kwargs.get("xrce_port")
    with RuntimeLock(active_workflows={"sim", "external-mode"}):
        isolated_port = _resolve_isolation_value(
            requested_port, "UAV_NAV_XRCE_PORT", DEFAULT_XRCE_PORT, low=1024, high=65535
        )
        _assert_xrce_port_available(isolated_port)
        with _RunnerSignalGuard():
            existing_sessions = set(_runtime_session_paths(ARTIFACT_ROOT))
            try:
                return _run_sim_unlocked(*args, **kwargs)
            except BaseException as error:
                _recover_unfinalized_session(error, existing_sessions)
                raise


def run_sim(*args: Any, **kwargs: Any) -> int:
    """Run SITL while excluding builds and other SITL sessions."""
    try:
        # SITL reads the verified install just like dataset replay. A shared
        # install lock allows the two isolated validation workflows to start
        # together while still excluding any build that could replace it.
        with BuildRuntimeLock(ROOT, exclusive=False):
            return _run_sim_with_runtime_lock(*args, **kwargs)
    except BuildRuntimeBusyError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


def _load_runtime_failures(session: Session) -> list[str]:
    try:
        value = json.loads((session.directory / "runtime.json").read_text(encoding="utf-8"))
        return [str(item) for item in value.get("failures", [])]
    except (OSError, ValueError):
        return []


def status() -> int:
    try:
        session_path, recovered = _resolve_latest_or_newest(ARTIFACT_ROOT)
    except FileNotFoundError:
        print(f"No runtime session under {ARTIFACT_ROOT}", file=sys.stderr)
        return 1
    if recovered:
        print(
            f"warning: runtime/latest is missing or stale; using {session_path.name}",
            file=sys.stderr,
        )
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


def _resolve_latest_or_newest(root: Path) -> tuple[Path, bool]:
    """Resolve latest, falling back to the newest durable session."""
    try:
        return resolve_latest(root), False
    except FileNotFoundError:
        sessions = _runtime_session_paths(root)
        if not sessions:
            raise
        return sessions[0], True


def stop() -> int:
    session_paths = _runtime_session_paths(ARTIFACT_ROOT)
    try:
        session_path, recovered_latest = _resolve_latest_or_newest(ARTIFACT_ROOT)
    except FileNotFoundError:
        latest = ARTIFACT_ROOT / "latest"
        if latest.is_symlink():
            latest.unlink()
        print(f"No runtime session under {ARTIFACT_ROOT}")
        return 0
    if session_path not in session_paths:
        session_paths.append(session_path)
    cleanup_failures: list[str] = []
    reports: dict[Path, dict[str, Any]] = {}
    for path in session_paths:
        session = Session.from_path(path)
        was_live = bool(session.live_records())
        failures = session.stop()
        if failures:
            existing = _load_runtime_failures(session)
            _write_runtime(
                session,
                failures=existing + [f"cleanup: {item}" for item in failures],
            )
            cleanup_failures.extend(f"{path.name}: {item}" for item in failures)
        session.mark_stopped("make stop")
        if was_live or path == session_path or not (path / "report.json").is_file():
            workflow = str(session.state().get("workflow", "sim"))
            config_path = RUNTIME_CONFIG / (
                "dataset.yaml" if workflow == "dataset" else "sim.yaml"
            )
            px4_value = session.state().get("px4_dir")
            px4_dir = Path(str(px4_value)) if px4_value else None
            interactive_workflow = (
                workflow in {"sim", "external-mode"}
                and not session.state().get("headless", False)
                and not session.state().get("auto_scenario", False)
            )
            reports[path] = report.build(
                session.directory,
                workflow,
                config_path,
                ROOT,
                px4_dir,
                observation_complete=interactive_workflow,
            )

    if recovered_latest:
        update_latest(ARTIFACT_ROOT, session_path)
    result = reports[session_path]
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


def _remove_generated_path(path: Path, removed: list[Path]) -> None:
    import shutil

    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
        removed.append(path)
    elif path.is_file() or path.is_symlink():
        path.unlink()
        removed.append(path)


def _clean_unlocked(*, clean_workspace_caches: bool = True) -> int:
    """Remove generated state while the caller owns the runtime lock."""
    import shutil

    removed: list[Path] = []
    for path in GENERATED_CLEAN_PATHS:
        resolved = path.resolve()
        if resolved == Path("/") or resolved == ROOT.resolve():
            raise ValueError(f"refusing unsafe clean path: {resolved}")
        if resolved == ARTIFACT_ROOT.parent.resolve() and path.is_dir() and not path.is_symlink():
            # Keep the directory and advisory lock inode alive for the whole
            # clean. Removing the lock path would let a new simulation start
            # concurrently while old sessions are still being deleted.
            for child in path.iterdir():
                if child.resolve() != ARTIFACT_ROOT.resolve():
                    _remove_generated_path(child, removed)
                    continue
                for runtime_child in child.iterdir():
                    if runtime_child.resolve() in {
                        RUNTIME_LOCK_PATH.resolve(),
                        DATASET_RUNTIME_LOCK_PATH.resolve(),
                        BUILD_RUNTIME_LOCK_PATH.resolve(),
                    }:
                        continue
                    _remove_generated_path(runtime_child, removed)
            continue
        _remove_generated_path(path, removed)

    if clean_workspace_caches:
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
        try:
            display = path.relative_to(ROOT)
        except ValueError:
            display = path
        print(f"removed {display}")
    return 0


def clean() -> int:
    """Safely remove generated state while preserving active simulations."""
    with RuntimeLock():
        # Keep the build/runtime lock inode alive while traversing the shared
        # artifact cache; otherwise a concurrent build can recreate the path
        # and defeat the stale-install exclusion.
        with BuildRuntimeLock(ROOT, exclusive=True):
            return _clean_unlocked()


def _number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def main() -> int:
    try:
        require_canonical_python()
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    dataset = sub.add_parser("dataset-check")
    dataset.add_argument("--dataset", required=True)
    dataset.add_argument("--rate", type=float, required=True)
    dataset.add_argument(
        "--ros-domain-id",
        type=int,
        default=None,
        help=f"isolated ROS 2 DDS domain (default: $UAV_NAV_DATASET_ROS_DOMAIN_ID or {DEFAULT_DATASET_ROS_DOMAIN_ID})",
    )
    dataset.add_argument("--rviz", action="store_true", help="launch RViz for this replay")
    dataset.add_argument(
        "--shadow-planning-goal-distance-m",
        type=float,
        default=5.0,
        help="bounded dataset shadow-planning goal distance; 0 disables planning",
    )
    sub.add_parser("sim-check")
    external_mode = sub.add_parser("external-mode-check")
    external_mode.add_argument(
        "--map-profile",
        choices=(
            "smoke", "open", "speed", "long_open", "long_open_slow", "long_featured", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "long_open_featured_speed", "long_open_featured_core_60", "long_open_featured_core_60_pv", "single_pillar_speed", "single_pillar_speed_pv",
            "corridor", "pillar", "occlusion", "occlusion_featured", "occlusion_degenerate",
            "tunnel_irregular", "tunnel_smooth", "forest_clutter", "no_path",
        ),
        default=None,
        help="legacy Gazebo map profile; prefer --map-scene/--test-case",
    )
    external_mode.add_argument(
        "--map-scene", choices=CANONICAL_SCENES, default=None,
        help="compact canonical scene family used by the external-mode scenario",
    )
    external_mode.add_argument(
        "--test-case", choices=TEST_CASES, default="positive",
        help="scene variant (positive, degenerate, detour or no_path)",
    )
    external_mode.add_argument(
        "--motion-preset", choices=MOTION_PRESETS, default="nominal",
        help="motion variant used when the scene provides one",
    )
    external_mode.add_argument(
        "--map-seed", type=int, default=0,
        help="deterministic seed for stochastic map profiles (for example forest_clutter)",
    )
    external_mode.add_argument(
        "--ros-domain-id", type=int, default=None,
        help=f"isolated ROS 2 DDS domain (default: $UAV_NAV_ROS_DOMAIN_ID or {DEFAULT_ROS_DOMAIN_ID})",
    )
    external_mode.add_argument(
        "--xrce-port", type=int, default=None,
        help=f"dedicated MicroXRCEAgent UDP port (default: $UAV_NAV_XRCE_PORT or {DEFAULT_XRCE_PORT})",
    )
    external_mode.add_argument(
        "--speed-cap-mps", type=float, default=None,
        help="temporary planner/tracker velocity upper bound for one benchmark run",
    )
    external_mode.add_argument(
        "--gazebo-native-diagnostic", action="store_true",
        help="diagnostic-only native Gazebo stats/process observer; not an acceptance gate",
    )
    sub.add_parser("sim")
    external_mode_gui = sub.add_parser(
        "external-mode-gui",
        aliases=("external-mode",),
        help="interactive Gazebo/RViz session with the External Mode mission node",
    )
    external_mode_gui.add_argument(
        "--map-profile",
        choices=(
            "smoke", "open", "speed", "long_open", "long_open_slow", "long_featured", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "long_open_featured_speed", "long_open_featured_core_60", "long_open_featured_core_60_pv", "single_pillar_speed", "single_pillar_speed_pv",
            "corridor", "pillar", "occlusion", "occlusion_featured", "occlusion_degenerate",
            "tunnel_irregular", "tunnel_smooth", "forest_clutter", "no_path",
        ),
        default=None,
        help="legacy Gazebo map profile; prefer --map-scene/--test-case",
    )
    external_mode_gui.add_argument(
        "--map-scene", choices=CANONICAL_SCENES, default=None,
        help="compact canonical scene family and static mission YAML",
    )
    external_mode_gui.add_argument(
        "--test-case", choices=TEST_CASES, default="positive",
        help="scene variant (positive, degenerate, detour or no_path)",
    )
    external_mode_gui.add_argument(
        "--motion-preset", choices=MOTION_PRESETS, default="nominal",
        help="motion variant used when the scene provides one",
    )
    external_mode_gui.add_argument(
        "--map-seed", type=int, default=0,
        help="deterministic seed for stochastic map profiles (for example forest_clutter)",
    )
    external_mode_gui.add_argument(
        "--ros-domain-id", type=int, default=None,
        help=f"isolated ROS 2 DDS domain (default: $UAV_NAV_ROS_DOMAIN_ID or {DEFAULT_ROS_DOMAIN_ID})",
    )
    external_mode_gui.add_argument(
        "--xrce-port", type=int, default=None,
        help=f"dedicated MicroXRCEAgent UDP port (default: $UAV_NAV_XRCE_PORT or {DEFAULT_XRCE_PORT})",
    )
    external_mode_gui.add_argument(
        "--speed-cap-mps", type=float, default=None,
        help="temporary planner/tracker velocity upper bound for one benchmark run",
    )
    external_mode_gui.add_argument(
        "--gazebo-native-diagnostic", action="store_true",
        help="diagnostic-only native Gazebo stats/process observer; not an acceptance gate",
    )
    external_mode_gui.add_argument(
        "--manual-takeoff", action="store_true",
        help="do not send ARM/TAKEOFF; wait for the operator before activating External Mode",
    )
    sub.add_parser("status")
    sub.add_parser("stop")
    sub.add_parser("clean")
    args = parser.parse_args()
    enable_rviz = args.command in {"sim", "external-mode-gui", "external-mode"} or (args.command == "dataset-check" and args.rviz)
    os.environ.update(RVIZ_ENV if enable_rviz else NO_RVIZ_ENV)
    if args.command == "dataset-check":
        return run_dataset(
            args.dataset,
            args.rate,
            enable_rviz=args.rviz,
            shadow_planning_goal_distance_m=args.shadow_planning_goal_distance_m,
            ros_domain_id=args.ros_domain_id,
        )
    if args.command == "sim-check":
        return run_sim(True)
    if args.command == "external-mode-check":
        return run_sim(
            True,
            control_interface="external_mode",
            map_profile=args.map_profile,
            map_scene=args.map_scene,
            test_case=args.test_case,
            motion_preset=args.motion_preset,
            map_seed=args.map_seed,
            ros_domain_id=args.ros_domain_id,
            xrce_port=args.xrce_port,
            speed_cap_mps=args.speed_cap_mps,
            gazebo_native_diagnostic=args.gazebo_native_diagnostic,
        )
    if args.command == "sim":
        return run_sim(False)
    if args.command in {"external-mode-gui", "external-mode"}:
        return run_sim(
            False,
            control_interface="external_mode",
            map_profile=args.map_profile,
            map_scene=args.map_scene,
            test_case=args.test_case,
            motion_preset=args.motion_preset,
            map_seed=args.map_seed,
            ros_domain_id=args.ros_domain_id,
            xrce_port=args.xrce_port,
            speed_cap_mps=args.speed_cap_mps,
            gazebo_native_diagnostic=args.gazebo_native_diagnostic,
            auto_scenario=True,
            manual_takeoff=args.manual_takeoff,
        )
    if args.command == "status":
        return status()
    if args.command == "stop":
        return stop()
    return clean()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeBusyError as error:
        print(f"RUNTIME_BUSY: {error}", file=sys.stderr)
        raise SystemExit(75) from error

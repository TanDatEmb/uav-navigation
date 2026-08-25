#!/usr/bin/env python3
"""Run colcon with reproducible build-mode-specific directories and flags."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys

RUNTIME_DIR = Path(__file__).resolve().parent
if str(RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(RUNTIME_DIR))

from build_provenance import MANIFEST_NAME, create_manifest, source_fingerprint, write_manifest_atomic
from runtime_environment import (
    BuildRuntimeBusyError,
    BuildRuntimeLock,
    CANONICAL_PYTHON,
    require_canonical_python,
)


ROOT = Path(__file__).resolve().parents[2]
ROS_SYSTEM_PYTHON_PATH = Path("/usr/lib/python3/dist-packages")
PARALLEL_WORKERS = os.environ.get("PARALLEL_WORKERS", "1")
MAKE_JOBS = os.environ.get("MAKE_JOBS", "1")
COLCON_FLAGS = shlex.split(os.environ.get("COLCON_FLAGS", ""))
STANDALONE_EXCLUDED_PACKAGES: tuple[str, ...] = ()
# The pinned px4_ros2_interface_lib submodule contains the product library and
# upstream example packages.  The examples are independently discoverable by
# colcon, but are not used by this workspace's runtime or acceptance path.
PX4_ROS2_EXAMPLE_PACKAGE_REGEX = r"^example_.*_cpp$"
BUILD_PX4_ROS2_EXAMPLES = os.environ.get("BUILD_PX4_ROS2_EXAMPLES", "0").lower() in {
    "1",
    "true",
    "yes",
}
# These packages are upstream examples/integration tests.  They are useful
# when an FMU and the upstream test harness are available, but they are not
# part of the product acceptance gate and otherwise make `make test` depend
# on an external PX4 process.
PRODUCT_TEST_EXCLUDED_PACKAGES: tuple[str, ...] = (
    "px4_ros2_cpp",
    "example_executor_with_multiple_modes_cpp",
    "example_global_navigation_cpp",
    "example_local_navigation_cpp",
    "example_mode_fw_attitude_cpp",
    "example_mode_goto_cpp",
    "example_mode_goto_global_cpp",
    "example_mode_manual_cpp",
    "example_mode_mission_cpp",
    "example_mode_rtl_replacement_cpp",
    "example_mode_vtol_cpp",
    "example_mode_with_executor_cpp",
    "example_rover_velocity_mode_cpp",
)
PRODUCT_TEST_PACKAGES: tuple[str, ...] = (
    "fast_lio_core",
    "fast_lio_ros",
    "fast_lio_tools",
    "navigation_common",
    "navigation_contracts",
    "navigation_execution",
    "navigation_runtime",
    "navigation_planning",
    "navigation_planning_backend",
    "px4_navigation_external_mode",
    "px4_odometry_bridge",
    "uav_simulation",
)
# `make build` is a product build, not a workspace-wide discovery build. The
# list intentionally excludes simulator/demo packages and lets colcon resolve
# only the transitive dependencies of the FAST-LIO -> mapping backend -> planner backend -> PX4
# path. An explicit `build --packages ...` remains available for development.
PRODUCT_BUILD_PACKAGES: tuple[str, ...] = (
    "fast_lio_ros",
    "fast_lio_tools",
    "navigation_common",
    "navigation_contracts",
    "navigation_execution",
    "navigation_bringup",
    "navigation_runtime",
    "px4_navigation_external_mode",
    "px4_odometry_bridge",
    "navigation_planning_backend",
    "navigation_planning",
)
MODES = {
    "release": {
        "build_type": "Release",
        "flags": (),
    },
    "profile": {
        "build_type": "RelWithDebInfo",
        "flags": (),
    },
    "debug": {
        "build_type": "Debug",
        "flags": (),
    },
    "asan": {
        "build_type": "Debug",
        "flags": ("-fsanitize=address", "-fno-omit-frame-pointer"),
    },
    "ubsan": {
        "build_type": "Debug",
        "flags": (
            "-fsanitize=undefined",
            "-fno-sanitize-recover=all",
            "-fno-omit-frame-pointer",
        ),
    },
    "tsan": {
        "build_type": "Debug",
        "flags": ("-fsanitize=thread", "-fno-omit-frame-pointer"),
    },
}


def mode_paths(mode: str) -> tuple[Path, Path, Path]:
    suffix = "" if mode == "release" else f"-{mode}"
    return (
        Path(os.environ.get("BUILD_BASE", ROOT / f"build{suffix}")),
        Path(os.environ.get("INSTALL_BASE", ROOT / f"install{suffix}")),
        Path(os.environ.get("LOG_BASE", ROOT / f"log{suffix}")),
    )


def test_result_base(mode: str) -> Path:
    suffix = "" if mode == "release" else f"-{mode}"
    return Path(os.environ.get("TEST_RESULT_BASE", ROOT / f"test-results{suffix}"))


def ros_environment(install: Path | None = None) -> dict[str, str]:
    require_canonical_python()
    setup_files = [Path("/opt/ros/jazzy/setup.bash")]
    if install is not None and (install / "setup.bash").is_file():
        setup_files.append(install / "setup.bash")
    command = " && ".join(
        [f"source {shlex.quote(str(path))}" for path in setup_files]
        + [f"{shlex.quote(str(CANONICAL_PYTHON))} -c 'import os; print(chr(0).join(f\"{{k}}={{v}}\" for k, v in os.environ.items()))'"]
    )
    result = subprocess.run(
        ["bash", "-c", command],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        text=False,
    )
    environment = os.environ.copy()
    # Keep ROS client logs inside workspace-owned artifacts. This makes
    # package tests hermetic in CI, containers, and managed runners where the
    # user home may be read-only.
    environment.setdefault("ROS_HOME", str(ROOT / ".ros"))
    environment.setdefault("ROS_LOG_DIR", str(ROOT / "log" / "ros"))
    for item in result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            environment[key.decode()] = value.decode()
    if ROS_SYSTEM_PYTHON_PATH.is_dir():
        python_paths = [
            item for item in environment.get("PYTHONPATH", "").split(os.pathsep) if item
        ]
        system_path = str(ROS_SYSTEM_PYTHON_PATH)
        if system_path not in python_paths:
            python_paths.append(system_path)
        environment["PYTHONPATH"] = os.pathsep.join(python_paths)
    return environment


def sanitizer_environment(mode: str, environment: dict[str, str]) -> None:
    if mode == "asan":
        environment.setdefault(
            "ASAN_OPTIONS", "abort_on_error=1:detect_leaks=1:disable_coredump=0"
        )
    elif mode == "ubsan":
        environment.setdefault(
            "UBSAN_OPTIONS", "halt_on_error=1:print_stacktrace=1"
        )
    elif mode == "tsan":
        environment.setdefault("TSAN_OPTIONS", "halt_on_error=1")


def _main_unlocked() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=sorted(MODES), default="release")
    subparsers = parser.add_subparsers(dest="action", required=True)
    for action in ("build", "test"):
        child = subparsers.add_parser(action)
        child.add_argument("--packages", nargs="+")
    subparsers.add_parser("check")
    args = parser.parse_args()

    build, install, log = mode_paths(args.mode)
    environment = ros_environment(install if args.action != "build" else None)
    sanitizer_environment(args.mode, environment)

    if args.action == "build":
        spec = MODES[args.mode]
        compile_flags = " ".join(spec["flags"])
        command = [
            "colcon",
            "--log-base",
            str(log),
            "build",
            "--base-paths",
            "src",
            "--build-base",
            str(build),
            "--install-base",
            str(install),
            "--symlink-install",
            "--parallel-workers",
            PARALLEL_WORKERS,
            "--executor",
            "sequential",
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={spec['build_type']}",
        ]
        if compile_flags:
            command.extend(
                [
                    f"-DCMAKE_C_FLAGS={compile_flags}",
                    f"-DCMAKE_CXX_FLAGS={compile_flags}",
                    f"-DCMAKE_EXE_LINKER_FLAGS={compile_flags}",
                    f"-DCMAKE_SHARED_LINKER_FLAGS={compile_flags}",
                ]
            )
        if not BUILD_PX4_ROS2_EXAMPLES:
            command.extend(
                [
                    "--packages-ignore-regex",
                    PX4_ROS2_EXAMPLE_PACKAGE_REGEX,
                ]
            )
        command.extend(COLCON_FLAGS)
    elif args.action == "test":
        command = [
            "colcon",
            "--log-base",
            str(log),
            "test",
            "--base-paths",
            "src",
            "--build-base",
            str(build),
            "--install-base",
            str(install),
            "--test-result-base",
            str(test_result_base(args.mode)),
            "--event-handlers",
            "console_cohesion+",
        ]
    else:
        command = [
            "colcon",
            "test-result",
            "--test-result-base",
            str(test_result_base(args.mode)),
            "--verbose",
        ]

    packages = getattr(args, "packages", None)
    if args.action in {"build", "test"} and STANDALONE_EXCLUDED_PACKAGES:
        command.extend(["--packages-skip", *STANDALONE_EXCLUDED_PACKAGES])
    if args.action == "test" and not packages:
        command.extend(["--packages-select", *PRODUCT_TEST_PACKAGES])
    if args.action == "build" and not packages:
        command.extend(["--packages-up-to", *PRODUCT_BUILD_PACKAGES])
    if packages:
        command.extend(["--packages-select", *packages])
    # GCC ThreadSanitizer can fail before main() under Linux's randomized
    # address layout ("unexpected memory mapping").  Run only the test
    # processes with ASLR disabled; the build itself remains unchanged.
    if args.mode == "tsan" and args.action == "test":
        command = ["setarch", os.uname().machine, "-R", *command]
    print("+", shlex.join(command), flush=True)
    if args.action == "build":
        environment["MAKEFLAGS"] = f"-j{MAKE_JOBS}"
    # Any Release build can replace files in the canonical install.  A
    # package-select build is deliberately *not* allowed to leave the old
    # authoritative certificate valid; only a subsequent full Release build
    # may recreate it.
    release_build = args.action == "build" and args.mode == "release"
    if release_build:
        (install / MANIFEST_NAME).unlink(missing_ok=True)
    full_product_build = release_build and not packages
    build_started_wall_ns = 0
    source_before = None
    if full_product_build:
        # A failed/interrupted full build must not leave the previous build
        # certificate looking authoritative.
        build_started_wall_ns = __import__("time").time_ns()
        source_before = source_fingerprint(ROOT)
    result = subprocess.run(
        command, cwd=ROOT, env=environment, check=False
    ).returncode
    if result == 0 and full_product_build:
        try:
            manifest = create_manifest(
                ROOT,
                install,
                mode=args.mode,
                authoritative=True,
                command=command,
                build_started_wall_ns=build_started_wall_ns,
                source_before=source_before,
            )
            path = write_manifest_atomic(install, manifest)
            print(f"Wrote authoritative build manifest: {path}", flush=True)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            print(f"Build provenance failed closed: {error}", file=sys.stderr)
            result = 1
    if result == 0 and args.action == "check":
        data_check = [str(CANONICAL_PYTHON), str(ROOT / "tools" / "data.py"), "check"]
        print("+", shlex.join(data_check), flush=True)
        result = subprocess.run(
            data_check, cwd=ROOT, env=environment, check=False
        ).returncode
    return result


def main() -> int:
    """Run one build/test/check operation under the canonical lock."""
    try:
        require_canonical_python()
        with BuildRuntimeLock(ROOT, exclusive=True):
            return _main_unlocked()
    except (BuildRuntimeBusyError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

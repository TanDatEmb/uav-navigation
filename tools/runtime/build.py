#!/usr/bin/env python3
"""Run colcon with reproducible build-mode-specific directories and flags."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
PARALLEL_WORKERS = os.environ.get("PARALLEL_WORKERS", "1")
MAKE_JOBS = os.environ.get("MAKE_JOBS", "1")
COLCON_FLAGS = shlex.split(os.environ.get("COLCON_FLAGS", ""))
STANDALONE_EXCLUDED_PACKAGES = ("px4_msgs", "px4_odometry_bridge")
MODES = {
    "release": {
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


def ros_environment(install: Path | None = None) -> dict[str, str]:
    setup_files = [Path("/opt/ros/jazzy/setup.bash")]
    if install is not None and (install / "setup.bash").is_file():
        setup_files.append(install / "setup.bash")
    command = " && ".join(
        [f"source {shlex.quote(str(path))}" for path in setup_files]
        + [f"{shlex.quote(sys.executable)} -c 'import os; print(chr(0).join(f\"{{k}}={{v}}\" for k, v in os.environ.items()))'"]
    )
    result = subprocess.run(
        ["bash", "-c", command],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        text=False,
    )
    environment = os.environ.copy()
    for item in result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            environment[key.decode()] = value.decode()
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


def main() -> int:
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
            "--event-handlers",
            "console_cohesion+",
        ]
    else:
        command = [
            "colcon",
            "test-result",
            "--test-result-base",
            str(build),
            "--verbose",
        ]

    packages = getattr(args, "packages", None)
    if args.action in {"build", "test"}:
        command.extend(["--packages-skip", *STANDALONE_EXCLUDED_PACKAGES])
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
    result = subprocess.run(
        command, cwd=ROOT, env=environment, check=False
    ).returncode
    if result == 0 and args.action == "check":
        data_check = [sys.executable, str(ROOT / "tools" / "data.py"), "check"]
        print("+", shlex.join(data_check), flush=True)
        result = subprocess.run(
            data_check, cwd=ROOT, env=environment, check=False
        ).returncode
    return result


if __name__ == "__main__":
    raise SystemExit(main())

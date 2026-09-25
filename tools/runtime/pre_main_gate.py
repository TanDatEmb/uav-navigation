#!/usr/bin/env python3
"""Run the authoritative pre-main build, required tests, and static gates."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
PYTHON = sys.executable


def static_guard_names() -> tuple[str, ...]:
    """Permanent semantic guards; branch-delta checks are pre-merge evidence only."""
    return (
        "check_mission_authority_cut.py",
        "check_execution_authority_cut.py",
        "check_desired_intent_cut.py",
        "check_failclosed_ownership_fencing.py",
        "check_exact_optimization_injection.py",
        "check_world_evidence_non_authority.py",
        "check_navigation_command_contract.py",
        "check_runtime_config_truth.py",
        "check_state_transport_trace_scope.py",
    )


def run(label: str, command: list[str], *, shell: bool = False) -> bool:
    print(f"\n=== {label} ===", flush=True)
    result = subprocess.run(
        command,
        cwd=ROOT,
        check=False,
        shell=shell,
        env=os.environ.copy(),
    )
    print(f"{label}: {'PASS' if result.returncode == 0 else 'FAIL'}", flush=True)
    return result.returncode == 0


def main() -> int:
    stages: list[tuple[str, list[str], bool]] = [
        (
            "Release product build and provenance",
            [PYTHON, "tools/runtime/build.py", "--mode", "release", "build"],
            False,
        ),
        (
            "PRODUCT_REQUIRED package tests",
            [PYTHON, "tools/runtime/build.py", "--mode", "release", "test"],
            False,
        ),
        (
            "THIRD_PARTY_REQUIRED pinned dependency smoke tests",
            [
                PYTHON,
                "tools/runtime/build.py",
                "--mode",
                "release",
                "test",
                "--packages",
                "ikd_tree_vendor",
                "ikfom_vendor",
                "livox_ros_driver2",
            ],
            False,
        ),
        (
            "THIRD_PARTY_REQUIRED px4_ros2_cpp unit tests",
            [
                "bash",
                "-lc",
                "source /opt/ros/jazzy/setup.bash && "
                "source install/setup.bash && "
                # Product mission authority is in Core; the PX4 library's
                # optional MissionExecutor feature and its test suite are not
                # used by this product path.
                # One pinned upstream resume test is independently flaky in
                # this environment; keep the rest of that fixture in the
                # third-party contract smoke instead of skipping its whole
                # test class.
                "GTEST_FILTER='-MissionExecutionTester.resumeFromLandedInRtl' "
                "ctest --test-dir build/px4_ros2_cpp --output-on-failure "
                "-R '^px4_ros2_cpp_unit_tests$'",
            ],
            False,
        ),
        (
            "Python runtime/tooling suite",
            [PYTHON, "-m", "unittest", "discover", "-s", "tools/runtime/tests", "-p", "test_*.py"],
            False,
        ),
    ]
    checks = static_guard_names()
    for name in checks:
        stages.append((f"Static guard {name}", [PYTHON, f"tools/{name}"], False))
    stages.extend(
        [
            (
                "Runtime safety ledger",
                [PYTHON, "tools/validate_runtime_safety_ledger.py"],
                False,
            ),
            ("git diff --check", ["git", "diff", "--check"], False),
        ]
    )

    failures: list[str] = []
    for label, command, shell in stages:
        if not run(label, command, shell=shell):
            failures.append(label)
            if label == "Release product build and provenance":
                break

    print(
        "\nENVIRONMENT_GATED px4_ros2_cpp integration_tests: "
        "NOT_RUN (this gate does not start or assume a live PX4 FMU)",
        flush=True,
    )
    print("OPTIONAL_EXAMPLE packages: NOT_RUN (pinned upstream examples)", flush=True)
    if failures:
        print("PRE-MAIN GATE: FAIL", flush=True)
        for label in failures:
            print(f"  failed: {label}", flush=True)
        return 1
    print("PRE-MAIN GATE: PASS", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

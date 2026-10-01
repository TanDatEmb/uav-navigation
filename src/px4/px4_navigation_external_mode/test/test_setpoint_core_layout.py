#!/usr/bin/env python3
"""Structural contract for the ROS-free px4_setpoint_core target."""

from __future__ import annotations

import re
import sys
from pathlib import Path


PURE_HEADERS = (
    "command_admission_assessment.hpp",
    "command_acceptance_gate.hpp",
    "tracking_envelope.hpp",
    "velocity_only_continuity.hpp",
    "px4_tracking_adapter.hpp",
    "certified_command_handoff.hpp",
    "mission_command_identity.hpp",
    "planner_recovery.hpp",
    "local_frame_alignment.hpp",
    "navigation_input_validation.hpp",
    "runtime_metrics_policy.hpp",
)

PURE_TEST_TARGETS = (
    "test_navigation_command",
    "test_tracking_envelope",
    "test_local_frame_alignment",
    "test_px4_tracking_adapter",
    "test_velocity_only_continuity",
)

HELPER_DESTINATIONS = {
    "floatRepresentable": "navigation_input_validation.hpp",
    "checkedEnuToNed": "local_frame_alignment.hpp",
    "checkedTimestampAdd": "planner_recovery.hpp",
}


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_setpoint_core_layout.py CMakeLists.txt")
    cmake = Path(sys.argv[1]).read_text(encoding="utf-8")
    errors: list[str] = []

    if "add_library(px4_setpoint_core STATIC" not in cmake:
        errors.append("missing STATIC px4_setpoint_core target")
    for header in PURE_HEADERS:
        if header not in cmake:
            errors.append(f"pure header is not listed in px4_setpoint_core: {header}")
    if "px4_setpoint_core" in cmake:
        core_block = cmake.split("add_library(px4_setpoint_core STATIC", 1)[1]
        core_block = core_block.split("add_library(${PROJECT_NAME}_adapter", 1)[0]
        for forbidden in ("rclcpp", "px4_ros2_cpp"):
            if forbidden in core_block:
                errors.append(f"forbidden ROS/PX4 dependency in core block: {forbidden}")
    for target in PURE_TEST_TARGETS:
        marker = f"target_link_libraries({target}"
        if marker not in cmake:
            errors.append(f"missing link declaration for {target}")
        else:
            link_block = cmake.split(marker, 1)[1].split(")", 1)[0]
            if "px4_setpoint_core" not in link_block:
                errors.append(f"{target} does not link px4_setpoint_core")

    for header in PURE_HEADERS:
        include = f'#include "px4_navigation_external_mode/{header}"'
        if include not in cmake:
            errors.append(f"anchor does not include pure header: {header}")
    for helper, header in HELPER_DESTINATIONS.items():
        header_path = Path(sys.argv[1]).parent / "include" / "px4_navigation_external_mode" / header
        if helper not in header_path.read_text(encoding="utf-8"):
            errors.append(f"{helper} is not moved to {header}")
        node_path = Path(sys.argv[1]).parent / "src" / "navigation_mode_node.cpp"
        node_text = node_path.read_text(encoding="utf-8")
        if re.search(rf"(?m)^(?:bool|std::optional<[^>]+>)\s+{re.escape(helper)}\(", node_text):
            errors.append(f"{helper} remains locally defined in navigation_mode_node.cpp")

    if "install(EXPORT" in cmake or "ament_export_targets" in cmake:
        errors.append("px4_setpoint_core must remain package-private; do not export a target")

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: px4_setpoint_core target and pure test links are structurally present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

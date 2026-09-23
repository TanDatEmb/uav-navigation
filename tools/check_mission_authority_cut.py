#!/usr/bin/env python3
"""Static product call-path guard for the first Core mission-authority cut."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "src/px4/px4_navigation_external_mode"
RUNTIME = ROOT / "src/runtime/navigation_runtime"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    adapter_text = "\n".join(
        (ADAPTER / path).read_text()
        for path in (
            "include/px4_navigation_external_mode/navigation_mode.hpp",
            "src/navigation_mode_node.cpp",
        )
    )
    for forbidden in (
        r"\bMissionController\b",
        r"\bMissionControllerEvent\b",
        r"\bgoal_publisher_\b",
        r"\bmission_terminal_\b",
        r"\bsafety_suffix_handoff_pending_\b",
        r"\blast_completed_waypoint_index_\b",
        r"\blast_completed_request_id_\b",
        r"\badvanceRequestId\s*\(",
        r"navigation\.mission_file",
    ):
        require(not re.search(forbidden, adapter_text),
                f"adapter product path still contains {forbidden}")
    require("create_publisher<navigation_contracts::msg::NavigationGoal>" not in adapter_text,
            "adapter still publishes a successor goal")
    runtime_text = (RUNTIME / "src/navigation_runtime_node.cpp").read_text()
    require("mission_progress_.emplace" in runtime_text,
            "Core does not own the loaded mission")
    require("makeMissionGoal(*mission_progress_" in runtime_text,
            "Core does not produce internal successor intent")
    require("applyValidatedGoalLocked(" in runtime_text,
            "Core does not reuse the goal transition")
    require("onCommandAdmission(" in runtime_text and
            "rememberMissionCommandIssued(command);" in runtime_text,
            "Core progression lacks exact adapter admission evidence")
    require("onNavigationCommand(" in adapter_text and
            "command_admission_publisher_->publish(receipt)" in adapter_text,
            "adapter does not report admitted command identity")
    require("message->mode_activation_id == mode_activation_id_" in adapter_text and
            "command.mode_activation_id = mission_progress_" in runtime_text,
            "command transport does not fence a prior mode activation")
    require("external NavigationGoal rejected: Core owns mission" in runtime_text,
            "external goal path can supersede Core mission")
    cmake = (ADAPTER / "CMakeLists.txt").read_text()
    adapter_link = cmake.split("target_link_libraries(${PROJECT_NAME}_adapter", 1)[1].split(")", 1)[0]
    require("${PROJECT_NAME}_contract" not in adapter_link,
            "adapter still links the legacy mission authority library")
    px4_launch = (ROOT / "src/navigation_bringup/launch/px4_external_mode.launch.py").read_text()
    require("navigation.mission_file" not in px4_launch,
            "PX4 launch still gives adapter a mission policy source")
    print("MISSION_AUTHORITY_STATIC_CHECK: PASS")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as error:
        print(f"MISSION_AUTHORITY_STATIC_CHECK: FAIL: {error}", file=sys.stderr)
        sys.exit(1)

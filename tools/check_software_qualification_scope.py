#!/usr/bin/env python3
"""Guard the C0-SW witness cut against unrelated flight-authority edits.

This is a change-scope check, not a proof of behavior equivalence. Runtime
parity and the source review remain separate gates.
"""

from pathlib import Path
import subprocess
import sys


BASE = "c422b8485a372e5b3a792682ef0773784b6cf1c4"
ROOT = Path(__file__).resolve().parents[1]
ALLOWED_PRODUCT = {
    "src/contracts/navigation_contracts/msg/NavigationExecutionDiagnostics.msg",
    "src/planning/navigation_planning/include/navigation_planning/candidate_bundle.hpp",
    "src/planning/navigation_planning_backend/src/planner_core/planner.cpp",
    "src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp",
    "src/runtime/navigation_runtime/include/navigation_runtime/retained_decision_observation.hpp",
    "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp",
    "src/runtime/navigation_runtime/test/test_navigation_runtime_terminal_monitor.cpp",
}


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True)


def main() -> int:
    changed = set(git("diff", "--name-only", BASE).splitlines())
    changed.update(git("ls-files", "--others", "--exclude-standard").splitlines())
    unexpected = sorted(
        path for path in changed if path.startswith("src/") and
        path not in ALLOWED_PRODUCT
    )
    if unexpected:
        print("unexpected product source changes:", *unexpected, sep="\n  ")
        return 1
    protected = (
        "src/contracts/navigation_contracts/msg/NavigationCommand.msg",
        "src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp",
    )
    if any(path in changed for path in protected):
        print("flight control or execution authority contract changed")
        return 1
    print("C0-SW source scope: PASS (behavior equivalence requires separate review)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

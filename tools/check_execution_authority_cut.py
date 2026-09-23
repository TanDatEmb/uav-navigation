#!/usr/bin/env python3
"""Static guard against restoring a second mutable Core execution owner."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "src/runtime/navigation_runtime"
EXECUTION = ROOT / "src/execution/navigation_execution"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    node_h = (RUNTIME / "include/navigation_runtime/navigation_runtime_node.hpp").read_text()
    node_cpp = (RUNTIME / "src/navigation_runtime_node.cpp").read_text()
    owner = (EXECUTION / "include/navigation_execution/execution_authority.hpp").read_text()
    product = "\n".join((RUNTIME / "include/navigation_runtime" / p).read_text()
                        for p in ("execution_lifecycle_view.hpp", "navigation_runtime_node.hpp"))

    require("class ExecutionAuthority" in owner, "typed execution owner is absent")
    require("struct ActiveRecord" in owner and "struct StagedRecord" in owner,
            "active and staged goals are not paired with their bundles")
    require("ExecutionLifecycleState lifecycle_" in owner,
            "lifecycle is not inside the execution owner")
    require(owner.count("mutable std::mutex mutex_") == 1,
            "execution owner does not have exactly one internal mutex")
    require("navigation_execution::ExecutionAuthority execution_authority_" in node_h,
            "RuntimeNode is not using the typed owner directly")
    for forbidden in (r"\bclass ExecutionEpisode\b", r"\bexecuting_goal_\b",
                      r"\bcommand_goal_epoch_\b", r"\bexecution_episode_\b"):
        require(not re.search(forbidden, product + "\n" + node_cpp),
                f"product retains execution mirror {forbidden}")
    require("execution_authority_.publishIfCurrent(" in node_cpp,
            "publication bypasses owner authorization")
    require("sameExecutionPublicationIdentity(" not in node_cpp,
            "RuntimeNode reconstructs final publication identity outside owner")
    require("mission_progress_.emplace" in node_cpp,
            "MissionProgress no longer owns mission activation")
    print("EXECUTION_AUTHORITY_STATIC_CHECK: PASS")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as error:
        print(f"EXECUTION_AUTHORITY_STATIC_CHECK: FAIL: {error}", file=sys.stderr)
        sys.exit(1)

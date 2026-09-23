#!/usr/bin/env python3
"""Guard the audited async destructive seams against accidental raw mutation.

This is a narrow source guard, not a C++ parser or a substitute for the race
tests. Global safety and localization-reset call sites are inventoried in the
corresponding audit artifact rather than forbidden by this script.
"""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
AUTHORITY = (
    ROOT
    / "src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp"
)


def section(source: str, begin: str, end: str) -> str:
    start = source.find(begin)
    if start < 0:
        raise ValueError(f"missing section start: {begin}")
    stop = source.find(end, start + len(begin))
    if stop < 0:
        raise ValueError(f"missing section end: {end}")
    return source[start:stop]


def main() -> int:
    source = RUNTIME.read_text(encoding="utf-8")
    owner = AUTHORITY.read_text(encoding="utf-8")
    errors: list[str] = []
    try:
        result_failure = section(
            source,
            "if (disposition == PlannerResultDisposition::FailClosed)",
            "if (disposition == PlannerResultDisposition::RestartFromRest)",
        )
        stopped_retry = section(
            source,
            "if (disposition == PlannerResultDisposition::RetryFromRest)",
            "if (disposition == PlannerResultDisposition::RetainCommittedCommand",
        )
        restart = section(
            source,
            "if (disposition == PlannerResultDisposition::RestartFromRest)",
            "if (disposition == PlannerResultDisposition::RetryFromRest)",
        )
        watchdog = section(
            source,
            "std::unique_lock<std::mutex> solve_activity_lock",
            "Eigen::Matrix<double, 3, 4> pvaj",
        )
        retained = section(
            source,
            "void NavigationRuntimeNode::validateRetainedCommand(",
            "void NavigationRuntimeNode::publishCommand()",
        )
        publisher = source[source.index("void NavigationRuntimeNode::publishCommand()") :]
    except ValueError as error:
        errors.append(str(error))
    else:
        for name, body, required in [
            ("planner failure", result_failure, "failClosedIfCurrentSnapshot"),
            ("stopped recovery", stopped_retry, "failClosedIfCurrentSnapshot"),
            ("planner watchdog", watchdog, "active_planner_solve_witness_"),
        ]:
            if required not in body:
                errors.append(f"{name}: missing exact async ownership witness")
            if "failClosedLocked()" in body or "pending_goal_owner_.clearGoal()" in body:
                errors.append(f"{name}: raw destructive mutation reintroduced")
        if "isCurrentSnapshot(*context.expected_execution)" not in retained:
            errors.append("retained result: missing solve execution witness")
        if "isCurrentSnapshot(execution_at_solve)" not in restart:
            errors.append("restart result: missing solve execution witness")
        if "failedExecutionLeaseIsCurrent(" not in retained:
            errors.append("retained result: missing causal state-lease check")
        if publisher.count("failedExecutionLeaseIsCurrent(") < 3:
            errors.append("publisher: missing callback-start/final lease supersession checks")
        if "suspendCommandForWorldFreshness(execution_at_command)" not in publisher:
            errors.append("publisher: missing world-suspension execution witness")
        if "clearIfCurrent(" not in watchdog:
            errors.append("watchdog: pending cleanup is not conditional")
    if "failClosedIfCurrentSnapshot(" not in owner or "suspendIfCurrentSnapshot(" not in owner:
        errors.append("ExecutionAuthority: missing atomic conditional mutation API")
    if errors:
        print("FAILCLOSED_OWNERSHIP_STATIC_CHECK: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("FAILCLOSED_OWNERSHIP_STATIC_CHECK: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

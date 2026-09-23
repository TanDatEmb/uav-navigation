#!/usr/bin/env python3
"""Narrow static guard for removable, write-only experiment instrumentation."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = ROOT / "src"
ALLOWED = {
    "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp",
    "src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp",
    "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp",
    "src/px4/px4_navigation_external_mode/src/mission_controller.cpp",
    "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/navigation_mode.hpp",
    "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/mission_controller.hpp",
    "src/contracts/navigation_contracts/include/navigation_contracts/audit_event_sink.hpp",
}
NAMES = re.compile(r"\b(?:audit_sink_|MissionGateAudit|emitWorldAudit|emitHoldAudit|emitLeaseAudit|receive_audit|mission_audit)\b")


def check():
    errors = []
    for path in SOURCE_ROOT.rglob("*"):
        if path.suffix not in {".cpp", ".hpp", ".h"} or not path.is_file():
            continue
        relative = str(path.relative_to(ROOT))
        content = path.read_text(errors="replace")
        if re.search(r"create_subscription\s*<\s*(?:navigation_contracts::msg::)?AuditEvent", content):
            errors.append(f"{relative}: control subscriber to audit topic")
        if NAMES.search(content) and relative not in ALLOWED:
            errors.append(f"{relative}: audit symbol outside experiment allowlist")
        if relative in ALLOWED and relative != "src/contracts/navigation_contracts/include/navigation_contracts/audit_event_sink.hpp":
            # Each audit symbol must be lexically guarded by the compile switch.
            stack = []
            for line_number, line in enumerate(content.splitlines(), 1):
                stripped = line.strip()
                if stripped.startswith("#if ") or stripped.startswith("#ifdef ") or stripped.startswith("#ifndef "):
                    stack.append("NAVIGATION_AUDIT_INSTRUMENTATION" in stripped)
                elif stripped.startswith("#endif") and stack:
                    stack.pop()
                if NAMES.search(line) and not any(stack):
                    errors.append(f"{relative}:{line_number}: audit symbol outside compile guard")
    for path in SOURCE_ROOT.rglob("*.cpp"):
        content = path.read_text(errors="replace")
        if '"/navigation/audit_event"' in content:
            errors.append(f"{path.relative_to(ROOT)}: audit topic in product cpp")
    return errors


if __name__ == "__main__":
    problems = check()
    for problem in problems:
        print(problem, file=sys.stderr)
    if problems:
        raise SystemExit(1)
    print("PASS: audit symbols confined to guarded experiment hooks; no audit subscriber")

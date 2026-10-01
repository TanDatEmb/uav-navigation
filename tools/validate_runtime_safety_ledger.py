#!/usr/bin/env python3
"""Validate the runtime-safety current contract.

This is intentionally a small, dependency-free structural guard. It does not
interpret safety semantics or certify evidence; it catches size drift,
duplicate register identifiers, missing removal conditions and broken local
links in the safety-document set.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SAFETY_DIR = ROOT / "docs/safety"
CURRENT = SAFETY_DIR / "runtime_safety_current.md"
MAX_CURRENT_LINES = 500


def register_ids(text: str, prefix: str) -> list[str]:
    return re.findall(rf"^\|\s*({prefix}-\d+)\s*\|", text, re.M)


def main() -> int:
    errors: list[str] = []
    if not CURRENT.is_file():
        print(f"missing required file: {CURRENT.relative_to(ROOT)}", file=sys.stderr)
        return 1

    current = CURRENT.read_text(encoding="utf-8")
    current_lines = current.count("\n")
    if current_lines > MAX_CURRENT_LINES:
        errors.append(f"current contract exceeds {MAX_CURRENT_LINES} lines: {current_lines}")

    gates = register_ids(current, "HG")
    bypasses = register_ids(current, "TB")
    for label, ids in (("gate", gates), ("bypass", bypasses)):
        if len(ids) != len(set(ids)):
            errors.append(f"duplicate {label} IDs in current contract")
    if "TB-003" in current and "Removal condition" not in current:
        errors.append("active TB-003 is missing an explicit removal condition")

    for path in sorted(SAFETY_DIR.glob("*.md")):
        for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", path.read_text(encoding="utf-8")):
            if re.match(r"[a-z]+:", target):
                continue
            if not (path.parent / target).resolve().exists():
                errors.append(f"broken link in {path.relative_to(ROOT)}: {target}")

    if errors:
        print("runtime safety ledger validation: FAIL", file=sys.stderr)
        print("\n".join(f"- {error}" for error in errors), file=sys.stderr)
        return 1
    print(
        "runtime safety ledger validation: PASS "
        f"(current={current_lines} lines, gates={len(gates)}, bypasses={len(bypasses)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

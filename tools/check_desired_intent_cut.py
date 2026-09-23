#!/usr/bin/env python3
"""Reject legacy desired-intent and execution-owner vocabulary in product code.

Historical artifacts and documentation are intentionally outside this scan.
The active_goal_ check is scoped to RuntimeNode: ExecutionAuthority may expose
the legitimate active_goal execution concept, while RuntimeNode must not use
that spelling for its desired planning intent.
"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "src/runtime/navigation_runtime"
EXECUTION = ROOT / "src/execution/navigation_execution"
PRODUCT_SUFFIXES = {".h", ".hh", ".hpp", ".cc", ".cpp", ".cxx"}


def product_files(*roots: Path) -> list[Path]:
    files: list[Path] = []
    for root in roots:
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in PRODUCT_SUFFIXES:
                continue
            relative_parts = path.relative_to(root).parts
            if "test" in relative_parts or "tests" in relative_parts:
                continue
            files.append(path)
    return sorted(files)


def scan(files: list[Path], patterns: list[tuple[str, re.Pattern[str]]]) -> list[str]:
    findings: list[str] = []
    for path in files:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as error:
            findings.append(f"{path.relative_to(ROOT)}: cannot read product source: {error}")
            continue
        for line_number, line in enumerate(lines, start=1):
            for label, pattern in patterns:
                if pattern.search(line):
                    findings.append(
                        f"{path.relative_to(ROOT)}:{line_number}: prohibited {label}: {line.strip()}"
                    )
    return findings


def main() -> int:
    runtime_files = product_files(RUNTIME)
    execution_files = product_files(EXECUTION)
    all_product_files = sorted(set(runtime_files + execution_files))

    # active_goal_ names the old RuntimeNode desired-goal mirror. The public
    # ExecutionAuthoritySnapshot.active_goal field is intentionally allowed.
    runtime_patterns = [
        ("RuntimeNode active_goal_ mirror", re.compile(r"\bactive_goal_(?!epoch_)")),
        ("active_goal_epoch_ mirror", re.compile(r"\bactive_goal_epoch_")),
        ("new_goal_ boolean", re.compile(r"\bnew_goal_")),
        ("hot_goal_transition_ boolean", re.compile(r"\bhot_goal_transition_")),
        ("RuntimeNode desired identity helper", re.compile(r"\bdesiredGoalIdentityMatchesLocked\b")),
        ("RuntimeNode execution identity helper", re.compile(r"\bexecutingCommandIdentityMatchesLocked\b")),
    ]
    legacy_patterns = [
        ("ExecutionTimelineStore", re.compile(r"\bExecutionTimelineStore\b")),
        ("CommittedBundleStore", re.compile(r"\bCommittedBundleStore\b")),
        ("ExecutionTimelineSnapshot", re.compile(r"\bExecutionTimelineSnapshot\b")),
        ("ExecutionEpisode internal concept", re.compile(r"\bExecutionEpisode(?:Snapshot|Phase)?\b")),
    ]
    findings = scan(runtime_files, runtime_patterns)
    findings.extend(scan(all_product_files, legacy_patterns))

    # The old owner name must not survive as an authoritative source/header
    # filename after the migration. Test fixtures and historical artifacts are
    # outside the product-source scan above.
    for path in all_product_files:
        lowered_name = path.name.lower()
        if "committed_bundle_store" in lowered_name:
            findings.append(
                f"{path.relative_to(ROOT)}: prohibited legacy source filename committed_bundle_store"
            )

    if findings:
        print("DESIRED_INTENT_STATIC_CHECK: FAIL")
        for finding in findings:
            print(f"- {finding}")
        return 1

    print("DESIRED_INTENT_STATIC_CHECK: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

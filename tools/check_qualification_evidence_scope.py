#!/usr/bin/env python3
"""Guard the evidence branch against product flight-behavior edits.

This branch is intentionally scoped to evidence tooling. The pinned product
source at the entry SHA must remain byte-identical, including uncommitted work.
"""

from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PRODUCT_PATHS = ("src",)


def changed_files(*arguments: str) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--name-only", *arguments, "--", *PRODUCT_PATHS],
        cwd=ROOT, text=True, capture_output=True, check=True,
    )
    return [line for line in result.stdout.splitlines() if line]


def validate_base(base: str) -> bool:
    available = subprocess.run(
        ["git", "cat-file", "-e", f"{base}^{{commit}}"],
        cwd=ROOT, text=True, capture_output=True, check=False,
    )
    if available.returncode != 0:
        print(f"SCOPE_BASE_UNAVAILABLE: {base}")
        return False
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", base, "HEAD"],
        cwd=ROOT, text=True, capture_output=True, check=False,
    )
    if ancestor.returncode != 0:
        print(f"SCOPE_BASE_INVALID: {base} is not an ancestor of HEAD")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="explicit reachable pre-merge baseline commit")
    args = parser.parse_args()
    if not validate_base(args.base):
        return 2
    changed = sorted(set(
        changed_files(args.base, "HEAD")
        + changed_files()
        + changed_files("--cached")
        + subprocess.check_output(
            ["git", "ls-files", "--others", "--exclude-standard", "--", *PRODUCT_PATHS],
            cwd=ROOT, text=True,
        ).splitlines()
    ))
    if changed:
        print("EVIDENCE_SCOPE_VIOLATION: product source changed")
        for path in changed:
            print(path)
        return 1
    print("EVIDENCE_SCOPE_PASS: product src/ byte-identical to base")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Guard the evidence branch against product flight-behavior edits.

This branch is intentionally scoped to evidence tooling. The pinned product
source at the entry SHA must remain byte-identical, including uncommitted work.
"""

from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
# The pre-main integration campaign freezes the already-approved architecture
# at the incoming World milestone.  Historical evidence-cut commits predate
# that architecture and are not a valid scope baseline here.
BASE = "0b477638d21ce60cdb42ed85fb7c2d568bf500ed"
PRODUCT_PATHS = ("src",)


def changed_files(*arguments: str) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--name-only", *arguments, "--", *PRODUCT_PATHS],
        cwd=ROOT, text=True, capture_output=True, check=True,
    )
    return [line for line in result.stdout.splitlines() if line]


def main() -> int:
    changed = sorted(set(
        changed_files(BASE, "HEAD")
        + changed_files()
        + changed_files("--cached")
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

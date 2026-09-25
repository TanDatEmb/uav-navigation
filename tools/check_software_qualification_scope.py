#!/usr/bin/env python3
"""Guard the C0-SW witness cut against unrelated flight-authority edits.

This is a change-scope check, not a proof of behavior equivalence. Runtime
parity and the source review remain separate gates.
"""

from pathlib import Path
import argparse
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
ALLOWED_PRODUCT: set[str] = set()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True)


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
    changed = set(git("diff", "--name-only", args.base).splitlines())
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

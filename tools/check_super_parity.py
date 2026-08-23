#!/usr/bin/env python3
"""Check vendored SUPER files against the pinned upstream checkout.

The ROS/ROG adapter is intentionally outside this list.  Only the small
portability includes needed by this workspace are allowed in core files.
"""

from __future__ import annotations

import argparse
import difflib
import subprocess
import sys
from pathlib import Path


PINNED_COMMIT = "2ad3419c127a617c6d7df6925e81a14175a9c096"
ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "src/planning/super_planner_vendor"

FILES = (
    "include/data_structure/cmd_traj.h",
    "include/data_structure/exp_traj.h",
    "include/data_structure/backup_traj.h",
    "include/path_search/astar.h",
    "include/super_core/config.hpp",
    "include/super_core/fov_checker.h",
    "include/super_core/log_utils.hpp",
    "include/super_core/super_planner.h",
    "include/super_core/super_ret_code.hpp",
    "include/traj_opt/backup_traj_optimizer_s4.h",
    "include/traj_opt/config.hpp",
    "src/super_core/astar.cpp",
    "src/super_core/super_planner.cpp",
    "src/traj_opt/backup_traj_optimizer_s4.cpp",
    "src/utils/trajectory.cpp",
)


def normalized(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").splitlines()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream-dir", type=Path, required=True)
    args = parser.parse_args()
    upstream = args.upstream_dir.resolve()
    if not upstream.is_dir():
        print(f"upstream directory does not exist: {upstream}", file=sys.stderr)
        return 2
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"cannot inspect upstream git checkout: {error}", file=sys.stderr)
        return 2
    if commit != PINNED_COMMIT:
        print(f"wrong upstream commit: {commit}; expected {PINNED_COMMIT}", file=sys.stderr)
        return 1

    failures = 0
    for relative in FILES:
        expected = upstream / relative
        actual = VENDOR / relative
        if not expected.is_file() or not actual.is_file():
            print(f"missing parity file: {relative}", file=sys.stderr)
            failures += 1
            continue
        expected_lines = normalized(expected)
        actual_lines = normalized(actual)
        comparable_lines = actual_lines
        if relative == "include/path_search/astar.h":
            comparable_lines = []
            for index, line in enumerate(actual_lines):
                if line == "#include <mutex>" or (
                    line == "" and index + 1 < len(actual_lines) and
                    actual_lines[index + 1] == "#include <mutex>"
                ):
                    continue
                comparable_lines.append(line)
        if relative == "src/super_core/super_planner.cpp":
            comparable_lines = [
                line for line in actual_lines
                if line not in {"#include <cmath>", "using std::isnan;"}
            ]
        if expected_lines == comparable_lines:
            continue
        diff = list(
            difflib.unified_diff(
                expected_lines, actual_lines,
                fromfile=f"upstream/{relative}", tofile=f"vendor/{relative}",
                n=1,
            )
        )
        print("".join(f"{line}\n" for line in diff), file=sys.stderr)
        failures += 1
    if failures:
        print(f"SUPER parity FAILED ({failures} file(s))", file=sys.stderr)
        return 1
    print(f"SUPER parity OK at {PINNED_COMMIT}: {len(FILES)} files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Audit the vendored planner backend core against the pinned upstream checkout.

Exact upstream files must remain byte-identical. Product-port files may differ
only when they are explicitly listed in ``DOCUMENTED_DELTA_FILES`` and
described in ``navigation_planning_backend/UPSTREAM.md``. A newly changed, missing, or
unclassified core file makes this check fail.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


PINNED_COMMIT = "2ad3419c127a617c6d7df6925e81a14175a9c096"
ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "src/planning/navigation_planning_backend"
PORT_MANIFEST = VENDOR / "PLANNER_BACKEND_PORT_SHA256.json"

# These are source-level ROS 2/product port deltas, not silently accepted
# upstream drift. Keep this list synchronized with UPSTREAM.md.
DOCUMENTED_DELTA_FILES = frozenset(
    {
        "include/data_structure/base/polytope.h",
        "include/data_structure/base/piece.h",
        "include/data_structure/base/trajectory.h",
        "include/data_structure/cmd_traj.h",
        "include/data_structure/exp_traj.h",
        "include/path_search/astar.h",
        "include/path_search/config.hpp",
        "include/super_core/ciri.h",
        "include/super_core/config.hpp",
        "include/super_core/corridor_generator.h",
        "include/super_core/fov_checker.h",
        "include/super_core/guide_endpoint.hpp",
        "include/super_core/super_planner.h",
        "include/super_core/super_ret_code.hpp",
        "include/traj_opt/backup_traj_optimizer_s4.h",
        "include/traj_opt/exp_traj_optimizer_s4.h",
        "include/traj_opt/yaw_traj_opt.h",
        "include/utils/geometry/quadrotor_flatness.hpp",
        "include/utils/geometry/quickhull.h",
        "include/utils/header/backward.hpp",
        "include/utils/header/color_text.hpp",
        "include/utils/header/eigen_alias.hpp",
        "include/utils/header/fmt_eigen.hpp",
        "include/utils/header/scope_timer.hpp",
        "include/utils/header/type_utils.hpp",
        "include/utils/header/yaml_loader.hpp",
        "include/utils/optimization/polynomial_interpolation.h",
        "include/utils/optimization/sdlp.h",
        "src/super_core/astar.cpp",
        "src/super_core/corridor_generator.cpp",
        "src/super_core/super_planner.cpp",
        "src/traj_opt/backup_traj_optimizer_s4.cpp",
        "src/traj_opt/exp_traj_optimizer_s4.cpp",
        "src/traj_opt/yaw_traj_opt.cpp",
        "src/utils/lbfgs.cpp",
        "src/utils/piece.cpp",
        "src/utils/trajectory.cpp",
        "src/utils/minco.cpp",
        "src/utils/optimization_utils.cpp",
        "src/utils/quickhull.cpp",
        "src/utils/sdlp.cpp",
    }
)

# Product-only helpers have no upstream counterpart, but are still part of the
# audited core surface and must be documented explicitly in UPSTREAM.md.
DOCUMENTED_VENDOR_ONLY_FILES = frozenset(
    {
        "include/super_core/backup_braking.hpp",
        "include/traj_opt/trajectory_dynamics.hpp",
    }
)

TRACKED_PREFIXES = (
    "include/data_structure/",
    "include/path_search/",
    "include/super_core/",
    "include/traj_opt/",
    "include/utils/geometry/",
    "include/utils/header/",
    "include/utils/optimization/",
    "src/super_core/",
    "src/traj_opt/",
    "src/utils/",
)
EXCLUDED_UPSTREAM_FILES = frozenset(
    {
        "include/utils/header/matplotlibcpp.hpp",
        "include/utils/header/tinycolormap.hpp",
        "src/super_core/fsm.cpp",
    }
)


def tracked_files(root: Path) -> set[str]:
    return {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and path.relative_to(root).as_posix().startswith(TRACKED_PREFIXES)
        and path.relative_to(root).as_posix() not in EXCLUDED_UPSTREAM_FILES
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream-dir", type=Path, required=True)
    args = parser.parse_args()
    upstream_repo = args.upstream_dir.resolve()
    upstream = upstream_repo / "super_planner"
    if not upstream.is_dir():
        print(
            f"upstream planner backend package does not exist: {upstream}", file=sys.stderr
        )
        return 2
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(upstream_repo), "rev-parse", "HEAD"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"cannot inspect upstream git checkout: {error}", file=sys.stderr)
        return 2
    if commit != PINNED_COMMIT:
        print(
            f"wrong upstream commit: {commit}; expected {PINNED_COMMIT}",
            file=sys.stderr,
        )
        return 1
    try:
        manifest = json.loads(PORT_MANIFEST.read_text(encoding="utf-8"))
        manifest_commit = manifest["pinned_upstream_commit"]
        manifest_files = manifest["files"]
        if not isinstance(manifest_files, dict) or not all(
            isinstance(path, str) and isinstance(digest, str)
            for path, digest in manifest_files.items()
        ):
            raise ValueError("files must be a string-to-string mapping")
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as error:
        print(f"invalid planner backend port manifest: {error}", file=sys.stderr)
        return 2
    if manifest_commit != PINNED_COMMIT:
        print(
            f"planner backend port manifest pins {manifest_commit}; expected {PINNED_COMMIT}",
            file=sys.stderr,
        )
        return 1

    upstream_files = tracked_files(upstream)
    vendor_files = tracked_files(VENDOR)
    failures: list[str] = []
    missing = sorted(upstream_files - vendor_files)
    extra = sorted(vendor_files - upstream_files - DOCUMENTED_VENDOR_ONLY_FILES)
    failures.extend(f"missing vendored core file: {path}" for path in missing)
    failures.extend(f"unclassified vendor core file: {path}" for path in extra)

    exact = 0
    documented = len(DOCUMENTED_VENDOR_ONLY_FILES & vendor_files)
    expected_manifest_files: set[str] = set()
    for relative in sorted(DOCUMENTED_VENDOR_ONLY_FILES & vendor_files):
        expected_manifest_files.add(relative)
        print(f"DOCUMENTED VENDOR DELTA  {relative}")
    for relative in sorted(upstream_files & vendor_files):
        if (upstream / relative).read_bytes() == (VENDOR / relative).read_bytes():
            exact += 1
            continue
        if relative not in DOCUMENTED_DELTA_FILES:
            failures.append(f"undocumented upstream drift: {relative}")
            continue
        documented += 1
        expected_manifest_files.add(relative)
        print(f"DOCUMENTED DELTA  {relative}")

    manifest_paths = set(manifest_files)
    for relative in sorted(expected_manifest_files - manifest_paths):
        failures.append(f"missing planner backend port manifest hash: {relative}")
    for relative in sorted(manifest_paths - expected_manifest_files):
        failures.append(f"stale planner backend port manifest hash: {relative}")
    for relative in sorted(expected_manifest_files & manifest_paths):
        actual_digest = sha256(VENDOR / relative)
        if manifest_files[relative] != actual_digest:
            failures.append(
                f"reviewed planner backend port hash changed: {relative} "
                f"(expected {manifest_files[relative]}, got {actual_digest})"
            )

    stale_allowlist = sorted(
        DOCUMENTED_DELTA_FILES - (upstream_files & vendor_files)
    )
    failures.extend(
        f"stale documented-delta entry: {path}" for path in stale_allowlist
    )
    stale_vendor_only = sorted(DOCUMENTED_VENDOR_ONLY_FILES - vendor_files)
    failures.extend(
        f"stale vendor-only delta entry: {path}" for path in stale_vendor_only
    )
    unexpected_upstream = sorted(DOCUMENTED_VENDOR_ONLY_FILES & upstream_files)
    failures.extend(
        f"vendor-only delta now exists upstream: {path}" for path in unexpected_upstream
    )

    if failures:
        for failure in failures:
            print(failure, file=sys.stderr)
        print(f"planner backend parity audit FAILED ({len(failures)} issue(s))", file=sys.stderr)
        return 1
    print(
        "planner backend parity audit OK at "
        f"{PINNED_COMMIT}: {exact} exact, {documented} documented delta"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

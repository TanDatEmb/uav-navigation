#!/usr/bin/env python3
"""Verify the frozen M1 vendor inventory without modifying it."""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
VENDOR_ROOT = ROOT / "src/navigation_estimator"
MANIFEST = ROOT / "tools/vendor_manifest.tsv"
PINNED = {
    "ikd_tree_vendor": "c0e36a16b6e4d557d3783b16911207f6398dd478",
    "ikfom_vendor": "59cfc095ca74425f9b330c7c04a5d74f68c6dd62",
}
DOCUMENTED_PATCHES = {
    "ikd_tree_vendor/vendor/ikd-Tree/ikd-Tree/ikd_Tree.cpp",
    "ikd_tree_vendor/vendor/ikd-Tree/ikd-Tree/ikd_Tree.h",
    "ikfom_vendor/vendor/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp",
}


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def main() -> int:
    failures: list[str] = []
    expected_files: set[str] = set()
    upstream_differences: set[str] = set()
    for raw in MANIFEST.read_text(encoding="utf-8").splitlines():
        if not raw or raw.startswith("#"):
            continue
        expected_current, expected_upstream, relative = raw.split()
        expected_files.add(relative)
        path = VENDOR_ROOT / relative
        if not path.is_file():
            failures.append(f"missing vendor file: {relative}")
            continue
        observed = sha256(path)
        if observed != expected_current:
            failures.append(
                f"vendor file changed: {relative}: {observed} != {expected_current}"
            )
        if expected_current != expected_upstream:
            upstream_differences.add(relative)

    actual_files = {
        path.relative_to(VENDOR_ROOT).as_posix()
        for package in ("ikd_tree_vendor/vendor/ikd-Tree",
                        "ikfom_vendor/vendor/IKFoM")
        for path in (VENDOR_ROOT / package).rglob("*")
        if path.is_file()
    }
    for extra in sorted(actual_files - expected_files):
        failures.append(f"undocumented vendor file: {extra}")
    if upstream_differences != DOCUMENTED_PATCHES:
        failures.append(
            "upstream differences do not match documented patch set: "
            f"{sorted(upstream_differences)}"
        )
    for package, commit in PINNED.items():
        upstream = (VENDOR_ROOT / package / "UPSTREAM.md").read_text(
            encoding="utf-8"
        )
        if commit not in upstream:
            failures.append(f"{package} does not declare pinned SHA {commit}")
        patches = VENDOR_ROOT / package / "PATCHES.md"
        if upstream_differences and package == "ikd_tree_vendor":
            if not patches.is_file():
                failures.append(f"{package} is missing PATCHES.md")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(
        "vendor freeze OK: 18 files, 2 pinned upstream SHAs, "
        "3 documented patched files"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

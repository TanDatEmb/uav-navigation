#!/usr/bin/env python3
"""Delete stale test results of the selected packages before `make test` (D30).

`colcon test-result` counts every xUnit XML under build/<pkg>, including the XML of tests that no
longer exist. This removes only result files: `*.xml` under build/<pkg>/test_results/ and
build/<pkg>/Testing/*/Test.xml. Directories, other files, symlinks and anything resolving outside
build/<pkg> are never touched. Standard library only.

Usage: clean_test_results.py [--build-base DIR] PKG...    (exit 0 ok, 2 invalid package name)
"""
import argparse
import os
import pathlib
import re
import sys
from typing import Dict, List, Sequence

_NAME = re.compile(r"^[A-Za-z0-9_]+$")


class InvalidPackageName(ValueError):
    pass


def _is_inside(path: pathlib.Path, root: pathlib.Path) -> bool:
    try:
        path.resolve().relative_to(root)
    except ValueError:
        return False
    return True


def _candidates(pkg_dir: pathlib.Path) -> List[pathlib.Path]:
    found: List[pathlib.Path] = []
    results = pkg_dir / "test_results"
    if results.is_dir() and not results.is_symlink():
        for dirpath, dirnames, filenames in os.walk(results, followlinks=False):
            dirnames[:] = [d for d in dirnames if not os.path.islink(os.path.join(dirpath, d))]
            found.extend(pathlib.Path(dirpath) / f for f in filenames if f.endswith(".xml"))
    testing = pkg_dir / "Testing"
    if testing.is_dir() and not testing.is_symlink():
        for stamp in testing.iterdir():
            if stamp.is_dir() and not stamp.is_symlink():
                found.append(stamp / "Test.xml")
    return found


def clean(build_base: pathlib.Path, packages: Sequence[str]) -> Dict[str, int]:
    """Remove stale result files of `packages` under build_base; returns package -> files removed."""
    packages = list(packages)
    for name in packages:
        if not _NAME.fullmatch(name):
            raise InvalidPackageName(f"invalid package name: {name!r}")
    base = pathlib.Path(build_base)
    removed: Dict[str, int] = {}
    for name in packages:
        pkg_dir = base / name
        count = 0
        if pkg_dir.is_dir() and not pkg_dir.is_symlink():
            root = pkg_dir.resolve()
            for path in _candidates(pkg_dir):
                if path.is_symlink() or not path.is_file() or not _is_inside(path, root):
                    continue
                path.unlink()
                count += 1
        removed[name] = removed.get(name, 0) + count
        print(f"clean_test_results: {name}: removed {count} result files")
    return removed


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-base", default="build", type=pathlib.Path)
    parser.add_argument("packages", nargs="+", metavar="PKG")
    args = parser.parse_args(argv)
    try:
        clean(args.build_base, args.packages)
    except InvalidPackageName as err:
        print(f"clean_test_results: {err}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

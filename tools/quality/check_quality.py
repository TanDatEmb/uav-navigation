#!/usr/bin/env python3
"""Fail-closed quality checks used by the opt-in Wave 4 gate."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


SELECTED_CLANG_TIDY = (
    "bugprone-*",
    "cppcoreguidelines-pro-type-member-init",
    "cppcoreguidelines-narrowing-conversions",
    "performance-unnecessary-copy-initialization",
)


def _run(command: list[str], root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=root, text=True, capture_output=True, check=False)


def changed_cpp_files(root: Path, baseline: str = "origin/main") -> list[Path]:
    names: set[str] = set()
    for command in (
        ["git", "diff", "--name-only", f"{baseline}...HEAD"],
        ["git", "diff", "--name-only"],
        ["git", "diff", "--cached", "--name-only"],
    ):
        result = _run(command, root)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "git diff failed")
        names.update(line.strip() for line in result.stdout.splitlines() if line.strip())
    suffixes = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
    return [root / name for name in sorted(names) if Path(name).suffix in suffixes and (root / name).is_file()]


def timer_violations(root: Path, allowlist: Path) -> list[str]:
    allowed: dict[str, str] = {}
    if allowlist.is_file():
        for line in allowlist.read_text(encoding="utf-8").splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            path, finding, *_ = line.split("\t")
            allowed[path] = finding
    violations: list[str] = []
    for path in sorted((root / "src").rglob("*")):
        if not path.is_file() or "test" in path.parts or "navigation_sitl_harness" in path.parts:
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except UnicodeDecodeError:
            continue
        for number, line in enumerate(lines, 1):
            if "create_wall_timer" not in line:
                continue
            relative = path.relative_to(root).as_posix()
            if relative not in allowed:
                violations.append(f"{relative}:{number}: {line.strip()}")
    return violations


def doc_layout_violations(root: Path, layout: Path) -> list[str]:
    package_names: set[str] = set()
    for package in (root / "src").rglob("package.xml"):
        if "examples" in package.parts:
            continue
        try:
            node = ET.parse(package).getroot().find("name")
        except ET.ParseError as error:
            return [f"invalid package.xml {package}: {error}"]
        if node is not None and node.text:
            package_names.add(node.text.strip())
    import re
    text = layout.read_text(encoding="utf-8")
    code_blocks = re.findall(r"```(?:text)?\n(.*?)```", text, flags=re.DOTALL)
    documented = set(re.findall(r"(?m)^\s*(?:src/)?([A-Za-z0-9_]+)/", "\n".join(code_blocks))) - {
        "src", "common", "contracts", "estimation", "mapping", "planning", "execution", "external",
        "px4", "runtime", "tools", "docs", "config",
        "px4_ros2_interface_lib",
    }
    missing = sorted(package_names - documented)
    extra = sorted(documented - package_names)
    return [*(f"package missing from repository_layout.md: {name}" for name in missing),
            *(f"repository_layout.md names unknown package: {name}" for name in extra)]


def _lizard_rows(command: str, files: Iterable[Path], root: Path) -> list[dict[str, object]]:
    paths = [str(path.relative_to(root)) for path in files]
    if not paths:
        return []
    result = _run([command, "--csv", *paths], root)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "lizard failed")
    rows: list[dict[str, object]] = []
    for values in csv.reader(result.stdout.splitlines()):
        if len(values) < 11:
            continue
        rows.append({"ccn": int(values[1]), "file": values[6], "function": values[8]})
    return rows


def baseline_rows(root: Path, lizard: str) -> list[dict[str, object]]:
    files = [
        path for path in sorted((root / "src").rglob("*"))
        if path.suffix in {".c", ".cc", ".cpp", ".cxx"}
        and "test" not in path.parts
        and "vendor" not in path.parts
        and "external" not in path.parts
    ]
    return _lizard_rows(lizard, files, root)


def write_baseline(root: Path, output: Path, lizard: str) -> None:
    rows = baseline_rows(root, lizard)
    values = {f"{row['file']}::{row['function']}": row["ccn"] for row in rows}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"schema": 1, "functions": values}, indent=2, sort_keys=True) + "\n",
                       encoding="utf-8")


def lizard_violations(root: Path, baseline: Path, lizard: str, files: list[Path]) -> list[str]:
    if not baseline.is_file():
        return [f"missing lizard baseline: {baseline}"]
    previous = json.loads(baseline.read_text(encoding="utf-8")).get("functions", {})
    violations: list[str] = []
    for row in _lizard_rows(lizard, files, root):
        key = f"{row['file']}::{row['function']}"
        old = previous.get(key)
        ccn = int(row["ccn"])
        if old is None and ccn > 15:
            violations.append(f"new function CCN>{15}: {key}={ccn}")
        elif old is not None and ccn > int(old):
            violations.append(f"function CCN increased: {key} {old}->{ccn}")
    return violations


def quality(root: Path, *, lizard: str = "lizard", baseline: Path | None = None) -> int:
    baseline = baseline or root / "tools/quality/lizard_baseline.json"
    failures: list[str] = []
    try:
        files = changed_cpp_files(root)
        if files:
            compile_db = root / "build/compile_commands.json"
            if not compile_db.is_file():
                failures.append(f"missing compile database for clang-tidy: {compile_db}")
            else:
                for path in files:
                    result = _run([
                        "clang-tidy", f"-checks={','.join(SELECTED_CLANG_TIDY)}",
                        "-p", str(compile_db.parent), str(path),
                    ], root)
                    if result.returncode:
                        failures.append(f"clang-tidy failed for {path.relative_to(root)}\n{result.stdout}{result.stderr}")
        failures.extend(timer_violations(root, root / "tools/quality/create_wall_timer_allowlist.tsv"))
        failures.extend(doc_layout_violations(root, root / "docs/architecture/repository_layout.md"))
        if files:
            failures.extend(lizard_violations(root, baseline, lizard, files))
        elif not baseline.is_file():
            failures.append(f"missing lizard baseline: {baseline}")
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        failures.append(str(error))
    for failure in failures:
        print(f"QUALITY: FAIL: {failure}")
    if not failures:
        print("QUALITY: PASS")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write-baseline", action="store_true")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--lizard", default="lizard")
    args = parser.parse_args()
    root = args.root.resolve()
    baseline = root / "tools/quality/lizard_baseline.json"
    if args.write_baseline:
        write_baseline(root, baseline, args.lizard)
        print(f"QUALITY: wrote baseline functions to {baseline}")
        return 0
    return quality(root, lizard=args.lizard, baseline=baseline)


if __name__ == "__main__":
    raise SystemExit(main())

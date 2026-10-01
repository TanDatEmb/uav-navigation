#!/usr/bin/env python3
"""Inventory physical/safety numeric candidates for WP-A6.

The scanner is intentionally conservative: it emits candidates for review,
not an assertion that every numeric token is a safety input.  The report
records the false-positive filters used when turning this candidate stream
into constants.csv.
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path


NUMBER = re.compile(
    r"(?<![A-Za-z0-9_.])[-+]?(?:\d[\d']*(?:\.\d[\d']*)?|\.\d+)"
    r"(?:[eE][-+]?\d+)?[fFlLuU]*(?![A-Za-z0-9_.])"
)
SEMANTIC = re.compile(
    r"(?:age|anchor|accel|jerk|speed|velocity|radius|tolerance|budget|limit|"
    r"timeout|lease|fresh|stale|retry|queue|horizon|window|margin|clearance|"
    r"resolution|voxel|goal|waypoint|corridor|envelope|thrust|mass|gravity|"
    r"period|duration|distance|lookahead|overlap|seed|fov|visibility|hold|"
    r"watchdog|sample|step|rate|time|height|range|density|probability|fraction|"
    r"ratio|capacity|count|size|length|depth|iterations?|association)",
    re.IGNORECASE,
)
PYTHON_SEMANTIC = re.compile(
    r"(?:DEFAULT_|threshold|stale|fresh|age|radius|residual|retry|queue|capacity|"
    r"timeout|speed|accel|jerk|lease|tolerance|anchor|collision|clearance|"
    r"world_observation_fault|gap_budget|acceptance_|mission_timeout|wall_timeout|"
    r"watchdog|minimum_fraction|synchronization|coverage|expected_hz|period_s|"
    r"vehicle_collision|stop_enter|stop_exit|stop_min_duration|deadline)",
    re.IGNORECASE,
)
COMPARISON = re.compile(r"(?:[<>]=?|==|!=|std::abs|std::max|std::min|clamp)")
HG_LINE = re.compile(r"^\| (HG-\d{3}) \|.*")


@dataclass(frozen=True)
class Occurrence:
    path: str
    line: int
    kind: str
    value: str
    text: str


def is_excluded(path: Path) -> bool:
    parts = set(path.parts)
    if {"test", "tests", "external", "vendor", "third_party"} & parts:
        return True
    return any(part.endswith("_vendor") for part in path.parts)


def source_files(root: Path) -> list[Path]:
    out: list[Path] = []
    for path in (root / "src").rglob("*"):
        if path.is_file() and path.suffix in {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"} and not is_excluded(path):
            out.append(path)
    return sorted(out)


def yaml_files(root: Path) -> list[Path]:
    paths = list((root / "config" / "runtime").rglob("*.yaml"))
    paths += [p for p in (root / "src").glob("*/*/config/*.yaml") if p.is_file()]
    return sorted(set(paths))


def python_files(root: Path) -> list[Path]:
    return sorted((root / "tools" / "runtime").glob("*.py"))


def number_tokens(text: str) -> list[str]:
    return [m.group(0) for m in NUMBER.finditer(text)]


def rel(root: Path, path: Path) -> str:
    return path.relative_to(root).as_posix()


def line_for_offset(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def cpp_occurrences(root: Path) -> list[Occurrence]:
    out: list[Occurrence] = []
    for path in source_files(root):
        text = path.read_text(errors="replace")
        lines = text.splitlines()
        for line_no, raw in enumerate(lines, 1):
            line = raw.split("//", 1)[0]
            nums = number_tokens(line)
            if not nums:
                continue
            lower = line.lower()
            if "constexpr" in lower or "static const" in lower or "static constexpr" in lower:
                for value in nums:
                    out.append(Occurrence(rel(root, path), line_no, "constexpr", value, raw.strip()))
            elif "declare_parameter" in lower or "loadparam(" in lower:
                for value in nums:
                    out.append(Occurrence(rel(root, path), line_no, "param_default", value, raw.strip()))
            elif SEMANTIC.search(line) and re.search(r"(?:[={]|::)", line):
                # Named field/member defaults are another parameter seam even
                # when they are not expressed through LoadParam.  Keep this
                # candidate stream review-oriented; constants.csv is the
                # semantic filter and authority map.
                for value in nums:
                    out.append(Occurrence(rel(root, path), line_no, "literal", value, raw.strip()))
            elif SEMANTIC.search(line) and COMPARISON.search(line):
                for value in nums:
                    out.append(Occurrence(rel(root, path), line_no, "literal", value, raw.strip()))

        # Multiline parameter calls: associate the default value with the line
        # where the numeric default appears and keep the call kind explicit.
        for match in re.finditer(
            r"(?:LoadParam|declare_parameter)(?:<[^()\n]*>)?\s*\((.*?)\)",
            text,
            re.DOTALL,
        ):
            body = match.group(1)
            nums = number_tokens(body)
            if not nums:
                continue
            start_line = line_for_offset(text, match.start())
            end_line = line_for_offset(text, match.end())
            for line_no in range(start_line, min(end_line, len(lines)) + 1):
                for value in number_tokens(lines[line_no - 1]):
                    kind = "param_default"
                    candidate = Occurrence(rel(root, path), line_no, kind, value, lines[line_no - 1].strip())
                    if candidate not in out:
                        out.append(candidate)
    return sorted(set(out), key=lambda x: (x.path, x.line, x.kind, x.value))


def yaml_occurrences(root: Path) -> list[Occurrence]:
    out: list[Occurrence] = []
    for path in yaml_files(root):
        for line_no, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
            content = raw.split("#", 1)[0]
            if not content.strip() or not number_tokens(content):
                continue
            # YAML scope is deliberately broader than C++: numeric config
            # keys, vectors and mission geometry are all profile candidates.
            for value in number_tokens(content):
                out.append(Occurrence(rel(root, path), line_no, "yaml", value, raw.strip()))
    return sorted(set(out), key=lambda x: (x.path, x.line, x.value))


def python_occurrences(root: Path) -> list[Occurrence]:
    out: list[Occurrence] = []
    for path in python_files(root):
        for line_no, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
            content = raw.split("#", 1)[0]
            if not number_tokens(content):
                continue
            if PYTHON_SEMANTIC.search(content) or re.search(
                r"vehicle_collision_radius|stale_after|residual|world_observation_fault|"
                r"(?:420|430|520)|threshold|default",
                content,
                re.IGNORECASE,
            ):
                for value in number_tokens(content):
                    out.append(Occurrence(rel(root, path), line_no, "python", value, raw.strip()))
    return sorted(set(out), key=lambda x: (x.path, x.line, x.value))


def doc_occurrences(root: Path) -> list[Occurrence]:
    path = root / "docs" / "safety" / "runtime_safety_current.md"
    out: list[Occurrence] = []
    for line_no, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
        if HG_LINE.match(raw):
            # The HG identifier is not a constant value.  Scan only the
            # contract text after the identifier and before the status column.
            fields = raw.split("|", 3)
            contract = fields[2] if len(fields) > 2 else raw
            for value in number_tokens(contract):
                out.append(Occurrence(rel(root, path), line_no, "doc", value, raw.strip()))
    return out


def all_occurrences(root: Path) -> list[Occurrence]:
    return cpp_occurrences(root) + yaml_occurrences(root) + python_occurrences(root) + doc_occurrences(root)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--csv", type=Path, help="write raw candidate stream as CSV")
    args = parser.parse_args()
    root = args.root.resolve()
    rows = all_occurrences(root)
    if args.csv:
        with args.csv.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["file", "line", "kind", "value", "text"])
            writer.writerows((r.path, r.line, r.kind, r.value, r.text) for r in rows)
    else:
        print("file:line|kind|value|text")
        for row in rows:
            print(f"{row.path}:{row.line}|{row.kind}|{row.value}|{row.text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

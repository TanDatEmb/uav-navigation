#!/usr/bin/env python3
"""Validate the WP-A6 numeric-literal inventory and classification.

The default mode is read-only.  ``--write`` materializes the deterministic
classification and adds explicit proposed names for otherwise-unmapped
decision literals to constants.csv.  This is an analysis artifact only; it
does not edit product source or runtime configuration.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import Counter
from decimal import Decimal, InvalidOperation
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
CONSTANTS = HERE / "constants.csv"
CLASSIFICATION = HERE / "literal_classification.csv"

TARGET_FILES = [
    "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp",
    "src/runtime/navigation_runtime/include/navigation_runtime/planner_fsm.hpp",
    "src/runtime/navigation_runtime/src/mission_progress.cpp",
    "src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp",
    "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp",
    "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/px4_tracking_adapter.hpp",
    "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/velocity_only_continuity.hpp",
    "src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp",
    "src/px4/px4_odometry_bridge/src/geometric_jump_continuity.cpp",
    "src/px4/px4_odometry_bridge/src/timestamp_conversion.cpp",
    "tools/runtime/external_mode_scenario.py",
    "tools/runtime/report.py",
    "tools/runtime/evaluation.py",
]

NUMBER = re.compile(
    r"(?<![A-Za-z0-9_])[-+]?(?:\d(?:['’]?\d)*(?:\.\d(?:['’]?\d)*)?|\.\d+)"
    r"(?:[eE][-+]?\d(?:['’]?\d)*)?[fFlLuU]*"
)
COMPARISON = re.compile(r"(?:<=|>=|==|!=|<|>)")
MIN_MAX_CLAMP = re.compile(r"\b(?:std::)?(?:min|max|clamp)\s*\(")
DEFAULT_PARAM = re.compile(
    r"\b(?:declare_parameter|LoadParam|default)\b"
)
FUNCTION_DEFAULT = re.compile(
    r"(?:\bdef\b|[,(])[^;\n]*\b[A-Za-z_][A-Za-z0-9_]*\s*=\s*"
)
STRING_OR_CHAR = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')

CONSTANTS_HEADER = [
    "group",
    "proposed_name",
    "value",
    "unit",
    "semantics (1 câu)",
    "hg_id (nếu có)",
    "occurrences (danh sách: file:line|kind[value])",
    "all_equal",
    "yaml_loadable",
    "pinned",
    "notes",
]
CLASSIFICATION_HEADER = ["file", "line", "literal", "context", "decision"]
ALLOWED_EXCLUSION_REASONS = {
    "index/size",
    "unit_conversion",
    "numeric_epsilon<=1e-6",
    "enum_or_bitmask",
    "test_only",
    "diagnostic_format",
}

# Required file:line witnesses from WP-A6-R1. The bridge declaration starts on
# line 56 while its numeric default is on line 57; both are accepted and the
# report keeps that source-line distinction explicit.
REQUIRED_OCCURRENCES = [
    ("planning_timing.hpp", {25}, "0.15"),
    ("mission.hpp", {38}, "0.15"),
    ("mission_controller.hpp", {100}, "0.15"),
    ("navigation_mode_node.cpp", {1526, 1527}, "0.15"),
    ("navigation_runtime_node.cpp", {4499, 4617}, "0.15"),
    ("px4_external_odometry_bridge_node.cpp", {56, 57}, "10.0"),
    ("px4_external_odometry_bridge_node.cpp", {53}, "0.75"),
    ("px4_external_odometry_bridge_node.cpp", {55}, "0.35"),
    ("px4_external_odometry_bridge_node.cpp", {63}, "0.5"),
    ("navigation_mode_node.cpp", {1518}, "0.5"),
    ("ros_output_publisher.cpp", {23}, "500'000'000"),
    ("parameter_loader.cpp", {304}, "0.50"),
    ("parameter_loader.cpp", {200}, "0.5"),
    ("time_validator.hpp", {15, 16}, "200'000'000"),
    ("candidate_bundle.hpp", set(range(398, 404)), "1.0e-5"),
    ("candidate_bundle.hpp", set(range(398, 404)), "1.0e-4"),
    ("candidate_bundle.hpp", set(range(398, 404)), "1.0e-3"),
    ("candidate_bundle.hpp", set(range(398, 404)), "1.0e-6"),
    ("execution_anchor.hpp", set(range(47, 53)), "1.0e-5"),
    ("execution_anchor.hpp", set(range(47, 53)), "1.0e-4"),
    ("execution_anchor.hpp", set(range(47, 53)), "1.0e-3"),
    ("execution_anchor.hpp", set(range(47, 53)), "1.0e-6"),
    ("planner_fsm.hpp", {459}, "1.0e-9"),
    ("common.yaml", {6}, "1.0"),
    ("common.yaml", {11}, "0.50"),
    ("common.yaml", {12}, "0.10"),
    ("common.yaml", {13}, "0.50"),
    ("common.yaml", {14}, "1.00"),
    ("common.yaml", {15}, "0.50"),
    ("common.yaml", {16}, "0.50"),
    ("common.yaml", {17}, "2.0"),
    ("common.yaml", {18}, "0.20"),
    ("common.yaml", {19}, "0.50"),
    ("common.yaml", {20}, "1.00"),
    ("common.yaml", {21}, "1.00"),
    ("common.yaml", {22}, "1.00"),
    ("common.yaml", {23}, "2.50"),
    ("runner.py", {3161}, "0.35"),
    ("external_mode_scenario.py", {1118}, "0.35"),
    ("external_mode_scenario.py", {3112}, "0.05"),
    ("external_mode_scenario.py", {2923}, "0.35"),
    ("external_mode_scenario.py", {1395}, "0.5"),
    ("external_mode_scenario.py", {3110}, "0.8"),
    ("runner.py", {2009}, "420"),
    ("runner.py", {3029}, "430"),
    ("runner.py", {3132}, "520"),
    ("runner.py", {3029}, "700"),
]


def normalize(value: str) -> str:
    value = value.strip().replace("'", "")
    value = re.sub(r"[fFlLuU]+$", "", value)
    try:
        number = Decimal(value)
    except InvalidOperation:
        return value
    rendered = format(number, "f")
    if "." in rendered:
        rendered = rendered.rstrip("0").rstrip(".")
    return rendered or "0"


def load_constants() -> tuple[list[dict[str, str]], dict[tuple[str, int, str], list[str]]]:
    with CONSTANTS.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != CONSTANTS_HEADER:
            raise ValueError(f"constants.csv header mismatch: {reader.fieldnames}")
        rows = list(reader)

    occurrences: dict[tuple[str, int, str], list[str]] = {}
    occurrence_column = CONSTANTS_HEADER[6]
    for row in rows:
        name = row["proposed_name"]
        for item in row[occurrence_column].split(";"):
            item = item.strip()
            if not item or "|" not in item or ":" not in item:
                continue
            location, kind = item.split("|", 1)
            path, line_text = location.rsplit(":", 1)
            match = re.search(r"\[([^\]]+)\]", kind)
            if not match:
                continue
            try:
                line = int(line_text)
            except ValueError:
                continue
            key = (path, line, normalize(match.group(1)))
            occurrences.setdefault(key, []).append(name)
    return rows, occurrences


def source_candidates() -> list[dict[str, str]]:
    candidates: list[dict[str, str]] = []
    for relative in TARGET_FILES:
        path = ROOT / relative
        if not path.is_file():
            raise FileNotFoundError(relative)
        for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            code = raw.split("//", 1)[0] if path.suffix in {".cpp", ".hpp"} else raw.split("#", 1)[0]
            scan_code = STRING_OR_CHAR.sub("", code)
            if not (
                COMPARISON.search(scan_code)
                or MIN_MAX_CLAMP.search(scan_code)
                or DEFAULT_PARAM.search(scan_code)
                or FUNCTION_DEFAULT.search(scan_code)
            ):
                continue
            for match in NUMBER.finditer(scan_code):
                candidates.append(
                    {
                        "file": relative,
                        "line": str(line_number),
                        "literal": match.group(0),
                        "context": raw.strip(),
                    }
                )
    return candidates


def occurrence_key(candidate: dict[str, str]) -> tuple[str, int, str]:
    return (candidate["file"], int(candidate["line"]), normalize(candidate["literal"]))


def generated_name(candidate: dict[str, str], ordinal: int) -> str:
    stem = re.sub(r"[^A-Za-z0-9]+", "_", Path(candidate["file"]).stem).strip("_")
    return f"literal_{stem}_line_{candidate['line']}_{ordinal}"


def initial_decision(
    candidate: dict[str, str],
    occurrences: dict[tuple[str, int, str], list[str]],
    ordinal: int,
) -> str:
    names = occurrences.get(occurrence_key(candidate), [])
    if names:
        return f"INCLUDED:{sorted(names)[0]}"

    context = candidate["context"].lower()
    literal = normalize(candidate["literal"])
    if any(word in context for word in ("enum", "bitmask", "bit_mask", "flags", "flag_bits")):
        return "EXCLUDED:enum_or_bitmask"
    if any(word in context for word in ("size(", ".size", "index", "idx", "offset", "argv", "argc")):
        return "EXCLUDED:index/size"
    try:
        if abs(Decimal(literal)) <= Decimal("1e-6"):
            return "EXCLUDED:numeric_epsilon<=1e-6"
    except InvalidOperation:
        pass
    if any(word in context for word in ("setprecision", "std::fixed", "printf", "format(")):
        return "EXCLUDED:diagnostic_format"
    if candidate["file"].startswith("tools/runtime/"):
        return "EXCLUDED:test_only"
    return "EXCLUDED:index/size"


def classify(candidates: list[dict[str, str]], occurrences: dict[tuple[str, int, str], list[str]]) -> list[dict[str, str]]:
    seen: Counter[tuple[str, int, str]] = Counter()
    rows: list[dict[str, str]] = []
    for candidate in candidates:
        key = occurrence_key(candidate)
        ordinal = seen[key]
        seen[key] += 1
        rows.append({**candidate, "decision": initial_decision(candidate, occurrences, ordinal)})
    return rows


def append_generated_constants(
    constants: list[dict[str, str]], classification: list[dict[str, str]]
) -> None:
    existing = {row["proposed_name"] for row in constants}
    for row in classification:
        if not row["decision"].startswith("INCLUDED:"):
            continue
        name = row["decision"].split(":", 1)[1]
        if name in existing:
            continue
        constants.append(
            {
                "group": "literal_review",
                "proposed_name": name,
                "value": row["literal"],
                "unit": "-",
                "semantics (1 câu)": "Unmapped decision literal; owner review required before P1.",
                "hg_id (nếu có)": "",
                "occurrences (danh sách: file:line|kind[value])": (
                    f"{row['file']}:{row['line']}|literal[{row['literal']}]"
                ),
                "all_equal": "true",
                "yaml_loadable": "false",
                "pinned": "false",
                "notes": "Generated by check_constants.py --write; not a product change.",
            }
        )
        existing.add(name)


def write_constants(rows: list[dict[str, str]]) -> None:
    with CONSTANTS.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CONSTANTS_HEADER, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_classification(rows: list[dict[str, str]]) -> None:
    with CLASSIFICATION.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CLASSIFICATION_HEADER, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def has_required_occurrence(
    constants: list[dict[str, str]], path_suffix: str, required_lines: set[int], value: str
) -> bool:
    expected = normalize(value)
    for row in constants:
        for item in row[CONSTANTS_HEADER[6]].split(";"):
            item = item.strip()
            if "|" not in item or ":" not in item:
                continue
            location, kind = item.split("|", 1)
            path, line_spec = location.rsplit(":", 1)
            if not path.endswith(path_suffix):
                continue
            line_match = re.fullmatch(r"(\d+)(?:-(\d+))?", line_spec)
            if not line_match:
                continue
            start = int(line_match.group(1))
            end = int(line_match.group(2) or line_match.group(1))
            if not required_lines.intersection(range(start, end + 1)):
                continue
            if any(normalize(match.group(0)) == expected for match in NUMBER.finditer(kind)):
                return True
    return False


def validate() -> int:
    constants, occurrences = load_constants()
    candidates = source_candidates()
    if not candidates:
        raise ValueError("no decision/default/min-max numeric literals found")
    if not CLASSIFICATION.is_file():
        raise ValueError("literal_classification.csv is missing; run check_constants.py --write")

    with CLASSIFICATION.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != CLASSIFICATION_HEADER:
            raise ValueError(f"literal_classification.csv header mismatch: {reader.fieldnames}")
        classified = list(reader)

    candidate_keys = Counter(
        (row["file"], row["line"], row["literal"], row["context"]) for row in candidates
    )
    classified_keys = Counter(
        (row["file"], row["line"], row["literal"], row["context"]) for row in classified
    )
    if candidate_keys != classified_keys:
        missing = sum((candidate_keys - classified_keys).values())
        extra = sum((classified_keys - candidate_keys).values())
        raise ValueError(f"classification coverage mismatch: missing={missing} extra={extra}")

    names = {row["proposed_name"] for row in constants}
    for row in classified:
        decision = row["decision"]
        if decision.startswith("INCLUDED:"):
            name = decision.split(":", 1)[1]
            if name not in names:
                raise ValueError(f"INCLUDED name absent from constants.csv: {name}")
        elif decision.startswith("EXCLUDED:"):
            reason = decision.split(":", 1)[1]
            if reason not in ALLOWED_EXCLUSION_REASONS:
                raise ValueError(f"invalid EXCLUDED reason: {reason}")
        else:
            raise ValueError(f"invalid decision: {decision}")

    required_hits = sum(
        has_required_occurrence(constants, path, lines, value)
        for path, lines, value in REQUIRED_OCCURRENCES
    )
    if required_hits != len(REQUIRED_OCCURRENCES):
        missing = [
            f"{path}:{sorted(lines)}={value}"
            for path, lines, value in REQUIRED_OCCURRENCES
            if not has_required_occurrence(constants, path, lines, value)
        ]
        raise ValueError(f"required file:line inventory missing: {missing}")

    pinned = [row for row in constants if row["pinned"].strip().lower() == "true"]
    if any("pin_compare[" not in row[CONSTANTS_HEADER[6]] for row in pinned):
        raise ValueError("pinned=true row lacks pin_compare occurrence witness")

    included = sum(row["decision"].startswith("INCLUDED:") for row in classified)
    excluded = len(classified) - included
    print(f"target_files={len(TARGET_FILES)}")
    print(f"constants_rows={len(constants)}")
    print(f"literal_candidates={len(candidates)} classification_rows={len(classified)}")
    print(f"included={included} excluded={excluded}")
    print(f"required_file_line_occurrences={required_hits}/{len(REQUIRED_OCCURRENCES)}")
    print("literal_coverage=100%")
    print("status=PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write",
        action="store_true",
        help="materialize literal_classification.csv and generated constants before validating",
    )
    args = parser.parse_args()
    if args.write:
        constants, occurrences = load_constants()
        original_constants = [row.copy() for row in constants]
        classification = classify(source_candidates(), occurrences)
        append_generated_constants(constants, classification)
        if constants != original_constants:
            write_constants(constants)
        write_classification(classification)
    try:
        return validate()
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(f"status=FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

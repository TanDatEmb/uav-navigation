#!/usr/bin/env python3
"""Check SafetyProfile key coverage and source-value agreement with A6."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import re
import sys
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.safety_profile.profile import load_profile


ORACLE = ROOT / "tools/safety_profile/oracle/a6_constants.csv"
SOURCE = ROOT / "config/safety_profile/sitl_current_as_is.yaml"
EXCLUDED = {
    "numerical_roundoff_epsilons": "K3: named constexpr roundoff is not a profile key",
    "world_observation_fault_duration_ms": "A5 rule 8: fault injection is not a product key",
}


def _number(value: str, unit: str) -> Any:
    value = value.strip()
    if value.startswith("[") and value.endswith("]"):
        body = value[1:-1]
        parts = re.split(r"[;/]", body)
        return [_number(part, unit) for part in parts if part.strip()]
    if "/" in value and re.fullmatch(r"[\d.eE+-]+(?:/[\d.eE+-]+)+", value):
        return [_number(part, unit) for part in value.split("/")]
    if value in {"true", "false"}:
        return value == "true"
    try:
        numeric = re.match(r"^[+'\-\d.eE]+", value.replace("'", ""))
        if numeric is None:
            return value
        result = float(numeric.group(0))
        if "ns" in value and unit == "s":
            result /= 1_000_000_000.0
        return result
    except ValueError:
        return value


def _profile_records(document: dict[str, Any]) -> dict[str, tuple[str, dict[str, Any]]]:
    result = {}

    def visit(node: Any, prefix: str) -> None:
        if not isinstance(node, dict):
            return
        if "value" in node:
            name = node.get("oracle_name", prefix.rsplit(".", 1)[-1])
            result.setdefault(name, (prefix, node))
            return
        for key, value in node.items():
            visit(value, f"{prefix}.{key}" if prefix else str(key))

    for section, values in document.items():
        if section not in {"schema_version", "profile", "overlays"}:
            visit(values, section)
    return result


def check() -> list[str]:
    errors: list[str] = []
    document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    with ORACLE.open(encoding="utf-8", newline="") as stream:
        oracle_rows = list(csv.DictReader(stream))
    records = _profile_records(document)
    loaded = load_profile(SOURCE)
    open_questions = set(document["profile"].get("open_questions", []))
    seen = set()

    for row in oracle_rows:
        name = row["proposed_name"]
        if name in EXCLUDED:
            if name in records:
                errors.append(f"{name}: excluded by rule but present in profile")
            continue
        item = records.get(name)
        if item is None:
            if name not in open_questions:
                errors.append(f"{name}: missing from profile without an OPEN_QUESTION")
            continue
        seen.add(name)
        path, record = item
        if record.get("unit") != row["unit"]:
            errors.append(f"{name}: unit {record.get('unit')!r} != oracle {row['unit']!r}")
            continue
        if record.get("value") == "DERIVED":
            expected = loaded.values[path]
        elif name == "anchor_pvaj_roundoff_tolerances":
            components = [float(value) for value in re.findall(
                r"=([0-9.eE+-]+)", str(record["value"]))]
            occurrences = {}
            for occurrence in row["occurrences (danh sách: file:line|kind[value])"].split(";"):
                source_and_kind = occurrence.strip().split("|", 1)
                if len(source_and_kind) == 2:
                    match = re.search(r"\[([^\]]*)\]", source_and_kind[1])
                    if match:
                        occurrences[source_and_kind[0].strip()] = _number(match.group(1), row["unit"])
            selected_values = [occurrences[source] for source in record.get("sources", [])
                               if source in occurrences]
            if selected_values != components * 2:
                errors.append(f"{name}: profile tolerances do not match source literals")
            expected = record["value"]
        elif isinstance(record.get("value"), str):
            # Descriptive, nonnumeric evidence rows inventory a source seam;
            # their A6 text is the oracle value, while numeric policy belongs
            # to the separately represented profile key.
            expected = row["value"]
        else:
            occurrences = {}
            for occurrence in row["occurrences (danh sách: file:line|kind[value])"].split(";"):
                source_and_kind = occurrence.strip().split("|", 1)
                if len(source_and_kind) != 2:
                    continue
                source, detail = source_and_kind
                match = re.search(r"\[([^\]]*)\]", detail)
                if match:
                    occurrences[source.strip()] = _number(match.group(1), row["unit"])
            selected = record.get("sources", [])
            selected_values = [occurrences[source] for source in selected if source in occurrences]
            if not selected_values:
                errors.append(f"{name}: profile source does not resolve to an A6 occurrence")
                continue
            expected = selected_values[0]
            if any(value != expected for value in selected_values[1:]):
                errors.append(f"{name}: selected profile sources disagree: {selected_values}")
                continue
        actual = loaded.values[path]
        values_match = actual == expected
        if isinstance(actual, (float, int)) and isinstance(expected, (float, int)):
            values_match = math.isclose(float(actual), float(expected), rel_tol=1e-12, abs_tol=1e-15)
        if not values_match:
            errors.append(f"{name}: profile value {actual!r} != selected source {expected!r}")

    undeclared = open_questions - {row["proposed_name"] for row in oracle_rows}
    if undeclared:
        errors.append(f"OPEN_QUESTION names not in A6 oracle: {sorted(undeclared)}")
    absent = {row["proposed_name"] for row in oracle_rows} - seen - set(EXCLUDED)
    if absent != open_questions:
        errors.append(f"oracle coverage discrepancy: absent={sorted(absent)} open_questions={sorted(open_questions)}")
    if open_questions:
        errors.append(f"unresolved A6 profile rows: {sorted(open_questions)}")
    print(f"semantic_rows={len(oracle_rows)} represented={len(seen)} excluded={len(EXCLUDED)} open_questions={len(open_questions)}")
    print(f"failures={len(errors)}")
    for error in errors:
        print(f"FAIL: {error}")
    return errors


if __name__ == "__main__":
    failures = check()
    raise SystemExit(1 if failures else 0)

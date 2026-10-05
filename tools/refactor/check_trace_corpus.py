#!/usr/bin/env python3
"""Fail-closed structural checker for the P4-0 decision-record corpus.

This checker validates evidence shape and site coverage only.  It deliberately
does not claim semantic replay; that requires a predicate/reducer dispatcher
and is a separate P4 deliverable.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


REQUIRED_FIELDS = (
    "schema_version",
    "corpus",
    "site_id",
    "event",
    "pre_state",
    "trigger",
    "predicates",
    "effects",
    "post_state",
)
VALID_CORPORA = {"K1", "K2", "K3"}


@dataclass(frozen=True)
class ValidationReport:
    errors: list[str]
    covered_sites: set[str]

    @property
    def ok(self) -> bool:
        return not self.errors


def _is_mapping(value: Any) -> bool:
    return isinstance(value, dict)


def _validate_record(record: Any, index: int, expected_sites: set[str], errors: list[str]) -> str | None:
    prefix = f"record[{index}]"
    if not _is_mapping(record):
        errors.append(f"{prefix}: expected object")
        return None

    for field in REQUIRED_FIELDS:
        if field not in record:
            errors.append(f"{prefix}.{field}: missing")

    site_id = record.get("site_id")
    if not isinstance(site_id, str) or not site_id:
        errors.append(f"{prefix}.site_id: expected non-empty string")
        site_id = None
    elif site_id not in expected_sites:
        errors.append(f"{prefix}.site_id: unknown {site_id}")

    schema_version = record.get("schema_version")
    if not isinstance(schema_version, str) or not schema_version:
        errors.append(f"{prefix}.schema_version: expected non-empty string")

    corpus = record.get("corpus")
    if corpus not in VALID_CORPORA:
        errors.append(f"{prefix}.corpus: expected one of K1/K2/K3")

    for field in ("event", "pre_state", "trigger", "post_state"):
        if field in record and not _is_mapping(record[field]):
            errors.append(f"{prefix}.{field}: expected object")

    predicates = record.get("predicates")
    if "predicates" in record and not isinstance(predicates, list):
        errors.append(f"{prefix}.predicates: expected array")
    elif isinstance(predicates, list):
        for predicate_index, predicate in enumerate(predicates):
            if not _is_mapping(predicate):
                errors.append(f"{prefix}.predicates[{predicate_index}]: expected object")
                continue
            for field in ("name", "args", "result"):
                if field not in predicate:
                    errors.append(f"{prefix}.predicates[{predicate_index}].{field}: missing")

    effects = record.get("effects")
    if "effects" in record and not isinstance(effects, list):
        errors.append(f"{prefix}.effects: expected array")
    elif isinstance(effects, list):
        if not effects:
            errors.append(f"{prefix}.effects: must not be empty")
        for effect_index, effect in enumerate(effects):
            if not _is_mapping(effect) or not isinstance(effect.get("type"), str) or not effect["type"]:
                errors.append(f"{prefix}.effects[{effect_index}].type: expected non-empty string")

    dropped = record.get("dropped", 0)
    if isinstance(dropped, bool) or not isinstance(dropped, int) or dropped < 0:
        errors.append(f"{prefix}.dropped: expected non-negative integer")
    elif dropped:
        errors.append(f"{prefix}.dropped: {dropped}")
    return site_id


def validate_corpus(
    records: Iterable[Any], expected_sites: set[str], unreached: dict[str, str]
) -> ValidationReport:
    errors: list[str] = []
    covered_sites: set[str] = set()
    expected_sites = set(expected_sites)
    unreached = dict(unreached)

    for site_id, reason in unreached.items():
        if site_id not in expected_sites:
            errors.append(f"unreached[{site_id}]: unknown site")
        elif not isinstance(reason, str) or not reason.strip():
            errors.append(f"unreached[{site_id}]: non-empty reason required")

    for index, record in enumerate(records):
        site_id = _validate_record(record, index, expected_sites, errors)
        if site_id is not None:
            covered_sites.add(site_id)

    missing = sorted(expected_sites - covered_sites - set(unreached))
    if missing:
        errors.append("coverage.missing_sites: " + ",".join(missing))
    return ValidationReport(errors, covered_sites)


def _load_jsonl(path: Path) -> list[Any]:
    records: list[Any] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc.msg}") from exc
    return records


def _load_expected_sites(path: Path) -> set[str]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if "rule_id" not in (reader.fieldnames or []):
            raise ValueError(f"{path}: missing rule_id column")
        sites = {row["rule_id"] for row in reader if row.get("rule_id")}
    if not sites:
        raise ValueError(f"{path}: no rule IDs")
    return sites


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", type=Path, required=True, help="P4-0 JSONL corpus")
    parser.add_argument("--decision-table", type=Path, required=True, help="A4 decision_table.csv")
    parser.add_argument("--unreached-json", type=Path, help="JSON object mapping site_id to reason")
    args = parser.parse_args(argv)

    try:
        records = _load_jsonl(args.corpus)
        expected_sites = _load_expected_sites(args.decision_table)
        unreached = {}
        if args.unreached_json:
            value = json.loads(args.unreached_json.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ValueError("--unreached-json must contain a JSON object")
            unreached = value
        report = validate_corpus(records, expected_sites, unreached)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"ok": False, "errors": [str(exc)]}, sort_keys=True))
        return 2

    result = {
        "ok": report.ok,
        "record_count": len(records),
        "covered_site_count": len(report.covered_sites),
        "errors": report.errors,
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

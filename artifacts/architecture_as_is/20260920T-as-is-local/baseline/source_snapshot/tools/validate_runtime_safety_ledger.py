#!/usr/bin/env python3
"""Validate the lossless runtime-safety ledger split.

This is intentionally a small, dependency-free structural guard. It does not
interpret safety semantics or certify evidence; it catches migration drift,
missing history, duplicate identifiers, and active-register omissions.
"""

from __future__ import annotations

import hashlib
import re
import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CURRENT = ROOT / "docs/safety/runtime_safety_current.md"
INDEX = ROOT / "docs/safety/runtime_safety_index.md"
ARCHIVE = ROOT / "docs/safety/archive/runtime_safety_legacy_full.md"
COMPAT = ROOT / "docs/architecture/runtime_safety_decision_ledger.md"

KNOWN_LIFECYCLE = {"ACTIVE", "SUPERSEDED", "REVERTED", "REMOVED", "REJECTED", "UNRESOLVED"}
KNOWN_IMPLEMENTATION = {"NOT_IMPLEMENTED", "IMPLEMENTED", "UNRESOLVED"}
KNOWN_EVIDENCE = {
    "UNVERIFIED",
    "UNIT_VERIFIED",
    "COMPONENT_VERIFIED",
    "INTEGRATION_PARTIAL",
    "INTEGRATION_VERIFIED",
    "QUALIFIED",
    "BLOCKED",
    "NOT_EVALUABLE",
    "UNRESOLVED",
}
KNOWN_AUTHORITY = {"PRODUCT", "DIAGNOSTIC_ONLY", "EXPERIMENT", "UNRESOLVED"}


def fail(errors: list[str], message: str) -> None:
    errors.append(message)


def github_slug(value: str) -> str:
    value = re.sub(r"[^a-z0-9 _-]", "", value.lower())
    return re.sub(r"[ _]+", "-", value)


def markdown_table_rows(text: str, prefix: str) -> list[tuple[str, ...]]:
    rows = []
    for line in text.splitlines():
        match = re.match(rf"^\|\s*({prefix}-\d+)\s*\|(.*)\|$", line)
        if match:
            rows.append((match.group(1),) + tuple(part.strip() for part in match.group(2).split("|")))
    return rows


def main() -> int:
    errors: list[str] = []
    for path in (CURRENT, INDEX, ARCHIVE, COMPAT):
        if not path.is_file():
            fail(errors, f"missing required file: {path.relative_to(ROOT)}")
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1

    current = CURRENT.read_text(encoding="utf-8")
    index = INDEX.read_text(encoding="utf-8")
    archive_bytes = ARCHIVE.read_bytes()
    archive = archive_bytes.decode("utf-8")

    current_lines = current.count("\n")
    if current_lines > 500:
        fail(errors, f"current contract exceeds 500 lines: {current_lines}")

    sha_match = re.search(r"Original/archive SHA-256: `([0-9a-f]{64})`", index)
    size_match = re.search(r"Original/archive size: `([0-9,]+)` lines, `([0-9,]+)` bytes", index)
    count_match = re.search(r"Dated decisions represented: `([0-9,]+)`", index)
    if not (sha_match and size_match and count_match):
        fail(errors, "index is missing migration provenance metadata")
    else:
        actual_sha = hashlib.sha256(archive_bytes).hexdigest()
        expected_sha = sha_match.group(1)
        if actual_sha != expected_sha:
            fail(errors, f"archive SHA-256 mismatch: index={expected_sha}, actual={actual_sha}")
        expected_lines, expected_bytes = (int(value.replace(",", "")) for value in size_match.groups())
        if archive.count("\n") != expected_lines:
            fail(errors, f"archive line count mismatch: index={expected_lines}, actual={archive.count(chr(10))}")
        if len(archive_bytes) != expected_bytes:
            fail(errors, f"archive byte count mismatch: index={expected_bytes}, actual={len(archive_bytes)}")

    archive_headings = []
    for line in archive.splitlines():
        match = re.match(r"^#{2,3} (\d{4}-\d\d-\d\d) - (.+)$", line)
        if match:
            date, title = match.groups()
            archive_headings.append((date, title, github_slug(f"{date} - {title}")))
    if len(archive_headings) != 706:
        fail(errors, f"unexpected dated decision count: {len(archive_headings)} (expected 706)")
    if len({slug for _, _, slug in archive_headings}) != len(archive_headings):
        fail(errors, "archive contains duplicate heading anchors")

    decision_rows = []
    in_history = False
    for line in index.splitlines():
        if line == "## Decision history":
            in_history = True
            continue
        if not in_history:
            continue
        match = re.match(
            r"^\|\s*(DEC-\d{8}-\d{3})\s*\|\s*(\d{4}-\d\d-\d\d)\s*\|\s*([^|]+)\|\s*([^|]+)\|\s*([^|]+)\|\s*([^|]+)\|\s*([^|]+)\|\s*([^|]+)\|\s*(.*?)\|\s*\[full entry\]\([^#]+#([^)]+)\)\s*\|$",
            line,
        )
        if match:
            decision_rows.append(tuple(part.strip() for part in match.groups()))

    if len(decision_rows) != len(archive_headings):
        fail(errors, f"index decision rows={len(decision_rows)} does not match archive={len(archive_headings)}")
    if len({row[0] for row in decision_rows}) != len(decision_rows):
        fail(errors, "duplicate decision IDs in index")
    seen_dates = Counter()
    for row, source in zip(decision_rows, archive_headings):
        decision_id, date, _layer, record, lifecycle, implementation, evidence, authority, _summary, anchor = row
        source_date, _title, source_anchor = source
        seen_dates[date] += 1
        expected_id = f"DEC-{date.replace('-', '')}-{seen_dates[date]:03d}"
        if decision_id != expected_id:
            fail(errors, f"non-deterministic decision ID: expected {expected_id}, found {decision_id}")
        if date != source_date or anchor != source_anchor:
            fail(errors, f"archive target mismatch for {decision_id}")
        if record != "ARCHIVED":
            fail(errors, f"non-archived decision record in index: {decision_id}")
        if lifecycle not in KNOWN_LIFECYCLE:
            fail(errors, f"invalid lifecycle for {decision_id}: {lifecycle}")
        if implementation not in KNOWN_IMPLEMENTATION:
            fail(errors, f"invalid implementation for {decision_id}: {implementation}")
        if evidence not in KNOWN_EVIDENCE:
            fail(errors, f"invalid evidence for {decision_id}: {evidence}")
        if authority not in KNOWN_AUTHORITY:
            fail(errors, f"invalid authority for {decision_id}: {authority}")

    archive_gate_rows = markdown_table_rows(archive, "HG")
    archive_bypass_rows = markdown_table_rows(archive, "TB")
    index_gate_rows = markdown_table_rows(index, "HG")
    index_bypass_rows = markdown_table_rows(index, "TB")
    current_gate_rows = markdown_table_rows(current, "HG")
    current_bypass_rows = markdown_table_rows(current, "TB")

    def unique_ids(rows: list[tuple[str, ...]], label: str) -> set[str]:
        ids = [row[0] for row in rows]
        if len(ids) != len(set(ids)):
            fail(errors, f"duplicate {label} IDs")
        return set(ids)

    archive_gates = unique_ids(archive_gate_rows, "archive gate")
    archive_bypasses = unique_ids(archive_bypass_rows, "archive bypass")
    index_gates = unique_ids(index_gate_rows, "index gate")
    index_bypasses = unique_ids(index_bypass_rows, "index bypass")
    if archive_gates != index_gates:
        fail(errors, "index gate IDs do not match the archived register")
    if archive_bypasses != index_bypasses:
        fail(errors, "index bypass IDs do not match the archived register")

    active_gate_ids = {
        row[0]
        for row in archive_gate_rows
        if not re.search(r"REMOVED|REJECTED|REVERTED", row[3], re.I)
    }
    if unique_ids(current_gate_rows, "current gate") != active_gate_ids:
        fail(errors, "current active gate IDs do not match the archived lifecycle register")
    active_bypass_ids = {
        row[0]
        for row in archive_bypass_rows
        if not re.search(r"REMOVED|REJECTED|REVERTED", row[-1], re.I)
    }
    if unique_ids(current_bypass_rows, "current bypass") != active_bypass_ids:
        fail(errors, "current active bypass IDs do not match the archived lifecycle register")
    if "TB-003" in current and "Removal condition" not in current:
        fail(errors, "active TB-003 is missing an explicit removal condition")

    archive_anchors = {anchor for _, _, anchor in archive_headings}
    for target in re.findall(r"\]\(archive/[^#)]+#([^)]+)\)", index):
        if target not in archive_anchors and target not in {"current-hard-gate-audit", "temporary-bypass-register"}:
            fail(errors, f"broken archive/index anchor: {target}")

    for decision_id in re.findall(r"\bDEC-\d{8}-\d{3}\b", current):
        if not re.search(rf"^\|\s*{re.escape(decision_id)}\s*\|", index, re.M):
            fail(errors, f"current decision summary is absent from index: {decision_id}")
    if "runtime_safety_current.md" not in COMPAT.read_text(encoding="utf-8"):
        fail(errors, "compatibility path does not point to the current contract")

    if errors:
        print("runtime safety ledger validation: FAIL", file=sys.stderr)
        print("\n".join(f"- {error}" for error in errors), file=sys.stderr)
        return 1
    print(
        "runtime safety ledger validation: PASS "
        f"(current={current_lines} lines, decisions={len(decision_rows)}, "
        f"gates={len(archive_gate_rows)}, active_gates={len(active_gate_ids)}, "
        f"bypasses={len(archive_bypass_rows)}, active_bypasses={len(active_bypass_ids)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

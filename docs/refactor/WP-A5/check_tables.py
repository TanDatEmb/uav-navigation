#!/usr/bin/env python3
"""Structural checker for the WP-A5 adapter/bridge decision tables.

This is a static inventory check.  It deliberately does not claim runtime,
SITL, PX4, or flight qualification evidence.  The target function list is a
data parameter so the same checker can be extended without hard-coding a
return count.
"""

from __future__ import annotations

import csv
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent

STATE_COLUMNS = [
    "var_id", "component", "cpp_name", "type", "owner_class", "writers",
    "readers", "threads", "guarding_lock", "reset_on", "partition",
]
EVENT_COLUMNS = [
    "event_id", "component", "source", "thread", "payload_fields", "notes",
]
DECISION_COLUMNS = [
    "rule_id", "event", "guard", "effects", "state_updates", "source",
    "exit_site", "predicates_used", "covered_by_test", "suspect",
]
PREDICATE_COLUMNS = ["name", "file:line", "signature", "pure", "semantics", "used_by_rules"]

EFFECTS = {
    "PUBLISH_COMMAND", "PUBLISH_HOLD_SETPOINT", "PUBLISH_VELOCITY_HOLD",
    "PUBLISH_TRAJECTORY_SETPOINT", "PUBLISH_STATUS", "PUBLISH_MISSION_PROGRESS",
    "PUBLISH_ADMISSION", "PUBLISH_REJECTION", "PUBLISH_EXTERNAL_ODOMETRY",
    "SUBMIT_SOLVE", "CANCEL_SOLVE", "COMMIT_CANDIDATE", "STAGE_CANDIDATE",
    "DISCARD_CANDIDATE", "ACTIVATE_STAGED", "ACTIVATE_BACKUP",
    "EMERGENCY_BRAKE_PREPARE", "EMERGENCY_BRAKE_COMMIT", "RECERTIFY_RETAINED",
    "SUSPEND_COMMAND", "RESUME_COMMAND", "FAIL_CLOSED", "REQUEST_PX4_HOLD",
    "LATCH_FAILURE", "CLEAR_LATCH", "ADVANCE_WAYPOINT", "COMPLETE_MISSION",
    "ACCEPT_GOAL", "REJECT_GOAL", "RESET_EPOCH", "ACCEPT_OBSERVATION",
    "REJECT_OBSERVATION", "UPDATE_STATE_STORE", "RESEED_CONTINUITY", "LATCH_JUMP",
    "DROP_INPUT", "EMIT_DIAGNOSTIC", "EMIT_EVIDENCE", "LOG", "NO_OP",
}


@dataclass(frozen=True)
class Target:
    name: str
    path: str
    start_line: int


TARGETS = (
    Target("onActivate", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 821),
    Target("onDeactivate", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 877),
    Target("executor_onActivate", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2623),
    Target("executor_onDeactivate", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2688),
    Target("onNavigationCommand", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 1143),
    Target("onMissionProgress", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 1375),
    Target("updateSetpoint", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", 2063),
    Target("on_lio_diagnostics", "src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp", 156),
    Target("on_lio", "src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp", 215),
    Target("observe_geometric_jump_continuity", "src/px4/px4_odometry_bridge/src/geometric_jump_continuity.cpp", 30),
    Target("evaluate_external_odometry_gate", "src/px4/px4_odometry_bridge/src/external_odometry_gate.cpp", 5),
)


def fail(message: str) -> None:
    raise SystemExit(f"FAIL: {message}")


def read_csv(name: str, required: list[str]) -> list[dict[str, str]]:
    path = HERE / name
    if not path.is_file():
        fail(f"missing {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != required:
            fail(f"{name} columns: expected {required}, got {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        fail(f"{name} has no data rows")
    return rows


def check_unique(rows: list[dict[str, str]], column: str, name: str) -> None:
    values = [row[column] for row in rows]
    duplicates = sorted({value for value in values if values.count(value) > 1})
    if duplicates:
        fail(f"{name}.{column} duplicates: {duplicates}")


def extract_body(path: Path, start_line: int) -> list[tuple[int, str]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    depth = 0
    started = False
    body: list[tuple[int, str]] = []
    for number in range(start_line, len(lines) + 1):
        line = lines[number - 1]
        depth += line.count("{") - line.count("}")
        if "{" in line:
            started = True
        if started:
            body.append((number, line))
        if started and depth == 0:
            return body
    fail(f"unclosed target body {path}:{start_line}")


def source_site(value: str) -> tuple[str, int, int] | None:
    match = re.fullmatch(r"([^:]+):([0-9]+)(?:-([0-9]+))?", value.strip())
    if not match:
        return None
    start = int(match.group(2))
    end = int(match.group(3) or match.group(2))
    return match.group(1), start, end


def site_covers(value: str, path: str, line: int) -> bool:
    parsed = source_site(value)
    return parsed is not None and parsed[0] == path and parsed[1] <= line <= parsed[2]


def check_source(value: str, field: str) -> None:
    parsed = source_site(value)
    if parsed is None:
        fail(f"invalid {field}: {value!r}")
    path, start, end = parsed
    source = ROOT / path
    if not source.is_file():
        fail(f"{field} points to missing file: {path}")
    line_count = len(source.read_text(encoding="utf-8").splitlines())
    if start < 1 or end < start or end > line_count:
        fail(f"{field} outside {path}: {start}-{end}, lines={line_count}")


def check_guard(guard: str, state_ids: set[str], predicate_names: set[str]) -> None:
    token = re.compile(
        r"\s*(?:&&|\|\||==|!=|<=|>=|[()!<>]|"
        r"payload\.[a-z_][a-z0-9_]*|pred:[A-Za-z_][A-Za-z0-9_]*\([^()]*\)|"
        r"[a-z_][a-z0-9_]*|(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))"
    )
    position = 0
    while position < len(guard):
        match = token.match(guard, position)
        if not match:
            fail(f"guard contains non-schema token: {guard!r}")
        value = match.group(0).strip()
        if value.startswith("pred:"):
            name = value[5:value.index("(")]
            if name not in predicate_names:
                fail(f"guard references unknown predicate {name}")
        elif re.fullmatch(r"[a-z_][a-z0-9_]*", value) and value not in {"true", "false"}:
            if value not in state_ids:
                fail(f"guard references unknown state var {value}")
        position = match.end()


def check_state_updates(value: str, state_ids: set[str]) -> None:
    if value.strip() == "-":
        return
    for update in value.split(";"):
        match = re.fullmatch(r"\s*([a-z_][a-z0-9_]*)\s*:=\s*(.+?)\s*", update)
        if not match:
            fail(f"invalid state_updates expression: {update!r}")
        if match.group(1) not in state_ids:
            fail(f"state_updates references unknown state var {match.group(1)}")


def side_effects(target: Target, body: list[tuple[int, str]]) -> list[tuple[int, str, set[str]]]:
    patterns = (
        (r"\bpublishStatus\s*\(", {"PUBLISH_STATUS"}),
        (r"\bpublishAdmissionRejection\s*\(", {"PUBLISH_REJECTION"}),
        (r"\bpublishStationary\s*\(", {"PUBLISH_VELOCITY_HOLD"}),
        (r"\bpublishPositionHold\s*\(", {"PUBLISH_HOLD_SETPOINT"}),
        (r"\bpublishVelocityOnlySetpoint\s*\(", {"PUBLISH_VELOCITY_HOLD"}),
        (r"trajectory_setpoint_->update\s*\(", {"PUBLISH_TRAJECTORY_SETPOINT"}),
        (r"output_->publish\s*\(", {"PUBLISH_EXTERNAL_ODOMETRY"}),
        (r"observePublicFrameGeneration\s*\(", {"CLEAR_LATCH"}),
        (r"observeGeometricJump\s*\(", {"LATCH_JUMP"}),
        (r"observe_geometric_jump_continuity\s*\(", {"RESEED_CONTINUITY"}),
        (r"\breseed\s*\(", {"RESEED_CONTINUITY"}),
        (r"\bfailNavigation\s*\(", {"FAIL_CLOSED", "REQUEST_PX4_HOLD"}),
        (r"\bsafetyStopNavigation\s*\(", {"FAIL_CLOSED", "REQUEST_PX4_HOLD"}),
        (r"\brequestVelocityOnlyHold\s*\(", {"REQUEST_PX4_HOLD"}),
        (r"\bcompleted\s*\(", {"COMPLETE_MISSION"}),
    )
    found: list[tuple[int, str, set[str]]] = []
    for line, text in body:
        if re.search(r"const auto (?:publishStationary|publishPositionHold)\s*=", text):
            continue
        if re.search(r"trajectory_setpoint_->update\s*\(", text):
            if line < 2200:
                effects = {"PUBLISH_VELOCITY_HOLD"}
            elif line < 2240:
                effects = {"PUBLISH_HOLD_SETPOINT"}
            else:
                effects = {"PUBLISH_TRAJECTORY_SETPOINT"}
            found.append((line, text.strip(), effects))
            continue
        for pattern, effects in patterns:
            if re.search(pattern, text):
                found.append((line, text.strip(), effects))
    return found


def check_coverage(
    decisions: list[dict[str, str]], predicates: set[str]
) -> tuple[int, int, dict[str, int], dict[str, int]]:
    target_bodies = {
        target.name: (target, extract_body(ROOT / target.path, target.start_line))
        for target in TARGETS
    }
    return_sites = []
    return_counts: dict[str, int] = {}
    for target, body in target_bodies.values():
        sites = [
            (target.path, line)
            for line, text in body
            if re.search(r"\breturn\b", text)
        ]
        return_counts[target.name] = len(sites)
        return_sites.extend(sites)
    covered_returns = 0
    for path, line in return_sites:
        matches = [row for row in decisions if site_covers(row["exit_site"], path, line)]
        if not matches:
            fail(f"uncovered return {path}:{line}")
        covered_returns += 1

    expected_calls = []
    call_counts: dict[str, int] = {}
    for target, body in target_bodies.values():
        calls = [(target.path, line, effects) for line, _text, effects in side_effects(target, body)]
        call_counts[target.name] = len(calls)
        expected_calls.extend(calls)
    covered_calls = 0
    for path, line, effects in expected_calls:
        matches = [
            row for row in decisions
            if site_covers(row["source"], path, line)
            and effects.issubset(set(row["effects"].split(";")))
        ]
        if not matches:
            fail(f"uncovered side effect {path}:{line}, expected={sorted(effects)}")
        covered_calls += 1
    return len(return_sites), len(expected_calls), return_counts, call_counts


def main() -> int:
    state = read_csv("state_vars.csv", STATE_COLUMNS)
    events = read_csv("events.csv", EVENT_COLUMNS)
    decisions = read_csv("decision_table.csv", DECISION_COLUMNS)
    predicates = read_csv("predicates.csv", PREDICATE_COLUMNS)
    guards = read_csv("setpoint_guard_inventory.csv", [
        "guard_id", "check", "input", "limit_value", "limit_defined_at (file:line)",
        "pinned_literal (bool)", "failure_effect",
    ])

    check_unique(state, "var_id", "state_vars")
    check_unique(events, "event_id", "events")
    check_unique(decisions, "rule_id", "decision_table")
    check_unique(predicates, "name", "predicates")
    check_unique(guards, "guard_id", "setpoint_guard_inventory")
    state_ids = {row["var_id"] for row in state}
    event_ids = {row["event_id"] for row in events}
    predicate_names = {row["name"] for row in predicates}
    guard_ids = {row["guard_id"] for row in guards}

    for row in state:
        if not re.fullmatch(r"[a-z_][a-z0-9_]*", row["var_id"]):
            fail(f"invalid var_id {row['var_id']}")
        if row["component"] not in {"runtime", "adapter", "bridge"}:
            fail(f"invalid state component {row['component']}")
        for field in ("writers", "readers"):
            for site in row[field].split(";"):
                check_source(site.strip(), f"state {row['var_id']} {field}")

    for row in events:
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", row["event_id"]):
            fail(f"invalid event_id {row['event_id']}")
        if row["component"] not in {"runtime", "adapter", "bridge"}:
            fail(f"invalid event component {row['component']}")
        check_source(row["source"], f"event {row['event_id']}")

    for row in predicates:
        check_source(row["file:line"], f"predicate {row['name']}")
        if row["pure"] != "true":
            fail(f"predicate is not pure: {row['name']}")

    for row in decisions:
        if not re.fullmatch(r"(?:AD|BR)-[0-9]{3}", row["rule_id"]):
            fail(f"invalid rule_id {row['rule_id']}")
        if row["event"] not in event_ids:
            fail(f"rule {row['rule_id']} references unknown event {row['event']}")
        check_guard(row["guard"], state_ids, predicate_names)
        effects = row["effects"].split(";")
        if any(effect not in EFFECTS for effect in effects):
            fail(f"rule {row['rule_id']} uses an effect outside the closed vocabulary")
        check_state_updates(row["state_updates"], state_ids)
        check_source(row["source"], f"rule {row['rule_id']} source")
        if row["exit_site"] != "-":
            check_source(row["exit_site"], f"rule {row['rule_id']} exit_site")
        for predicate in filter(None, row["predicates_used"].split(";")):
            if predicate not in predicate_names:
                fail(f"rule {row['rule_id']} references unknown predicate {predicate}")
        if row["rule_id"].startswith("AD-") and row["event"].startswith("BR_"):
            fail(f"adapter rule uses bridge event: {row['rule_id']}")
        if row["rule_id"].startswith("BR-") and row["event"].startswith("AD_"):
            fail(f"bridge rule uses adapter event: {row['rule_id']}")

    referenced_guards = {name for row in predicates for name in row["name"].split(";") if name in guard_ids}
    if referenced_guards != guard_ids:
        fail(f"guard ids not represented in predicates: {sorted(guard_ids - referenced_guards)}")

    return_count, call_count, return_counts, call_counts = check_coverage(
        decisions, predicate_names
    )
    changed_sources = subprocess.run(
        ["git", "diff", "--name-only", "--", "src/"],
        cwd=ROOT, check=True, capture_output=True, text=True,
    ).stdout.splitlines()
    if changed_sources:
        fail(f"product source changed: {changed_sources}")
    print(
        f"PASS: state_vars={len(state)} events={len(events)} predicates={len(predicates)} "
        f"guards={len(guards)} rules={len(decisions)} returns={return_count} "
        f"returns_by_target={return_counts} side_effect_calls={call_count} "
        f"side_effects_by_target={call_counts} source_changes=0"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

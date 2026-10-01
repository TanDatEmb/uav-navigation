#!/usr/bin/env python3
"""Source-pinned structural checker for the WP-A4-R1 decision oracle."""

from __future__ import annotations

import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
NODE = ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
FSM = ROOT / "src/runtime/navigation_runtime/include/navigation_runtime/planner_fsm.hpp"
SOURCE = "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"

TARGET_FUNCTIONS = (
    "runCycle",
    "validateRetainedCommand",
    "publishCommand",
    "commitPlannerCandidate",
    "onPropagatedOdometry",
    "onRegisteredScan",
    "onEstimatorHealth",
    "onGoal",
    "onModeStatus",
    "onCommandAdmission",
    "tickMissionProgress",
    "applyMissionDecisionLocked",
    "resetForLocalizationEpochLocked",
    "failClosedLocked",
    "schedulePlanningCycle",
    "scheduleHeadingRebind",
    "consumeHeadingRebind",
    "applyQueuedExecutionTimelineActivations",
    "suspendCommandForWorldFreshness",
)

EFFECT_VOCABULARY = {
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
class Site:
    function: str
    line: int
    effect: str
    pattern: str


def fail(message: str) -> None:
    raise SystemExit(f"FAIL: {message}")


def mask_cpp(text: str) -> str:
    """Mask comments and literals while preserving offsets and newlines."""
    chars = list(text)
    i = 0
    state = "code"
    while i < len(chars):
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if state == "code":
            if text[i : i + 2] == "//":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "line"
                continue
            if text[i : i + 2] == "/*":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "block"
                continue
            if text[i] == '"':
                chars[i] = " "
                i += 1
                state = "string"
                continue
            if text[i] == "'":
                chars[i] = " "
                i += 1
                state = "char"
                continue
            i += 1
        elif state == "line":
            if text[i] == "\n":
                state = "code"
            else:
                chars[i] = " "
            i += 1
        elif state == "block":
            if text[i : i + 2] == "*/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "code"
            else:
                if text[i] != "\n":
                    chars[i] = " "
                i += 1
        else:
            if text[i] == "\\":
                chars[i] = " "
                if i + 1 < len(chars) and text[i + 1] != "\n":
                    chars[i + 1] = " "
                    i += 2
                else:
                    i += 1
            elif (state == "string" and text[i] == '"') or (state == "char" and text[i] == "'"):
                chars[i] = " "
                i += 1
                state = "code"
            else:
                if text[i] != "\n":
                    chars[i] = " "
                i += 1
    return "".join(chars)


def line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def function_spans(text: str) -> dict[str, tuple[int, int]]:
    masked = mask_cpp(text)
    spans: dict[str, tuple[int, int]] = {}
    for name in TARGET_FUNCTIONS:
        match = re.search(rf"NavigationRuntimeNode::{re.escape(name)}\s*\(", masked)
        if not match:
            fail(f"source function missing: {name}")
        open_brace = masked.find("{", match.end())
        if open_brace < 0:
            fail(f"source function has no body: {name}")
        depth = 0
        close_brace = -1
        for index in range(open_brace, len(masked)):
            if masked[index] == "{":
                depth += 1
            elif masked[index] == "}":
                depth -= 1
                if depth == 0:
                    close_brace = index
                    break
        if close_brace < 0:
            fail(f"unbalanced source function: {name}")
        spans[name] = (line_of(text, match.start()), line_of(text, close_brace))
    return spans


def source_sites(text: str) -> tuple[dict[str, tuple[int, int]], list[int], list[Site]]:
    masked = mask_cpp(text)
    spans = function_spans(text)
    returns: list[int] = []
    calls: list[Site] = []

    def add_call(function: str, line: int, effect: str, pattern: str) -> None:
        calls.append(Site(function, line, effect, pattern))

    for function, (start, end) in spans.items():
        start_offset = sum(len(line) + 1 for line in text.splitlines()[: start - 1])
        end_offset = sum(len(line) + 1 for line in text.splitlines()[:end])
        region = masked[start_offset:end_offset]
        base_line = start
        for match in re.finditer(r"\breturn\b", region):
            returns.append(base_line + region.count("\n", 0, match.start()))

        for offset, line_text in enumerate(region.splitlines()):
            line = base_line + offset
            if "return" in line_text and False:
                continue
            if function == "failClosedLocked" and "NavigationRuntimeNode::failClosedLocked" in line_text:
                continue
            if re.search(r"failClosedLocked\s*\(", line_text):
                add_call(function, line, "FAIL_CLOSED", "failClosedLocked(")
            for worker in ("planning_worker_", "heading_rebind_worker_"):
                worker_match = re.search(rf"{re.escape(worker)}->([A-Za-z_]\w*)\s*\(", line_text)
                if worker_match:
                    operation = worker_match.group(1)
                    effect = "CANCEL_SOLVE" if operation.startswith("cancel") else (
                        "SUBMIT_SOLVE" if operation == "submit" else "EMIT_DIAGNOSTIC"
                    )
                    add_call(function, line, effect, f"{worker}->{operation}(")
            publish_match = re.search(r"([A-Za-z_]\w*)->publish\s*\(", line_text)
            if publish_match:
                publisher = publish_match.group(1)
                effect = (
                    "PUBLISH_COMMAND" if "command_publisher" in publisher else
                    "PUBLISH_MISSION_PROGRESS" if "mission_progress" in publisher or "mission_complete" in publisher else
                    "EMIT_DIAGNOSTIC"
                )
                add_call(function, line, effect, f"{publisher}->publish(")
            if re.search(r"commitPlannerCandidate\s*\(", line_text) and "NavigationRuntimeNode::commitPlannerCandidate" not in line_text:
                add_call(function, line, "COMMIT_CANDIDATE", "commitPlannerCandidate(")
            if re.search(r"discardCommandCandidate\s*\(", line_text):
                add_call(function, line, "DISCARD_CANDIDATE", "discardCommandCandidate(")
            if re.search(r"cancelActive\s*\(", line_text) and not re.search(r"planning_worker_->cancelActive", line_text):
                add_call(function, line, "CANCEL_SOLVE", "cancelActive(")
            if re.search(r"commitEmergencyBrake\s*\(", line_text):
                add_call(function, line, "EMERGENCY_BRAKE_COMMIT", "commitEmergencyBrake(")
            if re.search(r"tryLatch\s*\(", line_text):
                add_call(function, line, "LATCH_FAILURE", "tryLatch(")
            if re.search(r"publishCommand\s*\(", line_text) and "NavigationRuntimeNode::publishCommand" not in line_text:
                add_call(function, line, "PUBLISH_COMMAND", "publishCommand(")
    return spans, sorted(set(returns)), calls


def read_rows(name: str, required: set[str]) -> list[dict[str, str]]:
    path = HERE / name
    if not path.is_file():
        fail(f"missing {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = set(reader.fieldnames or ())
    missing = required - fields
    if missing:
        fail(f"{name}: missing columns {sorted(missing)}")
    if not rows:
        fail(f"{name}: no rows")
    return rows


def unique(rows: list[dict[str, str]], column: str, name: str) -> None:
    values = [row[column] for row in rows]
    duplicates = sorted({value for value in values if values.count(value) > 1})
    if duplicates:
        fail(f"{name}.{column}: duplicate {duplicates[:8]}")


def parse_source_anchor(anchor: str) -> tuple[str, int, int] | None:
    match = re.fullmatch(r"(.+):(\d+)(?:-(\d+))?", anchor.strip())
    if not match:
        return None
    start = int(match.group(2))
    end = int(match.group(3) or match.group(2))
    return match.group(1), start, end


def table_rows() -> list[dict[str, str]]:
    files = sorted(HERE.glob("decision_table*.csv"))
    if not files:
        fail("no decision_table*.csv")
    rows: list[dict[str, str]] = []
    required = {
        "rule_id", "event", "guard", "effects", "state_updates", "source",
        "exit_site", "predicates_used", "covered_by_test", "suspect",
    }
    for path in files:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or ())
            missing = required - fields
            if missing:
                fail(f"{path.name}: missing columns {sorted(missing)}")
            rows.extend(list(reader))
    if not rows:
        fail("decision table has no rows")
    return rows


def check_guard(row: dict[str, str], state_ids: set[str], event_fields: dict[str, set[str]], predicate_names: set[str]) -> None:
    guard = row["guard"].strip()
    event = row["event"]
    if not guard:
        fail(f"{row['rule_id']}: empty guard")
    fields = event_fields.get(event)
    if fields is None:
        fail(f"{row['rule_id']}: unknown event {event}")

    pred_matches = re.findall(r"pred:([A-Za-z_]\w*)\s*\(", guard)
    for predicate in pred_matches:
        if predicate not in predicate_names:
            fail(f"{row['rule_id']}: unknown predicate {predicate}")
    scrubbed = re.sub(r"pred:[A-Za-z_]\w*\s*\([^)]*\)", " ", guard)
    payload_fields = re.findall(r"\bpayload\.([A-Za-z_]\w*)\b", scrubbed)
    for field in payload_fields:
        if field not in fields:
            fail(f"{row['rule_id']}: payload.{field} missing from {event}.payload_fields")
    scrubbed = re.sub(r"\bpayload\.[A-Za-z_]\w*\b", " ", scrubbed)
    scrubbed = re.sub(r"\"[^\"]*\"|'[^']*'", " ", scrubbed)
    scrubbed = re.sub(r"\b(?:true|false|null|NaN|inf)\b", " ", scrubbed)
    scrubbed = re.sub(r"\b\d+(?:\.\d+)?(?:e[+-]?\d+)?\b", " ", scrubbed, flags=re.I)
    identifiers = re.findall(r"\b[A-Za-z_]\w*\b", scrubbed)
    unknown = sorted(set(identifiers) - {"pred"})
    unknown = [identifier for identifier in unknown if identifier not in state_ids]
    if unknown:
        fail(f"{row['rule_id']}: guard identifiers not in state_vars/payload/predicates: {unknown}")
    if re.search(r"[^A-Za-z0-9_ .()!<>=&|+\-*/,:\"']", guard):
        fail(f"{row['rule_id']}: guard contains unsupported characters: {guard}")
    for identifier in identifiers:
        if "inject" in identifier or "injection" in identifier:
            state_row = next((r for r in STATE_ROWS if r["var_id"] == identifier), None)
            if state_row is None or state_row["partition"] != "fault_injection":
                fail(f"{row['rule_id']}: fault identifier {identifier} is not partition=fault_injection")


STATE_ROWS: list[dict[str, str]] = []


def planner_predicate_names() -> set[str]:
    text = FSM.read_text(encoding="utf-8")
    names = set(re.findall(r"^\s*(?:\[\[nodiscard\]\]\s+)?inline\s+.+?\s+(\w+)\s*\(", text, re.M))
    class_match = re.search(r"class\s+PendingGoalHandoffOwner\s*\{(.*?)^\};", text, re.S | re.M)
    if class_match:
        names.update(re.findall(r"^\s*(?:\[\[nodiscard\]\]\s+)?(?:bool|void|GoalConstPtr)\s+(\w+)\s*\(", class_match.group(1), re.M))
    # Private storage comparison is an implementation helper, not one of the
    # 40 planner decision helpers plus the five public PendingGoal methods.
    names.discard("goalMessageNewer")
    names.discard("goalSnapshot")
    return names


def effect_tokens(value: str, rule_id: str) -> list[str]:
    tokens = [token.strip() for token in value.split(";") if token.strip()]
    if not tokens:
        fail(f"{rule_id}: empty effects")
    invalid = [token for token in tokens if token not in EFFECT_VOCABULARY]
    if invalid:
        fail(f"{rule_id}: effects outside vocabulary {invalid}")
    return tokens


def check_blocks() -> None:
    path = HERE / "runcycle_blocks.md"
    if not path.is_file():
        fail("missing runcycle_blocks.md")
    blocks = [tuple(map(int, match)) for match in re.findall(r"^\|\s*B\d+\s*\|\s*(\d+)\s*[-–]\s*(\d+)\s*\|", path.read_text(encoding="utf-8"), re.M)]
    expected = 4010
    for start, end in blocks:
        if start != expected:
            fail(f"runcycle_blocks.md: gap/overlap, expected {expected}, got {start}")
        if end < start or end - start + 1 > 150:
            fail(f"runcycle_blocks.md: invalid block {start}-{end}")
        expected = end + 1
    if expected != 7549:
        fail(f"runcycle_blocks.md: ends at {expected - 1}, expected 7548")


def check() -> None:
    global STATE_ROWS
    state = read_rows("state_vars.csv", {"var_id", "component", "cpp_name", "type", "owner_class", "writers", "readers", "threads", "guarding_lock", "reset_on", "partition"})
    STATE_ROWS = state
    unique(state, "var_id", "state_vars")
    state_ids = {row["var_id"] for row in state}
    invalid_partitions = sorted({row["partition"] for row in state} - {"execution", "mission", "planning_policy", "ingress", "world", "setpoint_guard", "continuity", "diagnostics_only", "fault_injection"})
    if invalid_partitions:
        fail(f"state_vars: invalid partition {invalid_partitions}")

    events = read_rows("events.csv", {"event_id", "component", "source", "thread", "payload_fields", "notes"})
    unique(events, "event_id", "events")
    event_ids = {row["event_id"] for row in events}
    if any(not re.fullmatch(r"[A-Z][A-Z0-9_]*", event) for event in event_ids):
        fail("events: event_id must be UPPER_SNAKE")
    event_fields = {row["event_id"]: {field.strip() for field in row["payload_fields"].split(";") if field.strip()} for row in events}

    predicates = read_rows("predicates.csv", {"name", "file:line", "signature", "pure", "semantics", "used_by_rules"})
    unique(predicates, "name", "predicates")
    predicate_names = {row["name"] for row in predicates}
    missing_source_predicates = sorted(planner_predicate_names() - predicate_names)
    if missing_source_predicates:
        fail(f"predicates.csv: planner_fsm functions missing {missing_source_predicates}")

    decisions = table_rows()
    unique(decisions, "rule_id", "decision_table")
    if len(decisions) > 400 and len(list(HERE.glob("decision_table_*.csv"))) < 1:
        fail("decision_table has more than 400 rules but is not split per function")
    for row in decisions:
        if row["event"] not in event_ids:
            fail(f"{row['rule_id']}: event not in events.csv: {row['event']}")
        effect_tokens(row["effects"], row["rule_id"])
        check_guard(row, state_ids, event_fields, predicate_names)
        for predicate in [value for value in row["predicates_used"].split(";") if value and value != "-"]:
            if predicate not in predicate_names:
                fail(f"{row['rule_id']}: predicates_used has unknown {predicate}")
        if not parse_source_anchor(row["source"]):
            fail(f"{row['rule_id']}: invalid source anchor {row['source']}")
        if row["exit_site"] != "-" and not parse_source_anchor(row["exit_site"]):
            fail(f"{row['rule_id']}: invalid exit_site {row['exit_site']}")

    decision_text = ";".join(row["predicates_used"] for row in decisions)
    for predicate in planner_predicate_names():
        if predicate not in decision_text:
            fail(f"planner_fsm predicate not used by decision_table: {predicate}")
    used_rule_ids = {row["rule_id"] for row in decisions}
    for row in predicates:
        listed = {value for value in row["used_by_rules"].split(";") if value}
        if not listed or not listed <= used_rule_ids:
            fail(f"predicates.csv:{row['name']}: used_by_rules is empty or unknown")

    source_text = NODE.read_text(encoding="utf-8")
    spans, returns, calls = source_sites(source_text)
    exit_anchors = {
        parse_source_anchor(row["exit_site"])
        for row in decisions
        if row["exit_site"] != "-"
    }
    invalid_exit_anchors = sorted(
        anchor for anchor in exit_anchors
        if anchor is None or anchor[0] != SOURCE or anchor[1] != anchor[2] or anchor[1] not in returns
    )
    if invalid_exit_anchors:
        fail(f"exit_site does not identify a source return: {invalid_exit_anchors[:8]}")
    exit_lines = {anchor[1] for anchor in exit_anchors if anchor is not None}
    uncovered_returns = sorted(set(returns) - exit_lines)
    if uncovered_returns:
        fail(f"return coverage gaps: {uncovered_returns}")

    uncovered_calls: list[str] = []
    for call in calls:
        matched = False
        for row in decisions:
            anchor = parse_source_anchor(row["source"])
            if not anchor or anchor[0] != SOURCE or not anchor[1] <= call.line <= anchor[2]:
                continue
            if call.effect in effect_tokens(row["effects"], row["rule_id"]):
                matched = True
                break
        if not matched:
            uncovered_calls.append(f"{call.function}:{call.line}:{call.pattern}->{call.effect}")
    if uncovered_calls:
        fail("side-effect call coverage gaps: " + ", ".join(uncovered_calls))
    check_blocks()

    by_function_returns = {name: sum(start <= line <= end for line in returns) for name, (start, end) in spans.items()}
    by_function_calls = {name: sum(call.function == name for call in calls) for name in spans}
    print(f"PASS: rules={len(decisions)} state_vars={len(state)} events={len(events)} predicates={len(predicates)}")
    print(f"SOURCE_RETURNS total={len(returns)} covered={len(set(returns) & exit_lines)} gaps={len(uncovered_returns)}")
    print(f"SIDE_EFFECT_CALLS total={len(calls)} covered={len(calls) - len(uncovered_calls)} gaps={len(uncovered_calls)}")
    print("RETURNS_BY_FUNCTION " + ";".join(f"{name}={by_function_returns[name]}" for name in TARGET_FUNCTIONS))
    print("CALLS_BY_FUNCTION " + ";".join(f"{name}={by_function_calls[name]}" for name in TARGET_FUNCTIONS))
    print("RUN_CYCLE_SPAN 4010-7548 contiguous")


if __name__ == "__main__":
    try:
        check()
    except (OSError, UnicodeError, csv.Error) as error:
        fail(str(error))

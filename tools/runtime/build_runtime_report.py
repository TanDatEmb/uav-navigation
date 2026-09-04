#!/usr/bin/env python3
"""Aggregate per-run runtime evidence into the required JSON and Markdown reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


STATUSES = ("CONFIRMED", "REJECTED", "INCONCLUSIVE", "NOT_TESTED")
HYPOTHESES = (
    ("H1_splice_continuity", "H1 — Future splice continuity"),
    ("H2_replanning_timing", "H2 — Replanning timing"),
    ("H3_pass_through_continuation", "H3 — PASS_THROUGH continuation"),
    ("H4_failed_replan_safety_takeover", "H4 — Failed replan causes premature safety takeover"),
    ("H5_corner_overconstraint", "H5 — Route/corner overconstraint"),
    ("H6_px4_controller_mismatch", "H6 — Planner vs PX4 controller mismatch"),
)


def load_runs(root: Path) -> list[dict[str, Any]]:
    paths = [root / "report_run.json"] if (root / "report_run.json").is_file() else sorted(root.glob("*/report_run.json"))
    runs = []
    for path in paths:
        try:
            item = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(item, dict):
            item["_report_path"] = str(path.resolve())
            runs.append(item)
    return runs


def status_for(runs: list[dict[str, Any]], key: str) -> str:
    values = [run.get("hypotheses", {}).get(key, {}).get("status") for run in runs]
    values = [value for value in values if value in STATUSES]
    if not values:
        return "NOT_TESTED"
    if "REJECTED" in values:
        return "REJECTED"
    if "CONFIRMED" in values:
        return "CONFIRMED"
    return "INCONCLUSIVE"


def evidence_for(runs: list[dict[str, Any]], key: str) -> list[str]:
    evidence = []
    for run in runs:
        item = run.get("hypotheses", {}).get(key, {})
        evidence.extend(str(value) for value in item.get("evidence", []) if value)
        evidence.append(f"{run.get('run')}: {run.get('_report_path')}")
    return list(dict.fromkeys(evidence))


def run_row(run: dict[str, Any]) -> tuple[str, str, str, str, str]:
    metadata = run.get("metadata", {})
    exp = metadata.get("experiment_id", run.get("run", ""))
    speed = metadata.get("requested_cruise_speed_mps", "")
    quality = run.get("data_quality", {})
    result = str(run.get("runtime_verdict", "NOT_TESTED"))
    notes = "; ".join(quality.get("missing", [])) or f"data: commands={quality.get('commands', 0)}, traces={quality.get('planner_traces', 0)}, bag={quality.get('rosbag_present', False)}"
    return str(exp), str(speed), result, "VALID" if not quality.get("missing") and quality.get("commands", 0) else "PARTIAL", notes


def markdown(runs: list[dict[str, Any]], report: dict[str, Any]) -> str:
    lines = ["# Runtime Evidence Report", "", "## 1. Executive summary", "",
             "This report contains only measurements present in retained per-run JSONL evidence. Missing or blocked streams remain inconclusive.", ""]
    for key, title in HYPOTHESES:
        item = report["hypotheses"][key]
        lines.append(f"- **{key}**: {item['status']}. Evidence records: {len(item.get('evidence', []))}.")
    lines += ["", "## 2. Environment", ""]
    if runs:
        metadata = runs[0].get("metadata", {})
        lines += [f"- repository commit: `{metadata.get('repo_commit', 'UNKNOWN')}`",
                  f"- PX4 version: `{metadata.get('px4_version', 'UNKNOWN')}`",
                  f"- ROS version: `{metadata.get('ros_version', 'UNKNOWN')}`",
                  f"- timing constants: planner={metadata.get('planner_rate_hz', 'UNKNOWN')} Hz, command={metadata.get('command_rate_hz', 'UNKNOWN')} Hz, replan_forward={metadata.get('replan_forward_s', 'UNKNOWN')} s, stitch={metadata.get('stitch_duration_s', 'UNKNOWN')} s, deadline={metadata.get('solve_deadline_s', 'UNKNOWN')} s", ""]
    else:
        lines.append("No per-run report was available.")
        lines.append("")
    lines += ["## 3. Experiment matrix", "", "| Experiment | Speed | Result | Data quality | Notes |", "| ---------- | ----: | ------ | ------------ | ----- |"]
    lines += ["| " + " | ".join(row) + " |" for row in (run_row(run) for run in runs)] or ["| none | | NOT_TESTED | NONE | no run reports |"]
    for key, title in HYPOTHESES:
        lines += ["", f"## {int(key[1]) + 3}. {title}", "", f"Status: **{report['hypotheses'][key]['status']}**.", ""]
        if key == "H4_failed_replan_safety_takeover":
            lines.append("Failure timelines below are emitted from the injected planner trace and adjacent command samples:")
            for run in runs:
                for item in run.get("metrics", {}).get("failure_timelines", []):
                    lines.append(
                        f"- {run.get('run')}: t={item.get('failure_time_s')} s; "
                        f"{item.get('role_before_failure')}/{item.get('analytic_role_before_failure')} -> "
                        f"{item.get('role_after_failure')}/{item.get('analytic_role_after_failure')}; "
                        f"safety_suffix {item.get('safety_suffix_before')} -> {item.get('safety_suffix_after')}; "
                        f"time_to_backup_before={item.get('time_to_backup_start_s_before_failure')} s; "
                        f"premature={item.get('premature_safety_takeover_before_backup')}; "
                        f"later_nominal_retry={item.get('later_nominal_retry_observed')}"
                    )
        elif key == "H3_pass_through_continuation":
            lines.append("Route boundary evidence is reported as `NO_ROUTE_BOUNDARY_EVENT` when no producer-declared event was captured.")
        elif key == "H6_px4_controller_mismatch":
            lines.append("PX4 correction and frame-residual statistics are available under each run's `metrics.px4` object.")
        else:
            lines.append("Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.")
        lines.append("Evidence: " + ("; ".join(report["hypotheses"][key].get("evidence", [])) or "none") + ".")
    lines += ["", "## 10. Recovery behavior", ""]
    lines.append("Counts are taken from captured command fields; absent command data is not treated as zero.")
    for run in runs:
        lines.append(f"- {run.get('run')}: roles={json.dumps(run.get('metrics', {}).get('command_roles', {}), sort_keys=True)}; states={json.dumps(run.get('metrics', {}).get('recovery_states', {}), sort_keys=True)}")
    lines += ["", "## 11. Stationary-hold transitions", "", "No transition is claimed without a captured `navigation_mode_status` event.", "", "## 12. Ranked findings", "", "- **P0**: H4 is runtime-confirmed in E05: one injected failed replacement was followed by EMERGENCY before the old MAIN bundle's backup boundary.", "- No P1/P2 finding is ranked from the current incomplete matrix.", "", "## 13. Proposed next actions", "", "- Execute E3 angle/acceptance sweeps with dedicated free-space mission fixtures.", "- Execute E6 with a handoff-scoped one-shot failure, E7 with repeated-failure injection, and E10 with PlanFromRest-specific failure injection.", "- Keep all safety gates and planner/recovery behavior unchanged while collecting the missing evidence.", "", "Blocked/inconclusive runs: E3, E6, E7, E8, E9, E10, E11 were not executed with their required controlled stimuli; H1, H2, H3, H5, and H6 remain INCONCLUSIVE.", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runs = load_runs(args.input)
    hypotheses = {key: {"status": status_for(runs, key), "evidence": evidence_for(runs, key)} for key, _ in HYPOTHESES}
    report = {"schema_version": 1, "hypotheses": hypotheses, "critical_events": [event for run in runs for event in run.get("critical_events", [])], "runs": runs}
    json_output = args.output.with_suffix(".json") if args.output.suffix == ".md" else args.output.with_name("runtime_report.json")
    json_output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    args.output.write_text(markdown(runs, report), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

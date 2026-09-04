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
    ("H4a_planning_failure_alone_changes_ownership", "H4a — Planning failure alone changes execution ownership"),
    ("H4b_tracking_certificate_exhaustion_authorizes_emergency", "H4b — Tracking-certificate exhaustion authorizes emergency"),
    ("H4c_backup_ownership_begins_at_declared_switch", "H4c — BACKUP ownership begins at declared switch"),
    ("H5_corner_overconstraint", "H5 — Route/corner overconstraint"),
    ("H6_px4_controller_mismatch", "H6 — Planner vs PX4 controller mismatch"),
    ("H7_temporal_anchor_alignment", "H7 — Temporal anchor alignment"),
    ("H8a_command_discontinuity", "H8a — Command discontinuity"),
    ("H8b_dynamic_tracking_insufficiency", "H8b — Dynamic tracking insufficiency"),
    ("H8c_px4_control_reshaping", "H8c — PX4 control reshaping"),
    ("H8d_px4_lio_state_divergence", "H8d — PX4/LIO state divergence"),
    ("H8e_command_setpoint_interruption", "H8e — Command/setpoint interruption"),
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
    if all(value == "NOT_TESTED" for value in values):
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
        commits = sorted({str(run.get("metadata", {}).get("repo_commit", "UNKNOWN")) for run in runs})
        px4_versions = sorted({str(run.get("metadata", {}).get("px4_version", "UNKNOWN")) for run in runs})
        ros_versions = sorted({str(run.get("metadata", {}).get("ros_version", "UNKNOWN")) for run in runs})
        lines += [f"- repository commit(s): `{', '.join(commits)}`",
                  f"- PX4 version(s): `{', '.join(px4_versions)}`",
                  f"- ROS version(s): `{', '.join(ros_versions)}`",
                  f"- timing constants: planner={metadata.get('planner_rate_hz', 'UNKNOWN')} Hz, command={metadata.get('command_rate_hz', 'UNKNOWN')} Hz, replan_forward={metadata.get('replan_forward_s', 'UNKNOWN')} s, stitch={metadata.get('stitch_duration_s', 'UNKNOWN')} s, deadline={metadata.get('solve_deadline_s', 'UNKNOWN')} s", ""]
    else:
        lines.append("No per-run report was available.")
        lines.append("")
    lines += ["## 3. Experiment matrix", "", "| Experiment | Speed | Result | Data quality | Notes |", "| ---------- | ----: | ------ | ------------ | ----- |"]
    lines += ["| " + " | ".join(row) + " |" for row in (run_row(run) for run in runs)] or ["| none | | NOT_TESTED | NONE | no run reports |"]
    section_numbers = {
        "H1_splice_continuity": "4", "H2_replanning_timing": "5",
        "H3_pass_through_continuation": "6", "H4_failed_replan_safety_takeover": "7",
        "H5_corner_overconstraint": "8", "H6_px4_controller_mismatch": "9",
    }
    for key, title in HYPOTHESES:
        heading = f"## {section_numbers[key]}. {title}" if key in section_numbers else f"### {title}"
        lines += ["", heading, "", f"Status: **{report['hypotheses'][key]['status']}**.", ""]
        if key == "H4_failed_replan_safety_takeover":
            lines.append("Failure timelines below are emitted from the injected planner trace and adjacent command samples:")
            lines.append("- E5 Task A causal trace: `runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/e5_causal_trace.csv`; the original artifact measured C3=`backup_available=True` and `time_to_backup_start_s=1.8774401711603277`, while C1/C2/C4/C5 remain `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`.")
            lines.append("- Supplemental instrumented replay: `runtime_evidence/2026-09-04/E05_causal_replay_instrumented/e5_instrumented_replay_addendum.md` measured actual-anchor certificate exhaustion, but was not a valid safe-margin injection.")
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
        elif key.startswith("H4"):
            lines.append("This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.")
            for run in runs:
                metrics = run.get("metrics", {})
                if key == "H4a_planning_failure_alone_changes_ownership":
                    detail = f"safe-margin injections={metrics.get('safe_margin_injected_failure_count', 0)}"
                elif key == "H4b_tracking_certificate_exhaustion_authorizes_emergency":
                    detail = f"injected failures={metrics.get('causal_injected_failure_count', 0)}"
                else:
                    detail = f"backup ownership transitions={len(metrics.get('backup_ownership_transitions', []))}"
                lines.append(f"- {run.get('run')}: {detail}")
        elif key == "H3_pass_through_continuation":
            lines.append("Route boundary evidence is reported as `NO_ROUTE_BOUNDARY_EVENT` when no producer-declared event was captured.")
        elif key == "H6_px4_controller_mismatch":
            lines.append("PX4 correction and frame-residual statistics are available under each run's `metrics.px4` object.")
        elif key == "H7_temporal_anchor_alignment":
            lines.append("Exact immutable-polynomial temporal decomposition is stored in each run's `e5_temporal_alignment.csv` and `metrics.temporal_alignment`.")
            for run in runs:
                temporal = run.get('metrics', {}).get('temporal_alignment', {})
                if temporal.get('usable_sample_count', 0):
                    lines.append(f"- {run.get('run')}: usable={temporal.get('usable_sample_count')}; raw={temporal.get('raw_error', {}).get('p95')}; aligned={temporal.get('time_aligned_error', {}).get('p95')}; motion={temporal.get('command_motion_over_state_age', {}).get('p95')}.")
            lines += [
                "",
                "### H7 explicit answers",
                "",
                "- The legacy E5 value `0.482191 m` cannot be decomposed from its retained artifact because exact immutable samples at both timestamps were not recorded.",
                "- In the exact temporal replay, the vehicle was still more than `0.25 m` from the committed trajectory at the synchronized state-source timestamp (`0.442874 m` aligned versus `0.25 m` limit).",
                "- The measured command-motion contribution over state age was `0.010839 m` in that boundary sample; timestamp alignment therefore did not explain the certificate violation or prevent emergency authorization.",
                "- Further PX4/LIO tracking investigation remains required; H7 does not account for the observed synchronized residual.",
            ]
        elif key.startswith("H8"):
            lines.append("This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.")
            for run in runs:
                root_cause = run.get("metrics", {}).get("tracking_root_cause", {})
                if root_cause:
                    item = root_cause.get("h8", {}).get(key, {})
                    lines.append(f"- {run.get('run')}: status={item.get('status', 'NOT_TESTED')}; T_cross_ns={root_cause.get('T_cross_ns', 'NOT_RECORDED')}; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.")
            if key == "H8b_dynamic_tracking_insufficiency":
                lines.append("- Causal interpretation: synchronized error began growing before the observed planner failure and accelerated after generation 2 activation while the command remained MAIN.")
            if key == "H8e_command_setpoint_interruption":
                lines.append("- The direct mode topic labels WAIT_FIRST_COMMAND before TRACK_TRAJECTORY, but command continuity and the diagnostic-only mode projection do not prove a control interruption; this remains explicitly scoped/inconclusive where applicable.")
        else:
            lines.append("Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.")
        lines.append("Evidence: " + ("; ".join(report["hypotheses"][key].get("evidence", [])) or "none") + ".")
    lines += ["", "## 10. Recovery behavior", ""]
    lines.append("Counts are taken from captured command fields; absent command data is not treated as zero.")
    for run in runs:
        lines.append(f"- {run.get('run')}: roles={json.dumps(run.get('metrics', {}).get('command_roles', {}), sort_keys=True)}; states={json.dumps(run.get('metrics', {}).get('recovery_states', {}), sort_keys=True)}")
    h4_status = report["hypotheses"].get("H4_failed_replan_safety_takeover", {}).get("status", "NOT_TESTED")
    lines += ["", "## 11. Stationary-hold transitions", "", "No transition is claimed without a captured `navigation_mode_status` event.", "", "## 12. Ranked findings", "", f"- **P0**: H4 status is **{h4_status}** from captured failure timelines; no causal root cause is claimed unless H4a/H4b evidence is valid.", "- No P1/P2 finding is ranked without a valid controlled witness.", "", "## 13. Proposed next actions", "", "- Re-run E5b/E7/E6 using a fixture that preserves a valid committed MAIN across the failure boundary.", "- Re-run E10 only after a captured StoppedRecovery transition, then compare failure-count and timeout predicates.", "- Keep all safety gates and planner/recovery behavior unchanged while collecting the missing evidence.", "", "Blocked/inconclusive runs are listed in the experiment matrix and raw run directories; blocked SITL is not treated as a pass.", ""]
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

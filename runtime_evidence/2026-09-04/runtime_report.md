# Runtime Evidence Report

## 1. Executive summary

This report contains only measurements present in retained per-run JSONL evidence. Missing or blocked streams remain inconclusive.

- **H1_splice_continuity**: INCONCLUSIVE. Evidence records: 11.
- **H2_replanning_timing**: INCONCLUSIVE. Evidence records: 11.
- **H3_pass_through_continuation**: INCONCLUSIVE. Evidence records: 11.
- **H4_failed_replan_safety_takeover**: CONFIRMED. Evidence records: 12.
- **H4a_planning_failure_alone_changes_ownership**: INCONCLUSIVE. Evidence records: 11.
- **H4b_tracking_certificate_exhaustion_authorizes_emergency**: INCONCLUSIVE. Evidence records: 11.
- **H4c_backup_ownership_begins_at_declared_switch**: CONFIRMED. Evidence records: 12.
- **H5_corner_overconstraint**: INCONCLUSIVE. Evidence records: 11.
- **H6_px4_controller_mismatch**: INCONCLUSIVE. Evidence records: 11.

## 2. Environment

- repository commit: `37e8dc7f9967990ef4c1401475e0d15abef14360`
- PX4 version: `deaff86ee335dd697677bcfc2415a23878e1b895`
- ROS version: `jazzy`
- timing constants: planner=5.0 Hz, command=50.0 Hz, replan_forward=0.4 s, stitch=0.4 s, deadline=0.18 s

## 3. Experiment matrix

| Experiment | Speed | Result | Data quality | Notes |
| ---------- | ----: | ------ | ------------ | ----- |
| E01_straight_3mps | 3.0 | FAIL | VALID | data: commands=533, traces=815, bag=False |
| E01_straight_5mps | 5.0 | BLOCKED | VALID | data: commands=112, traces=35, bag=True |
| E02_pass_straight_3mps | 3.0 | BLOCKED | VALID | data: commands=482, traces=51, bag=True |
| E05_causal_replay_instrumented | 3.0 | BLOCKED | VALID | data: commands=486, traces=67, bag=True |
| E05_causal_replay_instrumented_v2 | 3.0 | FAIL | PARTIAL | scenario.jsonl; samples.jsonl |
| E05_single_hot_replan_failure_cycle5 | 3.0 | BLOCKED | VALID | data: commands=120, traces=15, bag=True |
| E05b_single_failure_large_tracking_margin | 3.0 | BLOCKED | VALID | data: commands=475, traces=46, bag=True |
| E05b_single_failure_large_tracking_margin | 1.0 | BLOCKED | VALID | data: commands=290, traces=32, bag=True |
| E06_failed_hot_retarget_pass_through | 3.0 | BLOCKED | VALID | data: commands=188, traces=15, bag=True |
| E07_repeated_failures_until_backup | 3.0 | BLOCKED | VALID | data: commands=341, traces=37, bag=True |
| E10_stopped_recovery_plan_from_rest_failure_budget | 3.0 | BLOCKED | VALID | data: commands=239, traces=103, bag=True |

## 4. H1 — Future splice continuity

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 5. H2 — Replanning timing

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 6. H3 — PASS_THROUGH continuation

Status: **INCONCLUSIVE**.

Route boundary evidence is reported as `NO_ROUTE_BOUNDARY_EVENT` when no producer-declared event was captured.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 7. H4 — Failed replan causes premature safety takeover

Status: **CONFIRMED**.

Failure timelines below are emitted from the injected planner trace and adjacent command samples:
- E5 Task A causal trace: `runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/e5_causal_trace.csv`; the original artifact measured C3=`backup_available=True` and `time_to_backup_start_s=1.8774401711603277`, while C1/C2/C4/C5 remain `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`.
- Supplemental instrumented replay: `runtime_evidence/2026-09-04/E05_causal_replay_instrumented/e5_instrumented_replay_addendum.md` measured actual-anchor certificate exhaustion, but was not a valid safe-margin injection.
- E01_straight_3mps: t=26.928 s; EMERGENCY/EMERGENCY -> EMERGENCY/EMERGENCY; safety_suffix True -> True; time_to_backup_before=-0.016 s; premature=False; later_nominal_retry=False
- E01_straight_3mps: t=30.612000000000002 s; MAIN/MAIN -> MAIN/MAIN; safety_suffix False -> False; time_to_backup_before=0.9600539211773867 s; premature=False; later_nominal_retry=False
- E01_straight_5mps: t=25.34 s; EMERGENCY/EMERGENCY -> EMERGENCY/EMERGENCY; safety_suffix True -> True; time_to_backup_before=-0.012 s; premature=False; later_nominal_retry=False
- E02_pass_straight_3mps: t=32.96 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=0.9164707114546665 s; premature=True; later_nominal_retry=False
- E05_causal_replay_instrumented: t=31.624000000000002 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=0.854239 s; premature=True; later_nominal_retry=False
- E05_single_hot_replan_failure_cycle5: t=20.472 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=1.8774401711603277 s; premature=True; later_nominal_retry=False
- E05b_single_failure_large_tracking_margin_blocked_initial: t=29.364 s; MAIN/MAIN -> UNKNOWN/UNKNOWN; safety_suffix False -> None; time_to_backup_before=6.68502427487914 s; premature=False; later_nominal_retry=False
- E07_repeated_failures_until_backup: t=25.228 s; MAIN/MAIN -> UNKNOWN/UNKNOWN; safety_suffix False -> None; time_to_backup_before=5.9064208063192645 s; premature=False; later_nominal_retry=False
- E10_stopped_recovery_plan_from_rest_failure_budget: t=26.66 s; MAIN/MAIN -> MAIN/MAIN; safety_suffix False -> False; time_to_backup_before=0.857316 s; premature=False; later_nominal_retry=False
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; Injected failure was followed by a safety-suffix command while the prior command still had positive time_to_backup_start_s.; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4a — Planning failure alone changes execution ownership

Status: **INCONCLUSIVE**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: safe-margin injections=0
- E01_straight_5mps: safe-margin injections=0
- E02_pass_straight_3mps: safe-margin injections=0
- E05_causal_replay_instrumented: safe-margin injections=0
- E05_causal_replay_instrumented_v2: safe-margin injections=0
- E05_single_hot_replan_failure_cycle5: safe-margin injections=0
- E05b_single_failure_large_tracking_margin: safe-margin injections=0
- E05b_single_failure_large_tracking_margin_blocked_initial: safe-margin injections=0
- E06_failed_hot_retarget_pass_through: safe-margin injections=0
- E07_repeated_failures_until_backup: safe-margin injections=0
- E10_stopped_recovery_plan_from_rest_failure_budget: safe-margin injections=0
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4b — Tracking-certificate exhaustion authorizes emergency

Status: **INCONCLUSIVE**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: injected failures=0
- E01_straight_5mps: injected failures=0
- E02_pass_straight_3mps: injected failures=0
- E05_causal_replay_instrumented: injected failures=0
- E05_causal_replay_instrumented_v2: injected failures=0
- E05_single_hot_replan_failure_cycle5: injected failures=1
- E05b_single_failure_large_tracking_margin: injected failures=0
- E05b_single_failure_large_tracking_margin_blocked_initial: injected failures=0
- E06_failed_hot_retarget_pass_through: injected failures=0
- E07_repeated_failures_until_backup: injected failures=0
- E10_stopped_recovery_plan_from_rest_failure_budget: injected failures=0
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4c — BACKUP ownership begins at declared switch

Status: **CONFIRMED**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: backup ownership transitions=1
- E01_straight_5mps: backup ownership transitions=0
- E02_pass_straight_3mps: backup ownership transitions=0
- E05_causal_replay_instrumented: backup ownership transitions=0
- E05_causal_replay_instrumented_v2: backup ownership transitions=0
- E05_single_hot_replan_failure_cycle5: backup ownership transitions=0
- E05b_single_failure_large_tracking_margin: backup ownership transitions=0
- E05b_single_failure_large_tracking_margin_blocked_initial: backup ownership transitions=0
- E06_failed_hot_retarget_pass_through: backup ownership transitions=0
- E07_repeated_failures_until_backup: backup ownership transitions=0
- E10_stopped_recovery_plan_from_rest_failure_budget: backup ownership transitions=1
Evidence: 1 MAIN-to-BACKUP command transition(s) were compared with declared backup_start_time_s.; E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 8. H5 — Route/corner overconstraint

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 9. H6 — Planner vs PX4 controller mismatch

Status: **INCONCLUSIVE**.

PX4 correction and frame-residual statistics are available under each run's `metrics.px4` object.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 10. Recovery behavior

Counts are taken from captured command fields; absent command data is not treated as zero.
- E01_straight_3mps: roles={"BACKUP": 52, "EMERGENCY": 44, "MAIN": 436, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 190, "TRACK_BACKUP": 100, "TRACK_MAIN": 243}
- E01_straight_5mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 60, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 60}
- E02_pass_straight_3mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 430}
- E05_causal_replay_instrumented: roles={"BACKUP": 0, "EMERGENCY": 55, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 56, "TRACK_MAIN": 430}
- E05_causal_replay_instrumented_v2: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 0, "UNKNOWN": 0}; states={}
- E05_single_hot_replan_failure_cycle5: roles={"BACKUP": 0, "EMERGENCY": 38, "MAIN": 82, "UNKNOWN": 0}; states={"EMERGENCY_BRAKE": 40, "TRACK_MAIN": 80}
- E05b_single_failure_large_tracking_margin: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 475, "UNKNOWN": 0}; states={"TRACK_MAIN": 475}
- E05b_single_failure_large_tracking_margin_blocked_initial: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 290, "UNKNOWN": 0}; states={"TRACK_MAIN": 290}
- E06_failed_hot_retarget_pass_through: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 188, "UNKNOWN": 0}; states={"TRACK_MAIN": 188}
- E07_repeated_failures_until_backup: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 341, "UNKNOWN": 0}; states={"TRACK_MAIN": 341}
- E10_stopped_recovery_plan_from_rest_failure_budget: roles={"BACKUP": 40, "EMERGENCY": 0, "MAIN": 199, "UNKNOWN": 0}; states={"TRACK_BACKUP": 71, "TRACK_MAIN": 168}

## 11. Stationary-hold transitions

No transition is claimed without a captured `navigation_mode_status` event.

## 12. Ranked findings

- **P0**: H4 status is **CONFIRMED** from captured failure timelines; no causal root cause is claimed unless H4a/H4b evidence is valid.
- No P1/P2 finding is ranked without a valid controlled witness.

## 13. Proposed next actions

- Re-run E5b/E7/E6 using a fixture that preserves a valid committed MAIN across the failure boundary.
- Re-run E10 only after a captured StoppedRecovery transition, then compare failure-count and timeout predicates.
- Keep all safety gates and planner/recovery behavior unchanged while collecting the missing evidence.

Blocked/inconclusive runs are listed in the experiment matrix and raw run directories; blocked SITL is not treated as a pass.

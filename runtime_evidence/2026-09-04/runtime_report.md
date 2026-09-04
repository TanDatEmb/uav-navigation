# Runtime Evidence Report

## 1. Executive summary

This report contains only measurements present in retained per-run JSONL evidence. Missing or blocked streams remain inconclusive.

- **H1_splice_continuity**: INCONCLUSIVE. Evidence records: 15.
- **H2_replanning_timing**: INCONCLUSIVE. Evidence records: 15.
- **H3_pass_through_continuation**: INCONCLUSIVE. Evidence records: 15.
- **H4_failed_replan_safety_takeover**: CONFIRMED. Evidence records: 16.
- **H4a_planning_failure_alone_changes_ownership**: INCONCLUSIVE. Evidence records: 15.
- **H4b_tracking_certificate_exhaustion_authorizes_emergency**: INCONCLUSIVE. Evidence records: 15.
- **H4c_backup_ownership_begins_at_declared_switch**: CONFIRMED. Evidence records: 16.
- **H5_corner_overconstraint**: INCONCLUSIVE. Evidence records: 15.
- **H6_px4_controller_mismatch**: INCONCLUSIVE. Evidence records: 15.
- **H7_temporal_anchor_alignment**: REJECTED. Evidence records: 16.
- **H8a_command_discontinuity**: REJECTED. Evidence records: 17.
- **H8b_dynamic_tracking_insufficiency**: CONFIRMED. Evidence records: 17.
- **H8c_px4_control_reshaping**: INCONCLUSIVE. Evidence records: 17.
- **H8d_px4_lio_state_divergence**: REJECTED. Evidence records: 17.
- **H8e_command_setpoint_interruption**: INCONCLUSIVE. Evidence records: 18.

## 2. Environment

- repository commit(s): `37e8dc7f9967990ef4c1401475e0d15abef14360, 4b03d39b2d89d7f97dec29780f136f611a77190e, 56f8d379dc90ebadf1e5c51c8c35c7d362303198, cac82aad20771b13f42cf11bd1af93616e51b44b, cf5dcf093ecc6addd56a844bc60fc201d0d5d27f`
- PX4 version(s): `deaff86ee335dd697677bcfc2415a23878e1b895`
- ROS version(s): `jazzy`
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
| E05_temporal_alignment_replay_exact | 3.0 | BLOCKED | VALID | data: commands=481, traces=61, bag=True |
| E05_temporal_alignment_replay_exact | 3.0 | BLOCKED | VALID | data: commands=481, traces=62, bag=True |
| E05b_single_failure_large_tracking_margin | 3.0 | BLOCKED | VALID | data: commands=475, traces=46, bag=True |
| E05b_single_failure_large_tracking_margin | 1.0 | BLOCKED | VALID | data: commands=290, traces=32, bag=True |
| E06_failed_hot_retarget_pass_through | 3.0 | BLOCKED | VALID | data: commands=188, traces=15, bag=True |
| E07_repeated_failures_until_backup | 3.0 | BLOCKED | VALID | data: commands=341, traces=37, bag=True |
| E07_repeated_failures_until_backup_v2 | 3.0 | FAIL | PARTIAL | scenario.jsonl; samples.jsonl |
| E10_plan_from_rest_failure_budget | 3.0 | BLOCKED | PARTIAL | data: commands=0, traces=59, bag=True |
| E10_stopped_recovery_plan_from_rest_failure_budget | 3.0 | BLOCKED | VALID | data: commands=239, traces=103, bag=True |

## 4. H1 — Future splice continuity

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 5. H2 — Replanning timing

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 6. H3 — PASS_THROUGH continuation

Status: **INCONCLUSIVE**.

Route boundary evidence is reported as `NO_ROUTE_BOUNDARY_EVENT` when no producer-declared event was captured.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

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
- E05_temporal_alignment_replay_exact: t=30.252000000000002 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=0.922696 s; premature=True; later_nominal_retry=False
- E05_temporal_alignment_replay_exact_v2: t=29.932000000000002 s; EMERGENCY/EMERGENCY -> EMERGENCY/EMERGENCY; safety_suffix True -> True; time_to_backup_before=0.925026 s; premature=True; later_nominal_retry=False
- E05b_single_failure_large_tracking_margin_blocked_initial: t=29.364 s; MAIN/MAIN -> UNKNOWN/UNKNOWN; safety_suffix False -> None; time_to_backup_before=6.68502427487914 s; premature=False; later_nominal_retry=False
- E07_repeated_failures_until_backup: t=25.228 s; MAIN/MAIN -> UNKNOWN/UNKNOWN; safety_suffix False -> None; time_to_backup_before=5.9064208063192645 s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.076 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.232000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.392000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.552000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.716 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=25.872 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.032 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.192 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.356 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.512 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.672 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.836000000000002 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=26.992 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.152 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.316000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.472 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.636000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.792 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=27.952 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.112000000000002 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.272000000000002 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.436000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.596 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.756 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=28.912000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=29.072000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_initial_plan_from_rest_failures: t=29.244000000000003 s; UNKNOWN/UNKNOWN -> UNKNOWN/UNKNOWN; safety_suffix None -> None; time_to_backup_before=None s; premature=False; later_nominal_retry=False
- E10_stopped_recovery_plan_from_rest_failure_budget: t=26.66 s; MAIN/MAIN -> MAIN/MAIN; safety_suffix False -> False; time_to_backup_before=0.857316 s; premature=False; later_nominal_retry=False
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; Injected failure was followed by a safety-suffix command while the prior command still had positive time_to_backup_start_s.; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4a — Planning failure alone changes execution ownership

Status: **INCONCLUSIVE**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: safe-margin injections=0
- E01_straight_5mps: safe-margin injections=0
- E02_pass_straight_3mps: safe-margin injections=0
- E05_causal_replay_instrumented: safe-margin injections=0
- E05_causal_replay_instrumented_v2: safe-margin injections=0
- E05_single_hot_replan_failure_cycle5: safe-margin injections=0
- E05_temporal_alignment_replay_exact: safe-margin injections=0
- E05_temporal_alignment_replay_exact_v2: safe-margin injections=0
- E05b_single_failure_large_tracking_margin: safe-margin injections=0
- E05b_single_failure_large_tracking_margin_blocked_initial: safe-margin injections=0
- E06_failed_hot_retarget_pass_through: safe-margin injections=0
- E07_repeated_failures_until_backup: safe-margin injections=0
- E07_repeated_failures_until_backup_v2: safe-margin injections=0
- E10_initial_plan_from_rest_failures: safe-margin injections=0
- E10_stopped_recovery_plan_from_rest_failure_budget: safe-margin injections=0
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4b — Tracking-certificate exhaustion authorizes emergency

Status: **INCONCLUSIVE**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: injected failures=0
- E01_straight_5mps: injected failures=0
- E02_pass_straight_3mps: injected failures=0
- E05_causal_replay_instrumented: injected failures=0
- E05_causal_replay_instrumented_v2: injected failures=0
- E05_single_hot_replan_failure_cycle5: injected failures=1
- E05_temporal_alignment_replay_exact: injected failures=0
- E05_temporal_alignment_replay_exact_v2: injected failures=0
- E05b_single_failure_large_tracking_margin: injected failures=0
- E05b_single_failure_large_tracking_margin_blocked_initial: injected failures=0
- E06_failed_hot_retarget_pass_through: injected failures=0
- E07_repeated_failures_until_backup: injected failures=0
- E07_repeated_failures_until_backup_v2: injected failures=0
- E10_initial_plan_from_rest_failures: injected failures=0
- E10_stopped_recovery_plan_from_rest_failure_budget: injected failures=0
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H4c — BACKUP ownership begins at declared switch

Status: **CONFIRMED**.

This sub-hypothesis uses only causal telemetry from valid failure-injection runs; blocked stimuli remain inconclusive.
- E01_straight_3mps: backup ownership transitions=1
- E01_straight_5mps: backup ownership transitions=0
- E02_pass_straight_3mps: backup ownership transitions=0
- E05_causal_replay_instrumented: backup ownership transitions=0
- E05_causal_replay_instrumented_v2: backup ownership transitions=0
- E05_single_hot_replan_failure_cycle5: backup ownership transitions=0
- E05_temporal_alignment_replay_exact: backup ownership transitions=0
- E05_temporal_alignment_replay_exact_v2: backup ownership transitions=0
- E05b_single_failure_large_tracking_margin: backup ownership transitions=0
- E05b_single_failure_large_tracking_margin_blocked_initial: backup ownership transitions=0
- E06_failed_hot_retarget_pass_through: backup ownership transitions=0
- E07_repeated_failures_until_backup: backup ownership transitions=0
- E07_repeated_failures_until_backup_v2: backup ownership transitions=0
- E10_initial_plan_from_rest_failures: backup ownership transitions=0
- E10_stopped_recovery_plan_from_rest_failure_budget: backup ownership transitions=1
Evidence: 1 MAIN-to-BACKUP command transition(s) were compared with declared backup_start_time_s.; E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 8. H5 — Route/corner overconstraint

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 9. H6 — Planner vs PX4 controller mismatch

Status: **INCONCLUSIVE**.

PX4 correction and frame-residual statistics are available under each run's `metrics.px4` object.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H7 — Temporal anchor alignment

Status: **REJECTED**.

Exact immutable-polynomial temporal decomposition is stored in each run's `e5_temporal_alignment.csv` and `metrics.temporal_alignment`.
- E05_temporal_alignment_replay_exact: usable=1; raw=0.46515379425721387; aligned=0.42242751718868116; motion=0.04350460700352208.
- E05_temporal_alignment_replay_exact_v2: usable=1; raw=0.4532910267671672; aligned=0.4428736168588373; motion=0.010839246322547656.

### H7 explicit answers

- The legacy E5 value `0.482191 m` cannot be decomposed from its retained artifact because exact immutable samples at both timestamps were not recorded.
- In the exact temporal replay, the vehicle was still more than `0.25 m` from the committed trajectory at the synchronized state-source timestamp (`0.442874 m` aligned versus `0.25 m` limit).
- The measured command-motion contribution over state age was `0.010839 m` in that boundary sample; timestamp alignment therefore did not explain the certificate violation or prevent emergency authorization.
- Further PX4/LIO tracking investigation remains required; H7 does not account for the observed synchronized residual.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; 1 retained-validation sample(s) exceeded 0.25 m after exact state-source time alignment.; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H8a — Command discontinuity

Status: **REJECTED**.

This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.
- E05_temporal_alignment_replay_exact_v2: status=REJECTED; T_cross_ns=29319484858; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; Exact scenario scope: E05_temporal_alignment_replay_exact; run=external-mode-check-20260904T125955-39028; map=open/sanity_open.; Before T_cross, observed generation boundaries have no large P/V/A jump; the first boundary sharply preceding growth has ΔP=0.011877 m, ΔV=0.028492 m/s, ΔA=0.067050 m/s². The emergency boundary is post-crossing and is therefore not an initiating cause.; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H8b — Dynamic tracking insufficiency

Status: **CONFIRMED**.

This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.
- E05_temporal_alignment_replay_exact_v2: status=CONFIRMED; T_cross_ns=29319484858; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.
- Causal interpretation: synchronized error began growing before the observed planner failure and accelerated after generation 2 activation while the command remained MAIN.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; Exact scenario scope: E05_temporal_alignment_replay_exact; run=external-mode-check-20260904T125955-39028; map=open/sanity_open.; Synchronized LIO error grows before planner failure, with a measured slope of 0.43445373391010655 m/s after gen2 activation versus 0.014167034761644003 m/s earlier; the command remains MAIN and continuous while requesting a rapid speed ramp and multi-m/s² acceleration in the exact E5 window.; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H8c — PX4 control reshaping

Status: **INCONCLUSIVE**.

This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.
- E05_temporal_alignment_replay_exact_v2: status=INCONCLUSIVE; T_cross_ns=29319484858; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; Exact scenario scope: E05_temporal_alignment_replay_exact; run=external-mode-check-20260904T125955-39028; map=open/sanity_open.; PX4 effective setpoints are available and controller deltas are computed, but no predeclared quantitative threshold establishes that reshaping is significant enough to be causal.; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H8d — PX4/LIO state divergence

Status: **REJECTED**.

This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.
- E05_temporal_alignment_replay_exact_v2: status=REJECTED; T_cross_ns=29319484858; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; Exact scenario scope: E05_temporal_alignment_replay_exact; run=external-mode-check-20260904T125955-39028; map=open/sanity_open.; Using the fixed first-active-sample transform and rotating the recorded body twist by q_xyzw, frame residuals remain small/stable relative to the 0.25 m planner-to-state loss before T_cross; no residual growth explains the crossing.; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

### H8e — Command/setpoint interruption

Status: **INCONCLUSIVE**.

This classification is scoped to the exact E05 temporal-alignment replay; no control-run statistic is merged into it.
- E05_temporal_alignment_replay_exact_v2: status=INCONCLUSIVE; T_cross_ns=29319484858; evidence=`runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/e5_tracking_root_cause.md`.
- The direct mode topic labels WAIT_FIRST_COMMAND before TRACK_TRAJECTORY, but command continuity and the diagnostic-only mode projection do not prove a control interruption; this remains explicitly scoped/inconclusive where applicable.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_causal_replay_instrumented: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented/report_run.json; E05_causal_replay_instrumented_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_causal_replay_instrumented_v2/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json; E05_temporal_alignment_replay_exact: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/report_run.json; Exact scenario scope: E05_temporal_alignment_replay_exact; run=external-mode-check-20260904T125955-39028; map=open/sanity_open.; A non-track External Mode state appears before T_cross; causality requires further event correlation.; No pre-cross command gap >=100 ms was observed.; E05_temporal_alignment_replay_exact_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/report_run.json; E05b_single_failure_large_tracking_margin: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin/report_run.json; E05b_single_failure_large_tracking_margin_blocked_initial: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05b_single_failure_large_tracking_margin_blocked_initial/report_run.json; E06_failed_hot_retarget_pass_through: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E06_failed_hot_retarget_pass_through/report_run.json; E07_repeated_failures_until_backup: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup/report_run.json; E07_repeated_failures_until_backup_v2: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E07_repeated_failures_until_backup_v2/report_run.json; E10_initial_plan_from_rest_failures: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_initial_plan_from_rest_failures/report_run.json; E10_stopped_recovery_plan_from_rest_failure_budget: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E10_stopped_recovery_plan_from_rest_failure_budget/report_run.json.

## 10. Recovery behavior

Counts are taken from captured command fields; absent command data is not treated as zero.
- E01_straight_3mps: roles={"BACKUP": 52, "EMERGENCY": 44, "MAIN": 436, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 190, "TRACK_BACKUP": 100, "TRACK_MAIN": 243}
- E01_straight_5mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 60, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 60}
- E02_pass_straight_3mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 430}
- E05_causal_replay_instrumented: roles={"BACKUP": 0, "EMERGENCY": 55, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 56, "TRACK_MAIN": 430}
- E05_causal_replay_instrumented_v2: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 0, "UNKNOWN": 0}; states={}
- E05_single_hot_replan_failure_cycle5: roles={"BACKUP": 0, "EMERGENCY": 38, "MAIN": 82, "UNKNOWN": 0}; states={"EMERGENCY_BRAKE": 40, "TRACK_MAIN": 80}
- E05_temporal_alignment_replay_exact: roles={"BACKUP": 0, "EMERGENCY": 50, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 51, "TRACK_MAIN": 430}
- E05_temporal_alignment_replay_exact_v2: roles={"BACKUP": 0, "EMERGENCY": 50, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 51, "TRACK_MAIN": 430}
- E05b_single_failure_large_tracking_margin: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 475, "UNKNOWN": 0}; states={"TRACK_MAIN": 475}
- E05b_single_failure_large_tracking_margin_blocked_initial: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 290, "UNKNOWN": 0}; states={"TRACK_MAIN": 290}
- E06_failed_hot_retarget_pass_through: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 188, "UNKNOWN": 0}; states={"TRACK_MAIN": 188}
- E07_repeated_failures_until_backup: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 341, "UNKNOWN": 0}; states={"TRACK_MAIN": 341}
- E07_repeated_failures_until_backup_v2: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 0, "UNKNOWN": 0}; states={}
- E10_initial_plan_from_rest_failures: roles={"BACKUP": 0, "EMERGENCY": 0, "MAIN": 0, "UNKNOWN": 0}; states={}
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

## 14. H10 — Closed-loop attribution and usable dynamic envelope

H10 evidence is scenario-separated. `S_BAD_E5` is the partial-but-layer-valid new-observability run `external-mode-check-20260904T142605-80003`; `S_OPEN_CONTROL` is `external-mode-check-20260904T140220-64377`; `S_DYNAMIC_ID` contains only the two exploratory 1.0/2.0 m/s runs. Raw layers and event table are in `h10_exact_e5_closed_loop.csv`, `h10_open_control.csv`, `h10_dynamic_identification.csv`, and `h10_exact_e5_events.csv`.

- `S_BAD_E5` first synchronized LIO-command crossing of the 0.25 m certificate: `29.800 s` simulation time. At that sample: command/LIO error `0.250516 m`, planner speed `1.334 m/s`, planner acceleration `2.435 m/s²`, PX4 effective speed `1.503 m/s`, effective acceleration `3.737 m/s²`.
- The selected run recorded `PX4_INPUT_SETPOINT` immediately before External Mode publication and independent Gazebo ground truth. It did not record `injected_replan_failure=1`; its later natural planner failure/emergency is not an injected-failure witness.
- H10a: **INCONCLUSIVE**; H10b: **INCONCLUSIVE**; H10c: **INCONCLUSIVE**; H10d: **INCONCLUSIVE**; H10e: **INCONCLUSIVE**. The two exploratory dynamic runs do not satisfy the no-recovery, increasing-demand criterion, so no closed-loop envelope is claimed.
- The matched control remains separate: command/LIO tracking P95 is `0.448382 m` for `S_BAD_E5` versus `0.467124 m` for `S_OPEN_CONTROL`; LIO-GT position P95 is `0.166793 m` versus `0.162726 m`; PX4-GT position P95 is `0.244979 m` versus `0.204108 m`.

The H10 causal conclusion is limited to ordering: tracking degradation and the 0.25 m crossing precede the natural planner failure/emergency. The report does not claim that a planning failure caused the loss. No behavior, threshold, timing, gain, or production envelope was changed.

See [h10_analysis.md](/home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/h10_analysis.md), [runtime_report.json](/home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/runtime_report.json), and the generated figures under `/home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/h10_figures/`.

## 15. H10-Final — Closed-loop attribution and usable dynamic envelope

# H10-Final — Closed-loop attribution and usable dynamic envelope

## Scenario scope

- `S_BAD_E5`: `.artifacts/runtime/external-mode-check-20260904T145100-95777`; sanity_open / external_mode_open_route / 3.0 m/s. Raw bag retained; exact final injection validity is `False` because the immutable marker count is `0`.
- `S_OPEN_CONTROL`: `.artifacts/runtime/external-mode-check-20260904T140220-64377`; matched open control at 3.0 m/s. Statistics are separate and never pooled.
- `S_DYNAMIC_ID`: DYN-LONG `['.artifacts/runtime/external-mode-check-20260904T140416-66041', '.artifacts/runtime/external-mode-check-20260904T140614-67910']` and DYN-LAT `.artifacts/runtime/external-mode-check-20260904T145448-98065`. Segment IDs are analysis labels; no production mission behavior was changed.

## Evidence validity

The exact E5 final SITL artifact contains all canonical layers and independent `/sim/ground_truth/odometry`, but the requested cycle-5 hook produced zero immutable `injected_replan_failure=1` events. It is therefore not a valid exact injected-failure reproduction. DYN-LAT is layer-valid only before the observed RECOVERY_HOLD and is not a clean envelope pass.

## Exact E5 ordering

| Event | timestamp_ns | evidence |
|---|---:|---|
| bundle_activation | 29196000000 | e_lio=0.1068940951086756, planner_a=0.09322181503532215, recovery=TRACK_MAIN |
| sustained_error_growth_start | 29292000000 | e_lio=0.10789685026724816, planner_a=0.9246956280518678, recovery=TRACK_MAIN |
| tracking_error_0.10_m | 28748000000 | e_lio=0.10002732298187615, planner_a=0.06883862826673635, recovery=TRACK_MAIN |
| tracking_error_0.20_m | 29580000000 | e_lio=0.201056465380215, planner_a=3.279451917791536, recovery=TRACK_MAIN |
| tracking_error_0.25_m | 29660000000 | e_lio=0.2558371555530457, planner_a=3.4785770994203866, recovery=TRACK_MAIN |
| planner_failure_return | NOT_RECORDED | NOT_RECORDED |
| emergency_authorization | NOT_RECORDED | NOT_RECORDED |
| emergency_activation | NOT_RECORDED | NOT_RECORDED |

The observed ordering is tracking degradation/growth before the natural planner failure and emergency. Because the injected marker is absent, this run cannot support a causal claim that the injected planner failure caused the loss.

## Exact E5 vs matched open control

| Metric | S_BAD_E5 | S_OPEN_CONTROL |
|---|---:|---:|
| command-LIO tracking P95 | 0.413914 | 0.467124 |
| command-GT tracking P95 | 0.370167 | 0.583501 |
| LIO-GT position P95 | 0.173013 | 0.162726 |
| PX4-GT position P95 | 0.180157 | 0.204108 |
| planner acceleration P95 | 3.043304 | 3.824621 |
| planner jerk P95 | 7.252682 | 18.084679 |
| PX4 controller delta-V P95 | 4.306638 | 3.478612 |
| PX4 controller delta-A P95 | 6.197942 | 7.392377 |

## DYN-LONG / DYN-LAT

The available low-demand runs do not constitute the requested deterministic increasing-demand matrix. DYN-LAT entered `RECOVERY_HOLD` at the second lateral waypoint. Per-segment metrics are in `h10_dynamic_long.csv` and `h10_dynamic_lat.csv`; no nominal envelope value is claimed.

## Usable closed-loop envelope

`h10_final_dynamic_envelope.json` is `NOT_IDENTIFIED`. The required P95 <= 0.10 m, MAX <= 0.175 m, no-recovery, estimator-health criterion was not demonstrated by a clean controlled matrix. E5 demand/envelope ratios are therefore `NOT_IDENTIFIED`.

## H10 scenario-scoped classification

| Hypothesis | S_BAD_E5 | DYN-LONG | DYN-LAT |
|---|---|---|---|
| H10a planner demand exceeds usable closed-loop envelope | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |
| H10b PX4 materially reshapes planner command | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |
| H10c LIO materially contributes | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |
| H10d PX4 estimator materially contributes | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |
| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | INCONCLUSIVE | INCONCLUSIVE |

No H10 hypothesis is promoted to CONFIRMED by an invalid injection or a recovery-contaminated dynamic-ID run. This is an evidence limitation, not evidence that the previously established H8b result is false.

## Figures

- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig1_e5_tracking_error_events.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig2_e5_velocity_layers.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig3_e5_acceleration_layers.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig4_e5_estimator_gt_error.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig5_dyn_long_demand_tracking.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig6_dyn_lat_lateral_tracking.png`
- `runtime_evidence/2026-09-04/H10_FINAL/figures/fig7_e5_demand_vs_nominal_envelope.png`

# FIRST FIX RECOMMENDATION

- Selected first fix: `PLANNER_CLOSED_LOOP_ENVELOPE` (provisional; implementation deferred).
- Measured basis: the prior exact-E5 family already established dynamic tracking insufficiency, while this final run confirms the requested observability layers but not a valid injected failure or clean envelope boundary.
- Competing fixes are deferred because PX4 reshaping, LIO contribution, PX4 estimator contribution, and plant-following attribution remain INCONCLUSIVE in the scenario-scoped final package.
- Preserve unchanged: 0.25 m retained-command certificate, fail-closed recovery, estimator/world freshness and validity gates, backup/emergency safety checks, planner timing, and mission acceptance rules.
- Exact E5 regression: rerun the same map/route/speed and require exactly one `injected_replan_failure=1`, then verify demand/tracking/error ordering and that the selected fix removes unwanted loss without weakening the certificate.
- Matched open-control regression: same vehicle, PX4 parameters, planner/command rates, map, route, and speed; compare tracking and controller-correction distributions separately; no easier control run may invalidate an E5 finding.

## Closed-loop characterization harness

See `closed_loop_characterization.md` and `closed_loop_characterization.json` for the isolated MODE_PX4_LOCAL evidence package. The production fix remains recommendation-only; no production behavior was changed.

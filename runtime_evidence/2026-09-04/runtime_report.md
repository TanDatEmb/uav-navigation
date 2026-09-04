# Runtime Evidence Report

## 1. Executive summary

This report contains only measurements present in retained per-run JSONL evidence. Missing or blocked streams remain inconclusive.

- **H1_splice_continuity**: INCONCLUSIVE. Evidence records: 4.
- **H2_replanning_timing**: INCONCLUSIVE. Evidence records: 4.
- **H3_pass_through_continuation**: INCONCLUSIVE. Evidence records: 4.
- **H4_failed_replan_safety_takeover**: CONFIRMED. Evidence records: 5.
- **H5_corner_overconstraint**: INCONCLUSIVE. Evidence records: 4.
- **H6_px4_controller_mismatch**: INCONCLUSIVE. Evidence records: 4.

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
| E05_single_hot_replan_failure_cycle5 | 3.0 | BLOCKED | VALID | data: commands=120, traces=15, bag=True |

## 4. H1 — Future splice continuity

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 5. H2 — Replanning timing

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 6. H3 — PASS_THROUGH continuation

Status: **INCONCLUSIVE**.

Route boundary evidence is reported as `NO_ROUTE_BOUNDARY_EVENT` when no producer-declared event was captured.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 7. H4 — Failed replan causes premature safety takeover

Status: **CONFIRMED**.

Failure timelines below are emitted from the injected planner trace and adjacent command samples:
- E01_straight_3mps: t=26.928 s; EMERGENCY/EMERGENCY -> EMERGENCY/EMERGENCY; safety_suffix True -> True; time_to_backup_before=-0.016 s; premature=False; later_nominal_retry=False
- E01_straight_3mps: t=30.612000000000002 s; MAIN/MAIN -> MAIN/MAIN; safety_suffix False -> False; time_to_backup_before=0.9600539211773867 s; premature=False; later_nominal_retry=False
- E01_straight_5mps: t=25.34 s; EMERGENCY/EMERGENCY -> EMERGENCY/EMERGENCY; safety_suffix True -> True; time_to_backup_before=-0.012 s; premature=False; later_nominal_retry=False
- E02_pass_straight_3mps: t=32.96 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=0.9164707114546665 s; premature=True; later_nominal_retry=False
- E05_single_hot_replan_failure_cycle5: t=20.472 s; MAIN/MAIN -> EMERGENCY/EMERGENCY; safety_suffix False -> True; time_to_backup_before=1.8774401711603277 s; premature=True; later_nominal_retry=False
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; Injected failure was followed by a safety-suffix command while the prior command still had positive time_to_backup_start_s.; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 8. H5 — Route/corner overconstraint

Status: **INCONCLUSIVE**.

Per-run measured metrics are available under the corresponding `metrics` object in `report_run.json`.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 9. H6 — Planner vs PX4 controller mismatch

Status: **INCONCLUSIVE**.

PX4 correction and frame-residual statistics are available under each run's `metrics.px4` object.
Evidence: E01_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_3mps/report_run.json; E01_straight_5mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E01_straight_5mps/report_run.json; E02_pass_straight_3mps: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E02_pass_straight_3mps/report_run.json; E05_single_hot_replan_failure_cycle5: /home/letandat/Dev/uav-navigation/runtime_evidence/2026-09-04/E05_single_hot_replan_failure_cycle5/report_run.json.

## 10. Recovery behavior

Counts are taken from captured command fields; absent command data is not treated as zero.
- E01_straight_3mps: roles={"BACKUP": 52, "EMERGENCY": 44, "MAIN": 436, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 190, "TRACK_BACKUP": 100, "TRACK_MAIN": 243}
- E01_straight_5mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 60, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 60}
- E02_pass_straight_3mps: roles={"BACKUP": 0, "EMERGENCY": 51, "MAIN": 430, "UNKNOWN": 1}; states={"EMERGENCY_BRAKE": 52, "TRACK_MAIN": 430}
- E05_single_hot_replan_failure_cycle5: roles={"BACKUP": 0, "EMERGENCY": 38, "MAIN": 82, "UNKNOWN": 0}; states={"EMERGENCY_BRAKE": 40, "TRACK_MAIN": 80}

## 11. Stationary-hold transitions

No transition is claimed without a captured `navigation_mode_status` event.

## 12. Ranked findings

- **P0**: H4 is runtime-confirmed in E05: one injected failed replacement was followed by EMERGENCY before the old MAIN bundle's backup boundary.
- No P1/P2 finding is ranked from the current incomplete matrix.

## 13. Proposed next actions

- Execute E3 angle/acceptance sweeps with dedicated free-space mission fixtures.
- Execute E6 with a handoff-scoped one-shot failure, E7 with repeated-failure injection, and E10 with PlanFromRest-specific failure injection.
- Keep all safety gates and planner/recovery behavior unchanged while collecting the missing evidence.

Blocked/inconclusive runs: E3, E6, E7, E8, E9, E10, E11 were not executed with their required controlled stimuli; H1, H2, H3, H5, and H6 remain INCONCLUSIVE.

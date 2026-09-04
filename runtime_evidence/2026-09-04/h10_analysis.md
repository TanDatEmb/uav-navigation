# H10 Analysis

## Scenario scope

- S_BAD_E5: `.artifacts/runtime/external-mode-check-20260904T145100-95777`; exact stressed map/route at 3.0 m/s. The retained run is an observability run; injected failure is NOT_RECOrDED unless the trace says `injected_replan_failure=1`.
- S_OPEN_CONTROL: `.artifacts/runtime/external-mode-check-20260904T140220-64377`; matched open/straight control, 3.0 m/s; statistics remain separate.
- S_DYNAMIC_ID: ['.artifacts/runtime/external-mode-check-20260904T140416-66041', '.artifacts/runtime/external-mode-check-20260904T140614-67910', '.artifacts/runtime/external-mode-check-20260904T145448-98065']; separate low-demand controls, not a certified envelope matrix.

## Evidence status

S_BAD_E5 rows: 496; synchronized T_cross: 29660000000.
Layer B is the `PX4_INPUT_SETPOINT` diagnostic emitted immediately before External Mode calls `trajectory_setpoint_->update`; SITL ground truth is `/sim/ground_truth/odometry` and is independent of LIO/PX4 estimation.

## H10 classifications

| Hypothesis | Status | Scenario-scoped evidence |
|---|---|---|
| H10a planner demand exceeds usable envelope | INCONCLUSIVE | E5 demand and error are correlated, but no valid increasing-demand S_DYNAMIC_ID segment meets the criterion. |
| H10b PX4 materially reshapes command | INCONCLUSIVE | Four layers are present in the observability run; attribution requires a valid exact injected E5 boundary and synchronized finite PX4 input/effective data. |
| H10c LIO material contributor | INCONCLUSIVE | LIO-GT error is measured, but its contribution is not isolated from controller/plant response in the failed run. |
| H10d PX4 estimator material contributor | INCONCLUSIVE | PX4-GT position is measured; velocity estimator evidence is NOT_RECORDED in this run. |
| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | Effective setpoint and GT are present, but no clean dynamic-ID segment separates plant limitation from command reshaping. |

## Envelope

`h10_dynamic_envelope.json` remains `NOT_TESTED`; the two low-speed runs fail closed before providing the required no-recovery increasing-demand segments. No production limit is changed.

## Causal ordering

The CSV and retained event timestamps are authoritative. The selected S_BAD_E5 run has a natural planner failure/emergency boundary but no `injected_replan_failure=1`; therefore it cannot establish injected-failure causality. Earlier tracking growth must not be attributed to that later planner event.

## Figures

- `h10_fig1_velocity_layers.png`
- `h10_fig2_acceleration_layers.png`
- `h10_fig3_estimator_gt_error.png`
- `h10_fig4_command_gt_error.png`
- `h10_fig5_controller_correction_vs_error.png`
- `h10_fig6_dynamic_demand_vs_error.png`
- `h10_fig7_lio_px4_consistency.png`
- `h10_fig8_exact_e5_causal_timeline.png`

## FIRST FIX RECOMMENDATION

Do not implement a fix from this evidence phase. First obtain a valid S_DYNAMIC_ID matrix and a valid exact-E5 injected-failure run with all four control layers; then choose between planner-envelope coupling and controller/plant changes from measured attribution.

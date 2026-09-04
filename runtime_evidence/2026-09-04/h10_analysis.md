# H10 Analysis

## Scenario scope

- `S_BAD_E5`: `.artifacts/runtime/external-mode-check-20260904T142605-80003`; `sanity_open`, route `external_mode_open_route`, requested speed 3.0 m/s. This is the new-observability run.
- `S_OPEN_CONTROL`: `.artifacts/runtime/external-mode-check-20260904T140220-64377`; matched open/straight control, same requested speed and planner/PX4 configuration. Statistics are separate.
- `S_DYNAMIC_ID`: ['.artifacts/runtime/external-mode-check-20260904T140416-66041', '.artifacts/runtime/external-mode-check-20260904T140614-67910']; low-demand exploratory runs only; no certified envelope is claimed.

## Evidence validity

The selected S_BAD_E5 run has 497 canonical PX4-input samples and independent `/sim/ground_truth/odometry`, but `injected_replan_failure=1` was not observed. The natural planner failure and emergency therefore cannot be used as injected-failure causality.
Synchronized first 0.25 m crossing: `29800000000` ns. Raw layers are retained in `.artifacts/runtime/external-mode-check-20260904T142605-80003/h10_exact_e5_closed_loop.csv`.

## Required event table

| Event | Relative time (s) | e LIO (m) | planner speed | planner accel | PX4 eff speed | PX4 eff accel | mode / recovery |
|---|---:|---:|---:|---:|---:|---:|---|
| bundle_activation | 18.176000 | 0.13805611471642734 | 0.7612730707294 | 1.7746563682336638 | 0.8080259365269181 | 2.4118864322478855 | TRACK_TRAJECTORY / TRACK_MAIN |
| sustained_error_growth_start | 14.928000 | 0.0545425102614939 | 0.772129207595193 | 0.16923813057131737 | 0.7670343050839894 | 0.11729602422575584 | WAIT_FIRST_COMMAND / TRACK_MAIN |
| tracking_error_0.10_m | 16.848000 | 0.10229417020758427 | 0.5639090991555141 | 0.1341914149538564 | 0.5908094052951121 | 0.02260620172866957 | WAIT_FIRST_COMMAND / TRACK_MAIN |
| tracking_error_0.20_m | 18.368000 | 0.20518357246938168 | 1.088960622296833 | 1.9414473929500775 | 1.1756011073914796 | 2.8675098078333714 | TRACK_TRAJECTORY / TRACK_MAIN |
| tracking_error_0.25_m | 18.480000 | 0.25051644624762326 | 1.3342096011475064 | 2.4354340567592128 | 1.5025752628655942 | 3.7370414374603484 | TRACK_TRAJECTORY / TRACK_MAIN |
| injected_solve_start | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED / NOT_RECORDED |
| injected_solve_failure | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED / NOT_RECORDED |
| planner_failure_return | 19.040000 | 0.40456345909692293 | 2.7108909875392753 | 1.49527029277196 | 3.120415276576699 | 2.543507502655753 | TRACK_TRAJECTORY / TRACK_MAIN |
| emergency_authorization | 19.040000 | 0.40456345909692293 | 2.7108909875392753 | 1.49527029277196 | 3.120415276576699 | 2.543507502655753 | TRACK_TRAJECTORY / TRACK_MAIN |
| emergency_activation | 19.072000 | 0.08284583797897321 | 2.8349862887285995 | 0.03829542021428544 | 2.717475396724084 | 0.5519396105822187 | TRACK_TRAJECTORY / EMERGENCY_BRAKE |

`injected_solve_start` and `injected_solve_failure` are `NOT_RECORDED`, not inferred.

## Matched comparison (not pooled)

| Metric | S_BAD_E5 exact | S_OPEN_CONTROL |
|---|---:|---:|
| LIO-GT position P95 | P95=0.1667927637589658; MAX=0.17349606468547263 | P95=0.1627264064355332; MAX=0.1747898792689254 |
| PX4-GT position P95 | P95=0.2449790363730274; MAX=0.2644041170747711 | P95=0.20410819686560913; MAX=0.2213926134948847 |
| planner acceleration P95 | P95=3.7423418329720235; MAX=6.4968210247314 | P95=3.824620605435933; MAX=6.507464951280076 |
| planner jerk P95 | P95=18.065568665848712; MAX=24.47518569135113 | P95=18.08467926135224; MAX=24.494666656971347 |
| PX4 controller delta-V P95 | P95=3.519856850491304; MAX=4.1439215324023655 | P95=3.4786121098899154; MAX=4.124395662027808 |
| PX4 controller delta-A P95 | P95=6.3676374074927296; MAX=12.85493022557812 | P95=7.392377192203414; MAX=12.700351782895519 |
| command/LIO tracking P95 | P95=0.44838248643528783; MAX=0.9326030255222632 | P95=0.46712365500499065; MAX=0.9115805161662376 |

## H10 classifications

| Hypothesis | Status | Scenario-scoped conclusion |
|---|---|---|
| H10a planner demand exceeds usable closed-loop envelope | INCONCLUSIVE | Demand/error correlation is present, but no valid increasing-demand envelope matrix. |
| H10b PX4 controller materially reshapes planner trajectory | INCONCLUSIVE | Exact input/effective layers are recorded; causal isolation is incomplete. |
| H10c LIO estimator is a material contributor | INCONCLUSIVE | LIO-GT error is measured, not isolated as the cause. |
| H10d PX4 estimator is a material contributor | INCONCLUSIVE | PX4-GT position is measured; velocity attribution is NOT_RECORDED. |
| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | No clean ID segment separates plant limitation from reshaping. |

## Usable closed-loop envelope

Status: **NOT_TESTED**. Criterion: P95 tracking <= 0.10 m, MAX <= 0.175 m, no recovery/emergency, estimator health valid. No production planner limit was changed.

## Causal ordering

The evidence orders tracking degradation before the natural planner failure/emergency. It does not establish that an injected failure caused the degradation, because the required injection marker is absent.

## Figures

- `runtime_evidence/2026-09-04/h10_figures/h10_fig1_velocity_layers.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig2_acceleration_layers.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig3_estimator_gt_error.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig4_command_gt_error.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig5_controller_correction_vs_error.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig6_dynamic_demand_vs_error.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig7_lio_px4_consistency.png`
- `runtime_evidence/2026-09-04/h10_figures/h10_fig8_exact_e5_causal_timeline.png`

## FIRST FIX RECOMMENDATION

Do not implement a behavioral fix from this incomplete attribution. The highest-leverage next action is a valid controlled dynamics-ID run plus exact E5 rerun with the four synchronized layers; if that confirms H10a with small H10b, the first product change should be one product-owned `VehicleControlEnvelope` coupling planner demand to measured closed-loop limits. Preserve the 0.25 m certificate, fail-closed recovery, estimator-health gates, and exact E5/open-control regression tests.

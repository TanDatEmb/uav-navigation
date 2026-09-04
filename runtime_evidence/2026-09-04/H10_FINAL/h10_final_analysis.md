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

# Closed-loop characterization

## Scope and isolation

- MODE: `MODE_PX4_LOCAL`; analytic reference generated directly in fixed PX4 local NED.
- NavigationRuntime, A*, corridor/MINCO, mission FSM, backup/emergency recovery, PASS_THROUGH, and planner renewal were not used.
- Ground truth is the SITL authority; PX4 and LIO are independently transformed with fixed initialization transforms.
- Production behavior and PX4 gains were not changed.

## Run matrix

| Run | Profile | Result | Raw evidence |
|---|---|---|---|
| DYN-LONG | longitudinal | `COMPLETE` | `sim-check-20260904T160837-159859` |
| DYN-LAT | lateral | `COMPLETE` | `sim-check-20260904T161026-161202` |
| S_BAD_E5 | exact existing E5 generation 2 control | regression evidence only | existing H10_FINAL artifact |

## LIO observability and initialization


The textured characterization world records `/lio/diagnostics` and retains only fixed initial-frame transforms. The stationary initialization baseline is: DYN-LONG LIO position P95 `2.0148407176845406e-06` m and velocity P95 `0.0026809969834497164` m/s; DYN-LAT LIO position P95 `0.0` m and velocity P95 `0.005479798623642757` m/s. In the GOOD dynamic segments, LIO position P95 is `0.04234076114068607` m (longitudinal) and `0.08044430919369527` m (lateral), with horizontal values separately retained in JSON/CSV. This distinguishes initialization bias from the later 3D/vertical residual and does not silently reclassify it.

## Measured nominal envelope

Criterion: GOOD requires GT trajectory-error P95 <= 0.10 m, MAX <= 0.175 m, no recovery/failsafe, and valid estimator health.

| Quantity | Limit | Evidence |
|---|---:|---|
| velocity | 0.5624988750005627 m/s | DYN-LONG GOOD segment |
| longitudinal acceleration | 0.2165062314375 m/s² | DYN-LONG GOOD segment |
| deceleration | 0.21650369681250065 m/s² | DYN-LONG GOOD segment |
| jerk | 0.2804066718750005 m/s³ | DYN-LONG GOOD segment |
| lateral acceleration | 0.012500000000000008 m/s² | DYN-LAT GOOD segment |

The envelope is the largest observed GOOD region in these two runs, not a theoretical actuator limit. MARGINAL/UNUSABLE segments remain evidence and are not promoted to nominal authority.

## Segment evidence

| Profile | Segment | Quality | V max | A max | J max | A_lat max | GT P95 | GT MAX |
|---|---|---|---:|---:|---:|---:|---:|---:|
| longitudinal | LONG_00 | MARGINAL | 0.18749962500018758 | 0.0721687438125 | 0.09346889062499991 | 0.0 | 0.12741975187922375 | 0.12870057360270296 |
| longitudinal | LONG_01 | GOOD | 0.5624988750005627 | 0.2165062314375 | 0.2804066718750005 | 0.0 | 0.0927760180465981 | 0.09442378060048531 |
| longitudinal | LONG_02 | UNUSABLE | 1.5624944444493831 | 0.8018753728395053 | 1.3833370370370406 | 0.0 | 0.26249490323447516 | 0.27554481385468876 |
| longitudinal | LONG_03 | UNUSABLE | 2.999938560314574 | 1.847492385177602 | 3.784836710399999 | 0.0 | 0.5157288281119438 | 0.5393514427748846 |
| lateral | LAT_00 | GOOD | 0.25000000000000006 | 0.6249282954482642 | 2.565120795424519 | 0.012500000000000008 | 0.09287822107174992 | 0.10148052910908899 |
| lateral | LAT_01 | UNUSABLE | 0.5000000000000001 | 1.2500434810052383 | 5.138365322087976 | 0.05000000000000003 | 0.579542214824316 | 0.6166697777157483 |
| lateral | LAT_02 | UNUSABLE | 1.0000000000000002 | 2.4995631593916583 | 10.315091636940796 | 0.2000000000000001 | 0.7673929244765905 | 0.7859174021903216 |
| lateral | LAT_03 | UNUSABLE | 1.5000000000000002 | 3.750544626576008 | 15.56851322882326 | 0.4500000000000001 | 0.8976134643657074 | 0.9181765117088211 |

## Exact E5 demand comparison

Generation 2 window: `29196000000`–`30140000000` ns. The measured demand-to-nominal ratios are: `{"planner_acceleration_mps2": 16.121327706987056, "planner_deceleration_mps2": 0.0, "planner_jerk_mps3": 36.232964669534276, "planner_lateral_acceleration_mps2": 13.288942878941608, "planner_speed_mps": 4.967444405950419}`.
The production snapshot records `traj_opt/boundary max_vel=12.0 m/s`, `max_acc=12.0 m/s²`, and `max_jerk=30.0 m/s³`; these remain configuration facts, not experimentally granted nominal authority.

## Scenario-scoped attribution

| Attribution | S_BAD_E5 | DYN-LONG | DYN-LAT |
|---|---|---|---|
| CLC-A | INCONCLUSIVE | CONFIRMED | CONFIRMED |
| CLC-B | INCONCLUSIVE | CONFIRMED | CONFIRMED |
| CLC-C | INCONCLUSIVE | REJECTED | CONFIRMED |
| CLC-D | INCONCLUSIVE | CONFIRMED | CONFIRMED |
| CLC-E | CONFIRMED | CONFIRMED | CONFIRMED |

CLC-A/B are evaluated against GT, not PX4-vs-LIO disagreement. CLC-C uses a stated diagnostic threshold of P95 controller delta-V > 0.10 m/s or delta-A > 0.20 m/s on a GOOD segment; the effective setpoint is a controller-layer observation, not a production behavior change. CLC-D is evaluated by measured PX4 state versus effective setpoint.

## Limitations

- Only one deterministic run per dynamic profile is included; repeat distributions remain required before changing production thresholds.
- MODE_LIO_REFERENCED, command-rate A/B, and difficult-map navigation regressions are deferred.
- The generic runner status is not used as the harness verdict: the standard runtime report expects external odometry, while this isolated primary mode intentionally disables that production path. Harness validity is `scenario.json: reason=COMPLETE` with no harness failures.

# FIRST PRODUCTION FIX

Selected fix: `PLANNER_CLOSED_LOOP_ENVELOPE`.

Measured basis: the exact existing E5 generation-2 demand exceeds the independently measured nominal envelope in speed, longitudinal acceleration, jerk, and lateral acceleration. PX4 and LIO both have valid low-demand textured-world evidence, so estimator fusion is not the highest-leverage first action; controller-layer corrections and plant-following remain secondary characterization signals. The fix must introduce one product-owned envelope without changing safety gates or treating marginal/unusable segments as nominal authority.

Preserve unchanged: the 0.25 m tracking certificate, fail-closed recovery, backup/emergency checks, estimator/world freshness and health gates, planner timing, PX4 gains, and mission acceptance rules.

Regression: rerun the exact original E5 map/route/speed and verify the original tracking-loss/emergency safety trigger remains fail-closed while unwanted demand-driven loss is removed; rerun the matched open-control run at the same speed and compare tracking distributions separately. No easier control run may invalidate the E5 result.

# P0.8 Odometry Supervisor

## Result

**P0.8 status: PASS**

P0.8A prerequisite hardening and P0.8B are implemented on the frozen P0.7
baseline. The known AIST processing-lag finding is carried from P0.0 and is
not attributed to P0.8.

## Revision

- Parent: `d6bd7f0349d90167a26bf3710e65341f8b469404`
- Branch: `feat/p0.8-odometry-supervisor`
- Commits: `d03f47f`, `68ff73c`, `8c4f014`, `d106b79`, `1720fd2`, `48c62cf`, `26e126e`, `6011beb`
- PX4 v1.17 submodule: `src/external/px4_msgs` at `86d8239e962f6939e05c3737784f60c02fa884db`

## Plan alignment

P0.8A was completed before P0.8B. P0.9 in-flight reinitialization and P0.10
PX4 interface-library work were not started.

## P0.8A prerequisite review

Reset metadata uses separate bounded timestamp-sorted local-position and
attitude histories. Association uses `timestamp_sample` and reset counters,
not callback order. Counter deltas are modulo-256; one-step changes are
accepted, jumps greater than one fail closed, and missing metadata is not
converted to a zero reset. Position, velocity, and attitude continuity are
compensated in NED/FRD conventions, with transition samples suppressed and
stable post-reset samples required.

The bridge validates PX4 sample timestamps against real ROS
`now().nanoseconds()`. Stale, future, duplicate, regression, overflow, and
invalid timestamps are rejected. Simulation pause does not advance freshness.
Finite zero angular velocity is valid; NaN and infinity are rejected.

Covariance conversion is frame-specific. NED/FRD source terms are transformed
to the ROS `odom`/`base_link` contract, orientation covariance is serialized to
`[3,3]`, `[4,4]`, `[5,5]`, and unavailable angular covariance is not relabeled.
Nonfinite, asymmetric, non-PSD, and zero-source cases are diagnosed.

The ring buffer clears on timestamp regression and generation changes, then
requires `stable_samples`. Reset grace clears residual persistence and cannot
reopen a degraded LIO gate. Query component/covariance masks use named
constants, preserve request nanoseconds exactly, and never extrapolate across
a generation. Diagnostics publish schema version 1 and non-OK rejection
states periodically.

## Package boundary

`src/odometry_supervisor` depends only on `ament_cmake`, `rclcpp`, `nav_msgs`,
`diagnostic_msgs`, `std_msgs`, `builtin_interfaces`, `navigation_interfaces`,
and Eigen3. It has no dependency on `px4_msgs`, `fast_lio_core`, Livox, or
`tf2_ros`, and builds without the PX4 submodule.

## Input contracts

The node consumes `/lio/odometry_propagated` as primary state,
`/lio/odometry_corrected` as the correction freshness anchor, and
`/px4/odometry_ros` for availability/latest tracking. It parses named
diagnostics `fast_lio/estimator` and `px4_odometry_bridge`, requiring
`diagnostic_schema_version=1`. Canonical uppercase LIO states
(`TRACKING`, `LOST`, `DEGRADED`, `RESETTING`) and legacy test values are
accepted.

## Time alignment

The supervisor keeps a bounded propagated-LIO history and selects the newest
epoch already covered by the PX4 ring buffer. It then makes one asynchronous
request to `/px4/sample_odometry_at_time`, associates the response with a
sequence and captured LIO sample, and discards late or failed responses. This
is exact-epoch sampling, not latest-vs-latest comparison or extrapolation.
Only one query is outstanding. The maximum alignment gap is 50 ms and service
timeout is 100 ms.

## Shared-origin gate

Independent comparison requires:

```text
initial_prior_source = topic
initial_prior_applied = true
initial_prior_fallback_applied = false
initial_prior_reason = TOPIC_PRIOR_ACCEPTED
```

No online SE(3) alignment or silent offset subtraction is performed. The final
SITL diagnostics show `TOPIC_PRIOR_ACCEPTED` and rejected count `0`.

## Residual mathematics

Position is compared in `odom`. Body velocities are rotated into the common
world frame. Orientation uses shortest quaternion error; yaw is wrapped to
`[-pi, pi]`; position-error growth uses sample-time deltas. Invalid or
unavailable comparisons produce no residual and are never converted to zero.

## Independent mode

Independent mode is canonical. Persistent divergence or LIO loss closes the
external-odometry gate and, only when a valid independent PX4 reference is
available, latches a machine-readable reinitialization request for P0.9. P0.8
does not execute that request.

## Correlated mode

Correlated mode monitors the same residuals and closes the gate on LIO loss,
but does not request independent reinitialization. The corrected artifact
`.artifacts/simulation/odometry-supervisor-faults-20260803/correlated_unhealthy_correlated.json`
ended `DIVERGED: LIO_LOST` with `reinitialization_requested=false`.

## FSM

The states are `STARTUP`, `HEALTHY`, `SUSPECT`, `DEGRADED`, and latched
`DIVERGED`. LIO loss or state corruption diverges immediately. Residual and
stale-stream conditions use persistence. A stale propagated stream invalidates
comparison and cannot recover on a held old residual.

## Persistence and hysteresis

Canonical SIM thresholds are centralized in
`src/odometry_supervisor/config/odometry_supervisor.yaml`. Persistence uses
sample time, not callback count; clear ratio is `0.70`. Recovery also requires
current LIO validity and fresh propagated and corrected streams. Clock-pause
behavior is covered by the state-machine test and does not decay from wall
time.

## Output actions

Reliable status is published on `/navigation/odometry_supervisor/status` with
transient-local depth 1; diagnostics are published on
`/navigation/odometry_supervisor/diagnostics`. Outputs include health, reason
codes, comparison validity, ages, alignment gap, residuals, gate state, speed
recommendation, hover/failsafe recommendation, and reinitialization request
state. No TF or PX4 actuation/input topic is published.

## Unit tests

- P0.8A PX4 bridge focused tests: `19/19` passed.
- P0.8B supervisor focused tests: `24/24` passed.
- Final focused result: `319 tests, 0 errors, 0 failures, 0 skipped`.
- Fault-injector Python tests: `2/2` passed.

Tests cover residual frames, quaternion sign/wrap, single outliers,
persistence/hysteresis, stale propagated input, LIO loss, correlated mode,
reset grace, time-generation invalidation, clock pause, invalid comparison,
and configuration rejection.

## ROS integration tests

The focused ROS build/test passed after the final freshness-gate fix. The
test-only injector provides machine-readable artifacts for healthy, jump,
drift, velocity-bias, PX4-stale, LIO-stale, reset-generation,
diagnostic-corruption, and correlated-unhealthy cases under:

```text
.artifacts/simulation/odometry-supervisor-faults-20260803/
```

The `clock_pause` catalog and state-machine test are present; a separate
external `/clock` pause was not run in this session.

## Dataset

`make data-check DATASET=aist-mid360-drive` passed. Current replay artifact:

```text
.artifacts/datasets/aist-mid360-drive/68ff73c-replay-1.0x-20260803T020755039370Z
```

Estimator and replay exit codes were zero; 55,435 IMU and 2,772 LiDAR samples
were received/processed; final queues, drops, and overflow were zero. The
processing-lag predicate still triggered at maximum queue depth 161 with load
shedding count 62:

```text
Dataset result: FAIL (carried P0.0-F01)
```

This is a pre-existing frozen-baseline finding and is not attributed to P0.8.

## Healthy SITL

Validated session:

```text
.artifacts/simulation/px4-mid360-20260803-091737
```

After warm-up, the supervisor ran for more than 60 simulated seconds. The
post-warmup snapshot recorded:

```text
health = HEALTHY
reason = HEALTHY
comparison_valid = true
time_aligned = true
external_odometry_allowed = true
reinitialization_requested = false
alignment_gap_ns = 0
position_error_m = 0.0665
velocity_error_m_s = 0.0125
orientation_error_rad = 0.00658
px4_reset_generation = 1
px4_time_generation = 0
```

The session report records finite corrected/propagated outputs, accepted topic
prior, schema version 1, and no estimator, bridge, or supervisor crash. The
existing SIM finite-point warning remains without a claimed cause.

## Fault injection

| Scenario | Observed result |
|---|---|
| healthy | HEALTHY; comparison valid; gate open; no reinit |
| single position jump | HEALTHY after the single outlier; no false divergence |
| velocity bias | SUSPECT persistence observed |
| PX4 stale | HEALTHY LIO retained; comparison invalid; `PX4_STALE`; no reinit |
| LIO propagated stale | DEGRADED; comparison invalid; gate closed |
| LIO corrected stale | DEGRADED; gate closed |
| PX4 reset generation | Reset grace followed by healthy recovery; no false divergence |
| correlated unhealthy | DIVERGED on LIO loss; no reinitialization request |

## Carried findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate triggered | Pre-existing; not caused or addressed by P0.8 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning retained | Pre-existing; no root cause claimed |
| P0.7-F02 | Simulation observer | Warning | Finite-point warning retained in corrected SITL | No root cause claimed |

## New findings

No new P0.8 production finding remains. The initial SITL uppercase `TRACKING`
parser mismatch and stale-residual recovery path were corrected and tested.
The reported transient initial-prior rejection was not reproduced in the final
session; final diagnostics show `TOPIC_PRIOR_ACCEPTED` and rejected count `0`.

## Files changed

Changes are limited to P0.8A bridge/interface hardening, the new
`src/odometry_supervisor` package, and test-only fault-injection tooling. No
estimator registration path, PX4 source, SDF, URDF, YAML baseline profile,
Makefile, or protected `mid360_px4_gazebo.yaml` file was changed.

## Non-goals confirmed

P0.8 does not implement P0.9 in-flight reinitialization, P0.10 interface
library integration, online frame alignment, GNSS fusion, NIS monitoring, map
reset, TF publication, planner control, motor control, flight-mode changes,
or PX4 input topics.

## Acceptance checklist

- [x] P0.8A completed before P0.8B
- [x] Reset/time/covariance/diagnostic/query prerequisite hardening
- [x] Supervisor dependency boundary
- [x] Propagated primary and corrected freshness anchor
- [x] Asynchronous one-outstanding exact-epoch PX4 query
- [x] No latest-vs-latest fallback or extrapolation
- [x] Shared-origin gate and residual frame mathematics
- [x] Independent/correlated modes
- [x] FSM, persistence, hysteresis, reset/time-generation handling
- [x] Machine-readable status and output gate
- [x] Focused tests and fault-injector artifacts
- [x] Dataset check; replay finding carried as P0.0-F01
- [x] Healthy PX4 v1.17 SITL for >60 s post warm-up
- [x] `make build-safe`
- [x] `make test`
- [x] `make check`
- [x] `make vendor-check`
- [x] `make px4-ingress-check`
- [x] Final working tree clean

## Final conclusion

```text
P0.8 status: PASS

The P0.8 odometry supervisor and prerequisite hardening are implemented and
reproducible. The dataset processing-lag failure and SIM finite-point warning
remain pre-existing baseline findings. They do not invalidate P0.8 and must
not be attributed to subsequent changes without a measured regression against
the frozen baseline.
```

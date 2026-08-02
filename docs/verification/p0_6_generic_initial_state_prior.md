# P0.6 Generic InitialStatePrior Verification

## Result

PASS

## Revision

- Parent: `53e7f1fafb894440dec76e4bf2639718bc880615`
- Branch: `feat/p0.6-initial-state-prior`
- Commits: core, ROS, tests, and verification-doc commits listed by `git log`

## Existing initialization audit

- IMU initialization completion: `ImuInitializer::tryInitialize()` remains the
  authority for gravity, tilt, and biases.
- `estimator.initialize` boundary: exactly once after IMU initialization and
  prior resolution; constructor initialization was removed.
- Bootstrap map boundary: first map reference is captured after prior
  application and the first prediction.
- Queue-consumption behavior: topic-pending `processNext()` returns before the
  synchronizer, preserving queued LiDAR.

## Prior contract

- Reference frame: `odom`
- Body frame: `base_link`
- Sources: zero, fixed, topic
- Contexts: ground startup, in-flight reinitialization policy
- Component mask: position, velocity, attitude
- Attitude modes: none, yaw-only, full

## Startup ordering

`IMU samples -> IMU initializer -> prior wait/validation/fallback -> prior
geometry application -> one-time estimator initialization -> first prediction
-> bootstrap map`. A pending topic prior is a sensor-time gate; it does not
consume or discard a LiDAR scan.

## Scope and baseline

P0.6 was implemented from parent `53e7f1fafb894440dec76e4bf2639718bc880615`
on branch `feat/p0.6-initial-state-prior`. The change is limited to a generic
initial-state prior at estimator startup. It does not add PX4 messages,
NED/FRD conversion, `/fmu` dependencies, planner/safety behavior, or runtime
in-flight reinitialization.

The internal nominal state remains at `livox_imu_frame`. The public prior
contract is `odom -> base_link` and the static geometry is
`T_base_imu = ^base_link T_livox_imu_frame`. The prior is applied after the
existing IMU initializer has produced gravity, roll/pitch, gyro bias and
accelerometer bias, and before the first estimator prediction and bootstrap
map capture. The pipeline now has one estimator initialization call per
startup generation; the constructor no longer initializes the estimator with
an unqualified default state.

## Implementation audit

| Requirement | Evidence |
|---|---|
| Zero, fixed and topic sources | `initial_state_prior.hpp`, `initial_state_prior_policy.hpp`, `parameter_loader.cpp` |
| Ground startup and in-flight context policy | `InitialStatePriorContext`; in-flight is unit-tested and rejects timeout fallback |
| Position/velocity/attitude component mask | `InitialStatePriorMask`; none/yaw-only/full attitude modes |
| Topic message semantics | `FastLioNode::onInitialStatePrior`, `nav_msgs/msg/Odometry`, `header.stamp`, `odom`, `base_link` |
| Geometry conversion | `InitialStatePriorApplicator`; `R_OB = R_OI R_IB`, lever-arm position and velocity equations |
| Yaw-only and full attitude safety | world-Z wrapped yaw delta; full-attitude gravity-tilt threshold |
| Non-destructive pending gate | `FastLioPipeline::processNext` returns before `MeasurementSynchronizer`; `MeasurementBuffer::nextLidarStartTime` and pipeline queue test |
| Candidate mailbox | mutex-protected candidate, strictly increasing candidate timestamps, late/clock/frame/value rejection counters |
| Diagnostics | `InitialPriorDiagnostics` and `/lio/diagnostics` key/value fields |
| Covariance/process/map/TF isolation | prior only changes the startup nominal state; no P0.5R or point-cloud path changes |

Zero prior means base position zero. The resulting IMU position includes the
resolved lever arm, so it is not silently treated as an IMU-origin prior.
Velocity uses the base-frame twist and `omega_B x r_BI`; a nonzero lever arm
requires angular velocity. No wall clock is used in the core prior policy.

## Configuration

The three canonical profiles contain:

```yaml
initial_prior:
  source: zero
  context: ground_startup
  components: {position: true, velocity: true, attitude: yaw_only}
  topic: /initial_state_prior
```

The topic source is available without changing the canonical runtime default.
Its timeout, maximum age, explicit fallback, fixed values, and full-attitude
tilt threshold are declared and validated at the ROS boundary.

## Focused tests

Added tests cover:

- zero-position base semantics and static lever arm;
- yaw-only tilt preservation and wrap across `+/-pi`;
- full-attitude gravity-tilt rejection;
- base twist plus angular lever-arm velocity;
- in-flight reject-on-timeout policy validation;
- topic-prior pending state preserving queued LiDAR;
- valid topic prior applying once before bootstrap and rejecting a late candidate.

## Acceptance results

| Check | Result | Evidence |
|---|---|---|
| Core build | PASS | `cmake --build build/fast_lio_core -j2` |
| ROS build | PASS | `cmake --build build/fast_lio_ros -j2` |
| Focused prior tests | PASS | `test_initial_state_prior`, `test_initial_state_prior_pipeline` |
| Existing pipeline/measurement tests | PASS | focused `ctest` run |
| Canonical profile parse | PASS | `test_parameter_loader`: all canonical YAMLs loaded |
| Full repository tests/check/vendor | PASS | `make test`: 271 tests; `make check`; `make vendor-check` |
| Dataset runtime acceptance | PASS | AIST replay exit 0; 55,435 IMU and 2,772 LiDAR accepted; no drops/overflow/lag |
| SIM runtime acceptance | PASS | headless session exit/cleanup completed; no process crash; prior applied |

Dataset artifact:
`.artifacts/datasets/aist-mid360-drive/53e7f1f-replay-1.0x-20260802T134832519547Z/`.
The recorded `/lio/diagnostics` snapshots report `initial_prior_status=closed`,
`initial_prior_source=zero`, `initial_prior_applied=true`,
`initial_prior_fallback_applied=false`, `initial_prior_candidate_count=0`, and
`initial_prior_rejected_count=0`. The transport summary reports zero IMU/LiDAR
drops, zero overflow, zero processing-lag exceedance, 2,759 successful
corrections, and node/replay return code 0.

SIM artifact:
`.artifacts/simulation/px4-mid360-20260802-205410/`.
The final estimator diagnostic reports `initial_prior_status=closed`,
`initial_prior_source=zero`, `initial_prior_applied=true`, application epoch
`4220000000` in simulation time, and no fallback/candidates/rejections. The
workflow was stopped and cleaned up successfully. The observer retained the
baseline finite-point warning: finite ratio `0.40478515625`, `+Inf=82756`,
`-Inf=136`, with no process crash. This is a runtime observation retained for
later investigation, not a P0.6 prior failure.

The repository has no root `05_TASK_BACKLOG_AND_ACCEPTANCE.md` or
`task_backlog.yaml` file at this baseline, so neither file was fabricated or
modified. P0.5R carried findings remain unchanged and are not silently fixed
by P0.6.

## Findings carried forward

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.5R-R01 | Corrected/propagated covariance diagnostics | Observation | Aggregate fields retain last-writer semantics | Not addressed by P0.6 |
| P0.5R-R02 | Output covariance diagnostics | Observation | Output asymmetry is currently classified as non-PSD | Not addressed by P0.6 |
| P0.5R-R03 | Numerical Jacobian tests | Observation | Randomized-seed coverage can be expanded | Not addressed by P0.6 |
| P0.5R-R04 | Angular-rate covariance documentation | Observation | Wording cleanup remains deferred | Not addressed by P0.6 |

## Final status

P0.6 status: PASS

The generic initial-state prior is source-complete, reproducible, and accepted
by focused tests, the full repository gate, AIST replay, and headless SIM.
The dataset has no new P0.6 prior finding. The SIM finite-point condition is
retained as a baseline warning and was not attributed to the prior. P0.7 is
the recommended next task; P0.7 was not started in this change.

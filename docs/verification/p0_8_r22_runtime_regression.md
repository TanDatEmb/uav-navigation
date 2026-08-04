# P0.8-R22 — Runtime regression, exact-time prior, queried alignment, and external gates

Date: 2026-08-04
Repository: `/home/letandat/Dev/uav-navigation`
Branch: `fix/odometry-lifecycle-and-alignment`
Implementation HEAD used for static validation: `30708abbb110bde412fc19f7bb573c89a80a9535`

## Scope and boundary

This report isolates the first runtime regression that produced zero corrected
FAST-LIO output at `c853efb` and `38ad14c`, fixes exact-time physical prior
propagation, and records the queried PX4 alignment and external-odometry gate
implementation.

The following were not changed or claimed:

- no merge to `main`, P0.9/P0.10 work, planner/safety/occupancy/world-model,
  loop-closure, or global-localization work;
- no IKFoM or ikd-tree registration-math change and no registration-iteration
  increase;
- no Gazebo PointCloud repair and no threshold relaxation;
- no claim that in-flight reinitialization, EKF2 fusion, PX4 full fusion, or
  process restart is implemented. The lifecycle coordinator retains only an
  odometry-only last-good snapshot and emits a request; it never applies a
  warm-start state.

The protected configuration was not modified. Its final SHA-256 is:

`src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml`
`2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f`

## Provenance

All SITL runs used the same command profile, world/model, PX4 tree, and
`px4_msgs` submodule (`86d8239e962f6939e05c3737784f60c02fa884db`):

```text
make sim-px4-mid360 PX4_DIR=/home/letandat/Dev/Autopilot-p0.7-v1.17 \
  GZ_GUI=0 SIM_PROFILE=debug SUPERVISOR_ENABLED=1 ENABLE_RVIZ=0 \
  AUTO_SNAPSHOT=0 SESSION_ROOT=<external-artifact-root>
```

There is no explicit random seed in the checked-in world or runner. The
artifact roots below are machine-local and intentionally not committed:

- `98085ee`: `/tmp/uav-navigation-r22-artifacts/980/px4-mid360-20260804-080551`
- `c853efb`: `/tmp/uav-navigation-r22-artifacts/c853/px4-mid360-20260804-082017`
- `38ad14c`: `/tmp/uav-navigation-r22-artifacts/38ad-rerun/px4-mid360-20260804-082424`
- fixed runtime: `/tmp/uav-navigation-r22-artifacts/final/px4-mid360-20260804-090224`

Two later attempts at final HEAD `30708ab` reached the startup bridge timeout
and were excluded from runtime metrics as invalid setup attempts:
`final2/px4-mid360-20260804-090726` and
`final3/px4-mid360-20260804-090852`.

## Phase A — identical SITL regression matrix

| Source | Duration | Corrected / registered | Local maps | PointCloud finite ratio | Final diagnostic state | Result |
|---|---:|---:|---:|---:|---|---|
| `98085ee` | 39.5 s | 347 / 347 | 35 | 0.404785 | `TRACKING` | WARN |
| `c853efb` | 49.5 s | 0 / 0 | 0 | 0.404785 | `INITIALIZING_MAP` | WARN |
| `38ad14c` | 55.5 s | 0 / 0 | 0 | 0.404785 | `INITIALIZING_MAP` | WARN |
| fixed runtime at `cd6b686` | 117.0 s | 49 / 49 streams; 52 corrected scans | 5 sampled | 0.404785 | later `LOST` | WARN |

The fixed runtime is not a full acceptance pass. It proves that the former
zero-correction stage is crossed: `53` correction attempts produced `52`
successful corrections (`0.981132`). The run later hit input queue overflow
and processing lag, then lost the re-anchor/bracket condition:

```text
transport: processing_lag_exceeded=true, lidar_drop_count=312,
           registration_update_count=52
estimator: status=LOST,
           prediction_rejection_reason="Prediction IMU start bracket is missing"
propagated: requires_reanchor=true, continuity_reset_count=16
```

The unchanged finite ratio (`0.404785`) and repeated `+Inf` PointCloud values
remain a separate simulator/input-quality warning. This report does not claim
that downstream queue, PointCloud, or final-flight acceptance is fixed.

## First failing runtime stage and root cause

The binary search was:

1. `98085ee`: corrected output was present.
2. `c853efb`: accepted and synchronized LiDAR existed, but correction attempts
   and registration updates were zero.
3. `38ad14c`: reproduced the same zero-correction result.

At `c853efb`, the initial topic-prior fallback had been accepted, but the
pipeline still kept `propagation_start` at `scan.start_time`. The guard then
rejected because the physical estimator state epoch belonged to the prior.
The rejection occurred before `IkfomEstimator::predict()`, so it was not a
registration failure. The decisive diagnostic was
`PROPAGATION_START_DOES_NOT_MATCH_STATE_TIME`.

## Phase B — exact-time physical propagation

Implemented in `22ad86b1179185328a156c543337a086d67c0e65` and completed in
`30708ab`:

- initial prediction starts at the accepted prior's exact sample epoch;
- bounded IMU history is merged with the current group and exact-epoch
  duplicates are deduplicated with the current group taking precedence;
- strict clock-domain/order checks, both physical brackets, timestamp
  regression checks, and interval gap checks are fail-closed;
- only the interval `[start_time, end_time]` is gap-checked, so a gap before an
  already handled rebase is not incorrectly charged to the new prediction;
- direct `MeasurementGroup` callers retain their IMU samples in the same
  bounded history as ROS ingress;
- `IkfomEstimator::predict()` is the only prediction path; no handwritten IMU
  integrator was added;
- `propagated_to_application` is set only after a successful nonzero physical
  prediction;
- prior covariance is symmetrized and PSD-checked before filter/state mutation,
  with invalid covariance leaving the filter unchanged.

The failing physical-state test was added before the fix:
`PhysicalStatePredictionUsesPriorEpochWithoutSkippingOrDuplicatingImu`.
The related matrix also covers missing brackets and transactional invalid
covariance. All 10 `InitialStatePriorPipelineTest` cases pass.

## Phase C–G — queried alignment and lifecycle

Alignment now requests `/px4/sample_odometry_at_time` asynchronously at the
latest propagated LIO epoch. One bounded alignment request is outstanding at a
time; sequence, timeout, stale-response, exact response timestamp, response
frame (`px4_odom`/`base_link`), position/orientation validity masks, and current
LIO/PX4 reset/time generations are checked before a sample enters the bounded
estimator window. A queried response is used directly; equal-timestamp history
matching is not used for alignment.

The lifecycle is explicit: `UNALIGNED`, `COLLECTING`, `PROVISIONAL`,
`LOCKED`, `REVALIDATING`, and `INVALID`. A candidate must pass three successive
bounded stability windows; each next candidate must remain within the configured
translation/yaw step limits before the stable counter advances. A locked
transform is not adapted. Reset continuity alone does not reinitialize the
transform; public reset/time/LIO generation changes invalidate it and start a
new generation. Query failure/timeout moves the lifecycle to revalidation and
closes comparison authorization.

Yaw estimation has two explicit modes:

- `ORIENTATION_AIDED` consumes only samples whose per-sample authoritative flag
  is true;
- `MOTION_OBSERVED` uses centered weighted 2-D trajectories and rejects
  stationary windows as yaw-unobservable.

Final yaw is recomputed before final translations. The reported effective
  sample count is the weighted effective count, covariance is an estimate with
  the corresponding unbiased denominator, and uniform weights are used because
  PX4 source covariance is not consumed by the alignment estimator.

The lifecycle coordinator distinguishes startup invalidity from loss after
tracking confirmation. Its snapshot is explicitly odometry-only evidence, not
an applied warm-start or reinitialization success.

## Phase H–I — independent gates and external odometry

The supervisor publishes three separate truths:

- `cross_comparison_valid` — exact-time PX4 comparison evidence is valid;
- `external_measurement_publishable` — LIO freshness/quality/timestamp/
  covariance/generation/continuity prerequisites are valid;
- `external_measurement_authorized` — the configured reference policy and
  health state permit publication.

Independent mode may authorize a publishable LIO measurement without PX4
comparison evidence. Correlated mode requires fresh PX4 evidence, continuity,
post-reset stability, alignment, and a valid comparison. The external bridge
uses `external_measurement_authorized`, not a stale or implicit PX4 fusion
assumption.

External conversion rejects unknown, zero, negative, or non-finite diagonal
pose/twist covariance; it does not silently replace those values with `1e-6`.
Local/world and body-frame transforms remain separate. The generic
`fast_lio.launch.py` default for `enable_external_odometry` is now `false`;
PX4 external publication remains opt-in.

`timestamp_sample` provenance is explicit: PX4 `VehicleOdometry.timestamp_sample`
is converted to the ROS odometry header stamp used by the exact-time service,
while external input uses the LIO measurement timestamp for
`timestamp_sample` and wall/ROS now only for transport timestamping.

## Static validation

Commands were run serially where worker tests share build/test artifacts:

- `colcon build --packages-up-to odometry_supervisor px4_odometry_bridge navigation_bringup` — passed;
- `ctest --test-dir build/fast_lio_core --output-on-failure` — **24/24 passed**;
- `ctest --test-dir build/odometry_supervisor --output-on-failure` — **47/47 passed**;
- `ctest --test-dir build/px4_odometry_bridge --output-on-failure` — **2/2 passed**;
- `git diff --check` — passed;
- protected YAML SHA check — passed.

The current worktree is clean. Runtime logs, JSONL timelines, and generated
reports remain outside the repository under `/tmp/uav-navigation-r22-artifacts`.

## Acceptance status

P0.8-R22 is **partially complete**:

- first failing runtime stage: isolated;
- exact-time prior physical propagation: implemented and unit-tested;
- async exact-time queried alignment, lifecycle, yaw modes, covariance, and
  separate external gates: implemented and statically tested;
- full runtime acceptance/no-regression: **not proven**. The successful fixed
  runtime crossed the zero-correction regression but later degraded under queue
  overflow/processing lag; final-HEAD reruns were startup-invalid due bridge
  timeout. No claim is made beyond the evidence above.

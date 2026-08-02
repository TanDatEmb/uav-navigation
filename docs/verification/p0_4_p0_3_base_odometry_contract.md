# P0.4/P0.3 Base Odometry Contract

## Result

- P0.4 status: `PASS`
- P0.3 status: `PASS`

P0.4 was completed first because P0.3 must consume an angular velocity tied to
the same estimator epoch as the state. P0.3 then converts the complete state
at the ROS boundary and publishes the corrected/propagated base-link contract.

## Revisions

- Parent: `a4910a9dd4c4c228b5392a9d4ab790384f52e511`
- Branch: `feat/p0.2-base-link-state-converter`
- Commits:
  - `56c2d6e feat(lio): provide exact-epoch angular velocity`
  - `06be0c3 feat(ros): publish corrected and propagated base odometry`
  - `docs(verification): validate base odometry contract` (this report)

## Dependency rationale

P0.4 preceded P0.3 so both corrected and propagated ROS odometry paths use a
defined angular-velocity epoch, bias policy, and fail-closed missing-data
policy. The ROS layer does not synthesize angular velocity or use a latest
sample unrelated to the state timestamp.

## Exact-epoch angular velocity

### Corrected

- Epoch: `group.scan.end_time`, the timestamp of the corrected `StateEstimate`.
- Sample/bracket policy: use an IMU sample at the exact epoch when available;
  otherwise linearly interpolate raw gyro between strict bracketing samples in
  the same clock domain. Duplicate/non-monotonic samples, clock mismatch,
  non-finite values, and missing brackets are rejected.
- Bias source: `StateEstimate.state.gyro_bias_rad_s()` is subtracted after
  interpolation.
- Missing-data policy: no corrected kinematic result is produced; the ROS
  odometry path fails closed.

### Propagated

- Epoch: the propagated estimator timestamp `propagated_time`.
- Pairing policy: the propagated state is paired atomically with
  `history_.back()` at that exact timestamp; a timestamp mismatch is rejected.
- Replay policy: replay/reanchor updates the state and history first, then the
  latest history sample is paired with the published propagated state.
- Bias source: the same `StateEstimate.state.gyro_bias_rad_s()` subtraction
  policy as corrected output.

### Common policy

- Zero fallback: none. Missing or invalid angular velocity is a diagnostics
  failure and prevents publication.
- Extrapolation: none. The resolver only accepts an exact sample or a strict
  interpolation bracket.

## Base conversion

- `T_base_imu` source: one bounded static-TF resolution from
  `base_link` (target) to `livox_imu_frame` (source), provided by
  `robot_state_publisher:/tf_static`. The validated Xacro configuration uses
  `base_link -> livox_frame` translation `[0, 0, 0.28]` and
  `livox_frame -> livox_imu_frame` translation
  `[0.011, 0.02329, -0.04412]`.
- Cache policy: the resolved transform is stored in one immutable
  `BaseLinkStateConverter` shared by all output paths.
- Runtime lookup count: one successful static resolution during node startup;
  bounded retries are used only while waiting for the static publisher.
- Converter reused: corrected odometry, propagated odometry, and dynamic TF
  publication all use the same cached converter.

## Corrected ROS contract

- Topic: `/lio/odometry_corrected`.
- Header frame: `odom`.
- Child frame: `base_link`.
- Timestamp: exact corrected estimator epoch.
- Pose: estimator pose converted to the base body frame.
- Twist: linear velocity expressed in the base body frame and angular
  velocity converted to the base body frame.
- Publication is fail-closed if static geometry or finite angular velocity is
  unavailable.

## Propagated ROS contract

- Topic: `/lio/odometry_propagated`.
- Header frame: `odom`.
- Child frame: `base_link`.
- Timestamp: exact propagated estimator epoch.
- Pose and twist use the same base-link serializer and converter as corrected
  output.
- The dataset and SIM profiles used for this validation have propagated
  odometry disabled, so their live propagated publication count is zero. The
  propagated publisher and converter contract are covered by focused tests.

## TF ownership

- Dynamic TF: only the selected odometry owner publishes `odom -> base_link`.
- TF owner when propagated enabled: propagated output.
- TF owner when propagated disabled: corrected output.
- Duplicate authority: no duplicate IMU dynamic edge remains; corrected
  publication is suppressed while propagated output owns the dynamic TF.
- TF timestamp regression: timestamps that are duplicate or non-monotonic are
  suppressed (`timestamp <= last_published_timestamp`).
- Dataset evidence: corrected owner, 2,759 dynamic TF publications, zero
  timestamp suppressions and zero conversion failures.
- SIM evidence: corrected owner, 329 dynamic TF publications, zero timestamp
  suppressions and zero conversion failures.

## Static TF tree

The validated static tree is:

```text
base_link
└── livox_frame
    └── livox_imu_frame
```

The Xacro expansion and `check_urdf` validation passed. No `imu_link`,
`lidar_link`, `mid360_lidar_frame`, or `mid360_imu_frame` appears in the
canonical tree.

- Registered points frame: `livox_frame`.
- Local map frame: `odom`.

## Covariance unavailable policy

The odometry serializer preserves zero covariance arrays as an explicit
transitional `unavailable` representation. No covariance relabeling,
rotation, or projection is claimed in P0.3. Covariance projection is deferred
to P0.5.

## Test results

| Command | Exit | Result |
|---|---:|---|
| Focused P0.4 build/test (`fast_lio_core`, `fast_lio_ros`) | 0 | 21/21 core tests and 10/10 ROS tests passed before P0.3 wiring |
| Focused P0.3 ROS `ctest --test-dir build/fast_lio_ros --output-on-failure` | 0 | 11/11 passed |
| `make build MODE=release` | 0 | Build passed for all packages |
| `make test MODE=release` | 0 | 253 tests, 0 failures |
| `make check MODE=release` | 0 | 253 tests, 0 errors, 0 failures, 0 skipped; data guard passed |
| `make vendor-check` | 0 | 18 files, 2 pinned upstream SHAs, 3 documented patched files |
| Xacro + `check_urdf` static-frame validation | 0 | Canonical three-link tree and configured origins passed |
| `python3 -m unittest tools.runtime.tests.test_ros_replay -v` | 0 | 15 tests passed |
| `python3 -m pytest -q src/uav_description/test/test_sensor_frames_contract.py` | 0 | 2 tests passed |

## Dataset

`make data-check DATASET=aist-mid360-drive` passed. The successful replay was:

```text
make data-replay DATASET=aist-mid360-drive RATE=1.0 ENABLE_RVIZ=0
```

The authoritative artifact is:

`.artifacts/datasets/aist-mid360-drive/169b521-replay-1.0x-20260802T114957646171Z/summary.json`

The summary reports estimator exit `0`, replay exit `0`, and no failures:

- Received/processed IMU: `55,435 / 55,435`.
- Received/processed LiDAR: `2,772 / 2,772`.
- Synchronized groups: `2,771`.
- Final input/IMU/LiDAR queues: `0 / 0 / 0`.
- Maximum queue depth: `55`.
- IMU/LiDAR drops: `0 / 0`.
- Overflow: `false`, count `0`.
- Processing lag: `0 ns`; `processing_lag_exceeded=false`.
- Corrections: `2,759` successful, `1` failed attempt.
- Dynamic TF owner: `corrected`; publications `2,759`; timestamp suppressions
  `0`; conversion failures `0`.
- Static geometry: ready, source `robot_state_publisher:/tf_static`.

## SIM

The headless PX4/MID-360 workflow completed and was cleaned up successfully.
The session report is:

`.artifacts/simulation/px4-mid360-20260802-185455/REPORT.md`

The report recorded no process crash. Corrected odometry, registered points,
and dynamic TF continued to publish; the stream counts were 330 odometry,
330 registered-points, and 330 TF messages. The observer sampled 38 scans and
reported finite ratio `0.40478515625`, below the warning threshold `0.5` (error
threshold `0.1`), with no NaN XYZ values, 46,246 positive-Inf XYZ values, and
76 negative-Inf XYZ values in the sampled scans. The sampled point-cloud
frame was `livox_frame`.

```text
SIM workflow completed and cleaned up successfully. The finite-point condition is retained as a baseline warning for later investigation.
```

The artifacts do not prove a root cause; none is asserted.

## P0.0 findings

- P0.0-F01 — Dataset processing-lag predicate: the frozen baseline predicate
  was `latest_received_time_ns - latest_processed_time_ns >
  maximum_processing_lag_ms * 1,000,000`, with threshold `500 ms`.
  P0.0 recorded a triggering lag of `554.251202 ms` across 10 diagnostic
  records, with maximum `638.232363 ms`, maximum queue depth `169`, no drops,
  no overflow, and final queues drained. The current P0.3 replay did not
  trigger the predicate (`0 ns`, maximum queue depth `55`), but the finding
  remains a retained pre-existing frozen-baseline finding and is not
  reclassified by this task.
- P0.0-F02 — SIM observer finite-point warning: the current SIM observation
  also remains below the `0.5` warning threshold at `0.40478515625`. There was
  no process crash; corrected odometry, registered points, and TF continued,
  and cleanup succeeded. This remains a baseline warning without an asserted
  cause.

## Files changed

- P0.4 core: exact-epoch angular-velocity state/resolver, pipeline and
  propagated-worker integration, diagnostics, and focused tests.
- P0.3 ROS: static-TF resolver, shared base-link converter/serializer,
  corrected and propagated odometry publishers, TF ownership/monotonicity,
  runtime diagnostics, and focused tests.
- Launch/runtime wiring: canonical sensor-frame launch validation, replay
  static-frame fixture, and headless SIM launch arguments.
- This verification report.

No P0.5 covariance projection, PX4 estimator integration, supervisor/reinit,
safety, planner, occupancy, loop-closure, or global-localization work was
started.

## Acceptance checklists

### P0.4

- [x] Corrected angular velocity is tied to the corrected estimator epoch.
- [x] Exact-sample and strict-bracket interpolation policies are implemented.
- [x] IMU gyro bias is subtracted from the raw angular velocity.
- [x] No extrapolation and no zero fallback.
- [x] Propagated state and angular velocity are paired at one exact epoch.
- [x] Missing, duplicate, non-finite, and clock-mismatched samples fail closed.
- [x] Angular-velocity diagnostics are observable.
- [x] Focused P0.4 tests and full repository gates passed.

### P0.3

- [x] Static `T_base_imu` is resolved once and cached in one shared converter.
- [x] Corrected and propagated odometry use the same base-link serializer.
- [x] Corrected topic uses `odom -> base_link` and corrected epoch.
- [x] Propagated topic uses `odom -> base_link` and propagated epoch.
- [x] Twist values are expressed in the `base_link` body frame.
- [x] Dynamic TF ownership is exclusive for both enabled and disabled modes.
- [x] Duplicate and regressed TF timestamps are suppressed.
- [x] Canonical static tree and sensor-frame launch contract validated.
- [x] Covariance remains explicitly unavailable for P0.5.
- [x] Full build, test, check, vendor, dataset, and SIM gates completed.

## Final conclusion

```text
P0.4 status: PASS
P0.3 status: PASS

The exact-epoch angular velocity contract and the corrected/propagated
base-link ROS odometry contract were validated against the frozen parent
baseline. Dataset replay passed its current runtime gates, while the retained
P0.0 baseline findings remain explicitly separated from implementation status.
The SIM finite-point warning is retained for later investigation; no cause is
asserted from the available artifacts.
```

# P0.2 Base-Link State Converter

## Result

PASS

## Revision

- Parent: `c999805ec7c79f682204166aec0c07bef138c12b`
- Branch: `feat/p0.2-base-link-state-converter`
- Commits: `f1638c7` implementation, documentation commit at `HEAD`; no push

## Scope

Implemented a ROS/PX4-independent rigid-body converter in
`fast_lio_core`. It converts a `StateEstimate` whose reference point is
`livox_imu_frame` into kinematics at `base_link`, when the caller supplies
angular velocity at the same state epoch.

Deferred to P0.3:

- corrected and propagated ROS publisher integration;
- ROS odometry child-frame and twist contract changes;
- dynamic TF changes.

Deferred to P0.4:

- obtaining a bias-corrected gyro sample at the output epoch;
- runtime angular-velocity wiring.

Deferred to P0.5:

- covariance Jacobian projection;
- base-link covariance output.

No ROS topics, TF runtime behavior, estimator process model, IKFoM state,
deskew, registration, map, or point-processing path was changed.

## Transform convention

`RigidTransform(target, source, q, t)` represents `^target T_source`.

- `T_base_imu = ^base_link T_livox_imu_frame`.
- `T_imu_base = inverse(T_base_imu)`.
- `T_odom_imu = ^odom T_livox_imu_frame`, constructed from the state pose.
- `T_odom_base = T_odom_imu * T_imu_base`.

Pose conversion is performed through `RigidTransform::compose`; no manual
translation subtraction is used.

## Pose equations

With `r_BI` expressed in `base_link`:

```text
p_OB = position(T_odom_base)
R_OB = rotation(T_odom_base)
```

The result is `^odom T_base`, so its orientation maps base vectors into odom.

## Velocity equations

The caller supplies `omega_I` at exactly `StateEstimate.time`. The converter
computes:

```text
omega_B = R_BI * omega_I
v_OB = v_OI - R_OB * (omega_B cross r_BI)
v_B  = transpose(R_OB) * v_OB
```

There is no zero-angular-velocity fallback and no pose-only conversion API.

## Static transform source

- P0.1 source: `base_link -> livox_frame -> livox_imu_frame`, with the vehicle
  mount and internal Livox geometry kept as separate transforms.
- Construction boundary: caller resolves and composes the physical static
  transforms, then passes `T_base_imu` to the converter constructor.
- The converter does not duplicate `[0, 0, 0.28]`, factory extrinsics, YAML,
  Xacro, or TF lookup logic.
- Cached fields: `T_base_imu`, `T_imu_base`, `R_base_imu`, `r_base_imu`, and
  frame metadata.
- Runtime lookup count: zero.
- Inverse count: once, during construction.

## Converter API

```cpp
BaseLinkStateConverter(RigidTransform base_to_imu);
Result<RigidBodyState> convert(
    const StateEstimate& imu_estimate,
    const Eigen::Vector3d& angular_velocity_imu_rad_s) const;
```

`RigidBodyState` exposes explicit source, reference, and body frame IDs,
timestamp, `^reference T_body`, reference-expressed body linear velocity,
body-expressed linear velocity, and optional body-expressed angular velocity.
It intentionally has no covariance field. A successful `Result` is the state
validity signal; non-finite input is rejected without clamping or mutation.

## Frame validation

The static transform must be exactly:

```text
base_link -> livox_imu_frame
```

The constructor factory rejects reversed, unrelated, or non-finite transforms.
The existing `RigidTransform::Create` rejects non-finite coefficients and
near-zero quaternions; its constructor normalizes accepted quaternions.
Dynamic conversion rejects non-finite state and angular velocity input.

## Test matrix

| Test | Input | Expected | Result |
|---|---|---|---|
| Identity | identity `T_base_imu`, arbitrary pose/velocity/omega | pose preserved; velocity expressions and omega unchanged | pass |
| Hover lever arm | identity attitude, 3D `r_BI`, zero omega | position subtracts full XYZ lever arm; velocity unchanged | pass |
| Pure yaw, stationary base | `v_OB = 0`, nonzero `omega_B` and `r_BI` | lever-arm velocity cancels to zero | pass |
| Pure yaw, translating base | known `v_OB` plus synthetic lever-arm term | exact `v_OB` and body velocity recovered | pass |
| Roll/pitch/yaw | nonzero 3D attitude and asymmetric lever arm | rigid composition and full 3D correction | pass |
| Mounting rotation | non-identity `R_BI` and arbitrary `omega_I` | `omega_B = R_BI * omega_I` | pass |
| Arbitrary 3D state | nonzero pose, velocity, omega, and translation | analytical pose/twist result | pass |
| Frame direction | reversed/unrelated static frame IDs | `kInvalidArgument` | pass |
| Non-finite/quaternion | NaN/Inf state, omega, translation, and degenerate quaternion | explicit rejection | pass |
| Timestamp/no mutation | sensor-domain timestamp and copied inputs | exact timestamp and unchanged inputs/cache | pass |
| Covariance API | `RigidBodyState` type inspection | no covariance field or relabelled base covariance | pass |
| Point-processing guard | existing `T_odom_imu * T_imu_lidar` path | unchanged IMU-origin point transform | pass |

## Covariance policy

`StateEstimate::covariance` remains the IMU-origin tangent/error-state
covariance. P0.2 neither copies it into `RigidBodyState` nor labels it as
base-link covariance, and performs no Jacobian projection. No zero covariance
is synthesized.

## Point-processing invariant

The existing point path remains:

```text
p_odom = T_odom_imu * T_imu_lidar * p_lidar
```

`BaseLinkStateConverter` is not called by `ManifoldState::transformLidarPointToOdom`
and does not introduce `base_link` into deskew, registration, or map hot paths.
The existing frame-convention composition test remains green.

## Commands

| Command | Exit code | Result |
|---|---:|---|
| `make build MODE=release PACKAGES=fast_lio_core` | 0 | focused build pass |
| `make test MODE=release PACKAGES=fast_lio_core` | 0 | 20 CTest tests and repository Python suites pass |
| `make build MODE=release` | 0 | full build pass |
| `make test MODE=release` | 0 | full test pass |
| `make check MODE=release` | 0 | pass; 244 tests, no errors/failures |
| `make vendor-check` | 0 | pass |
| dataset replay/fetch | not run | not required; converter is not wired to runtime |
| PX4 SIM | not run | not required; no runtime behavior changed |

## Files changed

- `src/navigation_estimator/fast_lio_core/include/fast_lio_core/navigation/rigid_body_state.hpp`
- `src/navigation_estimator/fast_lio_core/include/fast_lio_core/navigation/base_link_state_converter.hpp`
- `src/navigation_estimator/fast_lio_core/src/navigation/base_link_state_converter.cpp`
- `src/navigation_estimator/fast_lio_core/test/test_base_link_state_converter.cpp`
- `docs/verification/p0_2_base_link_state_converter.md`

## Non-goals confirmed

- P0.0-F01 dataset processing-lag baseline: unchanged and not re-tested.
- P0.0-F02 SIM finite-point observer warning: unchanged and not re-tested.
- No ROS publisher, topic, child-frame, or dynamic TF change.
- No raw IMU acceleration transform.
- No per-sample or per-point TF lookup.
- No manual Z offset.
- No zero angular-velocity fallback.
- No covariance relabel or projection.
- No IKFoM/process-model/state-dimension/index change.
- P0.3, P0.4, and P0.5 were not started.

## Acceptance checklist

- [x] Started at parent `c999805`
- [x] Working tree initially clean
- [x] Converter is in ROS-independent `fast_lio_core`
- [x] Converter receives static `T_base_imu`
- [x] Converter caches transform and inverse
- [x] No TF lookup in `convert()`
- [x] Pose conversion uses composition with correct direction
- [x] Full 3D position lever arm
- [x] Angular velocity rotates IMU to base
- [x] Linear velocity lever-arm sign is covered by pure-yaw tests
- [x] Body-frame velocity uses `R_odom_base`
- [x] Required kinematic and validation tests pass
- [x] Timestamp preserved and inputs unmodified
- [x] No manual Z offset or zero angular fallback
- [x] No covariance relabel or projection
- [x] IKFoM/process model unchanged
- [x] Point-processing path unchanged
- [x] ROS topics, frame IDs, and dynamic TF unchanged
- [x] Focused tests pass
- [x] Full build/test/check/vendor-check pass
- [x] Working tree clean after commit
- [x] P0.3 not started
- [x] P0.4 not started
- [x] P0.5 not started

## Final conclusion

P0.2 status: PASS

The base-link state converter is implemented and mathematically validated in
the ROS-independent core. It preserves the IMU-origin estimator contract,
applies the cached static transform and correct lever-arm kinematics, and
does not alter current ROS runtime behavior. P0.3 may wire this converter into
the corrected and propagated output contracts.

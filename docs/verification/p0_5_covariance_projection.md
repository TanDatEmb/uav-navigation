# P0.5 Covariance Projection — superseded by P0.5R

This document records the historical P0.5/P0.5A audit at the pre-implementation
baseline. The state-conditional projection described here as a future task is
now implemented and verified in
`docs/verification/p0_5r_state_covariance_projection.md`; the deferred
per-sample gyro covariance decision remains unchanged.

## Roadmap correction

The original P0.5 requirement to obtain authoritative per-sample raw gyro
covariance before projecting odometry covariance is `INVALID / OVERSPECIFIED`.
It is superseded by `P0.5R — State covariance projection`.

`P0.5A` per-sample gyro covariance is `DEFERRED / NOT REQUIRED FOR CORE
ROADMAP`. No calibration artifact, bag rewrite, ROS message covariance
fallback, or fake covariance was added. AIST all-zero covariance remains the
normal unknown/unavailable ROS message condition; it does not prevent
FAST-LIO propagation, deskew, correction, or P0.4 epoch pairing.

`P0.5R` is `READY` for a separate implementation task. Its required output is
pose covariance at `base_link` expressed in `odom`, and base linear-velocity
covariance expressed in `base_link`, both projected from the full IKFoM
23-state covariance with full cross terms and analytical/numerical Jacobian
validation.

The conditional velocity covariance must not independently add a calibrated
`Qgyro` term. The current filter does not maintain the state/current-input
cross covariance needed to justify that addition, and adding it could
double-count process noise already represented in `P_state`.

## Result

`SUPERSEDED`

The historical audit correctly found no authoritative raw gyro covariance for
AIST or REAL, but that absence does not block state covariance projection. The
old conclusion that P0.5 required such a source was too strong. No covariance
projector or ROS covariance wiring was implemented on this revision.

## Revision

- Parent: `d67d5419453160c01eb79950b4853e50c2ef5e75`
- Branch: `feat/p0.5-covariance-projection`
- Commits: documentation-only blocker/audit commit for this report

## Scope

- Implemented: mandatory state/frame/noise audit and runtime source checks.
- Deferred: P0.5R implementation, ROS serialization, diagnostics, covariance
  tests, dataset acceptance, and SIM covariance acceptance.
- Explicitly not started: P0.5A per-sample gyro covariance calibration/source
  work.

## State index mapping

The mapping is confirmed by `ManifoldState` constants and the
`MTK_BUILD_MANIFOLD(IkfomState, ...)` declaration in
`fast_lio_core/estimation/ikfom_state.hpp`:

| Block | Offset | Dimension | Tangent convention |
|---|---:|---:|---|
| Position `p_odom_imu` | `ManifoldState::kPositionOffset` = 0 | 3 | Euclidean, odom |
| Orientation `R_odom_imu` | `kOrientationOffset` = 3 | 3 | `MTK::SO3` right-local |
| LiDAR–IMU rotation | `kExtrinsicRotationOffset` = 6 | 3 | `MTK::SO3`, fixed in production |
| LiDAR–IMU translation | `kExtrinsicPositionOffset` = 9 | 3 | Euclidean |
| Velocity `v_odom_imu` | `kVelocityOffset` = 12 | 3 | Euclidean, odom |
| Gyro bias | `kGyroBiasOffset` = 15 | 3 | Euclidean, rad/s |
| Accelerometer bias | `kAccelBiasOffset` = 18 | 3 | Euclidean, m/s² |
| Gravity on `S2` | `kGravityOffset` = 21 | 2 | `MTK::S2` tangent |

`ManifoldState::kErrorStateDimension == 23`. The offsets are contiguous and
non-overlapping; the compile-time constants are the intended projector API,
not numeric literals.

## Orientation perturbation convention

The upstream `MTK::SO3::boxplus` implementation is:

```text
R' = R * Exp(delta_theta)
```

This is a right-local perturbation. The corresponding upstream boxminus is
`Log(R_nominal.transpose() * R_perturbed)`. The existing IKFoM Jacobian test
also validates this right perturbation convention. P0.5 must derive its
output residuals from this convention rather than assuming a left perturbation.

## Source covariance

- Corrected source: `StateEstimate::covariance`, returned by
  `IkfomEstimator::covariance()` from `filter_.get_P()` after the corrected
  IKFoM update.
- Propagated source: the same 23x23 IKFoM covariance after propagated
  prediction/replay at the propagated state epoch.
- Units/frame: tangent covariance in the internal IMU-origin estimator state;
  it is not a ROS 6x6 covariance and cannot be relabeled.
- Validation already present upstream: finite, symmetric, and PSD checks are
  used for filter prediction/correction acceptance. P0.5 still needs its own
  scale-aware source/output validation and diagnostics.

## Gyro measurement covariance

- ROS source audit: `RosImuAdapter` currently copies angular velocity and
  acceleration only; it discards both ROS covariance arrays. `ImuSample` has no
  covariance field.
- AIST runtime source check: the recorded `/livox/imu` message reported
  `angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]`. This is an
  unavailable sentinel, not a valid zero-noise measurement covariance.
- REAL profile: no driver/config/datasheet covariance source is defined in the
  repository.
- SIM source: the SDF explicitly declares gyro noise `stddev=0.001 rad/s` per
  axis at 200 Hz, implying `Q_gyro = 1e-6 I (rad/s)^2` for that simulated
  measurement model. A live `/lidar/imu` snapshot confirmed
  `[9.99999997e-7, 0, 0, 0, 9.99999997e-7, 0, 0, 0, 9.99999997e-7]`.
  This does not provide the missing AIST/REAL source.
- Existing `IkfomEstimatorConfig::gyro_noise_standard_deviation` is used by
  `processNoise()` as process-noise covariance (`sigma * sigma`) during IKFoM
  prediction. It is not a documented per-sample raw gyro covariance and must
  not be reused for this purpose.
- Interpolation policy: not implemented. Once a valid per-sample source is
  available, independent endpoint samples must use
  `Q_interp=(1-alpha)^2 Q0 + alpha^2 Q1`.

## Output convention

The P0.5R convention is:

### Pose

- Error vector: `[δp_odom_base, δtheta_odom_base]`.
- Expression frame: `odom`.
- Orientation residual: left residual
  `Log(R_perturbed * R_nominal.transpose())` in `odom`, with the analytical
  Jacobian derived consistently from the right-local IKFoM state perturbation.

### Base linear velocity

- Error vector: `δv_base`.
- Expression frame: `base_link`.
- The linear-velocity covariance describes velocity at the base origin in
  `base_link`, conditional on the resolved bias-corrected gyro measurement at
  the output epoch. Angular-rate covariance is not a P0.5R acceptance gate.

## Analytical Jacobians

Not implemented on this documentation-only correction. P0.5R must use the cached P0.2
`BaseLinkStateConverter` geometry, fixed-size Eigen matrices, the full 23x23
covariance including cross terms, and no TF lookup or numerical Jacobian in
production.

## Numerical Jacobian validation

Not run. No projector exists on this superseded audit revision. The required test must
use the repository's `IkfomState::boxplus`, central differences over all 23
columns, pose Lie-log residuals, full twist recomputation, and report epsilon,
maximum error, and worst block.

## PSD policy

Not implemented. The required future policy is symmetry projection followed by
finite/symmetry/PSD validation, with scale-aware repair only for roundoff-size
negative eigenvalues. Materially non-PSD source or output must be rejected;
absolute-eigenvalue repair and arbitrary diagonal epsilon are prohibited.

## Unavailable policy

P0.5R must fail closed when the state covariance or projected output is
unavailable or invalid. It must not publish an odometry sample carrying
all-zero or fake state covariance. Dynamic TF may continue from a valid state,
with state validity and covariance validity kept separate. Angular-rate
covariance may remain unavailable in diagnostics and is not a PX4 egress gate.

## Corrected integration

Not implemented. Corrected output currently has the P0.3 base-link frame and
epoch contract, but its covariance remains transitional/unavailable.

## Propagated integration

Not implemented. The propagated state, resolved angular rate, and state
covariance must later be carried as one exact epoch/generation snapshot and
must use the same P0.5R projector as corrected output. A raw gyro covariance is
not part of that snapshot contract.

## ROS serialization

Not implemented. The eventual serializer must fill the valid pose and linear
velocity covariance blocks row-major, preserve cross terms, leave angular-rate
covariance explicitly unavailable, and reject all-zero state covariance.

## Diagnostics

Not implemented. The required availability, failure counters, covariance
traces/eigenvalues, roundoff-repair counter, reference frames, and gyro
covariance source diagnostics remain future P0.5 work.

## Focused tests

No P0.5 implementation tests were added. Existing audit evidence includes the
upstream right-perturbation Jacobian test and the source/static checks described
above.

## Full repository validation

No build/test/check was rerun because no source or configuration file changed.
The mandatory pre-task git gate passed:

| Command | Exit | Result |
|---|---:|---|
| `git status --short --branch` | 0 | Correct branch and clean tree |
| `git rev-parse HEAD` | 0 | `d67d5419453160c01eb79950b4853e50c2ef5e75` |

## Dataset

The source AIST bag was played read-only for an IMU covariance audit. The
first `/livox/imu` message exposed an all-zero angular covariance. Therefore
P0.5 dataset covariance acceptance was not attempted and no dataset runtime
finding was changed.

Source bag:

`/home/letandat/snap/code/253/.local/share/uav-nav/datasets/aist-mid360-drive/source/rosbag2_2024_04_16-14_17_01`

## SIM

The headless SIM workflow was started only to inspect the raw IMU covariance,
then stopped through the scoped session cleanup command. The live covariance
was finite and matched the SDF model (`1e-6 I`), but no P0.5 odometry
covariance was published or accepted because the projector was not implemented.

Session:

`.artifacts/simulation/px4-mid360-20260802-191402`

Cleanup completed successfully. The existing P0.0-F02 finite-point warning was
not modified or investigated.

## P0.0 findings

- P0.0-F01 — Dataset processing-lag baseline: retained unchanged; no P0.5
  replay was run and no lag tuning was performed.
- P0.0-F02 — SIM finite-point warning: retained unchanged; the audit-only SIM
  run did not change its interpretation or scope.

## Files changed

- `docs/verification/p0_5_covariance_projection.md` — this superseded audit and
  P0.5R roadmap correction.
- `docs/architecture/estimator_state.md` — confirmed 23-DoF mapping and
  right-local perturbation convention.
- `docs/architecture/frame_conventions.md` — documented output covariance
  frames and the fixed-transform condition.
- `docs/fast_lio.md` and
  `docs/verification/propagated_odometry_semantics.md` — corrected current
  P0.3 output/covariance documentation and recorded the P0.5R correction.

No source, config, test, Makefile, SDF, URDF, or YAML file was changed.

## Non-goals confirmed

No P0.6 Generic InitialStatePrior, P0.7 PX4 ingress bridge, P0.8 supervisor,
P0.9 reinitialization, P0.10 PX4 egress, process-model change, IKFoM rewrite,
extrinsic uncertainty, point/map covariance, point-processing change, TF
authority change, or covariance tuning was started.

## Acceptance checklist

- [x] Started at parent `d67d541`.
- [x] Correct branch and clean initial working tree.
- [x] State index mapping audited from source.
- [x] Orientation perturbation convention audited from upstream implementation.
- [x] Corrected/propagated covariance source identified as the same IKFoM 23x23
  covariance.
- [x] AIST all-zero gyro covariance and SIM valid covariance were measured.
- [x] Existing process-noise parameter was rejected as a gyro measurement
  covariance source.
- [x] No fabricated covariance or partial PASS was introduced.
- [x] P0.0-F01 and P0.0-F02 were left unchanged.
- [x] P0.6 was not started.
- [x] P0.5A per-sample gyro covariance is explicitly deferred.
- [x] Missing AIST/REAL raw gyro covariance does not block P0.5R.
- [ ] Analytical projector implemented.
- [ ] Numerical Jacobian acceptance passed.
- [ ] Corrected and propagated ROS covariance serialization passed.
- [ ] Dataset and SIM covariance acceptance passed.

## Final conclusion

```text
P0.5 old specification: INVALID / OVERSPECIFIED

P0.5A status: DEFERRED / NOT REQUIRED FOR CORE ROADMAP

P0.5R status: READY

The audit confirmed the 23-DoF IKFoM state layout, right-local orientation
perturbation, IMU-origin covariance semantics, and cached base-link geometry.
The next task is P0.5R: project pose and conditional base linear-velocity
covariance from `filter_.get_P()` with full cross terms. Do not block that work
on AIST/REAL per-sample gyro covariance and do not start P0.6 until P0.5R is
accepted.
```

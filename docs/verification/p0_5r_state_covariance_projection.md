# P0.5R State-Conditional Covariance Projection

## Status

`P0.5R: PASS`

P0.5R projects the full 23-dimensional IKFoM error-state covariance into the
base-link ROS odometry contract. The implementation is on branch
`feat/p0.5-covariance-projection`, based on parent `1f547a1`.

P0.5A per-sample gyro covariance remains deferred. P0.6 was not started.

## Contract

The source covariance is `StateEstimate::covariance`, at the same epoch as the
state and resolved angular velocity carried by `KinematicStateEstimate`. It is
an IMU-origin tangent covariance and is not copied or relabeled into a ROS
6x6 message.

The state ordering is fixed at 23 error dimensions:

| Block | Offset | Dimension |
|---|---:|---:|
| position | 0 | 3 |
| orientation, right-local SO(3) | 3 | 3 |
| IMU-to-LiDAR rotation | 6 | 3 |
| IMU-to-LiDAR translation | 9 | 3 |
| IMU velocity | 12 | 3 |
| gyro bias | 15 | 3 |
| accelerometer bias | 18 | 3 |
| gravity tangent | 21 | 2 |

The cached static transform is `^base_link T_livox_imu_frame`, with
`R_BI` and `r_BI` expressed in `base_link`. The projector uses:

```text
R_OB = R_OI R_IB
p_OB = p_OI - R_OB r_BI
omega_B = R_BI (y_g - b_g)
v_B = R_BO v_OI - omega_B x r_BI
```

The analytical Jacobians are the corresponding pose and twist Jacobians under
the right-local orientation convention. Position, orientation, velocity, and
gyro-bias blocks are used as specified; extrinsic, accelerometer-bias, and
gravity columns are zero in this fixed-calibration/conditional-gyro contract.
All covariance cross terms are retained by `J P J^T`.

## Validation and failure policy

The source and both projected 6x6 matrices are checked for finite values,
symmetry, positive semidefiniteness, positive trace, and nonzero information.
The symmetry tolerance is `1e-8`. The PSD tolerance is
`1e-10 * max(1, max_abs_diagonal)`. Only tiny negative eigenvalues are clamped
as roundoff repair; material asymmetry or non-PSD input fails closed.

An invalid projection prevents the affected odometry message from being
published. Dynamic TF continues through its existing authority, and the
registered-point/local-map path remains in `odom` and is not transformed by
the covariance feature. Angular-rate covariance is not fabricated and is not
an acceptance gate.

Corrected and propagated publishers use the same serializer and the same
projector geometry. The propagated worker's exact-epoch/generation pairing is
preserved; no process model, IMU covariance, TF authority, or point-processing
logic was changed.

Diagnostics include:

- semantic: `state_conditional_on_resolved_gyro`;
- expression frames: pose `odom`, twist `base_link`;
- availability, success/failure, source and output failure counters;
- traces, minimum eigenvalues, roundoff repairs, and projection timing.

## Focused evidence

| Check | Result |
|---|---|
| Analytical Jacobian vs `IkfomState::boxplus` finite difference | PASS; 7 projector tests |
| Full covariance and cross-covariance propagation | PASS |
| Zero lever arm and fixed-extrinsic columns | PASS |
| Nonfinite, asymmetric, material non-PSD, zero source fail-closed behavior | PASS |
| Tiny negative eigenvalue roundoff repair | PASS |
| Shared ROS serializer row-major pose/twist covariance | PASS |
| Runtime counters and timing diagnostics | PASS |
| `fast_lio_core` focused test suite | PASS; 22/22 |
| `fast_lio_ros` focused test suite | PASS; 11/11 |

## Dataset runtime acceptance

Artifact:
`.artifacts/datasets/aist-mid360-drive/1f547a1-replay-1.0x-20260802T130149577493Z/`

The read-only replay completed with estimator exit code 0 and replay exit code
0. The final diagnostic snapshot reported:

| Metric | Value |
|---|---:|
| covariance projection success | 2,759 |
| covariance projection failure | 0 |
| source nonfinite/asymmetry/non-PSD/zero | 0 / 0 / 0 / 0 |
| output pose nonfinite/non-PSD | 0 / 0 |
| output twist nonfinite/non-PSD | 0 / 0 |
| pose expression frame | `odom` |
| twist expression frame | `base_link` |
| final processing lag | `0 ns` |
| queue overflow/drop count | `false` / `0` |
| projection time mean/max | `42.087713 / 94 us` |

The existing baseline dataset processing-lag finding remains a separate
baseline acceptance result and was not investigated or modified by P0.5R.

## SIM runtime observation

Artifact:
`.artifacts/simulation/px4-mid360-20260802-200658/`

The headless SIM completed and was cleaned up with the scoped session stop
command. The final corrected estimator diagnostic reported 438 successful
covariance projections and 0 projection failures, source failures, or output
failures; the transport snapshot reported 430 successful projections. The
pose/twist expression frames were `odom`/`base_link`; 427 odometry,
registered-point, and TF samples were observed, with no process crash.

The report remains `WARN` because the existing finite-point observer warning
was triggered (`finite ratio 0.404785`, warning threshold `0.5`). No cause is
asserted here and no point-processing change was made.

`SIM workflow completed and cleaned up successfully. The finite-point
condition is retained as a baseline warning for later investigation.`

## Acceptance checklist

- [x] Parent `1f547a1` and required branch verified.
- [x] Full state mapping and right-local perturbation convention locked.
- [x] Analytical Jacobians verified against finite differences.
- [x] Full covariance cross terms projected with `J P J^T`.
- [x] Corrected and propagated outputs use the shared serializer/projector.
- [x] Invalid covariance fails closed without changing TF or point frames.
- [x] No gyro covariance source, process model, or covariance tuning added.
- [x] Focused build/tests passed.
- [x] Dataset replay diagnostics captured.
- [x] SIM diagnostics captured and session cleaned up.
- [x] P0.6 not started.

## Conclusion

P0.5R status: PASS

The state-conditional covariance projection is implemented and validated for
the corrected and propagated ROS odometry boundary. Pose covariance is
expressed in `odom`, twist covariance in `base_link`, and both are derived
from the full state covariance conditioned on the resolved exact-epoch gyro.
The existing dataset processing-lag finding and SIM finite-point warning are
retained as pre-existing runtime observations; neither is attributed to this
covariance change.

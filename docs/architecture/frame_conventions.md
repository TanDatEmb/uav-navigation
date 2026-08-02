# Frame conventions

M1 follows ROS REP-103: SI units and right-handed FLU axes. The TF tree is
`odom -> base_link -> livox_frame -> livox_imu_frame`. `odom` is a continuous local LIO
world frame with possible drift; it is not a global `map`, and M1 never publishes
`map -> odom`.

| Frame | Parent | Axes/origin | Transform source | Dynamic |
| --- | --- | --- | --- | --- |
| `odom` | none | local right-handed, Z up | LIO state | no |
| `base_link` | `odom` | X forward, Y left, Z up; UAV reference/near CoM | vehicle definition | yes |
| `livox_frame` | `base_link` | LiDAR sensing origin; FLU points | URDF/factory geometry | no |
| `livox_imu_frame` | `livox_frame` | IMU sensing origin; FLU measurements | URDF/factory geometry | no |

Notation `^A T_B` transforms coordinates from B to A. The core state estimates
`T_odom_imu`; point registration uses `T_odom_imu * T_imu_lidar * p_lidar`.
The URDF defines `base_link -> livox_frame -> livox_imu_frame`; runtime config
names must match it. The factory nominal `T_L_I` is the parent/child transform
with translation `[0.011, 0.02329, -0.04412] m` and identity rotation. FAST-LIO
uses its exact inverse `T_I_L` with translation `[-0.011, -0.02329, 0.04412] m`.
`T_imu_lidar` is derived from those static transforms
and is never an anonymous matrix. ROS quaternions are ordered `x, y, z, w`.

The real profile uses factory nominal geometry, not unit calibration. The AIST
profile intentionally retains its dataset-specific estimator extrinsic and
requires separate provenance. In SIM, `[0, 0, 0.28] m` is the vehicle mount
`base_link -> livox_frame`; it is not the internal LiDAR-to-IMU transform.
Future PX4
NED/FRD conversion is confined to a future PX4 boundary, not this repository.

## Odometry covariance contract

The P0.3 odometry topics use `header.frame_id=odom` and
`child_frame_id=base_link`. P0.5R publishes pose covariance as
`[delta p_odom_base, delta theta_odom_base]` in `odom` and twist covariance as
`[delta v_base, delta omega_base]` in `base_link`. Both are analytical
projections of the full IKFoM 23-DoF covariance with cross terms preserved; the
internal covariance is not relabeled. The velocity projection is conditional
on the resolved bias-corrected gyro sample paired with the output epoch.
Angular-rate covariance is not a P0.5R acceptance gate.

The old P0.5A per-sample gyro covariance requirement is deferred and does not
block P0.5R. `T_base_imu` is deterministic static calibration and is resolved
once/cached; it has no modeled calibration uncertainty in this contract.

## Initial-state prior contract

P0.6 accepts an optional generic initial-state prior with public semantics
`odom -> base_link`. The internal IKFoM nominal state remains expressed at
`livox_imu_frame`; the prior is converted through the cached
`T_base_imu = ^base_link T_livox_imu_frame` geometry before the one-time
estimator initialization. For a base prior `(p_OB, v_OB, omega_B)`, the
conversion is:

```text
R_OB = R_OI R_IB
p_OI = p_OB + R_OB r_BI
v_OI = R_OB v_B + R_OB (omega_B x r_BI)
```

Zero position means `p_OB = 0`, not `p_OI = 0`. The prior can provide zero,
fixed, or `nav_msgs/msg/Odometry` topic values; topic timestamps are sensor
timestamps from the message header and are never replaced with callback time.
Ground-startup timeout fallback is explicit in configuration. In-flight
reinitialization is defined and unit-tested as reject-on-timeout only; it is
not runtime-wired by P0.6. Position, velocity, and attitude components are
independently masked. Yaw-only changes world-Z yaw while preserving IMU
gravity-derived roll/pitch; full attitude is accepted only when its gravity
tilt agrees with the IMU initialization threshold.

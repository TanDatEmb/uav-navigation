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
`child_frame_id=base_link`. A future valid covariance projection must express
pose error as `[delta p_odom_base, delta theta_odom_base]` in `odom` and linear
velocity error as `delta v_base` in `base_link`. It must project the full IKFoM
23-DoF covariance; the internal covariance must not be relabeled. The velocity
projection is conditional on the resolved bias-corrected gyro sample at the
output epoch. Angular-rate covariance is not a P0.5R acceptance gate.

The old P0.5A per-sample gyro covariance requirement is deferred and does not
block P0.5R. `T_base_imu` is deterministic static calibration and is resolved
once/cached; it has no modeled calibration uncertainty in this contract.

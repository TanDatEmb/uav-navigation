# Frame conventions

M1 follows ROS REP-103: SI units and right-handed FLU axes. The TF tree is
`odom -> base_link -> {imu_link, lidar_link}`. `odom` is a continuous local LIO
world frame with possible drift; it is not a global `map`, and M1 never publishes
`map -> odom`.

| Frame | Parent | Axes/origin | Transform source | Dynamic |
| --- | --- | --- | --- | --- |
| `odom` | none | local right-handed, Z up | LIO state | no |
| `base_link` | `odom` | X forward, Y left, Z up; UAV reference/near CoM | vehicle definition | yes |
| `imu_link` | `base_link` | sensing origin; FLU IMU measurements | URDF/calibration | no |
| `lidar_link` | `base_link` | point-cloud origin; FLU points | URDF/calibration | no |

Notation `^A T_B` transforms coordinates from B to A. The core state estimates
`T_odom_imu`; point registration uses `T_odom_imu * T_imu_lidar * p_lidar`.
The URDF defines `base_link -> imu_link` and `base_link -> lidar_link`; runtime
config names must match it. `T_imu_lidar` is derived from those static transforms
and is never an anonymous matrix. ROS quaternions are ordered `x, y, z, w`.

All extrinsic numeric defaults are explicitly `PLACEHOLDER`, not calibration.
Measure and validate them before any real-data or flight claim. Future PX4
NED/FRD conversion is confined to a future PX4 boundary, not this repository.

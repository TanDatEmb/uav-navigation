# Frame conventions

The ROS graph uses explicit frame names. `lio_odom` is the local LIO world
frame and `px4_odom` is the PX4 ingress boundary frame. The repository-owned
TF tree is:
`lio_odom -> base_link -> livox_frame -> livox_imu_frame`.

| Producer | `header.frame_id` | `child_frame_id` | Topic |
|---|---|---|---|
| FAST-LIO corrected/propagated | `lio_odom` | `base_link` | `/lio/odometry_corrected`, `/lio/odometry_propagated` |
| PX4 ingress bridge | `px4_odom` | `base_link` | `/px4/estimator_odometry` |
| external odometry bridge input | `lio_odom` | `base_link` | `/lio/odometry_propagated` |
| Gazebo ground truth | `world`/`odom` | `base_link` | `/sim/ground_truth/odometry` |

## Coordinate contract

There are two independent basis changes. They must not be combined or replaced
with a guessed yaw:

```text
LIO/Gazebo world ENU (z-up)       PX4 NED (z-down)
  +x east                         +x north
  +y north                        +y east
  +z up                           +z down

                 [ 0  1  0 ]
p_ned = C_ned_enu [ 1  0  0 ] p_enu
                 [ 0  0 -1 ]

ROS base_link FLU                 PX4 body FRD
  +x forward                      +x forward
  +y left                         +y right
  +z up                           +z down

                 [ 1  0  0 ]
v_frd = C_frd_flu [ 0 -1  0 ] v_flu
                 [ 0  0 -1 ]
```

The Gazebo smoke world is ENU and the offboard scenario setpoints are NED.
The direct simulator measurement is therefore decisive: a NED setpoint
`[3, 0, -2]` appears in Gazebo as approximately `[0, 3, 2]`.

PX4 `VehicleOdometry` uses `pose_frame=NED` and `velocity_frame=NED` on the
`/fmu/in/vehicle_visual_odometry` boundary. Its quaternion is the passive
Hamilton quaternion body-FRD -> world-NED. Its angular velocity is always
body-FRD. The ingress bridge performs the inverse NED/FRD -> ENU/FLU conversion
before publishing ROS messages; the values on `/px4/estimator_odometry` are
already ROS ENU/FLU despite the PX4-origin frame name.

The bridge anchors the continuous `px4_odom` position at its first valid sample
and retains reset compensation for later PX4 resets.

The external-odometry bridge converts position, velocity, and attitude using
`C_ned_enu`; it converts only angular velocity using `C_frd_flu`. Thus
`x_px4=y_lio`, `y_px4=x_lio`, and `z_px4=-z_lio`. A PX4 `POSE_FRAME_FRD` sample
is rejected at ingress unless an explicit measured world alignment is added;
the bridge never invents a local yaw offset.

Timestamp mapping, covariance conversion, public frame generation, freshness,
finite-value validation, and geometric-jump latching are kept at this
boundary. Publication is additionally gated by the compact LIO health
diagnostics; no alignment or supervisor process is involved.

Notation `^A T_B` transforms coordinates from B to A. The core state estimates
`^lio_odom T_imu`; point registration uses
`^lio_odom T_imu * ^imu T_lidar * p_lidar`. ROS quaternions are ordered
`x, y, z, w`.

## Covariance

FAST-LIO publishes pose covariance as
`[delta p_lio_odom_base, delta theta_lio_odom_base]` in `lio_odom` and twist
covariance as `[delta v_base, delta omega_base]` in `base_link`. The PX4
bridge accepts only finite, positive variances and never treats missing
covariance as valid external odometry.

## Initial prior

The simulation configuration uses the explicit ground-startup prior contract:

```yaml
initial_prior:
  source: topic
  context: ground_startup
  source_frame: px4_odom
  source_frame_transform: startup_coincident
  topic: /px4/estimator_odometry
```

The dataset configuration uses the same-frame zero prior. These are workflow
inputs, not runtime alignment fallbacks.

`/px4/estimator_odometry` is subscribed only after PX4 NED/FRD has been decoded
by `FrameConverter`. The initial-prior callback copies the full converted
quaternion and body-FLU velocity; it does not extract a raw PX4 yaw angle.
`yaw_only` changes only the yaw relative to the gravity-aligned IMU attitude,
while preserving the measured roll/pitch. This makes the startup yaw reference
an explicit ENU quantity rather than a silent NED scalar.

The simulator ground-truth topic is independent of LIO and PX4 estimates. The
runtime report compares position, world/body velocity, angular velocity,
attitude, frame IDs, and absolute timestamps against it.

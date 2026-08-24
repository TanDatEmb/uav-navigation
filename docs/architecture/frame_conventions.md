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
| Gazebo ground truth (evaluation only) | `world`/`odom` | `base_link` | `/sim/ground_truth/odometry` |

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

## RViz profile

The project profile `src/navigation_bringup/rviz/fast_lio.rviz` uses
`lio_odom` as its fixed frame. It shows TF, `/lio/registered_points`, and
`/lio/odometry_propagated`; `/lio/odometry_corrected` is disabled by default
for optional comparison. It contains no map/planner visualization topics.

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

Simulation and replay use the LIO-local zero prior. The stationary IMU
initializer establishes the initial LIO state; LIO does not subscribe to PX4
or simulator odometry at startup:

```yaml
initial_prior:
  source: zero
  context: ground_startup
  source_frame: lio_odom
  source_frame_transform: same_frame
```

`/px4/estimator_odometry` remains an observation topic emitted by the ingress
bridge. It is not an input to FAST-LIO. This removes startup yaw/position
feedback from PX4 and makes the LIO estimate independent of simulator truth.

## Ground-truth isolation and PX4 estimator sources

`/sim/ground_truth/odometry` is an evaluation reference only. Its only
repository consumers are the runtime monitor and report. The external odometry
bridge subscribes only to `/lio/odometry_propagated` and additionally requires
the exact `lio_odom -> base_link` frame pair, so a Gazebo `world`/`odom`
message is rejected even when all numeric fields are valid.

PX4's upstream Gazebo bridge normally enables `SIM_GZ_EN_ODOM=1`. That setting
subscribes to the same Gazebo `odometry_with_covariance` topic and internally
publishes it as `vehicle_visual_odometry`, creating a second source that can
compete with the ROS LIO bridge. The project launcher sets the following before
PX4 `rcS` starts the Gazebo bridge and EKF2:

| PX4 parameter | Value | Purpose |
|---|---:|---|
| `SIM_GZ_EN_ODOM` | `0` | prohibit direct Gazebo truth -> `vehicle_visual_odometry` |
| `SIM_GZ_EN_GPS`, `SIM_GPS_USED`, `EKF2_GPS_CTRL` | `1`, `10`, `7` | enable normal simulated GNSS position, altitude, and velocity aiding |
| `SIM_GZ_EN_BARO`, `EKF2_BARO_CTRL` | `1`, `1` | enable normal barometer transport and height aiding |
| `EKF2_RNG_CTRL` | `1` | enable PX4's conditional rangefinder aiding when a source is available |
| `EKF2_MAG_TYPE` | `0` | use PX4's normal automatic magnetometer fusion |
| `EKF2_EV_CTRL` | `15` | enable EV horizontal position, vertical position, velocity, and yaw |
| `EKF2_HGT_REF` | `1` | select GNSS as the normal multisensor height reference |

The only runtime route into PX4's external-vision input remains:

```text
/lio/odometry_propagated (ENU/FLU, lio_odom -> base_link)
  -> px4_external_odometry_bridge (exact ENU/FLU -> NED/FRD conversion)
  -> /fmu/in/vehicle_visual_odometry
```

The PX4 startup log prints these effective parameters for each session. The
runtime monitor's `estimator_status_flags` are retained as telemetry only;
they do not claim per-sample fusion evidence. The report compares LIO, PX4
input, and PX4 output against simulator truth only after recording them.

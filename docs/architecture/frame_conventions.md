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

PX4 NED/FRD pose and velocity fields are decoded with explicit basis
conversions before the ingress bridge publishes ROS ENU/FLU data. The bridge
anchors the continuous `px4_odom` position at its first valid sample and
retains reset compensation for later PX4 resets.

The external-odometry bridge converts `lio_odom` to PX4 FRD and body-FRD
fields. Timestamp mapping, covariance conversion, public frame generation,
freshness, finite-value validation, and geometric-jump latching are kept at
this boundary. Publication is additionally gated by the compact LIO health
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

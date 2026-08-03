# Frame conventions

The ROS graph uses explicit frame names. `lio_odom` is the local LIO world
frame; `px4_odom` is the PX4 boundary world frame. Neither is silently
relabeled as the other. The TF tree owned by this repository is:
`lio_odom -> base_link -> livox_frame -> livox_imu_frame`. No
`map -> lio_odom` or global-localization transform is published.

| Producer | `header.frame_id` | `child_frame_id` | World contract | Topic |
| --- | --- | --- | --- | --- |
| FAST-LIO corrected/propagated | `lio_odom` | `base_link` | ROS local ENU/Z-up, continuous local origin | `/lio/odometry_corrected`, `/lio/odometry_propagated` |
| PX4 ingress bridge | `px4_odom` | `base_link` | PX4 source convention retained; NED is converted to ROS ENU, FRD remains PX4 local | `/px4/estimator_odometry` |
| supervisor status | `lio_odom` | n/a | metadata is reported in the LIO contract | `/navigation/odometry_supervisor/status` |

PX4 `VehicleOdometry.pose_frame` and `velocity_frame` are decoded before any
ROS message is published. NED pose/velocity uses the explicit ENU/NED basis;
FRD pose/velocity uses the explicit body/world axis basis and is marked
`px4_frd_local`. Mixed NED/FRD world fields are rejected. The bridge reports
`source_pose_frame`, `source_velocity_frame`, `world_convention`, output frame,
and conversion rejection reason in diagnostics.

The supervisor captures one alignment record only after FAST-LIO has accepted a
ground-startup prior from `px4_odom`. The configured transform is
`^lio_odom T_px4_odom = Identity` at that startup epoch, represented by
`WorldAlignment{target_frame=lio_odom, source_frame=px4_odom}`. This is an
explicit startup-coincident contract, not a sign flip, yaw offset, topic
relabel, or runtime threshold adjustment. Every PX4 service response is
transformed using the captured record before residual calculation. The record
contains source/target frames, epoch, PX4 reset/time generations, source, and
reinitialization count. A generation change invalidates the comparison and
requires a new startup-coincident alignment; missing or mismatched alignment
fails closed.

Notation `^A T_B` transforms coordinates from B to A. The core state estimates
`^lio_odom T_imu`; point registration uses
`^lio_odom T_imu * ^imu T_lidar * p_lidar`. ROS quaternions are ordered
`x, y, z, w`. The URDF defines `base_link -> livox_frame -> livox_imu_frame`.
The factory nominal `T_L_I` has translation `[0.011, 0.02329, -0.04412] m` and
identity rotation; FAST-LIO uses its exact inverse `T_I_L`.

## Odometry covariance contract

FAST-LIO publishes pose covariance as
`[delta p_lio_odom_base, delta theta_lio_odom_base]` in `lio_odom` and twist
covariance as `[delta v_base, delta omega_base]` in `base_link`. These are
analytical projections of the full IKFoM covariance with cross terms
preserved. PX4 covariance remains annotated with the source convention by
the bridge and is not treated as LIO covariance until the explicit alignment
has been applied.

## Initial-state prior contract

The PX4 simulation profile declares:

```yaml
frames: {odom: lio_odom, base: base_link, ...}
initial_prior:
  source: topic
  context: ground_startup
  source_frame: px4_odom
  source_frame_transform: startup_coincident
  topic: /px4/estimator_odometry
```

The prior is consumed once at startup, with its sensor timestamp preserved.
The ROS adapter accepts `px4_odom` only under the explicit
`ground_startup/startup_coincident` contract and records the provenance in
FAST-LIO diagnostics. Real and AIST profiles use `lio_odom` plus
`same_frame`. Timeout fallback remains explicit and is not accepted by the
canonical PX4/SITL gate.

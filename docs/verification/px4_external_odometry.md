# PX4 external odometry contract

`px4_external_odometry_bridge_node` consumes `/lio/odometry_propagated` only
when its message frames are exactly `lio_odom` and `base_link`. It publishes
the versioned `/fmu/in/vehicle_visual_odometry` topic using
`px4_msgs/msg/VehicleOdometry` with `POSE_FRAME_FRD` and
`VELOCITY_FRAME_BODY_FRD`.

## Frame and generation contract

The world and body basis changes are deliberately separate:

```text
C_world_frd_from_ros_local_zup = diag( 1, -1, -1 )
C_body_frd_from_body_flu       = diag( 1, -1, -1 )
R_frd_body = C_world * R_lio_odom_base_link * inverse(C_body)
```

Position is transformed with `C_world`; body linear and angular velocities are
transformed with `C_body`. Quaternions are normalized and `q/-q` is equivalent.
The LIO producer owns `lio_public_frame_generation`. The bridge maps it to
`reset_counter = generation % 256`; PX4 frame generation is not authoritative.

A geometric position/orientation jump closes the bridge latch once. Repeated
jumps, PX4 resets, supervisor gate changes, and stale data do not clear it or
increment the reset counter. Only a new valid LIO public generation or an
explicit operator reset can clear the latch.

## Covariance contract

The bridge requires position XYZ, orientation small-angle XYZ, and body linear
velocity XYZ covariance blocks. Each full 3x3 block is validated for finite
values, symmetry, PSD, and positive diagonal, transformed as `C P C^T`, and
only then reduced to the `VehicleOdometry` diagonal fields. Cross-covariance
blocks are ignored explicitly because `VehicleOdometry` transports only these
diagonal groups. No unavailable value is replaced by `1e-6` or any other
synthetic covariance floor.

## Timestamp and publication gates

The bridge reports measurement time in `timestamp_sample` and publication time
in `timestamp` through an explicit conversion result. P0.9-A proves only
`ROS_SIMULATION_TIME -> PX4_SIMULATION_TIME`; unresolved real-hardware time
conversion returns `TIME_DOMAIN_UNRESOLVED` and keeps publication closed.

The gate independently requires node readiness, PX4 input transport,
timestamp conversion, supervisor authorization, valid LIO public generation,
fresh corrected/propagated odometry, covariance, fresh supervisor status, the
message frame contract, and an unlatched geometric-jump state. `publisher_ready`
advertises only bridge node/transport readiness so supervisor authorization
does not form a circular dependency. `publication_ready` and
`publication_active` do not mean EKF2 fusion is active.

Diagnostics are published on `/px4/external_odometry_diagnostics`, including
the required gate fields, frame/generation/reset fields, timestamp provenance,
covariance status, publication counts, and jump counts. No EKF2 fusion,
aiding, innovation, or estimator-improvement claim is part of P0.9-A.

Focused conversion tests cover mixed orientation, q/-q, full non-diagonal
covariance transforms, unavailable/invalid covariance, frame rejection, and
small genuine positive variances without flooring.

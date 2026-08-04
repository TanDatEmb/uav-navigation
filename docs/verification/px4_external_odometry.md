# PX4 external odometry verification

`px4_external_odometry_bridge_node` subscribes to propagated `lio_odom` and
publishes `/fmu/in/vehicle_visual_odometry` with `POSE_FRAME_FRD`,
`VELOCITY_FRAME_BODY_FRD`, measurement-time `timestamp_sample`, node-time
`timestamp`, finite covariance floors, and a reset counter that changes only
on an observed output jump.

Publishing is suppressed while the supervisor gate is closed; no zero or stale
fallback sample is emitted. The bridge publishes readiness diagnostics on
`/px4/external_odometry_diagnostics`, which is an input to the supervisor gate.
Frame conversion tests cover ROS Z-up/FLU to PX4 FRD.

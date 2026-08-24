# Navigation layers

The current product path is a small number of explicit runtime boundaries:

```text
LiDAR + IMU
  -> FAST-LIO
     -> /lio/odometry_corrected
     -> /lio/odometry_propagated
     -> /lio/registered_points
  -> super_navigation_node
     -> ROG-MapROS + SUPER planner
     -> /navigation/super_command
  -> px4_navigation_external_mode
     -> /fmu/in/trajectory_setpoint
```

## FAST-LIO outputs

FAST-LIO validates message layout, timing, frames, initialization, deskew,
correction, and registration. Corrected odometry is emitted only after a
successful LiDAR correction in `Tracking`. Propagated odometry is the
high-rate state used by the planner and PX4 bridge. The registered cloud is
the current point-cloud boundary consumed by SUPER; it remains distinct from
FAST-LIO's internal nearest-neighbour registration structure.

All three LIO outputs use `lio_odom -> base_link`. The cloud consumed by SUPER
must carry `lio_odom` and finite XYZ fields. `/lio/diagnostics` is the
estimator health surface.

## Navigation runtime

`super_navigation_node` directly constructs `rog_map::ROGMapROS` and
`super_planner::SuperPlanner`. It pairs the newest cloud and propagated
odometry, rejects wrong-frame/stale inputs, updates ROG-Map, and runs SUPER's
`PlanFromRest`/`ReplanOnce` state machine. The configured loop is 10 Hz for
planning and 50 Hz for command sampling.

The runtime publishes `mars_quadrotor_msgs/PositionCommand` on
`/navigation/super_command` and `diagnostic_msgs/DiagnosticArray` on
`/navigation/diagnostics`. The diagnostic status name is
`super_navigation/super_planner`; its current counters include accepted and
dropped observations, planner cycles, published trajectories, stale inputs,
and processing exceptions.

The runtime keeps SUPER's committed main/backup trajectory state inside the
planner. A failed hot replan may retain a usable safety suffix or commit an
emergency brake from the freshest propagated position and velocity. If no
valid command or safety suffix exists, it fails closed. This is an in-process
planner policy, not a second ROS transport.

## PX4 boundary

PX4 External Mode consumes `/navigation/super_command` and
`/lio/odometry_propagated`. It validates freshness, frame, finite values, and
command anchoring before converting ROS ENU/FLU data to PX4 NED/FRD and
publishing the PX4 trajectory setpoint. Mission completion is a notification
on `/navigation/mission_complete`; the runtime does not issue LAND or
disarm.

## Deliberately absent architecture

The current source does not implement `MappingWorldNode`, `WorldSnapshot`,
`PlanningControllerNode`, `TrajectoryBundle`, or a separate
`navigation_mapping`/`navigation_planning` ROS product path. Those names must
not be used as current runtime documentation until the corresponding code and
message contracts exist.

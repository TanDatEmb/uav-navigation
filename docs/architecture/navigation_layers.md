# Navigation layers

The current product path is a small number of explicit runtime boundaries:

```text
LiDAR + IMU
  -> FAST-LIO
     -> /lio/odometry_corrected
     -> /lio/odometry_propagated
     -> /lio/registered_points
  -> navigation_mission + navigation_runtime_node
     -> mapping and planning backends
     -> /navigation/navigation_command
  -> px4_navigation_external_mode
     -> /fmu/in/trajectory_setpoint
```

## FAST-LIO outputs

FAST-LIO validates message layout, timing, frames, initialization, deskew,
correction, and registration. Corrected odometry is emitted only after a
successful LiDAR correction in `Tracking`. Propagated odometry is the
high-rate state used by the planner and PX4 bridge. The registered cloud is
the current point-cloud boundary consumed by the planner backend; it remains distinct from
FAST-LIO's internal nearest-neighbour registration structure.

All three LIO outputs use `lio_odom -> base_link`. The cloud consumed by the planner backend
must carry `lio_odom` and finite XYZ fields. Typed `/lio/health` is the
estimator control-plane health surface; `/lio/diagnostics` is observability
only.

## Navigation runtime

`navigation_mission` validates the mission schema before runtime or PX4
mission behavior is constructed. It owns waypoint identity/frame checks,
mission planning limits, and the typed world-model UNKNOWN policy. This is a
contract boundary, not a second runtime execution node; Python mission
readers remain tooling-only.

`navigation_runtime_node` currently constructs the map/planner implementation
through the transitional backend boundary. `navigation_planning` now defines the
ROS/vendor-free C++20 request/outcome/candidate and kinematic-state contracts;
runtime sends the typed propagated state directly to the backend adapter.
It pairs the newest cloud and propagated odometry, rejects wrong-frame/stale
inputs, updates the world model, and runs the backend's
`PlanFromRest`/`ReplanOnce` state machine. The configured loop is 10 Hz for
planning and 50 Hz for command sampling.

The runtime publishes `navigation_contracts/NavigationCommand` on
`/navigation/navigation_command` and `diagnostic_msgs/DiagnosticArray` on
`/navigation/diagnostics`. `NavigationCommand` carries the PVA sample plus
localization epoch, goal identity, world snapshot identity and committed
bundle provenance. The diagnostic status name is
`navigation_runtime/planner`; its current counters include accepted and
dropped observations, planner cycles, published trajectories, stale inputs,
and processing exceptions.

The product-owned `navigation_execution::CommittedBundleStore` is the sole
exposed command candidate store at the runtime/PX4 boundary, and
`CommandSampler` reads only that immutable store. The planner backend retains
private trajectory history temporarily for replan continuity; that history is
not an independent command path and remains an extraction debt until the
execution coordinator accepts the previous candidate explicitly.

## PX4 boundary

PX4 External Mode consumes `/navigation/navigation_command` and
`/lio/odometry_propagated`. It validates freshness, frame, finite values, and
command anchoring before converting ROS ENU/FLU data to PX4 NED/FRD and
publishing the PX4 trajectory setpoint. Mission completion is a notification
on `/navigation/mission_complete`; the runtime does not issue LAND or
disarm.

## Deliberately absent architecture

The current source does not implement `MappingWorldNode`,
`PlanningControllerNode`, or a separate mapping/planning ROS transport.
`navigation_mapping::MappingActor` owns the mutable map and immutable snapshot
construction; runtime owns worker scheduling, diagnostics publication and the
planner invocation until the next execution-authority extraction step.

# ADR-010: Backend ownership remains inside the navigation runtime

**Status:** accepted and implemented.

The current product path keeps the pinned mapping and planning backends in
one `navigation_runtime_node`. The node consumes FAST-LIO's
`/lio/registered_points` and `/lio/odometry_propagated`, updates the backend
world model, runs the planner, and publishes `/navigation/navigation_command`.

This boundary is intentionally in-process. It avoids an unimplemented ROS
snapshot protocol, duplicate occupancy representation, and a second planner
map. `RegistrationMap` remains FAST-LIO's internal nearest-neighbour map; the
planning world model is local to navigation and is not exposed as a generic
per-voxel service.

The runtime also owns the planner's committed main/backup trajectory state,
safety suffix checks, emergency braking, and `/navigation/diagnostics`.
PX4 External Mode is the next boundary and consumes the PVA command stream.

The following target architecture is not yet a separate ROS transport and
must not be documented as fully deployed: `MappingWorldNode`,
`PlanningControllerNode`, and `TrajectoryBundle`. The `navigation_mapping`
package now owns reusable lifecycle/snapshot primitives; backend map
construction remains in the runtime during migration.

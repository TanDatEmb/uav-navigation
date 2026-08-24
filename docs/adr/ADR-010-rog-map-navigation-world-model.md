# ADR-010: ROG-Map and SUPER remain inside the navigation runtime

**Status:** accepted and implemented.

The current product path keeps the ROG-Map and SUPER planner in one
`super_navigation_node`. The node consumes FAST-LIO's
`/lio/registered_points` and `/lio/odometry_propagated`, updates
`rog_map::ROGMapROS`, runs `super_planner::SuperPlanner`, and publishes
`/navigation/super_command`.

This boundary is intentionally in-process. It avoids an unimplemented ROS
snapshot protocol, duplicate occupancy representation, and a second planner
map. `RegistrationMap` remains FAST-LIO's internal nearest-neighbour map; the
ROG-Map instance is the navigation planner's local map and is not exposed as
a generic per-voxel service.

The runtime also owns the planner's committed main/backup trajectory state,
safety suffix checks, emergency braking, and `/navigation/diagnostics`.
PX4 External Mode is the next boundary and consumes the PVA command stream.

The following names are not current contracts and must not be documented as
implemented: `navigation_mapping`, `navigation_planning`,
`LidarMappingObservation`, `MappingWorldNode`, `WorldSnapshot`,
`PlanningControllerNode`, and `TrajectoryBundle`.

# navigation_contracts

This is a contract-only ROS 2 package. It contains the generic goal, mode
status, and odometry sampling interfaces shared across package boundaries.

Current navigation messages include:

- `NavigationGoal`: mission identity, waypoint index/request ID, target,
  acceptance radius, behavior, and optional next target;
- `NavigationModeStatus`: mode state/reason and explicit waypoint acceptance
  evidence;
- `NavigationCommand`: product-owned PVA sample with localization, goal,
  world-snapshot and committed-bundle provenance;
- `SampleOdometryAtTime`: service used by the PX4 odometry bridge.

The package does not own LiDAR drivers, FAST-LIO, ROG-Map, planner backend, or a mapping
observation stream. In particular, `LidarMappingObservation` is not a current
message in this repository. Runtime topic and ownership details are in
`docs/architecture/navigation_layers.md`.

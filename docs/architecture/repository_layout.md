# Repository layout

The repository is organized around the runtime boundaries that exist today:

```text
src/estimation/
  fast_lio_core/        ROS-independent estimator
  fast_lio_ros/         ROS adapters and published LIO outputs
  fast_lio_tools/       offline estimator use
  ikfom_vendor/         pinned IKFoM provenance
  ikd_tree_vendor/      pinned registration-map provenance
src/runtime/
  navigation_runtime/   super_navigation_node, ROG-Map and SUPER composition
src/mapping/
  rog_map_vendor/       pinned ROG-Map source, tests and provenance
src/planning/
  super_planner_vendor/ pinned SUPER source and provenance
src/external/
  px4_msgs/             pinned PX4 messages
  px4_ros2_interface_lib/ PX4 v1.17 ROS 2 Control Interface
  livox_ros_driver2/    Livox driver and message package
src/px4/                PX4 ingress and External Mode adapters
src/navigation_interfaces/ shared goal/status/service messages
src/navigation_bringup/ launch files and RViz profile
src/uav_description/    sensor-frame source of truth
src/uav_simulation/     Gazebo assets
config/runtime/         workflow and node configuration
tools/runtime/          runner, monitor, report and scenario harness
tools/benchmarks/       optional offline benchmark utilities
docs/                   architecture, ADRs, validation and benchmarks
```

`super_navigation_node` is the current composition boundary. It constructs
`rog_map::ROGMapROS` and `super_planner::SuperPlanner` in one process and
consumes the FAST-LIO registered cloud directly. The repository does not
currently contain product-owned `navigation_mapping` or
`navigation_planning` packages, nor a snapshot/bundle ROS transport.

Ownership rules:

- FAST-LIO owns estimation, corrected/propagated odometry, and its internal
  registration map;
- `navigation_runtime` owns the planning map update, SUPER solve/replan state,
  command sampling, safety fallback, and planner diagnostics;
- PX4 External Mode owns the command-to-PX4 control boundary and mission
  completion notification;
- `tools/runtime/report.py` is the only public report entrypoint and writes
  only `report.json` and `REPORT.html`;
- `src/navigation_bringup/rviz/fast_lio.rviz` is the only project navigation
  RViz profile.

Generated build/install/log trees and `.artifacts/runtime/` are local state,
not documentation or source ownership boundaries.

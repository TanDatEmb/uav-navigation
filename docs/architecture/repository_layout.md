# Repository layout

The active source tree is organized by product ownership. Third-party source
is kept behind explicit backend package boundaries; it is not a product API.

```text
src/common/
  navigation_common/   shared ROS time and ENU/NED, FLU/FRD conversions
src/contracts/
  navigation_contracts/ product ROS messages, services and boundary checks
src/estimation/
  fast_lio_core/        ROS-independent estimator
  fast_lio_ros/         ROS adapters and published LIO outputs
  fast_lio_tools/       offline estimator use
  ikfom_vendor/         pinned IKFoM provenance
  ikd_tree_vendor/      pinned registration-map provenance
src/runtime/
  navigation_runtime/   ROS composition and lifecycle wiring
src/mapping/
  navigation_mapping/   product-owned mapping worker and lifecycle accounting
  rog_map_vendor/       pinned ROG-Map source, tests and provenance
src/planning/
  navigation_planning/  pure C++20 planning contracts and candidate types
  navigation_planning_backend/ isolated planner implementation boundary
src/execution/
  navigation_execution/ immutable commit, freshness and sampling gates
src/external/
  px4_msgs/             pinned PX4 messages
  px4_ros2_interface_lib/ PX4 v1.17 ROS 2 Control Interface
  livox_ros_driver2/    Livox driver and message package
src/px4/                PX4 ingress and External Mode adapters
src/navigation_bringup/ launch files and RViz profile
src/uav_description/    sensor-frame source of truth
src/uav_simulation/     Gazebo assets
config/runtime/         workflow and node configuration
tools/runtime/          runner, monitor, report and scenario harness
tools/benchmarks/       optional offline benchmark utilities
docs/                   architecture, ADRs, validation and benchmarks
```

`navigation_runtime_node` is the current composition boundary. It consumes
typed estimator observations, uses the product-owned mapping lifecycle
primitive, still owns the mutable backend/world-snapshot composition, invokes
the pinned planner backend, and publishes the single product-owned
`navigation_contracts/NavigationCommand` stream. There is no snapshot/bundle
ROS transport. Planning contracts and execution commit/sampling primitives now
exist as product-owned libraries; runtime wiring remains an explicit migration
step and is not claimed complete here.

Ownership rules:

- FAST-LIO owns estimation, corrected/propagated odometry, and its internal
  registration map;
- `navigation_mapping` owns the bounded observation worker and exact lifecycle
  accounting;
- `navigation_runtime` still owns planning map update, backend solve/replan state,
  and diagnostics while extraction is in progress;
- `navigation_execution` owns the new immutable candidate commit and sampling
  boundary; direct runtime sampling remains a tracked cutover until integration;
- PX4 External Mode owns the command-to-PX4 control boundary and mission
  completion notification;
- `tools/runtime/report.py` is the only public report entrypoint and writes
  only `report.json` and `REPORT.html`;
- `src/navigation_bringup/rviz/fast_lio.rviz` is the only project navigation
  RViz profile.

Generated build/install/log trees and `.artifacts/runtime/` are local state,
not documentation or source ownership boundaries.

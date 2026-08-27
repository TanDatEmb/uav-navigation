# Repository layout

The active source tree is organized by product ownership. Third-party source
is kept behind explicit backend package boundaries; it is not a product API.

```text
src/common/
  navigation_common/   shared ROS time and ENU/NED, FLU/FRD conversions
src/contracts/
  navigation_contracts/ product ROS messages, services and boundary checks
  navigation_mission/   validated mission schema and typed mission policy
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

`navigation_mission` is the single C++ owner of mission YAML validation,
waypoint identity/frame checks, planning limits, and the typed UNKNOWN-space
policy. PX4 External Mode and runtime consume that contract; Python under
`tools/runtime/` may read mission files for orchestration and reporting but is
not a flight-authority parser.

`navigation_runtime_node` is the current composition boundary. It consumes
typed estimator observations, delegates mutable map integration and immutable
snapshot construction to `navigation_mapping`, invokes the pinned planner
backend, and publishes the single product-owned
`navigation_contracts/NavigationCommand` stream. There is no snapshot/bundle
ROS transport. Planning contracts and execution commit/sampling primitives now
exist as product-owned libraries and the runtime uses them as its command
authority. The mapping package may retain a private backend representation,
but its installed public headers expose only product-owned snapshot, metrics,
observation and world-model types.

Ownership rules:

- FAST-LIO owns estimation, corrected/propagated odometry, and its internal
  registration map;
- `navigation_mapping` owns the bounded observation worker, exact lifecycle
  accounting, mutable map integration, and immutable planning snapshot
  construction;
- `navigation_runtime` owns composition, snapshot publication into the planning
  lifecycle, backend solve/replan state, and runtime diagnostics; it does not
  own the mutable mapping backend;
- `navigation_execution` owns the immutable candidate commit and sampling
  boundary;
- PX4 External Mode owns the command-to-PX4 control boundary and mission
  completion notification;
- `tools/runtime/report.py` is the only public report entrypoint and writes
  only `report.json` and `REPORT.html`;
- `src/navigation_bringup/rviz/fast_lio.rviz` is the only project navigation
  RViz profile.

Generated build/install/log trees and `.artifacts/runtime/` are local state,
not documentation or source ownership boundaries.

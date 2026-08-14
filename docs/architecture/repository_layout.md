# Repository layout

```text
src/estimation/
  ikfom_vendor/       pinned dependency provenance
  ikd_tree_vendor/    pinned registration-map dependency
  fast_lio_core/      ROS-independent estimator
  fast_lio_ros/       ROS boundary and output publishing
  fast_lio_tools/     offline use of the same pipeline
src/mapping/
  rog_map_vendor/      pinned ROG-Map navigation world-model dependency
  navigation_mapping/  product-owned synchronous world-model core
src/runtime/
  navigation_runtime/  single ROS composition boundary for mapping and planning
src/planning/
  navigation_planning/  product-owned WorldModel consumer and A* baseline
src/external/
  px4_msgs/            pinned PX4 message package
  px4_ros2_interface_lib/ pinned PX4 v1.17 ROS 2 Control Interface library
  livox_ros_driver2/   pinned Livox driver and message package
src/navigation_interfaces/ generic navigation service/message contracts,
  including the LidarMappingObservation atomic mapping observation
src/px4/               PX4 ingress, external-odometry bridge, and External Mode adapter
src/navigation_bringup/ launch and visualization composition
src/uav_description/     sensor-frame source of truth
src/uav_simulation/      Gazebo Harmonic simulation assets
config/runtime/          common, dataset, simulation, external-mode, offboard, and mapping.yaml
                         navigation_runtime contracts
tools/runtime/           runner, monitor, report, process ownership, scenario
docs/                    architecture, runtime validation, and ADRs
```

FAST-LIO and `navigation_runtime` run as independent ROS 2 processes. The
runtime owns one `MappingPipeline`/`WorldModel` instance and calls the
`navigation_planning` library directly, not through a separate ROS voxel-query
process. See `docs/architecture/navigation_layers.md` for the
RegistrationMap vs navigation-world-model distinction and dependency
direction. Safety and mission packages remain intentionally absent. Configuration
names, frame names, and static transforms have one documented source of
truth; do not introduce duplicate workflow, observer, or report helpers.

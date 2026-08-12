# Repository layout

```text
src/navigation_estimator/
  ikfom_vendor/       pinned dependency provenance
  ikd_tree_vendor/    pinned registration-map dependency
  fast_lio_core/      ROS-independent estimator
  fast_lio_ros/       ROS boundary and output publishing
  fast_lio_tools/     offline use of the same pipeline
src/navigation_mapping/
  rog_map_vendor/      pinned ROG-Map navigation world-model dependency
  navigation_mapping/  product-owned mapper node (separate ROS 2 process)
src/external/
  px4_msgs/            pinned PX4 message package
  livox_ros_driver2/   pinned Livox driver and message package
src/navigation_interfaces/ generic navigation service/message contracts,
  including the LidarMappingObservation atomic mapping observation
src/px4_interface/     PX4 ingress and external-odometry bridge
src/navigation_bringup/ launch and visualization composition
src/uav_description/     sensor-frame source of truth
src/uav_simulation/      Gazebo Harmonic simulation assets
config/runtime/          common, dataset, simulation, offboard, and (P1)
                         mapping.yaml navigation_mapping_node contracts
tools/runtime/           runner, monitor, report, process ownership, scenario
docs/                    architecture, runtime validation, and ADRs
```

FAST-LIO and `navigation_mapping` run as independent ROS 2 processes; see
`docs/architecture/navigation_layers.md` for the RegistrationMap vs
navigation-world-model distinction and the P1 dependency direction. Planner,
safety, and mission packages remain intentionally absent. Configuration
names, frame names, and static transforms have one documented source of
truth; do not introduce duplicate workflow, observer, or report helpers.

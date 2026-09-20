# Package and profile scope

Classification is based on package manifests, CMake targets, source paths and launch/config. It does not prove each installed node runs in a particular deployment.

| Package | Path | Classification |
|---|---|---|
| navigation_common | src/common/navigation_common/package.xml | shared-production-dependency |
| navigation_contracts | src/contracts/navigation_contracts/package.xml | production-path-or-contract |
| navigation_mission | src/contracts/navigation_mission/package.xml | production-path-or-contract |
| fast_lio_core | src/estimation/fast_lio_core/package.xml | production-path-or-contract |
| fast_lio_ros | src/estimation/fast_lio_ros/package.xml | production-path-or-contract |
| fast_lio_tools | src/estimation/fast_lio_tools/package.xml | offline-support |
| ikd_tree_vendor | src/estimation/ikd_tree_vendor/package.xml | vendor-dependency |
| ikfom_vendor | src/estimation/ikfom_vendor/package.xml | vendor-dependency |
| navigation_execution | src/execution/navigation_execution/package.xml | production-path-or-contract |
| livox_ros_driver2 | src/external/livox_ros_driver2/package.xml | external-source-boundary |
| px4_msgs | src/external/px4_msgs/package.xml | PX4-vendored-message-boundary |
| px4_ros2_cpp | src/external/px4_ros2_interface_lib/px4_ros2_cpp/package.xml | PX4-ROS2-vendor-submodule |
| navigation_mapping | src/mapping/navigation_mapping/package.xml | production-path-or-contract |
| navigation_world_model | src/mapping/navigation_world_model/package.xml | production-path-or-contract |
| rog_map_vendor | src/mapping/rog_map_vendor/package.xml | vendor-dependency |
| navigation_bringup | src/navigation_bringup/package.xml | production-path-or-contract |
| navigation_planning | src/planning/navigation_planning/package.xml | production-path-or-contract |
| navigation_planning_backend | src/planning/navigation_planning_backend/package.xml | production-path-or-contract |
| px4_navigation_external_mode | src/px4/px4_navigation_external_mode/package.xml | production-path-or-contract |
| px4_odometry_bridge | src/px4/px4_odometry_bridge/package.xml | optional-bridge-default-disabled |
| navigation_runtime | src/runtime/navigation_runtime/package.xml | production-path-or-contract |
| uav_description | src/uav_description/package.xml | simulation-support |
| uav_simulation | src/uav_simulation/package.xml | simulation-only |

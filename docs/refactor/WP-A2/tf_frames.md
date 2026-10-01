# TF frame ICD (AS-IS)

Baseline: `main@7e0b850`. This is a source/config inventory, not a runtime
`tf2_echo` measurement.

## Canonical tree

```text
lio_odom
└── base_link                         dynamic
    └── livox_frame                   static, robot_state_publisher
        └── livox_imu_frame           static, robot_state_publisher
```

The canonical names are returned by
`src/estimation/fast_lio_core/include/fast_lio_core/geometry/frame_ids.hpp:7-10`:
`lio_odom`, `base_link`, `livox_imu_frame`, and `livox_frame`.

## Owners and lookups

| Edge / frame | Owner or consumer | Evidence |
|---|---|---|
| `base_link -> livox_frame` | `robot_state_publisher` publishes `/tf_static` from the xacro fixed joint | `src/uav_description/urdf/uav_sensor_frames.urdf.xacro:25-29`; `src/uav_description/launch/publish_sensor_frames.launch.py:30-57` |
| `livox_frame -> livox_imu_frame` | `robot_state_publisher` publishes `/tf_static` from the xacro fixed joint | `src/uav_description/urdf/uav_sensor_frames.urdf.xacro:30-34` |
| `lio_odom -> base_link` | FAST-LIO dynamic broadcaster; state frame names are taken from the converted estimate | `src/estimation/fast_lio_ros/src/ros_transform_publisher.cpp:22-35,40-70`; invoked at `src/estimation/fast_lio_ros/src/fast_lio_node.cpp:176-182` |
| `base_link <- livox_imu_frame` lookup | FAST-LIO static resolver; target=`parameters_.base_frame`, source=`parameters_.imu_frame`, time=`TimePointZero` | `src/estimation/fast_lio_ros/src/fast_lio_node.cpp:134-155`; `src/estimation/fast_lio_ros/src/ros_static_transform_resolver.cpp:13-30` |
| Sensor-frame launch | Runner starts canonical static tree and supplies mount/calibration explicitly | `tools/runtime/runner.py:3776-3783`; `src/navigation_bringup/launch/fast_lio.launch.py:23-54` |

`publish_sensor_frames:=true` is a runner/launch condition, so the static
edges are not created by `fast_lio_node` itself. The estimator fails during
startup when the required static geometry is unavailable
(`fast_lio_node.cpp:135-145`).

## Hard-coded strings versus source of truth

| Literal / parameter | Location | Classification |
|---|---|---|
| `lio_odom`, `base_link`, `livox_imu_frame`, `livox_frame` | `frame_ids.hpp:7-10` | canonical helper values |
| `frames.*`, `navigation_runtime.planning_frame`, `navigation.body_frame` | `src/estimation/fast_lio_ros/src/parameter_loader.cpp:151-154`; `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:682-683`; `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:179-182` | configurable inputs; PX4 mode additionally pins planning/body values at `navigation_mode_node.cpp:255-258` |
| `livox_frame` expected visibility frame | `src/uav_simulation/src/visibility_bridge.cpp:23,32`; runner override `tools/runtime/runner.py:3697` | duplicated literal; must agree with canonical lidar frame |
| `px4_odom` | `src/px4/px4_odometry_bridge/src/px4_odometry_bridge_node.cpp:39-43,522-529` | separate SITL bridge output frame; no TF broadcaster is created for it |
| `base_link` child in PX4 bridge output | `px4_odometry_bridge_node.cpp:528-529` | serialized `nav_msgs/Odometry` child frame, not a TF publication |
| `lio_odom` / `base_link` in runtime message construction | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:9354`; `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:2661` | parameter-backed message frame contract |

The PX4 bridge's `/px4/estimator_odometry` is an odometry topic with
`header.frame_id=px4_odom`; it does not extend the TF tree. Ground-truth and
scenario frame strings are evidence/test inputs and are not product TF owners.

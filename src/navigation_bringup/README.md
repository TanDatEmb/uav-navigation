# navigation_bringup

This package owns the ROS 2 launch composition for the single product path:
FAST-LIO inputs, native SUPER navigation, and PX4 External Mode.

`fast_lio.launch.py` composes:

- the canonical `uav_description` sensor-frame publisher;
- the `fast_lio_ros` estimator node;
- the optional PX4 external-odometry bridge;

`navigation_runtime.launch.py` starts only `super_navigation_node`. The planner
consumes the newest `PointCloud2` and `Odometry` frames, owns the ROG-Map update,
and publishes one native `mars_quadrotor_msgs/PolynomialTrajectory` on
`/navigation/super_trajectory`.

`px4_external_mode.launch.py` launches the PX4 v1.17 ROS 2 Control Interface
node. The node registers `NavigationMode` and its `NavigationModeExecutor`,
then consumes the native polynomial contract on `/navigation/super_trajectory`.
Its config is `config/runtime/external_mode.yaml`. It is the PX4 control
boundary; do not replace it with direct `OffboardControlMode` publishers.

Interactive workflows use the project-owned RViz profile in `rviz/`. The
profile shows the LIO registered scan, bounded local map, TF tree, corrected
LIO odometry, and the last successful navigation plan on
`/navigation/visualization/planned_path`. Vendor Livox RViz files remain with the external
driver and are not used as the navigation stack's runtime profile.

Send a stamped goal for one planning attempt, or repeat it for timing samples:

```bash
python3 tools/runtime/send_goal.py 5.0 0.0 1.0
python3 tools/runtime/send_goal.py 5.0 0.0 1.0 --repeat 20 --period 1.0
```

The native polynomial trajectory is published on `/navigation/super_trajectory`.
The RViz path is visualization-only.

Runtime behavior and parameter ownership remain in `config/runtime/` and
`tools/runtime/`. The launch file is intentionally a single declarative
`generate_launch_description()` entrypoint.

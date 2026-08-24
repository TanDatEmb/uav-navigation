# navigation_bringup

This package owns the launch files and the project RViz profile for the
current FAST-LIO, SUPER, and PX4 External Mode path.

## Launch composition

- `fast_lio.launch.py` starts the sensor description, FAST-LIO, and the
  optional PX4 external-odometry bridge;
- `navigation_runtime.launch.py` starts one
  `navigation_runtime/super_navigation_node`;
- `px4_external_mode.launch.py` starts
  `px4_navigation_external_mode_node`;
- `avoidance_mission.launch.py` starts the last two components and passes the
  same mission file to both.

The navigation node subscribes to `/lio/registered_points`,
`/lio/odometry_propagated`, `/navigation/goal`, and
`/navigation/mode_status`. It publishes PVA commands on
`/navigation/super_command` and diagnostics on `/navigation/diagnostics`.
ROG-Map and SUPER are constructed in this process; there is no separate
mapping ROS node in the current product path.

## Mission entrypoint

```bash
ros2 launch navigation_bringup avoidance_mission.launch.py \
  config_file:=$PWD/config/runtime/mapping.yaml \
  mission_file:=$PWD/config/runtime/missions/long_three_pillars.yaml \
  use_sim_time:=true
```

`mission_file` supplies the mission controller's waypoint behavior and the
dynamic limits used before SUPER optimizer construction. PX4 External Mode
remains the control boundary; do not replace it with direct
`OffboardControlMode` publishers.

## RViz profile

`rviz/fast_lio.rviz` is intentionally small and matches live topics:

- fixed frame `lio_odom`;
- TF and a grid;
- enabled `/lio/registered_points`;
- enabled `/lio/odometry_propagated`;
- disabled `/lio/odometry_corrected` for optional estimator comparison.

There are no current displays for `/navigation_mapping/visualization/*`,
`/navigation/visualization/*`, or a planned-path topic. SUPER's map and
trajectory visualization serializers are disabled in the runtime config.

## Goal and observability

```bash
python3 tools/runtime/send_goal.py 5.0 0.0 1.0
python3 tools/runtime/send_goal.py 5.0 0.0 1.0 --repeat 20 --period 1.0
ros2 topic echo /navigation/diagnostics
ros2 topic echo /navigation/super_command
```

Runtime validation and report generation are owned by `tools/runtime/`; this
package does not contain a second report or monitoring tool.

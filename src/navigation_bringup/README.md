# navigation_bringup

This package owns the thin ROS 2 launch composition for the navigation stack.
It does not contain estimator or PX4 business logic.

`fast_lio.launch.py` composes:

- the canonical `uav_description` sensor-frame publisher;
- the `fast_lio_ros` estimator node;
- the optional PX4 external-odometry bridge;

`navigation_runtime.launch.py` launches the `navigation_runtime` composition
boundary, which owns the navigation world-model
node as its own separate process. It is deliberately not included from
`fast_lio.launch.py`: FAST-LIO and the navigation world model run as
independent ROS 2 processes so a mapper crash or overload can never affect the
estimator (see docs/architecture/navigation_layers.md). Compose
both launch files from a parent launch description when both are needed.

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

The product trajectory is published separately on `/navigation/trajectory`;
the RViz path is visualization-only.

Runtime behavior and parameter ownership remain in `config/runtime/` and
`tools/runtime/`. The launch file is intentionally a single declarative
`generate_launch_description()` entrypoint.

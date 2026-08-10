# navigation_bringup

This package owns the thin ROS 2 launch composition for the navigation stack.
It does not contain estimator or PX4 business logic.

`fast_lio.launch.py` composes:

- the canonical `uav_description` sensor-frame publisher;
- the `fast_lio_ros` estimator node;
- the independent `rog_map_ros` navigation occupancy-map node;
- the optional PX4 external-odometry bridge;

Interactive workflows use the project-owned RViz profile in `rviz/`. The
profile shows the LIO registered scan, bounded local map, TF tree, and
corrected LIO odometry. Vendor Livox RViz files remain with the external
driver and are not used as the navigation stack's runtime profile.

Runtime behavior and parameter ownership remain in `config/runtime/` and
`tools/runtime/`. The launch file is intentionally a single declarative
`generate_launch_description()` entrypoint.

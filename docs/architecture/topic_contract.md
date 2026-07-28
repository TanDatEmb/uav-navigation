# ROS topic contract

Inputs are remappable and configured, never embedded in core code. Defaults are
`/lidar/points` and `/lidar/imu` (or `/imu/data` for an external IMU). The LiDAR
adapter selection is explicit: Livox custom message or `sensor_msgs/PointCloud2`.

| Topic | Type | Frame/policy |
| --- | --- | --- |
| `/lio/odometry` | `nav_msgs/msg/Odometry` | `odom`, child `base_link`; corrected tracking updates only |
| `/lio/registered_points` | `sensor_msgs/msg/PointCloud2` | `odom`; current corrected deskewed scan |
| `/lio/local_map` | `sensor_msgs/msg/PointCloud2` | `odom`; low-rate debug snapshot |
| `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | sensor, sync, init, deskew, registration, map, state |
| `/tf` | TF | dynamic `odom -> base_link` only |

Static `base_link` sensor transforms are published by URDF/robot_state_publisher,
not by the estimator node. The node must reject invalid frame/configuration
combinations with diagnostic context.

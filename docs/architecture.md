# Architecture — uav-navigation

## Design Goals

1. **Modular**: each package has one responsibility; dependencies are explicit.
2. **Testable**: pure functions live in `px4_common`; nodes are thin wrappers.
3. **PX4-native**: conventions align with PX4 frames and uORB message semantics.
4. **Simulation-first, reality-ready**: SITL is the default test surface; real-flight parameters are YAML-gated.
5. **Observable**: health, timing, and planning metrics are exposed as ROS 2 topics.

## Package Responsibilities

### `px4_msgs` (submodule)

- ROS 2 equivalents of PX4 uORB messages.
- Tracked from upstream `PX4/px4_msgs`.

### `px4_common`

- geometry, quaternion, NED↔ENU transforms
- interpolation, clamping, grid↔world helpers
- parameter loading helpers
- unit tests for all pure functions

### `px4_mapping`

- LiDAR/IMU odometry (IESKF / FAST-LIO2 successor)
- local occupancy / voxel map
- sensor simulation handlers
- NED bridge to PX4 EKF2 if required

### `px4_navigation`

- virtual scan / obstacle representation
- local planner: reactive (VFH+) + deliberative (A* / B-spline)
- state machine with safety modes
- trajectory follower

### `px4_ros_com`

- ROS 2 ↔ PX4 uORB DDS bridge
- frame transform publishing
- offboard mode manager / health watcher

### `px4_visualization`

- RViz configuration
- plotting / telemetry export helpers
- bag recording workflow

## Data Flow

```
sensors (LiDAR, IMU, camera, GPS if available)
        ↓
px4_mapping
        ↓
map_ned + odom + body pose
        ↓
px4_navigation
        ↓
trajectory_setpoint / goto_setpoint
        ↓
px4_ros_com
        ↓
PX4 uORB
```

## Configuration Hierarchy

1. **Compile-time** constants in headers (only for truly fixed values).
2. **Package-level YAML** in `src/<pkg>/config/`.
3. **Global YAML** in `config/` for cross-package orchestration.
4. **Runtime overrides** via launch arguments or `ros2 param set`.

## Status

This document is a living draft. Frame tree and message definitions will be added when the first packages land.

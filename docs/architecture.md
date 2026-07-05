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

- **Self-developed** PX4 ↔ ROS 2 bridge and transform helpers.
- Do NOT submodule upstream `PX4/px4_ros_com`; it is ROS 1 legacy and poorly maintained for ROS 2 Jazzy.
- Frame transform publishing (NED↔ENU, body↔baselink).
- Offboard mode manager / health watcher.
- ROS 2 message conversion utilities on top of `px4_common` math.

### `px4_visualization`

- RViz configuration
- plotting / telemetry export helpers
- bag recording workflow

## Time Synchronization

- **Adopted strategy**: PX4 Scenario A — ROS 2 nodes use the ROS 2 clock
  (`rclcpp::Clock`, `this->now()`, `header.stamp`). The MicroXRCE-DDS agent
  is responsible for translating between PX4 wall-clock (`timestamp_sample`)
  and ROS 2 time when crossing the PX4↔ROS 2 boundary.
- **Consequence**: We do **not** maintain a manual `px4_to_ros_offset`
  inside this workspace. Any node that publishes to PX4 (`vehicle_visual_odometry`,
  `trajectory_setpoint`, etc.) must set `timestamp` and `timestamp_sample`
  from the same ROS 2 clock source as the rest of the pipeline.
- **Exception**: If SITL real-time factor (RTF) deviates significantly from
  1.0 or if agent sync is proven unreliable, revisit the legacy manual-offset
  approach captured in `reviews/review_timestamp_sync.md`.

## Frame Tree

- `map_ned` — local North-East-Down world frame (PX4 default).
- `camera_init` — FAST-LIO2 initialization frame (legacy).
- `odom` / `base_link` — ROS REP-103 body frame, Forward-Left-Up.
- `aircraft` — PX4 body frame, Forward-Right-Down.
- Transforms are implemented in `px4_common::math` and exposed in
  `px4_ros_com::frame_transforms`.

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

## Repository Layout

Standard ROS 2 workspace:

```
uav-navigation/
├── src/
│   ├── px4_msgs/            # upstream PX4 uORB message definitions (submodule)
│   ├── px4_common/          # shared math, geometry, transforms
│   ├── px4_mapping/         # odometry, local voxel map
│   ├── px4_navigation/      # planner, controller, state machine
│   ├── px4_ros_com/         # ROS 2 ↔ PX4 bridge, TF publishers
│   └── px4_visualization/   # RViz, plotting, bag helpers
├── config/                  # global runtime parameters
├── launch/                  # top-level orchestration
├── docs/                    # conventions and architecture
├── tests/                   # integration tests
└── tools/                   # build, format, simulation scripts
```

## Status

- [x] `px4_common`: shared types, math, transforms, parameter helpers, tests.
- [x] `px4_mapping`: `VoxelHashMap`, `VoxelPool`, tests.
- [x] `px4_navigation`: `LocalPlanGrid`, `VirtualScan`, `AStarPlanner`, tests.
- [x] `px4_ros_com`: self-developed bridge/transforms package scaffold (`frame_transforms.hpp` + tests).
- [ ] `px4_ros_com`: aircraft↔baselink and ECEF helpers.
- [ ] `px4_mapping`: FAST-LIO2 / odometry node, NED bridge.
- [ ] `px4_navigation`: B-spline optimizer, controller, state machine.
- [ ] `px4_visualization`: not yet populated.
- [ ] `px4_ros_com`: executable bridge/offboard/health-watcher nodes.

This document is a living draft. Detailed frame tree and message definitions are
now in place; remaining package-specific sections will be added as nodes land.


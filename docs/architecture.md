# Architecture — uav-navigation

## Design Goals

1. **Modular:** each package owns one explicit responsibility.
2. **Testable:** algorithms are isolated from ROS 2 wrappers where practical.
3. **PX4-native at the boundary:** PX4 message semantics, frames, and boot-time
   timestamps are preserved when data crosses into or out of PX4.
4. **Simulation-first, hardware-ready:** Gazebo/PX4 SITL is the default
   integration surface; sensor and flight parameters remain configurable.
5. **Observable:** mapping, perception, timing, and health outputs are available
   through ROS 2 topics and logs.

## Package Responsibilities

### `fast_lio`

- consumes MID-360 point clouds and IMU data
- estimates a gravity-aligned 15-DOF LiDAR-inertial state
- maintains the incremental ikd-Tree scan map
- publishes odometry, the registered cloud, path, and `lio_world -> mid360_imu`
  TF in the ROS clock domain
- has no dependency on PX4 messages or `px4_ros2_utils`

### `px4_mapping`

- converts localization data at ROS 2/PX4 boundaries
- publishes external odometry to PX4 through `lio_px4_alignment` or the legacy
  `localization_bridge` path
- builds the accumulated sparse global voxel map
- contains compatibility nodes for the earlier `camera_init` localization path

### `px4_navigation`

- converts point-cloud obstacles into PX4 `ObstacleDistance`
- publishes a ROS `LaserScan` representation for debugging
- provides planning libraries (`LocalPlanGrid`, `AStarPlanner`, and
  `VirtualScan`)
- does not yet provide the deferred full navigation controller or planner-local
  map executable

### `px4_nav_common`

- owns project-specific mapping/navigation contracts, voxel types, frame IDs,
  occupancy constants, and grid helpers
- does not duplicate generic PX4/ROS 2 frame, time, QoS, or topic utilities

### `px4_ros2_utils` (submodule)

- provides generic frame conversions, timestamp synchronization, typed PX4
  topics, QoS profiles, parameter helpers, and message adapters
- is used directly by first-party packages at PX4 boundaries
- does not own application nodes or mission behavior

### External submodules

- `px4_msgs`: ROS 2 equivalents of PX4 uORB messages; its revision must match
  the PX4 firmware/bridge revision.
- `px4_ros2_interface_lib`: upstream PX4 ROS 2 interface library.

## Primary Runtime Data Flow

```text
MID-360 PointCloud2 + IMU
          │
          ▼
      fast_lio
  ├── /lio/odometry          [lio_world -> mid360_imu, ROS time]
  ├── /lio/cloud_registered  [lio_world, ROS time]
  └── /lio/path              [lio_world, ROS time]
          │
          ├──► global_mapper
          │       └──► /mapping/global
          │
          └──► lio_px4_alignment
                  └──► /fmu/in/vehicle_visual_odometry
                       [NED pose, FRD body, PX4 boot time]

Raw/local point cloud + PX4 vehicle odometry
          │
          ▼
 obstacle_perception
  ├──► /fmu/in/obstacle_distance [BODY_FRD, PX4 boot time]
  └──► /perception/scan_1d       [ROS debug topic]
```

`lio_px4_alignment` currently performs coordinate-representation and timestamp
conversion. It does **not** estimate a dynamic translation/yaw alignment between
an arbitrary `lio_world` origin and the PX4 EKF origin. The integration must
therefore establish compatible initialization semantics, or add a separately
validated alignment stage, before treating the output as globally north-aligned.

The older `lidar_odometry` + `localization_bridge` path remains in the repository
for compatibility and testing. It uses `camera_init`, `/localization/*`, and
`/world/cloud`; it is not the primary FAST-LIO runtime path.

## Clock Contract

The system contains two explicit clock domains.

### ROS clock domain

Use the node-owned ROS clock for:

- ROS message headers
- TF stamps
- freshness and timeout checks
- pose buffers and internal runtime state

### PX4 clock domain

PX4 integer fields `timestamp` and `timestamp_sample` are microseconds since PX4
boot. DDS transport does not convert those integer fields into `rclcpp::Time`.

Every first-party PX4 boundary uses `px4_ros2_utils::time::Timesync`:

- PX4 input to ROS processing: `Timesync::toROS(px4_timestamp_us)`
- ROS measurement/publication to PX4 output: `Timesync::toPX4(ros_time)`

`timestamp` records publication time. `timestamp_sample` records measurement
acquisition time. Conversion failure, clock-type mismatch, stale sync, overflow,
or a converted value of zero causes the sample to be deferred or dropped. Direct
`nanoseconds()/1000` casts and fallback clock guesses are forbidden.

Simulation mode uses a zero offset only when PX4 SITL and ROS nodes share the
documented `/clock` setup. Hardware/external mode is updated from
`TimesyncStatus`.

## Frame Contract

The authoritative definitions and conversion rules are in
[`frame_contract.md`](frame_contract.md).

Key boundaries:

- `lio_world`: gravity-aligned, Z-up localization world with arbitrary initial
  yaw; it is not automatically north-aligned.
- `mid360_imu`: FAST-LIO body state, FLU convention.
- `map_ned`: PX4 local world, North-East-Down.
- `vehicle_frd` / PX4 body fields: Forward-Right-Down.

Generic ENU-like/FLU to NED/FRD representation changes use
`px4_ros2_utils::frame`. No identity TF may be published between `lio_world` and
`map_ned`, because both axes and origins can differ.

## Output Taxonomy

These names are fixed; do not use “local map” for unrelated products.

| Output | Primary topic | Producer | Meaning |
| --- | --- | --- | --- |
| Registered LIO cloud | `/lio/cloud_registered` | `fast_lio` | Current scan transformed into `lio_world` |
| Global 3D map | `/mapping/global` | `global_mapper` | Full accumulated occupied voxel map for the active input frame |
| Planner-local 3D map | not implemented | deferred executable | Bounded map intended for a future 3D planner |
| Distance bins 2D | `/fmu/in/obstacle_distance` | `obstacle_perception` | 72-bin PX4 Collision Prevention input |
| Virtual scan 1D | `/perception/scan_1d` by the SITL script | `obstacle_perception` | ROS debug perception output, not a map |
| External odometry | `/fmu/in/vehicle_visual_odometry` | `lio_px4_alignment` | PX4 `VehicleOdometry`, not a map |

`/mapping/global` uses the incoming `lio_world` frame in the default
`input_source=lio_world` pipeline. Other supported mapper modes can publish the
map in `map_ned`; consumers must inspect the message `frame_id` rather than infer
it from the topic name.

## Primary Topic Contract

| Topic | Producer | Frame/semantics | Clock | QoS intent |
| --- | --- | --- | --- | --- |
| `/lio/odometry` | `fast_lio` | parent `lio_world`, child `mid360_imu` | ROS | reliable |
| `/lio/cloud_registered` | `fast_lio` | `lio_world` | ROS | reliable |
| `/lio/path` | `fast_lio` | `lio_world` | ROS | reliable |
| `/mapping/global` | `global_mapper` | active mapper world frame | ROS | reliable |
| `/fmu/in/vehicle_visual_odometry` | `lio_px4_alignment` | NED pose, FRD body; unavailable velocity is NaN | PX4 boot μs via `Timesync` | best effort |
| `/fmu/in/obstacle_distance` | `obstacle_perception` | `BODY_FRD` message frame | PX4 boot μs via `Timesync` | reliable |
| `/perception/scan_1d` | `obstacle_perception` | configured ROS debug frame | ROS | reliable |

A contract change must update the implementation, parameter YAML, tests, this
document, and relevant SITL analysis tooling together.

## Configuration Hierarchy

1. Compile-time constants only for true invariants.
2. Package defaults in `src/<package>/config/`.
3. Cross-package orchestration in top-level `config/` and `scripts/`.
4. Launch arguments and ROS CLI parameters as runtime overrides.

FAST-LIO has one canonical parameter source:
`src/fast_lio/config/fast_lio.params.yaml`. The node uses native ROS 2 parameter
declaration and standard launch/CLI precedence; it has no internal second YAML
parser.

## Current Scope Status (2026-07-15)

Implemented:

- FAST-LIO localization and incremental ikd-Tree mapping
- accumulated global voxel-map publication
- PX4 external-odometry message conversion with separate sample/publish times
- PX4 Collision Prevention distance bins and ROS scan visualization
- mapping/planning support libraries and regression tests

Deferred:

- controlled map-quality tuning A/B beyond the preserved baseline parameters
- dynamic `lio_world` to PX4-origin/yaw alignment estimation
- planner-local 3D map executable
- B-spline optimizer, full controller, and mission state machine
- loop closure and multi-session SLAM

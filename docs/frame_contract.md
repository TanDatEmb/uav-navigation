# Frame Contract — uav-navigation

**Version:** 1.0
**Updated:** 2026-07-15
**Scope:** MID-360, FAST-LIO, ROS 2 mapping, and PX4 message boundaries

## Frame Definitions

| Frame or convention | Axes | Owner | Meaning |
| --- | --- | --- | --- |
| `lio_world` | gravity-aligned, Z-up; initial yaw arbitrary | `fast_lio` | Local LIO world established at initialization |
| `mid360_imu` | Forward-Left-Up (FLU) | `fast_lio` | IMU/body state estimated by FAST-LIO |
| `mid360_lidar_frame` | sensor FLU | Gazebo MID-360 model | Raw simulated LiDAR message frame |
| `lidar_sensor_link` | sensor FLU | SITL visualization tooling | Project alias connected to `mid360_lidar_frame` |
| `map_ned` | North-East-Down (NED) | PX4 | PX4 local EKF world frame |
| `vehicle_frd` | Forward-Right-Down (FRD) | PX4 semantics | Vehicle body convention used by PX4 odometry |
| `aircraft` | FRD | ROS debug outputs | Project frame ID used for body-frame visualization |
| `camera_init` | legacy ENU-like world | compatibility nodes | Earlier localization frontend world frame |
| `base_link` | FLU | compatibility nodes | ROS REP-103 body frame |

`lio_world` is Z-up but is not automatically geodetically East-North-Up: its
initial yaw is arbitrary. A coordinate-representation conversion can change the
axis convention, but it cannot recover PX4 north alignment or origin translation.

## Transform Notation

Use `T_parent_child` for a pose of `child` expressed in `parent`:

```text
p_parent = T_parent_child * p_child
T_parent_sensor = T_parent_body * T_body_sensor
```

Names such as `T_total`, `body_pose`, `extrinsic`, or `world_transform` are not
acceptable when the source and destination frames are unclear.

FAST-LIO stores the fixed LiDAR extrinsic as `T_imu_lidar` and computes:

```text
T_lio_world_lidar = T_lio_world_imu * T_imu_lidar
```

The extrinsic is configured through `extrinsic_t` and `extrinsic_r` in the
canonical FAST-LIO parameter file.

## Active TF Chain

The simulation/visualization chain is:

```text
lio_world
└── mid360_imu                    dynamic, published by fast_lio
    └── lidar_sensor_link         static calibrated LiDAR-in-IMU transform
        └── mid360_lidar_frame    static identity alias for Gazebo messages
```

The PX4 `map_ned` frame is intentionally not connected to `lio_world` by an
identity static TF. Their origins and horizontal axes are not guaranteed to
match.

## Coordinate Representation Changes

Use `px4_ros2_utils::frame` at PX4 boundaries. Do not duplicate these operations
with local sign flips or hand-written matrices.

### World vector

```text
ENU-like -> NED: [x, y, z] -> [y, x, -z]
NED -> ENU-like: [x, y, z] -> [y, x, -z]
```

The operation is an involution. It only changes coordinate convention; it does
not estimate yaw/origin alignment.

### Body vector

```text
FLU -> FRD: [x, y, z] -> [x, -y, -z]
FRD -> FLU: [x, y, z] -> [x, -y, -z]
```

### Pose and orientation

Use the complete pose helpers, such as `enu_to_ned_pose`, so the world and body
basis changes are applied together. Do not derive quaternion conversion by
negating Euler angles.

Quaternion storage rules:

- Eigen constructor: `(w, x, y, z)`
- Eigen `coeffs()`: `[x, y, z, w]`
- PX4 arrays: `[w, x, y, z]`

## FAST-LIO Output Contract

| Topic | Parent/world frame | Child/body frame | Clock |
| --- | --- | --- | --- |
| `/lio/odometry` | `lio_world` | `mid360_imu` | ROS clock |
| `/lio/cloud_registered` | `lio_world` | n/a | ROS clock |
| `/lio/path` | `lio_world` | n/a | ROS clock |
| `/tf` | `lio_world` | `mid360_imu` | ROS clock |

FAST-LIO remains independent of PX4 and must not publish `map_ned` data directly.

## PX4 External-Odometry Boundary

`px4_mapping/lio_px4_bridge` consumes `/lio/odometry` and `/fmu/out/vehicle_odometry`
and publishes `VehicleOdometry` with:

- `pose_frame = POSE_FRAME_NED`
- position/orientation represented as NED world + FRD body
- `velocity_frame = VELOCITY_FRAME_UNKNOWN`
- velocity, angular velocity, and unavailable velocity variance set to NaN
- position/orientation covariance reordered through the shared frame utilities
- `timestamp_sample` derived from the LIO header through `Timesync`
- `timestamp` derived from publication time through `Timesync`

The node estimates a fixed `T_map_ned_lio_world` transform by comparing the LIO
pose with PX4 odometry at startup. After capture the same transform is applied
to every LIO pose. No per-scan point-cloud transform is performed here.

## Global Map Frames

`global_mapper` always operates in the `lio_world` frame:

- registered points from `/lio/cloud_registered`, `/mapping/occupancy/global`, and
  `/mapping/occupancy/local` all remain in `lio_world`.
- PX4-based mapper modes were removed in Commit 1; only the FAST-LIO lio_world
  pipeline remains supported.

Global retention is independent of distance eviction. Distance eviction is
disabled by default; `enable_distance_eviction=true` explicitly opts into the
legacy radius-bounded memory mode. Capacity and frame-age bounds remain active.

`/mapping/occupancy/local` is a radius-bounded view of `/mapping/occupancy/global`;
it does not define a new coordinate frame. Consumers must use
`PointCloud2.header.frame_id`; the topic name alone does not imply a frame.

## Legacy Compatibility Path

The legacy `localization_bridge` and `lio_px4_alignment` nodes have been removed
and replaced by the unified `lio_px4_bridge`. `camera_init`, `base_link`,
`/localization/*`, and `/world/cloud` are no longer used. New FAST-LIO
integration uses `/lio/*` topics and the explicit PX4 boundary.

## Validation Rules

1. Every ROS message with geometric data has a non-empty `frame_id`.
2. Parent/child direction is encoded in transform names.
3. ROS headers and TF use the node-owned ROS clock.
4. PX4 integer timestamps use PX4 boot time through `Timesync`.
5. PX4 timestamp zero is invalid and suppresses publication.
6. No identity TF is published between frames with different conventions or
   unverified origins.
7. Unit tests cover vector, pose, quaternion, and covariance conversions at the
   PX4 boundary.
8. A frame-contract change updates code, YAML, tests, architecture docs, RViz,
   and SITL analysis tooling together.

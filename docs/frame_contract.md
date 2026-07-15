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

`px4_mapping/lio_px4_alignment` consumes `/lio/odometry` and publishes
`VehicleOdometry` with:

- `pose_frame = POSE_FRAME_NED`
- position/orientation represented as NED world + FRD body
- `velocity_frame = VELOCITY_FRAME_UNKNOWN`
- velocity, angular velocity, and unavailable velocity variance set to NaN
- position/orientation covariance reordered through the shared frame utilities
- `timestamp_sample` derived from the LIO header through `Timesync`
- `timestamp` derived from publication time through `Timesync`

Despite its historical name, this node currently performs deterministic frame
and time conversion only. It does not estimate `T_map_ned_lio_world`. A separate,
validated alignment design is required if PX4 origin and north alignment are not
established by initialization assumptions.

## Global Map Frames

`global_mapper` preserves the active world frame:

- `input_source=lio_world`: registered points and `/mapping/global` remain in
  `lio_world`; distance-based local eviction is disabled for the accumulated map.
- PX4-based modes: map output is represented in `map_ned`.

Consumers must use `PointCloud2.header.frame_id`; the topic name alone does not
imply a frame.

## Legacy Compatibility Path

The older `lidar_odometry` and `localization_bridge` nodes use:

- `camera_init` as the localization world
- `base_link` as the ROS body
- `map_ned` at the PX4/world boundary

This path remains for compatibility and testing. New FAST-LIO integration should
use `/lio/*` and the explicit PX4 boundary.

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

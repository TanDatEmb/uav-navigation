# FAST-LIO Integration Guide

## Purpose

The `fast_lio` package provides MID-360 LiDAR-inertial odometry for the
`uav-navigation` workspace. It owns localization in `lio_world` and remains
independent of PX4. PX4 frame and timestamp conversion is performed by
`px4_mapping`.

## Data Flow

```text
PointCloud2 + sensor_msgs/Imu
             │
             ▼
         fast_lio
    ├── /lio/odometry
    ├── /lio/cloud_registered
    ├── /lio/path
    └── TF: lio_world -> mid360_imu
             │
             ├──► global_mapper -> /mapping/global
             └──► lio_px4_alignment
                       └──► /fmu/in/vehicle_visual_odometry
```

The filter uses a 15-DOF state and an incremental ikd-Tree map. See
`docs/ieskf_design.md` and `docs/ikdtree_architecture.md` for implementation
details.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/letandat/Dev/uav-navigation
colcon build --packages-up-to fast_lio
source install/setup.bash
colcon test --packages-select fast_lio
colcon test-result --all --verbose
```

## Run the Package Directly

```bash
ros2 launch fast_lio fast_lio_sim.launch.py \
  use_sim_time:=true \
  lidar_topic:=/lidar_360/points \
  imu_topic:=/imu/out
```

The launch file loads `config/fast_lio.params.yaml` and remaps the relative
outputs to `/lio/*`.

## Run the Supported SITL Stack

From the workspace root:

```bash
bash scripts/sim_launch.sh
```

Useful environment switches:

```bash
GZ_GUI=1 bash scripts/sim_launch.sh
RECORD_BAG=1 bash scripts/sim_launch.sh
ENABLE_EXTERNAL_ODOMETRY=1 bash scripts/sim_launch.sh
```

External odometry is disabled by default. Enabling it publishes FAST-LIO output
to PX4 through `lio_px4_alignment`; it does not command or arm the vehicle.
Stop the stack with:

```bash
bash scripts/sim_stop.sh
```

## Canonical Configuration

`config/fast_lio.params.yaml` is the only FAST-LIO parameter file. The node
declares every active value through the ROS 2 parameter API. Launch dictionaries
and CLI overrides follow normal ROS 2 precedence; no internal YAML parser writes
a second set of values.

### Active topics and frames

| Parameter | Active value | Meaning |
| --- | --- | --- |
| `imu_topic` | `/livox/imu` before launch remapping | IMU input |
| `lidar_topic` | `/livox/lidar` before launch remapping | PointCloud2 input |
| `world_frame` | `lio_world` | gravity-aligned Z-up LIO world |
| `body_frame` | `mid360_imu` | FLU IMU/body state |
| `lidar_frame` | `mid360_lidar` | Expected LiDAR sensor frame_id |

### LiDAR input decoder

The `PointCloud2Decoder` normalizes `sensor_msgs::PointCloud2` into a
`NormalizedLidarScan` before any synchronization, deskew, or map update.

Replaces the former `sim_mode` boolean with explicit per-point time
configuration:

| Parameter | SIM default | REAL MID-360 | Meaning |
| --- | --- | --- | --- |
| `lidar_input.profile` | `sim_xyzi_snapshot` | `mid360_pointcloud2` | Input profile |
| `lidar_input.time_field` | `""` (none) | `offset_time` | Per-point time field name |
| `lidar_input.time_unit` | `nanoseconds` | `nanoseconds` | Time unit: seconds/ms/us/ns |
| `lidar_input.header_stamp_is_scan_start` | `true` | `true` | Header stamp semantics |
| `lidar_input.require_per_point_time` | `false` | `true` | Reject if time field missing |
| `lidar_input.max_scan_duration_s` | `0.2` | `0.2` | Max scan duration |
| `lidar_input.min_range_m` | `0.5` | `0.5` | Min range filter |
| `lidar_input.max_range_m` | `100.0` | `100.0` | Max range filter |
| `lidar_input.filter_livox_tags` | `false` | `true` | Filter by Livox tag bits |
| `lidar_input.point_stride` | `1` | `1` | Keep every N-th point |

**Contract:** `curvature` in `PointXYZINormal` stores per-point relative time
in **seconds** (not milliseconds). Points are sorted by ascending time by
the decoder. `scan_start_time_s` and `scan_end_time_s` are computed by the
decoder and consumed directly by the synchronizer.

### Deskew (Layer 3)

The `LidarDeskewer` compensates point motion distortion using the forward-propagated IMU trajectory.

| Parameter | Value | Meaning |
| --- | --- | --- |
| `deskew.enabled` | `true` | Enable motion compensation |
| `deskew.traj_min_knots` | `2` | Minimum IMU trajectory knots required |

**Formula:** Uses SE(3) absolute transforms:
```
P_L_end = (T_W_L_end)⁻¹ * T_W_L_i * P_L_i
```
where `T_W_L_i = T_W_I_i * T_I_L` (LiDAR pose at point time).

Points are iterated forward (ascending time) with monotonic knot index for O(N+M) complexity. Points outside trajectory coverage are copied raw without compensation.

**Contract:**
- Requires `NormalizedLidarScan.has_per_point_time = true`
- Requires `ImuTrajectory.covers(point_time)` for each point
- SIM snapshots (zero duration) skip deskew automatically
- Output cloud replaces input in `SyncPackage::cloud`

### MeasurementSynchronizer

The `MeasurementSynchronizer` replaces the inline `syncPackage()` in the node.
It buffers LiDAR scans and IMU samples, then produces `SyncPackage` with
bracketing IMU samples at both scan boundaries.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `synchronizer.max_imu_samples` | 4000 | Max IMU samples in buffer |
| `synchronizer.max_lidar_scans` | 5 | Max LiDAR scans in buffer |
| `synchronizer.max_imu_gap_s` | 0.02 | Max allowed IMU gap within scan |
| `synchronizer.require_imu_before_scan_start` | true | Reject if no IMU before scan_start |
| `synchronizer.require_imu_after_scan_end` | true | Keep first IMU after scan_end as bracket |

**Contract:** `SyncPackage` now includes `imu_before_scan` and `imu_after_scan`
optional brackets for interpolation. The synchronizer retains boundary samples
between consecutive packages for continuity.

### Active mapping profile

| Parameter | Active value | Meaning |
| --- | --- | --- |
| `scan_resolution` | `0.3 m` | current-scan voxel filter leaf size |
| `map_resolution` | `0.3 m` | ikd-Tree voxel downsampling size |
| `cube_len` | `50.0 m` | sliding local-map cube edge |
| `det_range` | `40.0 m` | detection range used by map movement logic |
| `move_thresh` | `0.5` | local-map movement ratio |
| `near_search_num` | `5` | neighbors requested for plane fitting |
| `ieskf_max_iter` | `1` | maximum LiDAR re-linearizations in the active profile |

These values were preserved during the single-YAML migration. Resolution and
iteration changes require a controlled timing/map-quality A/B test; they must not
be changed as an undocumented cleanup side effect.

Example override:

```bash
ros2 run fast_lio fast_lio_node --ros-args \
  --params-file src/fast_lio/config/fast_lio.params.yaml \
  -p scan_resolution:=0.25 \
  -p map_resolution:=0.25 \
  -p ieskf_max_iter:=3
```

Invalid parameter combinations fail node construction instead of falling back
silently.

## LiDAR-to-IMU Extrinsic

The canonical profile defines the LiDAR pose in the IMU frame:

```yaml
extrinsic_t: [-0.011, -0.02329, 0.04412]
extrinsic_r: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

Translations are meters. `extrinsic_r` is a row-major 3×3 rotation matrix. The
extrinsic is fixed and is not estimated online.

## Frame Boundary

See `docs/frame_contract.md` for the authoritative contract.

- `lio_world` is gravity-aligned and Z-up, with arbitrary initial yaw.
- `mid360_imu` uses FLU body axes.
- `map_ned` and PX4 body fields use NED/FRD only after the explicit PX4 boundary.
- no identity TF is valid between `lio_world` and `map_ned`

`lio_px4_alignment` converts the pose representation with
`px4_ros2_utils::frame` and converts measurement/publication time with
`Timesync`. Despite the historical node name, it does not estimate a dynamic
PX4-origin or north-yaw alignment.

## Outputs

| Topic | Frame | Content |
| --- | --- | --- |
| `/lio/odometry` | `lio_world -> mid360_imu` | estimated IMU pose and covariance |
| `/lio/cloud_registered` | `lio_world` | current downsampled scan transformed by the estimated pose |
| `/lio/path` | `lio_world` | bounded pose history |
| `/tf` | `lio_world -> mid360_imu` | dynamic transform |

## Rosbag and Replay

```bash
ros2 launch fast_lio record_debug_bag.launch.py
ros2 launch fast_lio full_debug_session.launch.py with_px4:=true
```

The workspace SITL launcher can also record the integrated topic set:

```bash
RECORD_BAG=1 bash scripts/sim_launch.sh
```

Replay a recorded bag with simulation time:

```bash
ros2 bag play <bag-directory> --clock --rate 0.5
```

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No odometry | verify `/imu/out` and `/lidar_360/points`; confirm the stationary IMU window passes validation |
| Node exits during startup | inspect the fail-fast parameter error and canonical YAML values |
| Registered cloud has the wrong frame | verify `world_frame=lio_world` and the input sensor `frame_id` |
| Rapid drift or duplicated walls | verify the LiDAR/IMU extrinsic, time ordering, and map/scan resolutions |
| PX4 receives no external odometry | confirm the boundary node is enabled and `Timesync` yields non-zero PX4 timestamps |
| RViz TF errors | verify `lio_world -> mid360_imu` and the static MID-360 extrinsic chain; do not add `lio_world -> map_ned` identity TF |

## Known Boundaries

- no loop closure or multi-session map correction
- no online LiDAR/IMU extrinsic estimation
- PointCloud2 input only in the active pipeline; Livox CustomMsg is not an active
  input path (use a driver adapter to convert CustomMsg to canonical PointCloud2)
- dynamic `lio_world` to PX4-origin/yaw alignment is not implemented
- per-point time is stored in `curvature` as seconds (FAST-LIO2 convention);
  a future refactor may introduce a dedicated `LidarPoint` type

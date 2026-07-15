# FAST-LIO2 Integration Design for UAV Navigation

## Overview

Replace `lidar_odometry` (fake PX4-based LIO) with real FAST-LIO2 implementation supporting:
- Per-point timestamp (simulation + real hardware)
- IMU-based deskew
- Proper frame conventions (`T_parent_child`)

## Architecture

```
Gazebo MID-360 (gpu_lidar 720x28 @ 10Hz)
    │
    ▼
┌─────────────────────────────────────┐
│ livox_sim_adapter_node            │
│ - Adds per-point timestamp field  │
│ - Publishes Livox-like CustomMsg  │
│ - Config: sim_mode (uniform time) │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ fast_lio_node (adapted from        │
│ liangheming/FASTLIO2_ROS2)        │
│ - IESKF-based LIO                 │
│ - IMU propagation                 │
│ - Deskew with per-point time      │
└─────────────────────────────────────┘
    │
    ├──► /fast_lio/odometry (lio_world → imu)
    ├──► /fast_lio/cloud_registered (lio_world frame)
    └──► /fast_lio/path
    │
    ▼
┌─────────────────────────────────────┐
│ lio_px4_alignment (renamed from    │
│ localization_bridge)               │
│ - Aligns lio_world to map_ned      │
│ - Publishes EV to PX4              │
│ - Publishes cloud in map_ned       │
└─────────────────────────────────────┘
    │
    ▼
global_mapper (simplified)
```

## Frame Conventions

| Frame | Description | Convention |
|-------|-------------|------------|
| `mid360_lidar` | LiDAR optical center | FLU (ROS) |
| `mid360_imu` | IMU chip in MID-360 | FLU (ROS) |
| `lio_world` | FAST-LIO world frame | Gravity-aligned, arbitrary yaw |
| `map_ned` | PX4 local world | NED |
| `vehicle_frd` | PX4 body frame | FRD |

### Transform Naming

```cpp
// LiDAR in IMU frame (FAST-LIO extrinsic)
Sophus::SE3d T_mid360_imu_mid360_lidar;

// IMU in vehicle frame (calibration)
Sophus::SE3d T_vehicle_frd_mid360_imu;

// LIO world alignment (captured at init)
Sophus::SE3d T_map_ned_lio_world;

// Pose chain for EV
T_map_ned_vehicle_frd =
    T_map_ned_lio_world *
    T_lio_world_mid360_imu *
    T_vehicle_frd_mid360_imu.inverse();
```

## Simulated Per-Point Timestamp

### Gazebo Limitation
`gpu_lidar` outputs complete frame at 10Hz, no per-point timing.

### Solution: livox_sim_adapter_node

For each PointCloud2 from Gazebo:

1. Parse 720x28 = 20,160 points
2. Calculate relative time based on scan pattern:
   - Scan duration: 100ms
   - Horizontal: 720 samples over 360°
   - Vertical: 28 lines
   - Time per point: `t_rel = (h_idx * 28 + v_idx) / (720 * 28) * 0.1`

3. Add fields to output:
   ```
   x, y, z (float32)
   intensity (float32)
   time (float32)  # NEW: offset from scan start in seconds
   line (uint8)     # NEW: vertical line index 0-27
   ```

4. Output as `livox_ros_driver2::msg::CustomMsg` for compatibility

### Sim Mode

When `sim_mode: true`:
- All points get `time = 0.0`
- FAST-LIO skips deskew (no motion distortion in sim anyway)
- Used for testing pipeline without deskew complexity

When `sim_mode: false` (real hardware):
- Livox driver provides real `offset_time`
- Full deskew enabled

## FAST-LIO2 Core Components (Extracted)

From `liangheming/FASTLIO2_ROS2`:

### Keep (essential)
- `IESKF` - Iterated EKF for state estimation
- `IMUProcessor` - IMU propagation, initialization, bias estimation
- `LidarProcessor` - Feature extraction, correspondence, IKD-Tree map
- `MapBuilder` - Main processing loop, sync package
- Point types: `PointXYZINormal` with `curvature` field for time

### Modify
- Remove PGO, HBA, localizer (not needed for UAV real-time)
- Adapt to PX4 frame conventions (NED/FRD)
- Add sim timestamp injection
- Simplify config loading

### Dependencies
- `pcl_ros` - Point cloud utilities
- `livox_ros_driver2` - Message types (CustomMsg)
- `Sophus` - SE(3) operations
- `ikd-Tree` - Incremental KD-Tree (header-only)

## Implementation Phases

### Phase 1: Sim Timestamp Adapter
- [ ] Create `livox_sim_adapter` package
- [ ] Parse Gazebo PointCloud2
- [ ] Generate synthetic per-point timestamp
- [ ] Output CustomMsg format
- [ ] Test with existing cloud visualization

### Phase 2: FAST-LIO2 Core
- [ ] Create `fast_lio` package
- [ ] Port IESKF from reference
- [ ] Port IMUProcessor
- [ ] Port LidarProcessor (with ikd-Tree)
- [ ] Port MapBuilder main loop
- [ ] Add ROS2 node wrapper
- [ ] Config file for MID-360

### Phase 3: Frame Alignment
- [ ] Rename `localization_bridge` → `lio_px4_alignment`
- [ ] Implement SE(3) capture at alignment
- [ ] Handle reset/failsafe
- [ ] Publish EV odometry
- [ ] Transform cloud to map_ned

### Phase 4: Global Mapper Simplification
- [ ] Remove multi-mode input
- [ ] Single input: cloud in map_ned
- [ ] Single output: global voxel map + local map for planning
- [ ] RViz visualization

### Phase 5: Integration Test
- [ ] SITL with x500 + MID-360
- [ ] RViz: verify map consistency
- [ ] Test EV stability
- [ ] Test obstacle distance bins

## Configuration

### fast_lio.yaml
```yaml
# Sensor
imu_topic: "/livox/imu"
lidar_topic: "/livox/lidar"

# Extrinsic (LiDAR in IMU frame)
extrinsic_t: [-0.011, -0.02329, 0.04412]  # meters
extrinsic_r: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # identity for MID-360

# Filter
lidar_filter_num: 3
lidar_min_range: 0.5
lidar_max_range: 40.0
scan_resolution: 0.1
map_resolution: 0.2
cube_len: 50.0
det_range: 40.0
move_thresh: 0.5

# IMU noise
na: 0.01        # accel noise (m/s^2)
ng: 0.001       # gyro noise (rad/s)
nba: 0.0001     # accel bias noise
nbg: 0.00001    # gyro bias noise

# Algorithm
ieskf_max_iter: 3
gravity_align: true
esti_il: false  # don't estimate LiDAR-IMU extrinsic online

# Sim mode
sim_mode: true  # disables deskew, uniform timestamp
```

### lio_px4_alignment.yaml
```yaml
# Input
lio_odom_topic: "/fast_lio/odometry"
lio_cloud_topic: "/fast_lio/cloud_registered"
px4_odom_topic: "/fmu/out/vehicle_odometry"

# Output
ev_topic: "/fmu/in/vehicle_visual_odometry"
aligned_cloud_topic: "/mapping/cloud_aligned"

# Extrinsic (calibration)
vehicle_frd_T_mid360_imu:
  translation: [0.0, 0.0, 0.0]
  rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

# Alignment
capture_alignment_on_start: true
min_alignment_samples: 10
```

## Notes

1. **MID-360 IMU**: ICM-40609, 6-axis, ~2000 deg/s gyro, ~16g accel
2. **ikd-Tree**: Incremental KD-Tree from HKU, header-only
3. **Sophus**: Use `Sophus::SE3d` for all transforms, no manual R*p+t
4. **Time sync**: PX4 Timesync bridges ROS time and PX4 time
5. **Sim vs Real**: Single codebase, `sim_mode` parameter switches behavior

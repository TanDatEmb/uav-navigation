# Baseline Environment Snapshot

**Commit:** `b1f7d6d0ab18ad02b3bb9fe0cdec7e8b0178b63b`  
**Branch:** `main`  
**Date:** 2026-07-20

## Environment

| Component | Value |
|-----------|-------|
| ROS 2 Distro | Jazzy |
| GCC Version | UNKNOWN (to be determined at build time) |
| G++ Version | UNKNOWN (to be determined at build time) |
| Build Type | RelWithDebInfo |
| Target PX4 Firmware | v1.17 (per submodule branch) |
| Livox ROS Driver 2 | Required for MID360S CustomMsg |

## Submodules

| Submodule | Path | Branch | SHA |
|-----------|------|--------|-----|
| px4_msgs | src/px4_msgs | release/1.17 | UNKNOWN (run `git rev-parse HEAD` in submodule) |
| px4_ros2_interface_lib | src/px4_ros2_interface_lib | release/1.17 | UNKNOWN |
| px4_ros2_utils | src/px4_ros2_utils | main | UNKNOWN |

## Input Configuration

| Profile | YAML File | Adapter | LiDAR Topic | IMU Topic |
|---------|-----------|---------|-------------|-----------|
| sim | simulation.yaml | sim_snapshot | `/livox/mid360/points` | `/livox/mid360/imu` |
| mid360_custom | mid360_custom.yaml | mid360_custom | `/livox/mid360/lidar` | `/livox/mid360/imu` |
| mid360_pointcloud2 | mid360_pointcloud2.yaml | mid360_pointcloud2 | `/livox/mid360/points` | `/livox/mid360/imu` |

## Launch Commands

```bash
# Simulation
ros2 launch fast_lio lio.launch.py profile:=sim

# MID360S hardware - CustomMsg
ros2 launch fast_lio lio.launch.py profile:=mid360_custom use_sim_time:=false

# MID360S hardware - PointCloud2
ros2 launch fast_lio lio.launch.py profile:=mid360_pointcloud2 use_sim_time:=false
```

## Key Parameters (Effective Values)

| Parameter | Value | Source |
|-----------|-------|--------|
| `estimator.backend` | `ieskf` | **Baseline - Phase 2 will migrate to IKFoM** |
| `filter.scan_voxel_size_m` | `0.3` | Profile |
| `filter.map_voxel_size_m` | `0.3` | Profile |
| `mapping.cube_side_length_m` | `50.0` | Profile |
| `mapping.detection_range_m` | `40.0` | Profile |
| `matching.knn_count` | `5` | Common |
| `extrinsic.translation_m` | `[-0.011, -0.02329, 0.04412]` | MID360 datasheet |

## Phase 1 Changes

This baseline documents the state **before** Phase 1 refactoring. Phase 1 addresses:

### P0 Issues Fixed
1. **YAML config drift** - Multiple config files with `/**` scope, unused keys
2. **PX4 bridge alignment deadlock** - Removed alignment state machine, switched to Local-FRD
3. **VoxelPool UB** - Replaced malloc with vector + aligned_allocator

### Configuration Changes
- Consolidated to 4 YAML files: `common.yaml`, `mid360_custom.yaml`, `mid360_pointcloud2.yaml`, `simulation.yaml`
- Removed: `fast_lio.params.yaml`, `lio_common.yaml`, `profile_*.yaml`
- Schema: All parameters under `fast_lio.ros__parameters.*`

### PX4 Bridge Changes
- Frame: Local-FRD (`POSE_FRAME_FRD`) instead of aligned NED
- Removed: `align_to_px4`, `alignment_mode`, `px4_odom_topic` subscription
- No dependency on PX4 armed state or local position

## Limitations for Phase 2

- Estimator backend still `ieskf` (15-DOF custom implementation)
- State15 with extrinsic offline (not in filter state)
- Gravity constant (not estimated on S2 manifold)
- Point-to-plane matching thresholds may need tuning with IKFoM
- ikd-Tree wrapper not yet using FAST-LIO2 upstream insertion

## Known P0 Issues Remaining for Phase 2

1. **estimator_backend** - Running legacy IESKF, not IKFoM 23-state
2. **State15 covariance** - 15×15 instead of 23×23
3. **No gravity estimation** - Fixed world-frame gravity
4. **Offline extrinsic** - Not part of filter state

## Validation Status

See `phase1_validation.md` for build/test results.

## Phase 2 Work

- Migrate to IKFoM 23-state with S2 gravity
- Full FAST-LIO2 measurement update
- Proper extrinsic in state
- ikd-Tree insertion per upstream

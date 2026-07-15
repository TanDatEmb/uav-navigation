# FAST-LIO2 Integration Guide

## Overview

This package provides FAST-LIO2 LiDAR-Inertial Odometry integration for UAV navigation with PX4.

## Architecture

```
Gazebo Simulation
    ├── LiDAR (mid360_link) → PointCloud2
    └── IMU (mid360_link) → IMU messages
            ↓
    livox_sim_adapter (optional)
            ↓
    FAST-LIO2 (15-DOF IESKF)
            ↓
    /lio/odometry → lio_px4_alignment → /fmu/in/odometry
```

## Quick Start

### 1. Build Package

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build FAST-LIO2
cd ~/Dev/uav-navigation
colcon build --packages-select fast_lio

# Source workspace
source install/setup.bash
```

### 2. Run with Gazebo + PX4 SITL

```bash
# Terminal 1: Start PX4 SITL with Mid360
make px4_sitl gazebo-classic_x500_lidar_360

# Terminal 2: Run FAST-LIO2
ros2 launch fast_lio fast_lio_sim.launch.py
```

### 3. Visualize

```bash
# RViz with pre-configured layout
rviz2 -d ~/Dev/uav-navigation/assets/rviz/uav_navigation.rviz
```

## Configuration

### Topics (config/fast_lio.params.yaml)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `imu_topic` | `/livox/imu` | IMU input topic |
| `lidar_topic` | `/livox/lidar` | LiDAR input topic |
| `sim_mode` | `true` | Disable deskew for simulation |

### Algorithm Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_resolution` | 0.1m | Scan downsampling |
| `map_resolution` | 0.2m | Map downsampling |
| `cube_len` | 50m | Local map size |
| `ieskf_max_iter` | 3 | Max IESKF iterations |

### Extrinsic Calibration (MID-360)

```yaml
extrinsic_t: [-0.011, -0.02329, 0.04412]  # LiDAR in IMU frame (m)
extrinsic_r: [1.0, 0.0, 0.0,   # Identity rotation
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

## Frame Conventions

See `docs/frame_contract_v1.md` for detailed frame specifications.

| Frame | Description |
|-------|-------------|
| `lio_world` | FAST-LIO world frame (gravity-aligned) |
| `mid360_imu` | IMU sensor frame |
| `mid360_lidar` | LiDAR optical center |
| `map` | PX4 NED world frame |

## Debug with Rosbag

### Record Debug Session

```bash
# Basic recording
ros2 launch fast_lio record_debug_bag.launch.py

# With custom path
ros2 launch fast_lio record_debug_bag.launch.py bag_path:=$HOME/bags/session1

# Full debug session with recording
ros2 launch fast_lio full_debug_session.launch.py with_px4:=true
```

### Recorded Topics

| Category | Topics |
|----------|--------|
| **IMU** | `/livox/imu`, `/imu/out`, `/fmu/out/vehicle_imu` |
| **LiDAR** | `/livox/lidar`, `/livox/lidar/pointcloud` |
| **LIO Output** | `/lio/odometry`, `/lio/path`, `/lio/registered_points` |
| **PX4** | `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_attitude` |
| **Transforms** | `/tf`, `/tf_static` |

### Playback for Analysis

```bash
# Analyze bag
./scripts/analyze_bag.sh ~/bags/fastlio_debug_20250714_101234

# Play at 50% speed with controls
./scripts/play_bag_debug.sh ~/bags/fastlio_debug_20250714_101234

# Play at 10% speed, start paused
./scripts/play_bag_debug.sh ~/bags/fastlio_debug_20250714_101234 -r 0.1 --pause

# Direct playback
ros2 bag play ~/bags/fastlio_debug_20250714_101234 --clock --rate 0.5
```

### Debug Workflow

1. **Record**: Run `full_debug_session.launch.py` during flight
2. **Analyze**: Use `analyze_bag.sh` to extract statistics
3. **Replay**: Use `play_bag_debug.sh` with RViz
4. **Iterate**: Fix issues and replay with new code

## Troubleshooting

### Build Errors

```bash
# Missing Sophus (optional - uses Eigen fallback)
sudo apt install ros-jazzy-sophus

# Missing PCL
sudo apt install libpcl-dev ros-jazzy-pcl-ros
```

### Runtime Issues

| Symptom | Solution |
|---------|----------|
| No odometry output | Check IMU and LiDAR topics |
| Drifting rapidly | Verify extrinsic calibration |
| High latency | Increase `scan_resolution` |

## Performance

- **Target**: < 5ms per scan @ 10Hz
- **Typical**: ~3ms per scan
- **Bottleneck**: Point-to-plane ICP (O(N) with PCL KD-tree)

## TODO

- [x] Complete ikd-Tree integration
- [x] Add lio_px4_alignment node
- [x] Add rosbag recording for debug
- [ ] Support Livox CustomMsg format
- [ ] Add loop closure detection
- [ ] Multi-session SLAM
- [ ] Add automated flight test validation

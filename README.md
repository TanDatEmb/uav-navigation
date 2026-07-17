# uav-navigation

ROS 2 Jazzy and PX4 mapping/perception workspace for UAV operation in GPS-denied
and cluttered environments. The current baseline centers on Livox MID-360
LiDAR-inertial odometry, accumulated 3D mapping, PX4 external odometry, and PX4
Collision Prevention input.

## Current Scope

Implemented and maintained:

- FAST-LIO2-style 15-DOF IESKF localization for the MID-360 pipeline
- incremental ikd-Tree scan map, accumulated global occupancy, and a 15 m rolling local view
- explicit ROS ENU-like/FLU to PX4 NED/FRD boundary conversion
- ROS/PX4 timestamp conversion through `px4_ros2_utils::time::Timesync`
- PX4 `ObstacleDistance` generation and a ROS `LaserScan` debug view
- Gazebo Harmonic + PX4 SITL orchestration and RViz/rosbag tooling

Deferred by the current scope lock:

- B-spline trajectory optimization
- a complete navigation controller and mission state machine
- executable planner/controller integration for the local map
- loop closure and multi-session SLAM

## Runtime Architecture

```text
Gazebo MID-360 or physical sensor
    ├── PointCloud2
    └── sensor_msgs/Imu
             │
             ▼
         fast_lio
    ├── /lio/odometry
    ├── /lio/cloud_registered
    └── /lio/path
             │
             ├──► px4_mapping/global_mapper
             │             ├──► /mapping/occupancy/global
             │             └──► /mapping/occupancy/local
             │
             └──► px4_mapping/lio_px4_bridge
                         └──► /fmu/in/vehicle_visual_odometry

Point cloud + PX4 vehicle odometry
             │
             ▼
px4_navigation/obstacle_perception
    ├── /fmu/in/obstacle_distance
    └── /perception/scan_1d (debug)
```

FAST-LIO remains independent of PX4. Generic frame, time, QoS, parameter, and
typed-topic helpers are applied only at ROS 2/PX4 boundaries through the
`px4_ros2_utils` submodule.

## Repository Layout

```text
uav-navigation/
├── src/
│   ├── fast_lio/                  # MID-360 LiDAR-inertial odometry
│   ├── px4_mapping/               # global map and PX4 odometry boundaries
│   ├── px4_nav_common/            # project-specific mapping/navigation contracts
│   ├── px4_navigation/            # obstacle perception and planning libraries
│   ├── px4_msgs/                  # upstream PX4 messages (submodule)
│   ├── px4_ros2_interface_lib/    # upstream PX4 ROS 2 interface (submodule)
│   └── px4_ros2_utils/            # generic PX4/ROS 2 utilities (submodule)
├── assets/                        # RViz configuration
├── config/                        # cross-package runtime configuration
├── docs/                          # architecture and engineering contracts
├── scripts/                       # SITL orchestration and analysis
├── tests/                         # workspace integration tests
├── tools/                         # build, formatting, and PX4 helper scripts
└── vendor/px4_autopilot_extras/   # reproducible PX4/Gazebo additions
```

## Build and Test

```bash
source /opt/ros/jazzy/setup.bash
cd /home/letandat/Dev/uav-navigation

git submodule update --init --recursive
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

For the actively developed mapping path only:

```bash
colcon build --packages-up-to fast_lio px4_mapping px4_navigation
colcon test --packages-select fast_lio px4_mapping px4_navigation
```

## Simulation

The supported orchestrator is:

```bash
bash scripts/sim_launch.sh
```

It starts the required bridges and nodes for PX4 SITL with Gazebo Harmonic.
External odometry publication is opt-in through the script configuration. Stop
the session with `bash scripts/sim_stop.sh`.

## Documentation

- [`docs/architecture.md`](docs/architecture.md): package boundaries, data flow,
  topics, frames, clocks, and current scope
- [`docs/frame_contract.md`](docs/frame_contract.md): authoritative frame and
  conversion contract
- [`docs/conventions.md`](docs/conventions.md): coding and verification rules
- [`docs/ieskf_design.md`](docs/ieskf_design.md): implemented FAST-LIO filter
  design
- [`docs/ikdtree_architecture.md`](docs/ikdtree_architecture.md): implemented
  spatial-index architecture
- [`src/fast_lio/INTEGRATION.md`](src/fast_lio/INTEGRATION.md): FAST-LIO package
  operation and configuration

All first-party source comments, commit messages, and repository documentation
are maintained in English.

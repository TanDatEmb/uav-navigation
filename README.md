# uav-navigation

ROS 2 Jazzy + PX4 autopilot navigation stack for autonomous UAV operations in GPS-denied and cluttered environments.

This repository is the next-generation, clean-room redesign of the mapping and navigation pipeline. It carries forward the proven architecture from the legacy `Mapping_and_Navigation_for_PX4_UAV` prototype while enforcing strict modularity, testability, and PX4-native conventions from the first commit.

## Architecture

```
LiDAR + IMU / depth / stereo
        ↓
┌───────────────────────┐
│      px4_mapping      │  ← odometry, local voxel map, sensor handlers
└───────────────────────┘
        ↓
┌───────────────────────┐
│     px4_navigation    │  ← reactive + deliberative planner, state machine
└───────────────────────┘
        ↓
┌───────────────────────┐
│  px4_navigation_common │  ← mapping ↔ navigation contracts
│      px4_ros2_utils    │  ← external PX4 ↔ ROS 2 utilities (submodule)
└───────────────────────┘
        ↓
         PX4
```

## Repository Layout

Standard ROS 2 workspace structure:

```
uav-navigation/
├── src/
│   ├── px4_msgs/                 # upstream PX4 uORB message definitions (submodule)
│   ├── px4_ros2_utils/           # external PX4 ↔ ROS 2 utilities (submodule)
│   ├── px4_navigation_common/    # project-specific types, transforms, helpers
│   ├── px4_mapping/              # LiDAR odometry + global mapping bridge
│   └── px4_navigation/           # local planning, trajectory generation, control
├── config/                       # global runtime parameters
├── launch/                  # top-level orchestration launch files
├── docs/                    # conventions, frame definitions, architecture
├── tests/                   # integration tests
└── tools/                   # helper scripts
```

## Conventions

See `docs/conventions.md`.

Highlights:

- ROS 2 C++ Style Guide + PX4 safety-critical supplements.
- Classes `PascalCase`, functions/members `snake_case_` for members.
- All comments in English.
- Declare every parameter; load before use.
- No magic numbers; use named constants or YAML parameters.
- Single setpoint publish per control cycle.

## Build

```bash
cd /home/letandat/Dev/uav-navigation
colcon build --symlink-install
```

Or use the safe build helper:

```bash
cd /home/letandat/Dev/uav-navigation
./tools/build.sh
```

## Status

Initial repository scaffolding. First feature package under design.

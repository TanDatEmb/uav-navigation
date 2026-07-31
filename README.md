# UAV Navigation

ROS 2 Jazzy workspace for reliable LiDAR–IMU odometry and an
incremental **registration map** in `odom`. The present scope ends at corrected
odometry, registered points, and a local registration map; it intentionally
does not contain planning, occupancy/world modelling, safety, or PX4 integration.

## Prerequisites

Ubuntu 24.04, ROS 2 Jazzy, and a C++20 compiler are the supported baseline.
Source the ROS installation before building:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The first build validates package metadata and configuration files. Hardware
operation additionally requires measured LiDAR/IMU extrinsics and confirmed
sensor timestamp/frame semantics; defaults marked `PLACEHOLDER` are not a
calibration and must not be used for flight or real-data acceptance.

## Packages

- `ikfom_vendor`: pinned IKFoM provenance and an explicit CMake interface target.
- `fast_lio_core`: estimator algorithm (ROS-independent).
- `fast_lio_ros`: ROS 2 adapters and corrected-output publisher.
- `fast_lio_tools`: offline evaluation using the same core pipeline.
- `navigation_bringup`: composition launch/configuration entry points.
- `uav_description`: source-of-truth sensor-frame URDF/Xacro.
- `uav_simulation`: Gazebo Harmonic launch assets for simultaneous-scan testing.

Read the consolidated [FAST-LIO guide](docs/fast_lio.md) before selecting a
sensor configuration or running dataset acceptance. Read
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before distributing artifacts:
vendored GPL-2.0-only dependencies require a distribution-license review.

## Run boundaries

Simulation may use `simultaneous_scan`, which explicitly bypasses deskew because
all rays share one timestamp. Real sensors must use per-point timing only after
the message layout and timestamps have been verified. Corrected odometry is
published only after a successful LiDAR correction while tracking.

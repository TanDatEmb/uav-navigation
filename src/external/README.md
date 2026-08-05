# External ROS dependencies

This directory contains the pinned external ROS packages used by the
workspace. They are built as normal workspace packages; no vcstool manifest or
external overlay is required.

## `px4_msgs`

`px4_msgs` is tracked as a Git submodule at `src/external/px4_msgs`.

- PX4 release: `v1.17.0`
- pinned commit: `86d8239e962f6939e05c3737784f60c02fa884db`
- consumer: `src/px4_interface/px4_odometry_bridge`

Initialize the submodule after cloning:

```bash
git submodule update --init --recursive
```

The package is then built by the normal workspace build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Do not replace this checkout with a second `px4_msgs` package from an overlay;
the parent repository intentionally pins the message definitions used by the
PX4 bridge.

## `livox_ros_driver2`

The complete pinned upstream package is stored at
`src/external/livox_ros_driver2`. Its ROS package name remains
`livox_ros_driver2`, so estimator and bag contracts do not change. Upstream
provenance and the local offline-build patch are documented in that package's
`UPSTREAM.md` and `PATCHES.md`.

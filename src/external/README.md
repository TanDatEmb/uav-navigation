# External ROS dependencies

This directory contains the pinned external ROS packages used by the
workspace. They are built as normal workspace packages; no vcstool manifest or
external overlay is required.

## `px4_msgs`

`px4_msgs` is tracked as a Git submodule at `src/external/px4_msgs`.

- PX4 release: `v1.17.0`
- pinned commit: `86d8239e962f6939e05c3737784f60c02fa884db`
- consumer: `src/px4/px4_odometry_bridge`
- license: BSD 3-Clause (`src/external/px4_msgs/LICENSE`)

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

## `px4_ros2_interface_lib`

`px4_ros2_interface_lib` is tracked as a Git submodule at
`src/external/px4_ros2_interface_lib`, using the `release/1.17` branch pinned
to commit `4a3370f084ac6f1ef001a4afa2b007845ffd0837`. It provides the
`px4_ros2_cpp` package used by `px4_navigation_external_mode`.

The project-owned node must use `ModeBase`, `ModeExecutorBase`, and the
library setpoint types. It must not replace the library with direct
`px4_msgs`/Offboard publishing.

## `livox_ros_driver2`

The complete pinned upstream package is stored at
`src/external/livox_ros_driver2`. Its ROS package name remains
`livox_ros_driver2`, so estimator and bag contracts do not change. Upstream
provenance and the local offline-build patch are documented in that package's
`UPSTREAM.md` and `PATCHES.md`. The copied driver source is MIT-licensed; see
`src/external/livox_ros_driver2/LICENSE`. The bundled RapidJSON notices are in
`src/external/livox_ros_driver2/3rdparty/rapidjson/license.txt`.

The consolidated workspace inventory is in
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md).

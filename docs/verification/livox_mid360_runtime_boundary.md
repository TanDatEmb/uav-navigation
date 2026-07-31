# Livox Mid-360 runtime boundary

## Pinned interface

The ROS boundary is pinned to the official
[`livox_ros_driver2` 1.2.6 release](https://github.com/Livox-SDK/livox_ros_driver2/releases/tag/1.2.6),
commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`.
`livox_ros_driver2/msg/CustomMsg` and `CustomPoint` are copied byte-for-byte
from that commit. Their checksums and license provenance are recorded in
`src/navigation_estimator/livox_ros_driver2_interface/UPSTREAM.md`.

The local `livox_ros_driver2` package is intentionally interface-only. The
hardware driver needs Livox SDK2 (`livox_lidar_api.h` and
`liblivox_lidar_sdk_shared.so`), which is not available in this environment.
This package therefore provides no network transport, device configuration,
launch file, or `livox_ros_driver2_node` executable. It must not be represented
as a working sensor driver.

For hardware deployment, install and run the full pinned driver in a separate
deployment workspace with its SDK dependency. Do not overlay both packages in
one colcon workspace because they correctly share the same ROS package identity.
FAST-LIO consumers keep the same message contract.

## Message and time semantics

The adapter consumes only official fields:

- `timebase` is the absolute first-point time in integer nanoseconds.
- `offset_time` is copied directly as the point-relative integer nanosecond
  offset. No scale factor or synthetic timing is applied.
- `x`, `y`, `z`, `reflectivity`, `tag`, and `line` are preserved.
- `point_num` must equal `points.size()`, the frame must match configuration,
  points must be finite, and time arithmetic must fit the core integer range.

The production policy is `require_header_match`: `header.stamp` and `timebase`
must represent the same integer nanosecond epoch, and `timebase` remains the
authoritative scan start. An explicit `timebase_authoritative` policy exists
only for legacy/offline input that is known to have a divergent header.

All Mid-360 LiDAR and IMU messages are tagged `sensor_time` in the core. This is
a clock-domain label, not an assertion that the sensor clock is synchronized to
UTC or ROS wall time. A deployment requiring a different epoch must perform
that synchronization at the sensor/driver boundary and configure both streams
to the same domain.

## Topics and QoS

The real configuration subscribes to the upstream defaults:

- LiDAR: `/livox/lidar`, `livox_ros_driver2/msg/CustomMsg`
- IMU: `/livox/imu`, `sensor_msgs/msg/Imu`

Both subscriptions use reliable, volatile, keep-last depth 256 QoS to match the
official ROS 2 driver publisher construction at the pinned revision. Unit tests
lock the message conversion, nanosecond offset behavior, timestamp policy,
clock-domain propagation, and both QoS profiles.

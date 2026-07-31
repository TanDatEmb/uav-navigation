#!/usr/bin/env bash
set -euo pipefail
echo '=== Gazebo topics ==='
gz topic -l | grep -E '(/sim/mid360|/world/px4_lio_smoke/clock)' || true
echo; echo '=== ROS topics ==='
ros2 topic list | grep -E '^(/clock|/lidar/points|/lidar/imu|/lio/)' || true
echo; echo 'Expected checks:'
echo '  ros2 topic hz /lidar/points       # ~10 Hz'
echo '  ros2 topic hz /lidar/imu          # ~200 Hz'
echo '  ros2 topic hz /lio/odometry_corrected'
echo '  ros2 topic echo /lio/diagnostics --once'

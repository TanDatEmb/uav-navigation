#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <new-output-directory>" >&2
  exit 64
fi

output_dir=$1
if [[ -e "$output_dir" ]]; then
  echo "refusing to overwrite existing path: $output_dir" >&2
  exit 65
fi

require_type() {
  local topic=$1
  local expected=$2
  local actual
  actual=$(ros2 topic type "$topic" 2>/dev/null || true)
  if [[ "$actual" != "$expected" ]]; then
    echo "topic $topic has type '$actual', expected '$expected'" >&2
    exit 66
  fi
}

require_type /livox/lidar livox_ros_driver2/msg/CustomMsg
require_type /livox/imu sensor_msgs/msg/Imu
require_type /tf tf2_msgs/msg/TFMessage
require_type /tf_static tf2_msgs/msg/TFMessage

echo "Recording raw physical Mid-360 messages. Do not run with simulator topics."
ros2 bag record --storage mcap --output "$output_dir" \
  /livox/lidar /livox/imu /tf /tf_static

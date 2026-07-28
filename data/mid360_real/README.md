# Real Livox Mid-360 datasets

This directory contains only manifests and documentation in Git. It must never
contain generated points, fake CSV, or simulator output represented as a real
capture. Large bags (`*.mcap`, `*.db3`, `metadata.yaml`) are ignored by Git.

A usable capture is a ROS 2 MCAP bag containing these unremapped topics exactly:

| Topic | Required type |
| --- | --- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/imu` | `sensor_msgs/msg/Imu` |
| `/tf` | `tf2_msgs/msg/TFMessage` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` |

`CustomMsg.timebase` and every `CustomPoint.offset_time` are part of the data
contract and must remain in the original bag. A PointCloud2 conversion, CSV
export, or synthetically produced bag is not an equivalent acquisition record.

Record a physical device with `tools/datasets/record_mid360_bag.sh`, then create
and verify an actual hash manifest:

```bash
tools/datasets/record_mid360_bag.sh data/mid360_real/<capture-id>
python3 tools/datasets/create_mid360_manifest.py data/mid360_real/<capture-id> \
  --dataset-id <capture-id> --source-kind physical_capture
python3 tools/datasets/verify_mid360_dataset.py data/mid360_real/<capture-id> \
  # checks hashes and ros2 bag metadata by default
```

Replay preserves original message type and timebase:

```bash
tools/datasets/replay_mid360_bag.sh data/mid360_real/<capture-id>
```

The Livox ROS Driver 2 documentation identifies its Mid-360 custom-message
layout as a `uint64 timebase` plus per-point `uint32 offset_time`; use the
installed driver version and record its commit/version in the manifest. See the
[official driver documentation](https://github.com/Livox-SDK/livox_ros_driver2).

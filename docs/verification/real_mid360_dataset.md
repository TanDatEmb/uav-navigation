# Real Mid-360 acquisition, verification, and replay

M1 real-data evidence must originate from a physical Mid-360 capture or a
redistributable official download with documented origin and license. Simulation
output, point-index timing, converted PointCloud2, synthetic CSV, and an
unverified third-party file are not substitutes.

The required raw ROS interfaces are `/livox/lidar` as
`livox_ros_driver2/msg/CustomMsg`, `/livox/imu` as `sensor_msgs/msg/Imu`, plus
`/tf` and `/tf_static`. Preserve `CustomMsg.timebase` and each point's
`offset_time` in the rosbag; this is why the workflow records raw messages
instead of converting them. Livox documents this CustomMsg layout and supplies
Mid-360 launch modes in its [official ROS Driver 2 repository](https://github.com/Livox-SDK/livox_ros_driver2).

```bash
# physical device only; fails closed if a required type is absent/wrong
bash tools/datasets/record_mid360_bag.sh data/mid360_real/<capture-id>

# hash all recorded bytes and declare source/driver/license provenance
python3 tools/datasets/create_mid360_manifest.py data/mid360_real/<capture-id> \
  --dataset-id <capture-id> --source-kind physical_capture \
  --driver-version <driver-tag-or-commit>

# verify hashes and inspect actual rosbag topic metadata
python3 tools/datasets/verify_mid360_dataset.py data/mid360_real/<capture-id> \
  # checks hashes and ros2 bag metadata by default

# original topics and message types are replayed unchanged
bash tools/datasets/replay_mid360_bag.sh data/mid360_real/<capture-id>
```

The repository intentionally contains no large bag and does not download one:
there is no verified source URL, checksum, dataset license, or size-safe
official artifact currently recorded. This is a blocker for a real-dataset M1
run, not a reason to manufacture data. The generated `manifest.json` is the
checksum/provenance hand-off, and the validation report must name its SHA-256.

The current dependency-light `fast_lio_tools` CSV reader is not a Livox CustomMsg
reader and does not itself prove preservation of `timebase`/`offset_time`.
Consequently, successful acquisition/replay verification is necessary but does
not yet prove offline estimator replay until a raw-bag adapter is implemented.

# FAST-LIO

This repository provides a ROS 2 Jazzy LiDAR–IMU estimator. The
ROS-independent `fast_lio_core` owns synchronization, deskew, IKFoM correction,
and the local ikd-Tree registration map. `fast_lio_ros` is a thin message,
frame, time, QoS, and publication boundary. `fast_lio_tools` runs the same
pipeline offline.

## Modes and configuration

- `mid360_real.yaml`: real Mid-360 topics, fixed production extrinsic, sensor
  time, and per-point timing.
- `mid360_px4_gazebo.yaml`: PX4 Gazebo PointCloud2 topics and explicit
  `simultaneous_scan` timing.
- `mid360_aist_replay.yaml`: verified AIST PointCloud2 timestamp contract, extrinsic, topics,
  and reliable replay QoS.

All three profiles are under
`src/navigation_estimator/fast_lio_ros/config/`. Scenario names belong in
dataset metadata, not estimator configuration.

All canonical profiles use `base_link -> livox_frame -> livox_imu_frame`.
FAST-LIO continues to publish the transitional dynamic edge
`odom -> livox_imu_frame`; P0.1 does not introduce `odom -> base_link`.

## Build and test

```bash
make clean
make build MODE=release
make test
make check
make vendor-check
```

The vendor check is read-only. It verifies the pinned upstream SHAs, frozen
file inventory, current file hashes, and the three documented upstream
differences.

## Dataset workflow

The only dataset entrypoint is `tools/data.py`. There is one active catalog
entry: `aist-mid360-drive`. Archives, prepared bags, and provenance live under
`$UAV_NAV_DATA_HOME`, or `$XDG_DATA_HOME/uav-nav` by default.

```bash
make data-list
make data-fetch DATASET=aist-mid360-drive
make data-check DATASET=aist-mid360-drive
make data-smoke DATASET=aist-mid360-drive
make data-run DATASET=aist-mid360-drive
make data-replay DATASET=aist-mid360-drive RATE=1.0
make data-view DATASET=aist-mid360-drive
make data-report DATASET=aist-mid360-drive
make data-test DATASET=aist-mid360-drive
```

`data-replay` automatically closes the FAST-LIO node, rosbag recorder,
rosbag player, diagnostics collector, and RViz after the bag drains or the
run is interrupted. A replay is limited to 900 seconds by default; tune it
with `REPLAY_TIMEOUT`, `REPLAY_READINESS_TIMEOUT`, and
`REPLAY_DRAIN_TIMEOUT`. If the terminal or replay wrapper is killed, clean up
only the repository-owned registered process groups with:

```bash
make data-replay-stop
# alias:
make replay-stop
```

Use `DRY_RUN=1 make data-replay-stop` to inspect the groups before signaling
them. The cleanup uses the per-run process-group registry and never performs a
global `pkill` of unrelated ROS or RViz processes.

`make data-replay` opens the canonical FAST-LIO RViz view by default. For CI,
remote shells, and other headless runs, set `ENABLE_RVIZ=0`. The `data-test`
and `runtime-repro` workflows always remain headless.

`data-fetch` downloads with resume support, verifies the catalog checksum,
extracts into a temporary directory, writes a LiDAR/IMU-only ROS bag, records
provenance, and atomically installs the prepared dataset. `data-check`
validates the catalog, archive checksum, bag topics and message types,
canonical config, provenance counts, canonical sensor header frames, and the
Git raw-data guard. Prepared schema v1 bags are rejected by `data-check` and
rebuilt atomically by the next `data-fetch`; the normalizer changes only
`header.frame_id` and records source/canonical frame IDs in `status.json`.

## ROS topics

- `/lio/odometry_corrected`: corrected `nav_msgs/msg/Odometry`, published only
  after a successful LiDAR correction. Its pose is the converted base pose in
  `odom`; `header.frame_id=odom`, `child_frame_id=base_link`. Linear and
  angular twist are expressed in the `base_link` child frame.
- `/lio/odometry_propagated`: high-rate propagated `nav_msgs/msg/Odometry`
  produced from the latest committed correction and the IMU stream. It is a
  low-latency prediction between corrected outputs, not an independent global
  estimate; it uses the same `odom`/`base_link` contract when enabled, and
  continuity and correction-age status are reported in `/lio/diagnostics`.
- `/lio/registered_points`: corrected registered scan in `odom`.
- `/lio/local_map`: local registration-map snapshot in `odom`.
- `/lio/diagnostics`: estimator and transport counters, processing
  percentiles, queue depth, lag, and drops.
- `/tf` and `/tf_static`: configured frame transforms.

`trajectory.csv` has the same corrected-only semantic. P0.5R now exports pose
and conditional base-link twist covariance from the full 23-state covariance
through the shared corrected/propagated serializer. Pose covariance is
expressed in `odom`; twist covariance is expressed in `base_link`. A resolved,
bias-corrected gyro at the exact output epoch is part of the conditional
velocity semantics. The deferred per-sample gyro covariance source is not a
gate, and angular-rate covariance remains outside this contract.
The real Mid-360 profile uses `/livox/imu` and `/livox/lidar`, `livox_custom`
per-point sensor timing, the pinned hardware extrinsic, bounded runtime
queues, and propagated odometry enabled at 50 Hz. This configuration is ready
for a real-hardware test; repository acceptance does not claim that hardware
has been validated.

## Artifacts

Offline artifacts are written below
`.artifacts/datasets/<dataset>/<run-id>/`:

```text
run.json
summary.json
diagnostics.csv
trajectory.csv
corrections.csv
local_map.pcd
stdout.log
stderr.log
```

`data-report` reads the newest artifact without rerunning the estimator and
prints timing distributions, acceptance counts, deskew/correction counts, and
map size. It may add `report.json` beside the artifact. ROS replay artifacts
also contain replay diagnostics and the recorded ROS output bag.

The local map is a bounded registration structure in `odom`, not a persistent
world model. Soft point pressure runs budgeted crop work; hard pressure must
recover to the configured target or report a hard recovery failure.

`output.publish_local_map` controls the periodic debug snapshot end to end.
When enabled, the pipeline takes a snapshot on the first successful LiDAR
correction and then every 10 successful corrections, and the ROS adapter
publishes it. Rejected or failed corrections do not advance this fixed
cadence. When disabled, the runtime does not snapshot, copy, sort, or publish
the debug map; `snapshot_us` and the snapshot point count remain zero. The
offline final `local_map.pcd` is still obtained once from the registration map
after processing and is unaffected by this flag.

The two voxel parameters serve different stages. The
`preprocessing.scan_voxel_size_m` filter reduces each incoming scan before
registration, trading scan detail for per-correction work.
`mapping.registration_map.voxel_size_m` controls insertion density in the
registration map, trading map detail for memory and neighbor-search cost.
Canonical profiles must specify both explicitly; changing one does not change
the other.

## Known limitations

- Propagated odometry is a high-rate prediction between corrections; corrected
  odometry remains the authoritative LiDAR-corrected output.
- No global SLAM or persistent global map.
- No loop closure.
- No relocalization or global localization.
- No flight-controller control integration.
- No multi-dataset accuracy acceptance; validation currently uses the AIST
  dataset, which has no ground truth.

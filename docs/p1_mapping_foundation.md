# P1 navigation mapping foundation

Verdict at this checkout: `P1 NOT YET ACCEPTED`.

The implementation is code-complete for the local foundation and passes the
clean workspace build plus unit tests. Full acceptance remains open until the
external AIST bag, headless SITL prerequisites, and interactive RViz checklist
are run on the target machine. No runtime threshold is claimed without those
artifacts.

## Locked contract

At HEAD `38e665df2e87e8ec95b0ae79c15b22c4e3416eaa`, LiDAR enters
`FastLioNode::onLidar` or `onLivoxCustom`, then bounded input queues feed
`FastLioPipeline::pushLidar`. `MeasurementSynchronizer` canonicalizes the scan
interval and rejects timestamp regressions. `ScanDeskewer::deskew` uses the
configured scan-end reference; the resulting `LidarScan` is expressed in
`livox_frame` at `scan.end_time`. `PointCloudPreprocessor` owns finite/range
and registration filtering; the accepted AIST registration voxel remains
`preprocessing.scan_voxel_size_m: 0.9`.

Correction success is gated by `ProcessResult::hasCorrectedOutput()` after
IKFoM correction. Corrected odometry is serialized as `lio_odom -> base_link`.
Static sensor geometry is owned by `uav_description` and resolved from
`/tf_static`.

## Architecture

```text
deskew once at FAST-LIO scan end
   +--> registration voxel 0.9 m --> IKFoM + ikd-Tree
   +--> shared immutable handoff --> /lio/deskewed_points
                                      + /lio/odometry_corrected
                                      --> exact nanosecond pairing
                                      --> static-TF frame transform
                                      --> independent rog_map_ros worker
                                      --> bounded occupancy/free/unknown map
```

`fast_lio_core` has no dependency on either mapping package. `rog_map_core`
contains the independent voxel container, raycasting, inflation,
sliding-window eviction, validity state and the P2 query boundary.
`rog_map_ros` owns ROS subscriptions, exact caches, TF conversion, lifecycle
and diagnostics. Its depth-1 latest-only queue and worker are outside the
FAST-LIO process.

## Topics and license

`/lio/deskewed_points` is published only after successful corrected output.
Its stamp is the integer ROS `sec/nanosec` scan-end stamp, frame is
`livox_frame`, points are finite and range-gated, and it never uses the
registration voxel. The output worker owns the only navigation-output voxel
filter (`0.80 m` in the dataset/simulation profiles, selected after the AIST
timing check); mapping does not filter again. This remains finer than the
localization registration voxel (`0.90 m`). The bounded navigation map uses
`0.30 m` voxels in production profiles to keep the update budget finite. QoS
is best-effort, volatile, keep-last 1.

For RViz/debug sessions, `rog_map_ros` can additionally publish occupied and
inflated voxel centers as `sensor_msgs/msg/PointCloud2` on
`/rog_map/occupied_voxels` and `/rog_map/inflated_voxels`. This is disabled by
default and enabled by the replay/interactive launch path; it does not change
map insertion, pruning, or planner queries.

ROG-Map was inspected at `https://github.com/hku-mars/ROG-Map`, main SHA
`df59c21304579a13fb3875100f8ce9523ba379a0`. Its `LICENSE` is GPL-3.0. No
ROG-Map or SUPER source was copied or linked into this Apache-2.0 workspace;
`rog_map_core` is original project code, not a relicensed upstream derivative.

## Verification and remaining acceptance

```bash
source /opt/ros/jazzy/setup.bash
PARALLEL_WORKERS=1 MAKE_JOBS=1 python3 tools/runtime/build.py build
python3 tools/runtime/build.py test
python3 -m unittest discover -s tools/tests -p 'test_*.py' -v
python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v
```

P1 tests cover FREE/OCCUPIED/UNKNOWN, raycasting, inflation, sliding,
validity, exact integer timestamp pairing, bounded duplicate/capacity handling,
and numeric identity/yaw/extrinsic transforms. AIST OFF/publisher/full
comparison, memory plateau, headless SITL and interactive RViz evidence remain
required for a PASS verdict. The available AIST full run at 1.0x is recorded at
`.artifacts/runtime/dataset-20260810T023230-61232`: 55435 IMU, 2756 corrected
odom, 2756 paired/integrated mapping observations, zero mapping drops or
timestamp mismatches, map queue bound 1, 72 shifts, allocated voxels 204999,
and mapping update p99 72.4 ms against the measured 100 ms scan period. This is
strong full-path evidence, but it does not replace the missing OFF/publisher
comparison or the two SITL validation levels required by the prompt.

P2 may use `NavigationMap::query`, `isInflatedOccupied`, `resolution`,
`localBounds`, and `validity`. Inputs are always `lio_odom`; `kOutside` and
`kUnknown` are not free, and only `kActive` is planner-safe.

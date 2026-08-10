# P1 navigation mapping foundation

Verdict at this checkout: `P1 NOT YET ACCEPTED`.

The implementation is code-complete for the local foundation and passes the
clean workspace build plus unit tests. Full acceptance remains open until the
external AIST bag, headless SITL prerequisites, and interactive RViz checklist
are run on the target machine. No runtime threshold is claimed without those
artifacts.

## Locked contract

At reviewed baseline `12264df520221cb5668215dda371d44349982cef`, LiDAR enters
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

For RViz/debug sessions, `rog_map_ros` can publish derived visualization views:
`/rog_map/occupied_voxels` contains centers whose log odds exceed the occupied
threshold, `/rog_map/inflated_voxels` is the full derived keep-out set, and
`/rog_map/inflation_surface` is its six-connected boundary. The core does not
store a second inflated map: planner collision checks use
`NavigationMap::query()` and `NavigationMap::isInflatedOccupied()`, while the
topics are visualization-only. `/rog_map/local_bounds` is the current
axis-aligned sliding-window box in `lio_odom`. Visualization is disabled in
headless/production profiles and uses lazy, subscriber-gated, latest-only work
on a separate worker; it does not change map insertion, pruning, or planner
queries.

The navigation observation contract uses `mapping.observation.min_range_m:
0.50`; points below it produce neither FREE nor OCCUPIED updates. Finite hits
within maximum range produce a FREE ray and OCCUPIED endpoint. Points beyond
maximum range produce only a FREE ray clipped at that range. The map
resolution is `0.30 m`; the navigation observation voxel remains `0.80 m`,
while FAST-LIO registration remains `0.90 m`.

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

P1 tests cover FREE/OCCUPIED/UNKNOWN, finite-hit versus truncated-ray versus
minimum-range behavior, spherical inflation, six-connected surface
extraction, sliding validity, exact integer timestamp pairing, bounded
duplicate/capacity handling, and numeric identity/yaw/extrinsic transforms.
Post-change AIST OFF is recorded at
`.artifacts/runtime/dataset-20260810T072954-26445/REPORT.md`; AIST full with
RViz subscribers is recorded at
`.artifacts/runtime/dataset-20260810T073455-27290/REPORT.md`. Both preserve all
2756 corrections and full mapping preserves 2756 paired/integrated observations
with queue bound 1 and map p99 below 100 ms. Headless SITL artifacts are
recorded in `docs/p1_acceptance_results.md`; they failed due to simulator
external-odometry freshness/OFFBOARD loss. Manual yaw/frame and lifecycle
clear evidence is still required, so the verdict remains NOT YET ACCEPTED.

P2 may use `NavigationMap::query`, `isInflatedOccupied`, `resolution`,
`localBounds`, and `validity`. Inputs are always `lio_odom`; `kOutside` and
`kUnknown` are not free, and only `kActive` is planner-safe.

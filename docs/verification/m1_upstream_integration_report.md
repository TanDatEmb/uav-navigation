# M1 upstream integration report

Date: 2026-07-28  
Branch: `fix/m1-upstream-fastlio2`  
Baseline: `49b2fba1406653cbf29b9328e4309745c5a1d53e`

## Result

The software integration gates for upstream IKFoM, upstream ikd-Tree, and the
typed Livox Mid-360 `CustomMsg` boundary are implemented and tested. M1 as a
sensor-system acceptance milestone is **not accepted**: no real Mid-360 bag was
acquired or replayed, and the live Gazebo ROS bridge could not run in this
environment. These are recorded failures/blockers, not conditional passes.

## Package tree

```text
src/navigation_estimator/
  fast_lio_core
  fast_lio_ros
  fast_lio_tools
  ikd_tree_vendor
  ikfom_vendor
  livox_ros_driver2_interface  (upstream message definitions only)
src/navigation_bringup
src/uav_description
src/uav_simulation
tools/benchmarks
tools/datasets
tools/simulation
```

## Pinned upstream code in use

| Component | Repository and commit | Runtime evidence |
| --- | --- | --- |
| IKFoM | `hku-mars/IKFoM` `59cfc095ca74425f9b330c7c04a5d74f68c6dd62` | `IkfomFilter` is `esekfom::esekf`; prediction and iterated update call the vendored API |
| FAST-LIO reference | `hku-mars/FAST_LIO` `7cc4175de6f8ba2edf34bab02a42195b141027e9` | state ordering and process functions are ported with source annotation |
| ikd-Tree | `hku-mars/ikd-Tree` `c0e36a16b6e4d557d3783b16911207f6398dd478` | production map owns `KD_TREE` and calls build, incremental add, nearest search, and box deletion |
| Livox messages | `Livox-SDK/livox_ros_driver2` `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` (1.2.6) | byte-matched message definitions and typed adapter; this repository does not claim to ship the hardware driver executable |

Licenses, source URLs, pinning and distribution caveats are in
`THIRD_PARTY_NOTICES.md` and each vendor package's provenance file.

## Runtime semantics

- State/covariance dimension is the FAST-LIO2 IKFoM 24-dimensional nominal,
  23-DoF tangent state including SO(3) pose/extrinsic and S2 gravity.
- Extrinsics are fixed by default and are not updated by the LiDAR measurement.
- Rejected LiDAR updates restore the predicted IKFoM state and covariance.
- The old custom iterated filter, propagation model and manifold box
  implementation were removed from production.
- The old exhaustive voxel registration backend was removed. Production kNN
  uses upstream `Nearest_Search`; local cropping uses upstream box deletion.
- `ProcessResult` separates predicted state from an optional corrected estimate.
  Odometry, TF, registered scan and map publication require a valid correction.
- Livox point time is `timebase + offset_time` with integer nanoseconds.
  Invalid timing/clock-domain inputs fail closed; there is no REAL-to-SIM
  fallback.

## Verification

Final commands:

```text
colcon build --symlink-install
Summary: 10 packages finished
elapsed=4.78 s, max_rss=551568 KiB

colcon test
Summary: 105 tests, 0 errors, 0 failures, 0 skipped
elapsed=1.99 s, max_rss=136056 KiB
```

Coverage includes upstream vendor smoke tests, IKFoM prediction/update
transactionality, no-double-ingestion, timestamp/clock faults, deskew,
differential kNN against a brute-force test oracle, incremental insertion,
box deletion, Livox field/timing conversion, QoS, output gating, and simulation
asset contracts.

The upstream-only ikd-Tree benchmark smoke completed; hashes and reproduction
commands are in `docs/verification/ikd_tree_benchmark.md`. It is not Raspberry
Pi 5 performance evidence.

## Unmet acceptance evidence

| Gate | Status | Required next action |
| --- | --- | --- |
| Real Mid-360 dataset | **FAIL / not run** | Acquire a lawful raw ROS 2 bag containing `livox_ros_driver2/msg/CustomMsg` and IMU, generate checksum manifest, replay it through the typed node, and archive metrics/trajectory/map comparison |
| Live Gazebo harness | **BLOCKED by environment** | Install ROS Jazzy `ros_gz_bridge` and `xacro`, run the static/yaw/axis/vertical/square scenarios, and archive checker output |
| Measured extrinsic | **FAIL / placeholder** | Measure and version the Mid-360-to-IMU transform before hardware acceptance |
| Pi 5 benchmark | **NOT RUN** | Run scan-stage and ikd-Tree benchmark on the target with thermal/CPU metadata |

The public Zenodo/Mendeley datasets discovered during investigation were not
downloaded because their size, message/topic suitability, and redistribution
conditions were not verified within this run. A manifest template and
fail-closed verifier are provided; they are not evidence of replay.

## Commit sequence

```text
6105c77 docs: audit M1 upstream integration gaps
740b5d1 build: vendor pinned IKFoM source
26e482a build: vendor pinned ikd-Tree source
b1cbc93 refactor: replace custom filter with IKFoM estimator
a99a085 refactor: remove duplicate estimator machinery
13dfaa6 feat: use upstream ikd-tree registration map
96d144e feat: add complete Gazebo M1 sensor harness
47417fe feat: add Mid-360 dataset ingestion and manifest
8c8fc32 feat: add ikd-tree benchmark and license notices
b794b77 feat: integrate pinned Livox message boundary
bfadc4f fix: gate ROS outputs on corrected estimates
```

No planner, PX4 fusion, collision avoidance, safety logic, or controller was
added.

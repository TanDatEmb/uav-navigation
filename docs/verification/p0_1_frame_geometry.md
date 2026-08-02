# P0.1 Frame Naming and Geometry

## Result

PASS

## Revisions

- Parent: `134d3ee88b48224a3eb272340fc4594d83538388`
- Branch: `fix/p0.1-frame-geometry`
- Commits: `3772105`, `9878b47`, `50e5efd`, `879f4c1`, and the final
  documentation commit at `HEAD`; no push

## Scope

P0.1 updates the physical frame contract and the corresponding ROS, URDF,
Gazebo, driver, profile, dataset-preparation, test, and documentation
boundaries. The estimator process model, IKFoM state, registration, map,
timestamp policy, topic names, and point-transform implementation are outside
this task.

The frozen parent is `134d3ee88b48224a3eb272340fc4594d83538388`, the corrected
P0.0 report commit based on frozen implementation baseline
`29fa4752bd7b54e78cd0a2a81e42292c1611d290`. Work is on
`fix/p0.1-frame-geometry`.

## Canonical contract

The physical sensor tree is:

```text
base_link -> livox_frame -> livox_imu_frame
```

The factory nominal transform `T_L_I` has parent `livox_frame`, child
`livox_imu_frame`, translation `[0.011, 0.02329, -0.04412] m`, and identity
rotation. FAST-LIO consumes the exact inverse `T_I_L`:

```text
translation_imu_lidar: [-0.011, -0.02329, 0.04412]
rotation_imu_lidar_xyzw: [0, 0, 0, 1]
```

The core test covers origin, unit-axis, and asymmetric-point round trips in
both directions. The real profile labels this as factory nominal, not unit
calibration. The AIST profile retains `[-0.019391, -0.000278, 0.080926] m` as
a dataset-specific estimator extrinsic and documents that it must not be
generalized without calibration provenance.

The SIM model keeps the vehicle mount `[0, 0, 0.28] m` as
`base_link -> livox_frame`. Its internal IMU sensor pose is the nonzero factory
nominal translation above; vehicle mounting and internal LiDAR/IMU geometry are
separate quantities.

| Transform | Translation | Rotation | Source/provenance |
|---|---:|---|---|
| `base_link -> livox_frame`, SIM mount | `[0, 0, 0.28] m` | identity | vehicle SDF mount |
| `T_L_I`, factory | `[0.011, 0.02329, -0.04412] m` | identity | MID-360 factory nominal |
| `T_I_L`, factory | `[-0.011, -0.02329, 0.04412] m` | identity | exact inverse of `T_L_I` |
| `T_I_L`, AIST | `[-0.019391, -0.000278, 0.080926] m` | identity | AIST dataset-specific |

## Runtime and launch policy

FAST-LIO's transitional dynamic edge remains `odom -> livox_imu_frame`.
`navigation_bringup/fast_lio.launch.py` defaults `publish_sensor_frames` to
false with the required P0.3 note. The standalone sensor-frame launch remains
available, requires an explicit vehicle mount, and cannot silently substitute
a zero mount. No `odom -> base_link` edge is introduced in P0.1.

The Livox driver now accepts distinct `frame_id` and `imu_frame_id` parameters,
rejects empty or aliased values, and assigns the LiDAR and IMU message headers
from that contract. MID-360 launch files keep all existing topic names.

## Dataset preparation contract

`tools/data.py` uses prepared schema version 2. It selects the same LiDAR and
IMU topics, deserializes only the selected sensor messages, rewrites only
`header.frame_id` to `livox_frame` or `livox_imu_frame`, and writes the original
bag timestamp. `status.json` records the source frame-ID counts, canonical
frame IDs, normalized counts, selected counts, and the conversion policy.

`data-check` rejects schema-v1 prepared bags. A subsequent `data-fetch` rebuilds
an old prepared directory atomically from the verified source archive. No
source archive was modified. The rebuilt prepared bag has schema v2.

## Driver contract

| Message | Topic | `frame_id` |
|---|---|---|
| LiDAR `PointCloud2`/`CustomMsg` | existing Livox topic | `livox_frame` |
| IMU `sensor_msgs/msg/Imu` | existing Livox topic | `livox_imu_frame` |

## Profile contract

| Profile | `frames.imu` | `frames.lidar` | `input.imu_frame` | `input.lidar_frame` | `T_I_L` source |
|---|---|---|---|---|---|
| Real | `livox_imu_frame` | `livox_frame` | `livox_imu_frame` | `livox_frame` | factory nominal inverse |
| AIST | `livox_imu_frame` | `livox_frame` | `livox_imu_frame` | `livox_frame` | dataset-specific |
| PX4 Gazebo | `livox_imu_frame` | `livox_frame` | `livox_imu_frame` | `livox_frame` | factory nominal inverse |

## Transitional TF policy

- Dynamic edge: `odom -> livox_imu_frame`.
- Static subtree: validated separately as `base_link -> livox_frame -> livox_imu_frame`.
- Why not combined: canonical FAST-LIO launch defaults static publishing off, avoiding duplicate TF authority.
- Deferred task: P0.3 makes `odom -> base_link` the sole dynamic edge.

## SIM geometry

- Mount: `base_link -> livox_frame = [0, 0, 0.28] m`.
- Internal geometry: LiDAR pose zero, IMU pose `[0.011, 0.02329, -0.04412] m`.
- Resolved validation: `gz sdf -p` passed for `lidar_mid360/model.sdf`; resolved output retained distinct poses and frame IDs. The full x500 include requires the external PX4 `x500` model and was exercised by the PX4 session.
- Session report: `.artifacts/simulation/px4-mid360-20260802-175654/REPORT.md`.
- Cleanup: `make sim-px4-mid360-stop` exited zero; no owned processes remained.

## Dataset normalization

- Source headers: LiDAR `livox_frame`, IMU `livox_frame`.
- Canonical headers: LiDAR `livox_frame`, IMU `livox_imu_frame`.
- Modified fields: only sensor message `header.frame_id`.
- Unmodified fields: topic name, message type, bag timestamp, message header timestamp, numeric sensor/point payload, order, and counts.
- Prepared schema: v1 rejected/rebuilt; v2 installed atomically.
- Message counts: LiDAR `2772 -> 2772`, IMU `55435 -> 55435`.
- Provenance: `status.json` records source frame-ID counts, canonical frames, normalized counts, modified fields, tool version, and derived bag hash.
- Prepared bag evidence: `/home/letandat/snap/code/253/.local/share/uav-nav/datasets/aist-mid360-drive/status.json` and `lio/`.

## Test results

| Command | Exit | Result |
|---|---:|---|
| `make build MODE=release` | 0 | pass |
| `make test MODE=release` | 0 | pass; CTest and Python suites |
| `make check` | 0 | pass; 234 tests, no errors/failures |
| `make vendor-check` | 0 | pass |
| focused frame/dataset Python tests | 0 | 13 passed |
| Xacro source/launch contract test | 0 | XML and launch-argument checks pass; ROS `xacro` executable is unavailable in this environment |
| `make data-fetch DATASET=aist-mid360-drive` | 0 | schema v2 rebuild |
| `make data-check DATASET=aist-mid360-drive` | 0 | counts/type/checksum pass |
| `make data-replay ... RATE=1.0 ENABLE_RVIZ=0` | 0 | replay and cleanup pass |
| `make sim-px4-mid360-headless AUTO_SNAPSHOT=1` | 0 | session start pass |
| `make sim-px4-mid360-check` | 0 | topic/rate/diagnostic snapshot pass |
| `make sim-px4-mid360-stop` | 0 | cleanup pass |

## Runtime comparison

### Dataset

| Metric | P0.0 | P0.1 | Delta |
|---|---:|---:|---:|
| processing-lag threshold | 500 ms | 500 ms | unchanged |
| processing lag predicate | triggered | not triggered | improved; no regression |
| maximum queue high-water | 169 | 46 | -123 |
| received/processed LiDAR | 2772/2772 | 2772/2772 | unchanged |
| received/processed IMU | 55435/55435 | 55435/55435 | unchanged |
| final queue depth | 0 | 0 | unchanged |
| drop / overflow | 0 / 0 | 0 / 0 | unchanged |

P0.1 replay artifact: `.artifacts/datasets/aist-mid360-drive/134d3ee-replay-1.0x-20260802T105110783040Z/summary.json`.

### SIM

| Metric | P0.0 | P0.1 | Delta |
|---|---:|---:|---:|
| LiDAR raw frame | `mid360_lidar_frame` | `livox_frame` | contract corrected |
| IMU raw frame | `mid360_imu_frame` | `livox_imu_frame` | contract corrected |
| corrected odometry child | `mid360_imu_frame` | `livox_imu_frame` | contract corrected |
| process crashes | 0 | 0 | unchanged |
| finite ratio | 0.404785 | 0.404785 | unchanged |
| corrected odometry / registered points | present | present | no loss |

P0.1 SIM evidence is under `.artifacts/simulation/px4-mid360-20260802-175654/`.

## Frozen baseline findings

The P0.0 runtime findings remain historical baseline observations:

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate triggered | Not addressed by P0.1 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning triggered | Not addressed by P0.1 |

Neither finding is called a P0.1 regression without a measured before/after
runtime comparison. P0.1 does not investigate or repair either finding.

## Existing findings

- `P0.0-F01`: dataset processing-lag predicate was a frozen-baseline finding; P0.1 did not tune performance. The P0.1 run did not trigger it and showed no drops/overflow or queue accumulation.
- `P0.0-F02`: SIM finite-point warning remains at the same finite ratio and is retained for later investigation. No root cause is asserted.

## New findings

- None.

## Acceptance checklist

- [x] Branch starts from `134d3ee`
- [x] Working tree initially clean
- [x] Core frame identifiers use `livox_frame` and `livox_imu_frame`
- [x] Static Xacro tree is `base_link -> livox_frame -> livox_imu_frame`
- [x] No zero mounting fallback for static geometry
- [x] Factory `T_L_I` modeled and inverse tested
- [x] Real uses factory nominal inverse
- [x] SIM uses factory nominal inverse
- [x] AIST retains dataset-specific extrinsic
- [x] Livox driver emits distinct LiDAR/IMU frame IDs
- [x] MID360/MID360S launches pass both frame IDs
- [x] AIST prepared bag has distinct canonical frame IDs
- [x] Dataset normalization has provenance and schema invalidation
- [x] SIM LiDAR and IMU are not colocated
- [x] Mount and internal geometry are separate
- [x] Canonical profile frame validation rejects aliased frames
- [x] Xacro source and launch argument contract is covered by focused tests
- [x] Runtime Xacro expansion remains an environment limitation: the ROS `xacro` package/executable is unavailable
- [x] Canonical FAST-LIO launch avoids duplicate TF authority
- [x] Dynamic output remains at IMU origin
- [x] Registered points and map remain in `odom`
- [x] Build, test, check, and vendor check pass
- [x] Dataset raw frames and counts pass validation
- [x] SIM raw frames and runtime observations pass validation
- [x] P0.2 not started
- [x] Working tree clean after commit

## Conclusion

P0.1 status: PASS

The canonical frame and geometry contract is implemented at the core, ROS,
driver, description, simulation, profile, and dataset boundaries. P0.0-F01 and
P0.0-F02 remain pre-existing baseline findings and are not attributed to P0.1.
P0.1 does not introduce `odom -> base_link`, process-model changes, timestamp
changes, topic changes, or runtime tuning.

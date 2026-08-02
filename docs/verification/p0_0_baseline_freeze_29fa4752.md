# P0.0 Baseline Freeze — 29fa4752

## Result

PASS

P0.0 is a baseline-freeze task. The frozen revision, build, unit/static tests, diagnostics, topic/TF snapshots, and PX4 MID-360 session evidence were captured successfully. Dataset and SIM runtime findings are reported separately below; they do not invalidate the freeze. No source, configuration, launch, SDF/URDF, Makefile, test, catalog, driver, or `.gitignore` file was changed.

## P0.0 task status

PASS

- Exact frozen HEAD was captured.
- Release build, `make test`, `make check`, and `make vendor-check` passed.
- Dataset and SIM diagnostics, topic snapshots, and TF snapshot were captured.
- The working tree was clean before and after the report commit.
- No estimation logic or runtime behavior was changed.
- P0.1 was not started.

## Frozen revision

- Repository: `https://github.com/TanDatEmb/uav-navigation`
- Branch before work: `main`
- Branch after work: `chore/p0.0-baseline-freeze`
- Frozen commit: `29fa4752bd7b54e78cd0a2a81e42292c1611d290`
- Commit message: `fix(runtime): preserve transport state across diagnostics statuses`
- Working tree before: clean (`## main...origin/main`)
- Working tree after commit: clean; only this report is tracked by this task
- Baseline was checked after `git fetch origin --prune`, `git switch main`, and `git pull --ff-only origin main`.

## Environment

- OS: Ubuntu 24.04.4 LTS
- ROS: Jazzy
- Compiler: GCC/G++ 13.3.0
- RMW: `RMW_IMPLEMENTATION` unset; ROS default was used
- Gazebo: Gazebo Sim 8.11.0
- PX4: `/home/letandat/Dev/Autopilot`, `build/px4_sitl_default/bin/px4` executable
- Workspace: `/home/letandat/Dev/uav-navigation`
- Build mode: `MODE=release` (`RelWithDebInfo`)
- Python: 3.12.3

Environment and Git command logs are under `.artifacts/baseline/p0_0_29fa4752/{git,environment}/`.

## Build and tests

| Command | Exit code | Duration | Result | Artifact |
|---|---:|---:|---|---|
| `source /opt/ros/jazzy/setup.bash && make build MODE=release` | 0 | 91.158 s | PASS; all packages finished | `build/make_build.*` |
| `source /opt/ros/jazzy/setup.bash && make test MODE=release` | 0 | 21.266 s | PASS; 19 CTest targets and tool unittest groups passed | `tests/make_test.*` |
| `source /opt/ros/jazzy/setup.bash && make check MODE=release` | 0 | 0.731 s | PASS; 225 tests, 0 errors, 0 failures, 0 skipped | `tests/make_check.*` |
| `source /opt/ros/jazzy/setup.bash && make vendor-check` | 0 | 0.181 s | PASS; 18 files, 2 pinned SHAs, 3 documented patched files | `tests/make_vendor_check.*` |

The build emitted one existing benchmark warning: `m1_ikd_tree_benchmark` has no `install` target. It did not affect the exit code.

## Profile hashes

| File | SHA-256 |
|---|---|
| `Makefile` | `f1221a98ac88be2c6c60746ab92dd28316d9afdeee5755219b4d5ef3927c8b20` |
| `README.md` | `18d767ab882fe96eb91b07c607edd9211f531a43a5c673e38b46511f6ab8b3d6` |
| `docs/fast_lio.md` | `5cd0e7fefcac0570a8484255588f6e06a5d57a5b4711acbb26747d45a0cb8a09` |
| `datasets/catalog/aist-mid360-drive.yaml` | `a248793e1577aa1cadd43b97d76efe1982a8718077ae6194cebbd25340112a0e` |
| `src/navigation_estimator/fast_lio_ros/config/mid360_real.yaml` | `f3d089dff8bc59807112c974163e4be20ce1a368f420572b16eae637a4728070` |
| `src/navigation_estimator/fast_lio_ros/config/mid360_aist_replay.yaml` | `87203b323801bf9bbbfbf2aab8d172bec2cfc69d86fa1843476d4847a9a18730` |
| `src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml` | `eced9f2d2c02b7b6b3483166bfa914a591e62bd96b02e09de812ef071733687f` |
| `src/uav_simulation/models/lidar_mid360/model.sdf` | `a1546c4fa87102bfdf9d05be9a5b4ff0d3df6711cb5e17c0d175eea3692ffa0b` |
| `src/uav_simulation/models/x500_mid360/model.sdf` | `50ca21b9f379a23212cf9bba2f2c6abcddb36585dc38213cb62711f794cfd34f` |
| `src/uav_simulation/bridge/px4_mid360_bridge.yaml` | `4ff546490e6a9a888fb48acc79fbfbc2ec8b80a14258fb193b274db5c7b05ec1` |
| `tools/data.py` | `0d497a443a318cc9c915d2ee203cee192bde37c3ed88732f2b4c56fb79e02806` |
| `tools/runtime/build.py` | `8e0697a008328316ee7b4ac8fe113cf2c15b13db84c7fd5c0a1f73f03e0680d1` |
| `tools/runtime/ros_replay.py` | `6da0dcf7aad3a1a7b3bcc8bbffd2cacaf56a1ca6b8c0a3b248036bd58e00bb65` |
| `tools/simulation/start_px4_mid360_session.sh` | `f6f3a84a329fdccfe0b29b1ff514637e23de6554dc700ba517c50fd472d83c88` |
| `tools/simulation/config/px4_mid360_observer.yaml` | `4bc7486a0e417aec9e18080d63cfed13bcf118a742ca937b89327dd2f9403a67` |

The canonical hash output is `profiles/profile_hashes.sha256`.

## Dataset runtime acceptance

FAIL

### Finding ID: P0.0-F01 — Dataset processing-lag predicate triggered

- Predicate: the runtime latches `processing_lag_exceeded=true` when `latest_received_time_ns - latest_processed_time_ns > runtime.maximum_processing_lag_ms * 1,000,000`; the frozen AIST profile threshold is `maximum_processing_lag_ms: 500`, i.e. `500,000,000 ns`.
- Acceptance behavior: `tools/runtime/ros_replay.py` rejects any final state with `processing_lag_exceeded=true` as `maximum processing lag exceeded`.
- First triggering evidence: line 564 of `diagnostics.jsonl`, `collector_wall_time_ns=1785664871793854306`, with `processing_lag_ns=554,251,202 ns` (`554.251202 ms`) and `processing_lag_exceeded=true`.
- Trigger duration/evidence count: 10 diagnostic records carried `processing_lag_exceeded=true`; the maximum recorded lag was `638,232,363 ns` (`638.232363 ms`).
- Received/processed LiDAR: `2,772 / 2,772`.
- Final queue depth: input `0`, IMU `0`, LiDAR `0`.
- Maximum queue depth: `169`.
- Drop count: IMU `0`, LiDAR `0`.
- Overflow count: `0` (`overflow_detected=false`).
- Node exit code: `0`.
- Replay exit code: `0` for the internal bag player; the acceptance wrapper exited `2` because of this predicate.
- Evidence: `.artifacts/datasets/aist-mid360-drive/29fa475-replay-1.0x-20260802T095628434641Z/diagnostics.jsonl`, `summary.json`, `diagnostics_state.json`, and `.artifacts/baseline/p0_0_29fa4752/dataset/replay_*`.

Đây là hành vi tồn tại tại frozen baseline, không phải lỗi do P0.1 gây ra. Không điều tra hoặc sửa processing lag trong task hiệu chỉnh này.

- Dataset identity: `aist-mid360-drive`
- Dataset state: `ready`; checksum verified; tracked-blob guard OK
- Source bag: `/home/letandat/snap/code/253/.local/share/uav-nav/datasets/aist-mid360-drive/lio`
- New replay run: `.artifacts/datasets/aist-mid360-drive/29fa475-replay-1.0x-20260802T095628434641Z`
- Replay rate: `1.0`
- Commands: `make data-list`; `make data-check DATASET=aist-mid360-drive`; `make data-replay DATASET=aist-mid360-drive RATE=1.0 ENABLE_RVIZ=0`; `make data-report DATASET=aist-mid360-drive`
- Dataset input: 2,772 LiDAR and 55,435 IMU messages
- LiDAR received/processed: 2,772 / 2,772
- IMU received/processed: 55,435 / 55,435
- Synchronized scans: 2,771
- Correction attempted/success/failure: 2,760 / 2,759 / 1
- Deskew: sampled estimator diagnostics reported `deskew_applied=true`; a scalar final deskew count was not exposed by the new transport summary
- Drop/overflow: 0 / 0
- Invalid timestamp: 0
- Timestamp regression: 0
- Final input/IMU/LiDAR queue depth: 0 / 0 / 0
- Maximum queue depth: 169
- Processing lag: `processing_lag_exceeded=true` in the final transport snapshot; `processing_lag_ns=0` at final drain
- Processing duration: final mean scan processing `19169.442 us`; replay wall duration `290.243 s`
- Node exit: internal estimator return code 0
- Replay exit: internal bag-player return code 0; acceptance wrapper exit 2
- NaN/Inf: no nonfinite XYZ rejection (`reject_reason_nonfinite_xyz=0`); the replay summary does not contain a separate complete-output finite-point scalar
- Corrected output: present; output MCAP has 2,759 `/lio/odometry_corrected` messages
- Registered points: present; output MCAP has 2,759 messages
- Local map: present; output MCAP has 276 messages; sampled diagnostics reported map point count 53,276

Dataset runtime acceptance is `FAIL` only for Finding `P0.0-F01`. The final run otherwise drained completely with no drops, overflow, invalid timestamps, or timestamp regressions. Compared with the historical reference, LiDAR/IMU/synchronized/correction counts match; maximum queue depth is `169` versus `19`, and wall time is `290.243 s` versus approximately `122.43 s`.

`make data-report` returned 0 but selected an older `29fa475-run-20260801T152535311085Z` report artifact. The new run's `summary.json` and `diagnostics_state.json` are therefore the authoritative evidence copied under `dataset/replay_*`; the stale report selection is retained as observed evidence and not used for new-run counts.

## Dataset topic contract

The raw contract comes from the source bag; output contract and headers were captured by replaying only the recorded output MCAP after the run.

| Topic | Type | Publisher | frame_id | child_frame_id | Status |
|---|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | source bag player | `livox_frame` | — | present, 2,772 input messages |
| `/livox/imu` | `sensor_msgs/msg/Imu` | source bag player | `livox_frame` | — | present, 55,435 input messages |
| `/lio/odometry_corrected` | `nav_msgs/msg/Odometry` | FAST-LIO / recorded output player | `odom` | `imu_link` | present |
| `/lio/odometry_propagated` | — | — | — | — | expected absent because `propagated_odometry.enabled=false` |
| `/lio/registered_points` | `sensor_msgs/msg/PointCloud2` | FAST-LIO / recorded output player | `odom` | — | present |
| `/lio/local_map` | `sensor_msgs/msg/PointCloud2` | FAST-LIO / recorded output player | `odom` | — | present |
| `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | FAST-LIO / recorded output player | — | — | present; corrected/deskew/map diagnostics observed |

The complete topic/QoS snapshots are `dataset/source_bag_info.*`, `dataset/output_bag_info.*`, `dataset/output_topic_snapshot.*`, `dataset/topic_headers.*`, and `dataset/output_headers.*`.

## SIM runtime observations

### Finding ID: P0.0-F02 — SIM observer finite-point warning

- Severity: `WARNING`.
- Topic: `/lidar/points` (`sensor_msgs/msg/PointCloud2`).
- Sample count: `179` sampled scans in the session report.
- Sampled point count: `2,048` points; finite XYZ count `829`; finite ratio `0.40478515625`.
- Warning threshold: observer `finite_warn_threshold=0.5`; error threshold was `0.1`.
- Process crash: none (`process_crashes=0`).
- Corrected odometry: continued publishing; session report recorded `1,562` messages at `8.59 Hz`.
- Registered points: continued publishing; session report recorded `1,562` messages at `8.59 Hz`.
- Session report: `.artifacts/baseline/p0_0_29fa4752/simulation/px4-mid360-20260802-170339/REPORT.md`.
- Cleanup: completed successfully, exit `0`, with no scoped orphan process remaining.

SIM workflow completed and cleaned up successfully. The finite-point condition is retained as a baseline warning for later investigation. The artifacts do not prove a root cause, so none is asserted here.

- Workflow: canonical `make sim-px4-mid360-headless`
- PX4 path: `/home/letandat/Dev/Autopilot`
- Session ID: `px4-mid360-20260802-170339`
- Session artifact: `.artifacts/baseline/p0_0_29fa4752/simulation/px4-mid360-20260802-170339`
- Commands: `make sim-px4-mid360-headless ...`; `make sim-px4-mid360-check`; `make sim-px4-mid360-report`; `make sim-px4-mid360-stop`
- Runtime before cleanup: 179.0 s
- Process crash count: 0
- Cleanup: exit 0; no scoped orphan process remained
- `/clock`: active, 250.08 Hz
- `/lidar/imu`: 200.05 Hz
- `/lidar/points`: 10.00 Hz
- `/lio/odometry_corrected`: 8.59 Hz
- `/lio/registered_points`: 8.59 Hz
- `/lio/local_map`: 0.86 Hz
- `/lio/diagnostics`: 10.59 Hz
- Timestamp regressions: 0 for observed streams
- Session report: `simulation/.../REPORT.md`, overall `WARN`

The observer warning is material: sampled point clouds had finite ratio `0.404785`, with `0` NaN XYZ, `2` negative-Inf XYZ, and `1,217` positive-Inf XYZ in the sampled cloud. FAST-LIO diagnostics still showed successful corrections, `navigation_valid=true`, and zero nonfinite XYZ rejection. This is recorded as a baseline observation, not corrected in P0.0.

SIM runtime observation is `WARNING` for Finding `P0.0-F02`; it is not a P0.0 task failure.

## Simulation topic contract

| Topic | Type | Publisher | frame_id | child_frame_id | Rate |
|---|---|---|---|---|---:|
| `/clock` | `rosgraph_msgs/msg/Clock` | `px4_mid360_bridge` | — | — | 250.08 Hz |
| `/lidar/imu` | `sensor_msgs/msg/Imu` | `px4_mid360_bridge` | `mid360_imu_frame` | — | 200.05 Hz |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | `px4_mid360_bridge` | `mid360_lidar_frame` | — | 10.00 Hz |
| `/lio/odometry_corrected` | `nav_msgs/msg/Odometry` | `fast_lio` | `odom` | `mid360_imu_frame` | 8.59 Hz |
| `/lio/registered_points` | `sensor_msgs/msg/PointCloud2` | `fast_lio` | not captured in the single-message snapshot | — | 8.59 Hz |
| `/lio/local_map` | `sensor_msgs/msg/PointCloud2` | `fast_lio` | not captured in the single-message snapshot | — | 0.86 Hz |
| `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `fast_lio` | — | — | 10.59 Hz |

`/lio/odometry_propagated` and `/tf_static` were absent as observed; both are consistent with the current SIM profile/runtime. The raw single-message and QoS snapshot is `simulation/sim_topic_snapshot.*`.

## TF snapshot

| Parent | Child | Static/Dynamic | Authority | Translation | Rotation |
|---|---|---|---|---|---|
| `odom` | `mid360_imu_frame` | dynamic | `default_authority` | not emitted by `view_frames` | not emitted by `view_frames` |

The `tf2_tools view_frames` artifact reports average rate 9.286 Hz and a 2.8 s buffer. The sensor assembly mount remains the unmodified `[0, 0, 0.28] m` `base_link -> mid360_link` transform from the frozen SDF; the internal SIM LiDAR/IMU origin transform remains zero.

## Confirmed baseline inconsistencies

- B-01 — Real raw frames use the same name: `lidar_frame=livox_frame`, `imu_frame=livox_frame`.
- B-02 — AIST raw frames use the same name: `lidar_frame=livox_frame`, `imu_frame=livox_frame`.
- B-03 — SIM places IMU and LiDAR at the same origin in `model.sdf`.
- B-04 — SIM mounting is separate from internal extrinsic: `base_link -> mid360_link=[0,0,0.28] m` while internal LiDAR–IMU translation is zero.
- B-05 — Current corrected odometry is `odom -> imu_link` / `odom -> mid360_imu_frame`, not `odom -> base_link`.
- B-06 — ROS pose/twist covariance output is not fully exported.
- B-07 — Propagated profile differs: real enabled; AIST and SIM disabled.

These are observations only. No P0.1 frame, geometry, covariance, angular-velocity, publisher, or process-model change was started.

## Frozen baseline findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate triggered | Not addressed by P0.1 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning triggered | Not addressed by P0.1 |

## Acceptance checklist

- [x] Exact HEAD captured
- [x] Working tree clean
- [x] Build passed
- [x] `make test` passed
- [x] `make check` passed
- [x] vendor check passed
- [x] Topic snapshot captured
- [x] TF snapshot captured
- [x] Profile hashes captured
- [x] Dataset diagnostics captured
- [x] SIM diagnostics captured
- [x] No source/config changes
- [x] P0.1 not started

## Files changed

- `docs/verification/p0_0_baseline_freeze_29fa4752.md`

## Artifact locations

- `.artifacts/baseline/p0_0_29fa4752/`
- `.artifacts/datasets/aist-mid360-drive/29fa475-replay-1.0x-20260802T095628434641Z/`

Generated build/install/log/dataset/session artifacts are not tracked. The AIST output MCAP is retained at the external run path above and is intentionally not copied into the baseline evidence directory.

## Reproduction commands

```bash
source /opt/ros/jazzy/setup.bash
make build MODE=release
make test MODE=release
make check MODE=release
make vendor-check
make data-list
make data-check DATASET=aist-mid360-drive
make data-replay DATASET=aist-mid360-drive RATE=1.0 ENABLE_RVIZ=0
make data-report DATASET=aist-mid360-drive
make sim-px4-mid360-headless PX4_DIR="$HOME/Dev/Autopilot" ENABLE_RVIZ=0 AUTO_SNAPSHOT=1
make sim-px4-mid360-check
make sim-px4-mid360-report
make sim-px4-mid360-stop
```

## Final conclusion

P0.0 status: PASS

The repository baseline was successfully frozen and is reproducible.
The dataset processing-lag failure and SIM finite-point warning are retained
as pre-existing baseline findings. They do not invalidate the baseline freeze
and must not be attributed to subsequent frame-geometry changes without a
measured regression against this baseline.

Frozen baseline: `main@29fa4752bd7b54e78cd0a2a81e42292c1611d290`
Branch: `chore/p0.0-baseline-freeze`
Commit created: amended report commit, `docs(verification): freeze P0.0 baseline`
Tracked files changed: `docs/verification/p0_0_baseline_freeze_29fa4752.md`
Artifact root: `.artifacts/baseline/p0_0_29fa4752/`
Build/test result: PASS
Dataset runtime acceptance: FAIL — `P0.0-F01`
SIM runtime observations: WARNING — `P0.0-F02`; workflow completed and cleaned up
Known baseline inconsistencies: B-01 through B-07 above
Blockers: none for P0.0 freeze; runtime findings retained for later work
Recommended next task: `P0.1 — Frame naming and geometry`

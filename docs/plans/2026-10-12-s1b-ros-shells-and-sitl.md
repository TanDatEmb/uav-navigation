# S1b: LIO and PX4-Bridge ROS Shells, SITL Hover, F34 Check, Log Calibration

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish slice S1 (spec §7.2: "Hover ở mode Position của PX4, EKF2 fuse EV, log `T`. Kiểm giả thuyết F34"):
- implement the two LIO-core decisions that touch safety behaviour (D33: O15, O16);
- add the ROS shells: the `uavnav_lio` node over the product assembly `LioRunner`, the `uavnav_px4_bridge` node over the pure bridge core, and the `/events` publisher;
- add the PX4 profile parameters (D20, D28, D33/O23), the SITL bring-up with an early probe of the risky SITL assumptions, and a scripted, bounded hover smoke test;
- decide F34 in SITL and set the starting degeneracy thresholds from SITL logs (§3.2: "việc của lát S1").

The τ re-evaluation (O13) moves to S2: it needs flights with real motion (S2 straight-line flights at 1–5 m/s), and every S1b hover already records the output tracking error it will use.

**Architecture:**
- Every decision stays in a ROS-free library: `uavnav_lio_core` (changed only in Tasks 1–2), a new ingest library inside `uavnav_lio`, and a new `Px4Bridge` class in the bridge core target. The ROS targets only convert messages, spin one executor and publish.
- The `lio` process has exactly the three threads of §6.4: the executor thread, which is the ingest thread; the estimator thread, which `LioRunner` starts; the events thread, which the recorder starts.

  The `px4_bridge` process has the executor thread and the events thread.
- One `EventRecorder` per process writes through a fan-out sink to the JSONL file and to `/events`.
- SITL runs are driven by one bounded script and judged by pure, unit-tested Python checks.

**Tech Stack:** C++23 (gcc 13.3), Eigen 3, ament_cmake, GoogleTest, rclcpp (ROS 2 Jazzy), px4_msgs v1.17.0, Gazebo Harmonic with `ros_gz_bridge`, PX4 v1.17 SITL, MicroXRCEAgent, Python 3 `unittest` and `rclpy`, GNU make.

**Spec:** [SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md) §0 (profiles), §1, §3, §4.1, §6.1–§6.6, §7.1, §7.2 (S1).
- Decisions in [DECISIONS.md](../architecture/DECISIONS.md): D7, D11, D15, D17, D20–D22, D25, D26, D28–D32; facts F3, F13, F14, F22, F23, F27, F33–F36; D33 (O15–O25, summarised below).
- Predecessors: [S1a plan](2026-10-06-s1a-lio-and-bridge-core.md) and [D30 rework plan](2026-10-07-s1a-d30-rework.md), both "Carry-over to S1b" lists. Each open item is mapped in "Carry-over coverage" or "Not in this plan".
- TRACEABILITY rows: P2, P3, P7, F14, F22, F23, F34, the τ row (O13), "Event log", "Config ba tầng", "Message v2", and the D33 code rows (Task 14). The τ row moves to S2.

---

## Decisions (D33)

On 2026-10-12 the owner approved the recommended option for every open entry (D33 in [DECISIONS.md](../architecture/DECISIONS.md); spec §3.1, §3.3, §3.4, §3.5, §4.1, §6.5, §7.1 and §7.2 are updated). No design deviation is open, so the §7.4 stop rule is lifted. A new deviation found while executing this plan stops the task: record it as an O-entry and wait for the owner.

| ID | Decided | Tasks |
|---|---|---|
| O15 | A `kScanGood` result counts as corrected only if its age at application (newest IMU time − scan end) is ≤ `gap_degraded`. Otherwise it becomes `kScanCorrectionRejected` and is not offered to the predictor. | 1 |
| O16 | A restart answer that the predictor buffer can no longer match is refused (`LioRestartRejected`, stale reason). The state stays RESTARTING; the retry uses a fresh seed. | 2 |
| O17 | Spec text only: `frontend_config.hpp` also carries the `ImuGap` threshold, and `config.hpp` is included by the assembly, `LioPipeline`, the node and tests. | none |
| O18 | Spec text only: the predictor lookup keeps the OR rule (a bracket ≤ `kPredictorDtMax`, or within the newest dt). | none |
| O19 | Package layout. `src/core/uavnav_ros` holds the shared ROS helpers: time conversion, `RosEventSink`, QoS. The bridge node is a second target of `uavnav_px4_bridge`, and the core target stays ROS-free. `uavnav_lio` has a ROS-free ingest target and a node target. Launch files live in `src/uavnav_bringup`. The sim packages move to `src/sim/`. | 3, 6, 8, 10, 11 |
| O20 | `lio` inputs. The topics are `/lidar/imu` (m/s²) and `/lidar/points` (`PointCloud2`). A Gazebo scan is one instant (`start = end = stamp`), and non-finite points are dropped and counted. The base←LiDAR mount is a new tier-(b) `kLioNodeSpecs` table in `config/lio/node_sim.yaml`, and `base_T_imu = base_T_lidar · imu_T_lidar⁻¹`. Frame ids are tier (a). One pending scan waits for IMU coverage without blocking; it is dropped with an event when a newer scan arrives (`kSuperseded`) or after 100 ms of ROS time (`kImuCoverageTimeout`). A stamp more than 0.5 s ahead of ROS now is refused. Livox `CustomMsg` waits for hardware. | 7, 8 |
| O21 | Outputs. QoS: sensor inputs `SensorDataQoS` 5; `/lio/state` best effort 10; `/lio/odometry`, `/lio/health` and `/alignment` reliable 10; `/events` reliable 1000; PX4 best effort 5. `/lio/state` publishes one sample per 10 ms sensor-time cell. Each sample carries the base_link pose, velocity in base_link with the ω×r lever arm, the raw gyro as angular velocity, NaN covariances, and the latched reset delta. The node publishes on every call and counts overwritten odometry. `/lio/health` is published for every `HealthOutput`. `/alignment` is published on every `on_lio`/`on_tick` output, and on every `on_px4` output with a reason; `raw_*` is NaN when absent. | 7–10 |
| O22 | Bridge inputs. EV is blocked by a newer same-epoch non-TRACKING health sample (stamp ≥ the odometry stamp). Every non-TRACKING health sample calls `on_lio(tracking=false)`, so VALID→FROZEN. A PX4 sample with `!xy_valid` or `!z_valid` is skipped with an event, and its counters are applied at the next valid sample. PX4 µs are converted by the range-checked `from_px4_us`. `on_tick` runs every 100 ms. In `VehicleOdometry`, `timestamp` is ROS now and `angular_velocity` is NaN. | 5, 9, 10 |
| O23 | PX4 parameters. Both profiles: `EKF2_EV_DELAY=0`, `EKF2_DELAY_MAX=200`, `EKF2_EV_NOISE_MD=0`, `EKF2_EV_QMIN=0`. GPS profile: `EKF2_EV_CTRL=7`, `EKF2_GPS_CTRL=7`, `EKF2_HGT_REF=1`. No-GPS profile: `EKF2_EV_CTRL=15`, `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=3`. Harness only: `UXRCE_DDS_SYNCT=0`, `SIM_GZ_EN_ODOM=0`, `SIM_GZ_EN_GPS=1`, `COM_RC_IN_MODE=1`, `NAV_DLL_ACT=0`, `CBRK_SUPPLY_CHK`. | 3, 11 |
| O24 | Hover in Position mode, with centred sticks on `/fmu/in/manual_control_input` (`SOURCE_MAVLINK_0`, 20 Hz). **The approved fallback is Hold (`AUTO_LOITER`)**, chosen by the Task 3 probe. F34 is a positive check at spawn yaw 1.2 rad in both profiles, without a NED control. | 3, 11, 12 |
| O25 | `/lio/scan` and the restart caller (with the seed `T⁻¹ × PX4 pose`) move to S3/S4. In S1b a LOST LIO stays RESTARTING, EV stops, and `T` goes FROZEN, then INVALID. | "Not in this plan" |

Also accepted with D33:
- the S1a-T8 and D30 interface additions;
- worker death by `std::terminate`;
- commit trailers that name the model that did the work.

The remaining spec-sync items are for the architect, not this plan:
- §3.2 hysteresis is the lifecycle's consecutive-scan counts;
- §3.1 gains TRACKING→LOST on σ;
- §4.1 gains INVALID→INIT.

---

## Global Constraints

- **Tree and staging:** work in this tree on `rebuild/v2`; no worktree, clone or temporary build directory (CLAUDE.local.md). Stage with `git add <path>` only; never `git add -A`, `git add .`, `git stash -u`, `git clean` or `git reset --hard`. **Run `git diff --cached --stat` before every commit**: other agents share this index, so unstage anything you did not intend.
- **Build and test only through the Makefile:** `make build PKGS="<pkgs>"`, `make test PKGS="<pkgs>"`, `make sanitize`, `tools/uavnav/gate.sh <stage>`, and the new `make sitl-build` and `make sitl-hover`. They apply `-j3` (`-j2` for sanitize), two colcon workers and `nice`; never call bare `colcon` or raise the limits; at most 3 subagents. `build/` and `install/` are a shared incremental build and are never deleted.
- **SITL:** never while a build, test or sanitize runs (the script refuses while `colcon` runs); one run at a time, headless (`GZ_GUI=0`), bounded at 300 s; artifacts only under `log/sitl/`; never part of `make test` or `make gate`.
- **Deleting or moving tracked files** (AGENTS.md §6): reference sweep across all file types, then explicit `git rm`/`git mv` paths; the first build after a move uses `COLCON_EXTRA=--cmake-clean-cache`. If the permission system blocks a deletion, do not retry or work around it: write a script with one `git rm` per path, dry-run it (`-n`) and hand it to the owner.
- **No ROS in the cores:** `uavnav_core`, `uavnav_lio_core`, `fast_lio_core`, `uavnav_px4_bridge_core` and `uavnav_lio_ingest` include no rclcpp or message headers and read no clock. Only node targets, `uavnav_ros` and test harnesses use ROS.
- **Time:** every external stamp goes through a range-checked converter (`time::from_stamp`, `from_px4_us`); one `TimeSnapshot` (steady + ROS) per callback; sensor and PX4 time are SensorTime in SITL (D29).
- **Config (D22):** tier (a) in a `limits.hpp` with a derivation comment; tier (b) in a typed spec table with bounds (an unknown or out-of-range key refuses start); tier (c) in profile files. A node takes only string-path ROS parameters (`config_file`, `node_config_file` for `lio`, `events_jsonl`) plus `use_sim_time`; launch files have no defaults.
- **Events:** one recorder per process; every shell decision (input refused, scan dropped, EV gated, PX4 sample skipped, alignment transition) emits one event with a typed reason; per-sample rejections are rate-limited like the frontend's.
- **Tests:** synthetic data only; no wall-clock assertion tighter than 1 s; "never blocks" and "returns" checks use `std::async` with `wait_for(std::chrono::seconds(10))`; in-process ROS tests bound every wait at 60 s.
- **Commits:** `type(scope): description`, blank line, body (cite the spec section and the decision), blank line, trailer `Co-Authored-By: <the model that actually did the work> <noreply@anthropic.com>`; written as three `-m` arguments.

## Review Focus

1. **A scan arrives before its IMU coverage and the bounded wait expires.** The scan is held without blocking and retried on each IMU sample. After 100 ms of ROS time it is dropped with `kImuCoverageTimeout`, so it never reaches the core ahead of its IMU. Pinned by `LioIngest.ScanWaitingForImuIsDroppedAfterTheBoundedWait` (Task 7).
2. **A PX4 double reset during hover** (xy +2 and heading +1 in one sample). `T` moves by exactly the frame change G, the vehicle point stays continuous, and no later pair is rejected. Pinned by `Px4Bridge.DoubleResetDuringHoverKeepsTheVehiclePointContinuous` (Task 9).
3. **LIO LOST while PX4 keeps flying.** No EV is sent for odometry stamped at or before a non-TRACKING health of the same epoch (O22(a)); after a LOST health no further EV follows because LIO publishes odometry only in TRACKING and has no LOST→TRACKING edge (LIO-3). The alignment goes VALID→FROZEN, not STALE→INVALID. Pinned by `Px4Bridge.LioLostStopsEvAndFreezesAlignment` (Task 9).
4. **ROS callbacks on several threads never break `LioRunner`'s single-ingest contract.** Overlapping calls are refused with an event and never raced, and one callback group keeps even a multi-threaded executor serialised. Pinned by `LioIngest.ConcurrentCallsAreRefusedNotRaced` (Task 7) and `LioNode.MultiThreadedExecutorKeepsCallbacksSerialised` (Task 8).
5. **Non-finite or garbage ROS messages are refused at the boundary with a reason and change nothing.** This covers negative stamps, `nanosec` ≥ 1e9, NaN IMU data, malformed clouds, a wrong frame, PX4 µs overflow and a NaN heading. Pinned by `LioRosConversion.RejectsGarbageMessages` (Task 8), `Px4Bridge.GarbageInputsAreRejectedWithEvents` (Task 9) and `Px4BridgeRos.RejectsGarbagePx4Messages` (Task 10).

---

## S1b runtime design (binds Tasks 6–11)

| Topic | Decision |
|---|---|
| `lio` threads (§3.5, §6.4) | `main` builds, in order: the node; the recorder (`TeeSink(JsonlSink, RosEventSink)`); `LioIngest`, which creates `LioRunner`. It then spins one `SingleThreadedExecutor`, whose thread is the ingest thread. Both subscriptions are in one `MutuallyExclusive` group. `LioIngest` refuses overlapping calls (atomic try-enter, no lock). No timers. |
| Backend clock | `LioClock` returns `{time::steady_now(), latest ROS time}`. The ROS time is an atomic the ingest thread stores at every callback, so the clock never blocks (`stop()` joins the backend). |
| `px4_bridge` threads | One `SingleThreadedExecutor` and one callback group: three subscriptions plus the 100 ms tick, calling the single-threaded `Px4Bridge`. The recorder writer is the events thread. |
| `/events` | `RosEventSink::write` runs on the recorder writer (§6.1) and publishes one `EventRecord.msg` per record, with no lock held. |
| Shutdown | SIGINT: the pre-shutdown hook closes the node's callback gate (waits for any in-flight callback; later callbacks return at once), so no callback can overlap the next step; then `LioIngest::stop()` (`LioRunner::stop`, which joins the backend before the channels go); then the recorder is destroyed (drain and flush); then the publishers and the node. |
| Topics | `lio` in: `/lidar/imu`, `/lidar/points`; out: `/lio/state`, `/lio/health`, `/lio/odometry`, `/events`. `px4_bridge` in: `/lio/odometry`, `/lio/health`, `/fmu/out/vehicle_local_position_v1`; out: `/fmu/in/vehicle_visual_odometry`, `/alignment`, `/events`. A PX4 topic gets the suffix `_v<N>` when its message has `MESSAGE_VERSION` > 0. |
| Config files | `config/lio/sim.yaml` (unchanged schema); new `config/lio/node_sim.yaml`, `config/px4_bridge/sim.yaml`, `config/px4/profiles/{gps,no_gps}.yaml`. |
| Tier (a) | `uavnav/lio_node/limits.hpp`: `kStatePeriod` 10 ms, `kScanImuWaitMax` 100 ms, `kMaxStampAhead` 500 ms, frame ids. The bridge `limits.hpp` gains `kTickPeriod` 100 ms, `kMaxStampAhead` 500 ms and `kClockMode = kSimulationIdentity` (D29). |

## File Structure

```text
src/estimation/uavnav_lio_core/  (modify) T1-T2: frontend, output_predictor, types.hpp, runner.hpp, backend.cpp comment, tests
src/sim/uav_simulation, src/sim/uav_description   (git mv) T3
config/px4/profiles/{gps,no_gps}.yaml, tools/uavnav/{px4_profile,sitl_probe,sitl_checks}.py, tools/uavnav/sitl_hover.sh   (new) T3
src/core/uavnav_core/            (modify) T4: angles.hpp, tee_sink.hpp/.cpp, time.hpp, spsc_queue.hpp comment, tests
src/px4/uavnav_px4_bridge/       (modify) T5: px4_time, ev_encoder, alignment; T9: bridge.hpp/.cpp; T10: node target
config/px4_bridge/sim.yaml (new) T5; config/lio/node_sim.yaml (new) T7
src/core/uavnav_ros/             (new)    T6: time_msg.hpp, ros_event_sink.hpp/.cpp, qos.hpp, tests
src/estimation/uavnav_lio/       (new)    T7: ingest library; T8: ros_conversion + lio_node
src/uavnav_bringup/, tools/uavnav/sitl_hover.py   (new) T11; tools/uavnav/lio_calibration.py (new) T13
tools/simulation/run_px4_mid360.sh, tools/simulation/README.md, Makefile, README.md, .agents/skills/build-and-test/SKILL.md   (modify)
```

---

### Task 1: Result-age rule for scan corrections (D33/O15)

Spec §3.1, §3.5, D31, D33/O15.

**Files:** under `src/estimation/uavnav_lio_core/`:
- `include/uavnav/lio/types.hpp`: `EstimatorEventReason::kResultTooOld` ("RESULT_TOO_OLD") and its catalogue line;
- `src/frontend.cpp`; `include/uavnav/lio/frontend.hpp` (the gap-semantics comment);
- `test/test_frontend.cpp`.

**Rule** (no signature changes), in `apply_scan` for `kScanGood`:
1. `age = newest IMU time − r.t`.
2. If `age > cfg_.lifecycle.gap_degraded` (equal counts): do **not** call `on_correction`; emit `ScanResultTooOld` (`kResultTooOld`; values `sensor_time_s`, `age_s`, `gap_degraded_s`); feed `kScanCorrectionRejected`; publish no odometry and leave `last_good_scan_t_` unchanged.
3. Otherwise, as today.

Not offering the stale snapshot to the predictor is part of D33/O15 (fail closed): `/lio/state` does not move on a result the lifecycle refuses.

- [ ] **Step 1: Write the failing tests** (`FakeBackend` as in the existing suite)

  ```cpp
  TEST(LioFrontend, ResultOlderThanGapDegradedIsARejectedCorrection) {
    // TRACKING; kScanGood with t = newest IMU - 0.27 s -> ScanResultTooOld (age_s 0.27 +-1e-9), no odometry, tracking_error
    // unchanged (predictor untouched), correction_age_s not reset; the third one -> DEGRADED with CORRECTION_REJECTED
  }
  TEST(LioFrontend, ResultAgeEqualToGapDegradedCounts) { /* age exactly 0.25 s -> applied, odometry published */ }
  TEST(LioFrontend, SaturatedBackendReachesLostInsteadOfFlapping) {
    // DEGRADED; for 3 s a kScanGood aged 0.27 s every 100 ms while IMU flows at 200 Hz: never TRACKING;
    // LOST (kLidarGapLost) at the first IMU t > last applied scan end + 0.5 s
  }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core`.
  Expected:
  - the first and third tests fail (the 0.27 s result confirms TRACKING);
  - `ResultAgeEqualToGapDegradedCounts` already passes (it pins the inclusive boundary for Step 3);
  - every existing test passes.

- [ ] **Step 3: Implement the rule.**

- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sanitize`.
  Expected: all pass (`LioRunner.ConcurrentIngestAndBackendAccountForEveryScan` asserts accounting, not states); no `WARNING: ThreadSanitizer`.

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core
  git commit -m "fix(lio): count a scan correction only when its result is not older than gap_degraded" \
    -m "SYSTEM_DESIGN §3.1/§3.5, D31, O15 (a): results aged (0.25, 0.30] s no longer confirm TRACKING; a saturated backend ends in LOST instead of flapping DEGRADED<->TRACKING." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 2: Refuse a restart answer the predictor cannot match (D33/O16)

Spec §3.4, §3.5, D31, D33/O16.

**Files:** under `src/estimation/uavnav_lio_core/`:
- `include/uavnav/lio/output_predictor.hpp`, `src/output_predictor.cpp`;
- `include/uavnav/lio/types.hpp`: `EstimatorReason::kStaleAnswer` ("STALE_ANSWER"), `EstimatorEventReason::kRestartAnswerStale` ("RESTART_ANSWER_STALE"), catalogue;
- `src/frontend.cpp`;
- `include/uavnav/lio/runner.hpp` (rewrite "Late restart answer");
- `include/uavnav/lio/outputs.hpp` (invariant `OdometryOutput::reset_counter == epoch − 1`);
- `src/backend.cpp` (a comment on the "clear map, then seed" order);
- `test/test_output_predictor.cpp`, `test/test_frontend.cpp`, `test/test_runner.cpp`.

**Interfaces:**

```cpp
/// The match rule of on_correction / reset_to, changing nothing: Ok when the buffer covers t, else the reason.
Result<void, PredictorReason> OutputPredictor::covers(time::SensorTime t) const noexcept;
```

`apply_restart` refuses a `kRestartSeeded` when `!predictor_.covers(r.snapshot->t)`:
- `requested_epoch_ = epoch_`;
- `LioRestartRejected` with reason `kRestartAnswerStale` (values `new_epoch`, `sensor_time_s`, `seed_age_s`);
- `out.restart = unexpected(kStaleAnswer)`;
- the state stays RESTARTING, with no `reset_to`.

The backend has already switched to the new epoch. The next `restart()` reuses that epoch number with a newer `t`, which `restart_refusal` accepts (it checks only `t ≥ ESKF time`). Scans admitted in between carry the old epoch and come back `kScanNotProcessed`.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(OutputPredictor, CoversMatchesTheCorrectionRule) { /* jittered buffer, stamps swept over [oldest - 10 ms, newest + 10 ms] by 1 ms:
     covers(t).has_value() == on_correction(copy) Ok; latest() unchanged by covers */ }
  TEST(LioFrontend, LateRestartAnswerIsRefusedAndStaysRestarting) {
    // RESTARTING; restart() at t0; 400 ms of IMU; kRestartSeeded epoch 2 at t0 -> LioRestartRejected RESTART_ANSWER_STALE,
    // restart error kStaleAnswer, RESTARTING, epoch 1, next StateOutput t > previous (no rewind); a new restart() is accepted
  }
  TEST(LioFrontend, TimelyRestartAnswerAtTheBufferBoundaryIsApplied) { /* answer exactly 300 ms old -> INITIALIZING, epoch 2 */ }
  TEST(LioFrontend, ResetCounterIsEpochMinusOne) { /* two applied restarts and one refused: every OdometryOutput reset_counter == epoch - 1 */ }
  TEST(LioRunner, LateRestartAnswerIsRefusedAndARetrySucceeds) {
    // replaces LateRestartAnswerRestartsThePredictorAndRecovers: gate closed for 400 ms of IMU -> refused, RESTARTING, epoch 1;
    // retry -> epoch 2, INITIALIZING; StateOutput times never decrease
  }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio_core`. Expected: compile error, `OutputPredictor::covers` is missing.

- [ ] **Step 3: Implement**, delete `LioRunner.LateRestartAnswerRestartsThePredictorAndRecovers`, and fix the "no rewind" comment in `frontend.cpp`.

- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sanitize`. Expected: all pass, no TSan warning.

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core
  git commit -m "fix(lio): refuse a restart answer the output predictor can no longer match" \
    -m "SYSTEM_DESIGN §3.4/§3.5, D31, O16 (a): a kRestartSeeded older than the predictor buffer no longer rewinds /lio/state; the frontend stays RESTARTING and a retry reuses the epoch." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 3: Bring-up foundation and an early SITL probe of the risky assumptions

Spec §0 (profiles), §6.2 (c), §7.1, §7.2; D20, D26, D28, D29; D33 (O19, O20, O23, O24).

This task runs before any node exists, so that the three assumptions the rest of the plan builds on are checked on the real simulator first.
1. PX4 accepts Position mode driven by a stick stream from ROS (O24).
2. The Gazebo cloud has the layout O20 expects: float32 `x,y,z`, `inf` no-returns, one stamp per scan.
3. The sensor stamps and the PX4 time follow `/clock` (D29 identity).

The GPS profile gives PX4 a position without EV, so the probe needs neither LIO nor the bridge.

**Files:**
- Move (`git mv`): `src/uav_simulation` → `src/sim/uav_simulation`, `src/uav_description` → `src/sim/uav_description`.
- Create: `config/px4/profiles/gps.yaml`, `config/px4/profiles/no_gps.yaml`; `tools/uavnav/px4_profile.py`, `tools/uavnav/tests/test_px4_profile.py`; `tools/uavnav/sitl_hover.sh` (orchestrator; Task 11 extends it); `tools/uavnav/sitl_probe.py` (rclpy probe), `tools/uavnav/sitl_checks.py` (pure, stdlib only), `tools/uavnav/tests/test_sitl_checks.py`.
- Modify: `tools/simulation/run_px4_mid360.sh`, `tools/simulation/README.md` (drop the references to the removed `runner.py`), `Makefile` (target `sitl-probe`, kept out of `test` and `gate`), `README.md`, `.agents/skills/build-and-test/SKILL.md`.

**Interfaces:**
- **`px4_profile.py`:**

  ```python
  class ProfileError(ValueError): ...
  @dataclasses.dataclass(frozen=True)
  class Profile: localization: str; px4_params: dict[str, int | float]   # localization "gps" | "no_gps"
  def load(path: pathlib.Path) -> Profile   # only allow-listed EKF2_* names (O23); EKF2_EV_CTRL 7 for gps, 15 for no_gps (D20);
                                            # EKF2_EV_QMIN 0 (D28); anything else -> ProfileError naming the key
  def main(argv) -> int   # --profile PATH --format env -> sorted "export PX4_PARAM_<NAME>=<value>" lines; 0 ok, 2 invalid
  ```

- **`run_px4_mid360.sh`:** The profile cases are replaced by `eval "$(python3 tools/uavnav/px4_profile.py --profile "$UAVNAV_PX4_PROFILE" --format env)"`. The profile is required, the harness parameters follow O23, an optional `UAVNAV_SPAWN_YAW_RAD` sets the spawn yaw with `gz service -s /world/${WORLD_NAME}/set_pose` on `x500_mid360` before PX4 attaches, and paths point to `src/sim/`.
- **`make sitl-probe PROFILE=gps`** runs `tools/uavnav/sitl_hover.sh --probe`: The script exits 2 when `pgrep -f "colcon (build|test)"` finds a process. It makes `log/sitl/<UTC stamp>-<profile>-probe/` and starts, each in its own process group, `run_px4_mid360.sh` (`GZ_GUI=0`), `MicroXRCEAgent udp4 -p 8888`, the `ros_gz_bridge parameter_bridge` with `px4_mid360_control_bridge.yaml`, and `uav_simulation gz_lidar_bridge`. It then runs `sitl_probe.py` and kills every group on EXIT, INT and TERM. The run is bounded at 240 s.
- **`sitl_probe.py`** runs these phases, each with a timeout: (1) `wait_ready`: `xy_valid` (≤ 90 s); (2) `posctl` + `arm` with centred sticks on `/fmu/in/manual_control_input` at 20 Hz; (3) `climb` to 2.5 m; (4) `hover` for 20 s; (5) `land`.

  It records `/lidar/points` (fields, `point_step`, the non-finite ratio, stamp), `/lidar/imu` (rate and stamps), `/clock`, `/fmu/out/vehicle_local_position_v1` and `/fmu/out/vehicle_status_v1`.
- **`sitl_checks.evaluate_probe(run_dir) -> dict`** writes `summary.json` with these checks. Missing data fails a check.
  - `posctl_accepted`: `nav_state` POSCTL, armed, and hover within 0.5 m of the climb height.
  - `cloud_layout`: FLOAT32 `x`,`y`,`z` and frame `livox_frame`.
  - `cloud_non_finite_ratio`: the ratio is reported.
  - `imu_rate`: 200 ± 10 Hz.
  - `scan_stamp_vs_imu`: p99 of (scan stamp − newest IMU stamp at arrival) ≤ 50 ms.
  - `px4_time_identity`: p99 of |`vehicle_local_position.timestamp` − `/clock`| ≤ 20 ms.

- [ ] **Step 1: Reference sweep for the move.** Run: `grep -rnE "src/uav_simulation|src/uav_description" --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=.git . | grep -v "^./docs/plans/"`.
  Expected: hits only in `tools/simulation/` and this task's files. Resolve any other hit, or list it with a reason, before the move.

- [ ] **Step 2: Write the failing Python tests.**
  - `Px4ProfileTest`: `test_d20_ev_ctrl_per_profile` (7 and 15; a gps file with 15 → `ProfileError`); `test_ev_qmin_is_zero`; `test_unknown_param_rejected`; `test_env_output`.
  - `SitlChecksTest`: `test_probe_pass_on_synthetic_records`; `test_probe_fails_when_posctl_rejected`; `test_probe_fails_on_float64_cloud`; `test_missing_topic_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `ModuleNotFoundError`, `GATE_RESULT=FAIL`.

- [ ] **Step 3: Implement, move the packages, write the script and the docs.** Then:

  ```bash
  tools/uavnav/gate.sh python
  make build PKGS="uav_simulation uav_description" COLCON_EXTRA=--cmake-clean-cache && make test PKGS="uav_simulation uav_description"
  bash -n tools/simulation/run_px4_mid360.sh tools/uavnav/sitl_hover.sh
  ```

  Expected: `GATE_RESULT=PASS`; the build and asset tests pass; both scripts parse.

- [ ] **Step 4: Probe** (owner machine time, about 5 min, nothing else running). Run: `make sitl-probe PROFILE=gps`.
  Expected: `summary.json` passes every check except possibly `posctl_accepted`.
  - **`posctl_accepted` fails:** switch the harness hover mode to the approved D33/O24 fallback, Hold (`AUTO_LOITER` via `VEHICLE_CMD_DO_SET_MODE` 1/4/3). Re-run the probe and record the result in the commit body.
  - **`cloud_layout`, `scan_stamp_vs_imu` or `px4_time_identity` fails:** stop. The O20 or D29 assumption is wrong. Record a new O-entry and wait for the owner before Task 7.

- [ ] **Step 5: Commit** (`git mv` already staged the moves)

  ```bash
  git diff --cached --stat
  git add config/px4 tools/uavnav/px4_profile.py tools/uavnav/sitl_hover.sh tools/uavnav/sitl_probe.py tools/uavnav/sitl_checks.py \
          tools/uavnav/tests/test_px4_profile.py tools/uavnav/tests/test_sitl_checks.py tools/simulation Makefile README.md \
          .agents/skills/build-and-test/SKILL.md
  git commit -m "feat(sitl): add PX4 localization profiles, the SITL start contract and a probe of the SITL assumptions" \
    -m "SYSTEM_DESIGN §0/§6.2(c)/§7.1/§7.2, D20, D28, D29, D33 (O19, O20, O23, O24): profiles validated by px4_profile.py; sim packages moved to src/sim; probe gps: <paste summary.json, incl. the hover mode chosen>." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 4: `uavnav_core` angles header, `to_seconds` of an instant, `TeeSink`; remove duplicated helpers

Spec §6.1, §6.3, D25. Carry-over: one angles header before the nodes add more copies.

**Files:**
- Create (under `src/core/uavnav_core/`): `include/uavnav/core/angles.hpp`, `include/uavnav/core/tee_sink.hpp`, `src/tee_sink.cpp`, `test/test_angles.cpp`, `test/test_tee_sink.cpp`.
- Modify (under `src/core/uavnav_core/`): `include/uavnav/core/time.hpp`, `test/test_time.cpp`, `include/uavnav/core/spsc_queue.hpp` (comment: `sizeof` ≈ `Capacity * sizeof(T)`, put large queues on the heap), `CMakeLists.txt`.
- Callers: `src/estimation/uavnav_lio_core/src/{frontend.cpp,output_predictor.cpp,backend_math.cpp,backend_math.hpp}`; `src/estimation/uavnav_lio_core/include/uavnav/lio/types.hpp` (one `usable(const Eigen::Quaterniond&)` for both blocks); `src/px4/uavnav_px4_bridge/src/alignment.cpp`.

**Interfaces:**

```cpp
// angles.hpp (uavnav::angles, no Eigen)
constexpr double wrap_pi(double a) noexcept;                        // [-pi, pi]; NaN stays NaN
inline double yaw_from_rotation(double r10, double r00) noexcept;   // atan2(r10, r00)
// time.hpp
template <class Tag> constexpr double to_seconds(TimePoint<Tag> t);  // ns / 1e9
// tee_sink.hpp (uavnav::events)
class TeeSink final : public EventSink {
 public:
  static Result<std::unique_ptr<TeeSink>, RecorderError> create(std::unique_ptr<EventSink> a, std::unique_ptr<EventSink> b);  // kNoSink on null
  Result<void, SinkError> write(std::span<const EventRecord> batch) override;  // always writes both; kIo if either failed or threw
};
```

- [ ] **Step 1: Write the failing tests:** `Angles.WrapPiKeepsRangeAndNan` (3.5 → 3.5 − 2π; −π stays; NaN stays NaN); `Time.ToSecondsOfTimePoint` (1.5 s); `TeeSink.WritesBothAndReportsEitherFailure` (the first sink fails: kIo, and the second is still written); `TeeSink.ThrowingSinkDoesNotStopTheOther`.

  Run: `make build PKGS=uavnav_core`. Expected: compile error, `angles.hpp` missing.

- [ ] **Step 2: Implement, then replace every copy.**
  Run: `grep -rnE "double (yaw_of|wrap|wrap_pi|seconds)\(|bool usable\(" src/estimation/uavnav_lio_core src/px4/uavnav_px4_bridge`.
  Expected: only the `usable` in `types.hpp`.

- [ ] **Step 3: Verify.** Run: `make build && make test && make sanitize`. Expected: all pass, no TSan warning.

- [ ] **Step 4: Commit**

  ```bash
  git diff --cached --stat
  git add src/core/uavnav_core src/estimation/uavnav_lio_core src/px4/uavnav_px4_bridge
  git commit -m "refactor(core): add angles and TeeSink, remove duplicated angle and time helpers" \
    -m "SYSTEM_DESIGN §6.1/§6.3, D25: one fan-out sink lets one recorder per process write JSONL and /events; one wrap/yaw helper before the S1b nodes." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 5: Bridge core additions: PX4 µs converter, EV quaternion reason, reason fix, `sim.yaml`

Spec §4.1, D21, D22, D29; D33/O22. Covers the S1a carry-over items for the alignment and the EV encoder.

**Files:**
- Under `src/px4/uavnav_px4_bridge/`: modify `include/uavnav/px4bridge/{px4_time,ev_encoder}.hpp`, `src/{px4_time,ev_encoder,alignment}.cpp`; modify `test/{test_px4_time,test_ev_encoder,test_alignment,test_alignment_config}.cpp`; modify `CMakeLists.txt` (test definition `UAVNAV_REPO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"`).
- Create `config/px4_bridge/sim.yaml`: the six `kAlignmentSpecs` keys at the beta values 2.0, 20, 0.5, 0.0873, 1.0, 10.0, with a header in the style of `config/lio/sim.yaml`.

**Interfaces:**

```cpp
// px4_time.hpp: Px4TimeError gains kOutOfRange ("OUT_OF_RANGE")
/// PX4 microseconds (timestamp_sample) -> SensorTime; kSimulationIdentity only (else kRealtimeNotSupported);
/// us > time::kMaxStampSeconds * 1'000'000 -> kOutOfRange, checked before the multiply.
Result<time::SensorTime, Px4TimeError> from_px4_us(std::uint64_t us, ClockMode mode);
// ev_encoder.hpp: EvReason gains kBadQuaternion ("BAD_QUATERNION") for a zero or non-unit q (was kNonFiniteState)
```

Alignment fix: a FROZEN→VALID call whose pair is jump-rejected reports `kPairRejectedJump`, not `kLioTracking`.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Px4Time, FromPx4UsIsRangeChecked) { /* 0 -> 0; 1'234'567 us -> 1'234'567'000 ns; max ok; max+1 and UINT64_MAX -> kOutOfRange;
     kRealtime -> kRealtimeNotSupported */ }
  TEST(EvEncoder, BadQuaternionHasItsOwnReason) { /* zero q and |q| = 1.01 -> kBadQuaternion; NaN q -> kNonFiniteState */ }
  TEST(EvEncoder, NonUnitQuaternionWithRotatedCovariance) { /* |q| = 1 + 5e-4, yaw 1.0, anisotropic world block -> diag(R^T S R) (1e-6) */ }
  TEST(Alignment, FrozenToValidJumpRejectedPairKeepsItsReason) { /* FROZEN, tracking pair 1 m off -> kPairRejectedJump */ }
  TEST(AlignmentConfig, SimYamlLoads) { /* load_params_file(UAVNAV_REPO_CONFIG_DIR "/px4_bridge/sim.yaml") == BetaConfig() */ }
  ```

  Run: `make build PKGS=uavnav_px4_bridge`. Expected: compile error (`from_px4_us`, `kBadQuaternion` missing).

- [ ] **Step 2: Implement.**

- [ ] **Step 3: Verify.** Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: all pass, and the D30 reset and boundary tests are unchanged.

- [ ] **Step 4: Commit**

  ```bash
  git diff --cached --stat
  git add src/px4/uavnav_px4_bridge config/px4_bridge/sim.yaml
  git commit -m "feat(px4-bridge): add the PX4 microsecond converter and the SITL alignment config" \
    -m "SYSTEM_DESIGN §3.4/§4.1, D21, D22, D29: range-checked timestamp_sample -> SensorTime; kBadQuaternion; FROZEN->VALID jump keeps its reason; config/px4_bridge/sim.yaml is the single source of the alignment values." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 6: `uavnav_ros`: ROS time conversion, `RosEventSink`, product QoS

Spec §6.1, §6.3, §6.5; O19, O21.

**Files:** create under `src/core/uavnav_ros/`:
- `package.xml` (`rclcpp`, `builtin_interfaces`, `uavnav_core`, `uavnav_interfaces`), `CMakeLists.txt`;
- `include/uavnav/ros/time_msg.hpp`, `include/uavnav/ros/ros_event_sink.hpp`, `src/ros_event_sink.cpp`, `include/uavnav/ros/qos.hpp`;
- `test/test_time_msg.cpp`, `test/test_ros_event_sink.cpp`, `test/test_qos.cpp`.

**Interfaces** (`uavnav::rosutil`):

```cpp
template <class Tag> Result<time::TimePoint<Tag>, time::TimeError> from_msg(const builtin_interfaces::msg::Time&);  // via from_stamp
template <class Tag> Result<builtin_interfaces::msg::Time, time::TimeError> to_msg(time::TimePoint<Tag> t);  // kNegativeSeconds for t.ns < 0; kOutOfRange above int32 seconds
time::TimeSnapshot snapshot(const rclcpp::Clock& clock) noexcept;                    // steady_now() + clock.now(); ros{0} if unconvertible
class RosEventSink final : public events::EventSink {                                // uavnav_interfaces/msg/EventRecord on /events
 public:
  static Result<std::unique_ptr<RosEventSink>, RosSinkError> create(rclcpp::Node& node);  // kPublisherCreateFailed; created on the node's thread
  Result<void, events::SinkError> write(std::span<const events::EventRecord> batch) override;  // kIo if publish threw
};
namespace qos { rclcpp::QoS sensor_input(); rclcpp::QoS state_output(); rclcpp::QoS reliable_output();
                rclcpp::QoS events(); rclcpp::QoS px4(); }   // values of O21
```

- [ ] **Step 1: Write the failing tests:** `RosTime.FromMsgIsRangeChecked`: {−1, 0} → `kNegativeSeconds`; {1, 1e9} → `kNanosecondsOutOfRange`; `RosTime.ToMsgRoundTrip`; `RosEventSink.PublishesEveryRecordFieldByField`: in-process subscription; 3 records, one with 16 values, arrive with strings, identity and values in order; wait ≤ 60 s; `RosQos.ProductProfilesMatchTheDesign`.

  Run: `make build PKGS=uavnav_ros`. Expected: build failure on the missing headers.

- [ ] **Step 2: Implement.** Then run: `make build PKGS=uavnav_ros && make test PKGS=uavnav_ros`. Expected: all pass.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add src/core/uavnav_ros
  git commit -m "feat(ros): add uavnav_ros with time conversion, /events sink and product QoS" \
    -m "SYSTEM_DESIGN §6.1/§6.3/§6.5, O19, O21: the shared ROS shell utilities; RosEventSink runs on the recorder's writer thread." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 7: `uavnav_lio` ingest library (ROS-free): node config, pending scan, decimation, state mapping

Spec §3.3, §3.5, §6.2, §6.4; O20, O21. `LioIngest` is the only caller of `LioRunner`; nothing in `uavnav_lio` uses `LioPipeline` (D31).

**Files:**
- Create under `src/estimation/uavnav_lio/`: `package.xml`, `CMakeLists.txt` (target `uavnav_lio_ingest`, depending on `uavnav_lio_core` and `uavnav_core` only); `include/uavnav/lio_node/{limits,node_config,ingest}.hpp`, `src/{node_config,ingest}.cpp`; `test/room_scene.hpp` (a 10 m box-room cloud and a stationary 200 Hz IMU stream), `test/test_node_config.cpp`, `test/test_ingest.cpp`.
- Create `config/lio/node_sim.yaml`.
- Modify `Makefile`: `SANITIZE_PKGS` gains `uavnav_lio` (only the ingest library and its tests are built under `UAVNAV_SANITIZE`).

**Interfaces** (`uavnav::lio_node`):

```cpp
inline constexpr auto kLioNodeSpecs = /* the six mount keys of O20 */;
struct LioNodeConfig { Eigen::Isometry3d base_T_lidar; };
Result<LioNodeConfig, config::ConfigError> load_lio_node_config(const config::ParamValues&);
Eigen::Isometry3d compose_base_T_imu(const LioNodeConfig&, const lio::LioConfig&);   // base_T_lidar * imu_T_lidar^-1
enum class IngestReason : std::uint8_t { kAccepted, kReentered, kStampInFuture, kImuCoverageTimeout, kSuperseded, kRunnerRejected };
class StateDecimator { public: explicit StateDecimator(time::Duration period); bool due(time::SensorTime t) noexcept; };
struct LioStateOut { time::SensorTime t; std::uint32_t epoch, reset_counter; lio::LioState state;
                     Eigen::Vector3d p_world_base; Eigen::Quaterniond q_world_base; Eigen::Vector3d v_base_mps, w_base_rad_s;
                     lio::ResetDelta last_reset; };   // zero before the first reset
struct IngestOutputs { std::vector<LioStateOut> states; std::optional<lio::OdometryOutput> odometry;
                       std::optional<lio::HealthOutput> health; };
struct IngestStats { std::uint64_t scans_waited, scans_dropped_timeout, scans_superseded, odometry_overwritten, reentered; };
class SharedRosTime { public: void store(time::RosTime) noexcept; time::TimeSnapshot snapshot() const noexcept; };   // atomic
class LioIngest {
 public:
  static Result<std::unique_ptr<LioIngest>, config::ConfigError> create(const lio::LioConfig&, const LioNodeConfig&,
                                                                        events::EventRecorder&, SharedRosTime&);
  Result<IngestOutputs, IngestReason> on_imu(const lio::ImuInput&, const time::TimeSnapshot& now);
  Result<IngestOutputs, IngestReason> on_scan(lio::ScanInput&&, const time::TimeSnapshot& now);
  void stop() noexcept;   // LioRunner::stop(); idempotent
  IngestStats stats() const noexcept;
};
```

**Rules:**
- **Overlap:** an atomic try-enter comes first. An overlapping call returns `kReentered`, emits a rate-limited `LioIngestReentered` and touches nothing.
- **Stamps:** a stamp after `now.ros + kMaxStampAhead` gives `kStampInFuture` (rate-limited `LioInputRejected`).
- **Pending scan (O20):** on every call, first drop the scan if it has expired, then push it once the newest accepted IMU stamp is ≥ `scan.end`. Any runner refusal is `kRunnerRejected`; the core emits its own event.
- **States:** pass through `StateDecimator(kStatePeriod)` and are mapped per O21 with the last raw gyro. A `restart` output updates `last_reset`.
- **Events:** `LioScanDropped` (`kImuCoverageTimeout`/`kSuperseded`; values `scan_end_s`, `waited_s`); `IngestSummary` at `stop()`, with the stats and the runner's queue stats. This is the SITL measurement of `kBackendRequestCapacity` and `kMaxScansInFlight`.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(LioNodeConfig, SimYamlLoadsAndComposesBaseTImu) { /* node_sim.yaml + sim.yaml -> base_T_imu t = (0.011, 0.02329, 0.28 - 0.04412) */ }
  TEST(LioNodeConfig, UnknownAndOutOfRangeKeysAreRejected) { /* extra key -> kUnknownKey; yaw 4.0 -> kOutOfRange */ }
  TEST(StateDecimator, OnePerTenMillisecondsOfSensorTime) { /* 200 Hz +-1 ms jitter for 10 s -> 1000 +-1; a 0.37 s jump keeps the phase */ }
  TEST(LioIngest, ScanWaitsForImuCoverageThenIsPushed) { /* scan end 5 ms after newest IMU -> held; next IMU covers it -> pushed */ }
  TEST(LioIngest, ScanWaitingForImuIsDroppedAfterTheBoundedWait) {   // Review Focus 1
    // IMU stops at t; a scan ending t + 2 ms arrives at ROS r; a call at r + 99 ms keeps it, one at r + 101 ms drops it:
    // LioScanDropped IMU_COVERAGE_TIMEOUT, frontend scans_admitted unchanged; each call inside std::async, wait_for(10 s) ready
  }
  TEST(LioIngest, NewerScanSupersedesTheWaitingOne) { /* two uncovered scans -> first kSuperseded, second pushed once covered */ }
  TEST(LioIngest, FutureStampIsRefused) { /* now.ros + 0.6 s -> kStampInFuture, nothing pushed */ }
  TEST(LioIngest, ConcurrentCallsAreRefusedNotRaced) {   // Review Focus 4
    // two threads call on_imu 20'000 times each: every result is Ok, kReentered or kRunnerRejected (interleaved stamps arrive
    // out of order: the core refuses them); Ok count == frontend imu_accepted; TSan clean under make sanitize
  }
  TEST(LioIngest, StateIsBaseLinkPoseAndBodyVelocity) { /* yaw rate 1 rad/s, IMU 0.25 m from base: v_base == w x r (1e-6) */ }
  TEST(LioIngest, ResetDeltaIsLatchedOnLaterStates) { /* after a restart answer every later state carries it and reset_counter + 1 */ }
  TEST(LioIngest, ReachesTrackingOnASyntheticRoom) { /* room_scene, paced: TRACKING, >= 1 odometry, states 100 Hz +-1% of sensor time */ }
  TEST(LioIngest, DestroyWithQueuedWorkReturns) { /* 2 scans + 300 IMU queued; destroy in std::async -> ready in 10 s; IngestSummary */ }
  ```

  Run: `make build PKGS=uavnav_lio`. Expected: compile error, `ingest.hpp` missing.

- [ ] **Step 2: Implement.** Size check: `wc -l src/estimation/uavnav_lio/src/ingest.cpp`, at most 350 lines.

- [ ] **Step 3: Verify.** Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio && make sanitize`. Expected: all pass, no `WARNING: ThreadSanitizer`.

- [ ] **Step 4: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio config/lio/node_sim.yaml Makefile
  git commit -m "feat(lio): add the ROS-free ingest layer over LioRunner" \
    -m "SYSTEM_DESIGN §3.3/§3.5/§6.4, D31, O20, O21: bounded non-blocking wait for IMU coverage, 100 Hz state decimation on sensor time, base_link state mapping, refused overlapping calls; LioPipeline is never used." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 8: `lio` node: message conversion, executor, recorder, shutdown

Spec §3, §6.1, §6.4, §6.5; O20, O21. D30 carry-over: the backend joins before the channels die.

**Files:**
- Create under `src/estimation/uavnav_lio/`: `include/uavnav/lio_node/ros_conversion.hpp`, `src/ros_conversion.cpp`, `src/lio_node.cpp` (`LioNode` + `main`), `test/test_ros_conversion.cpp`, `test/test_lio_node.cpp`.
- Modify `package.xml` (`rclcpp`, `sensor_msgs`, `uavnav_interfaces`, `uavnav_ros`) and `CMakeLists.txt` (target `uavnav_lio_ros`, executable `lio_node`; both skipped under `UAVNAV_SANITIZE`).

**Interfaces:**

```cpp
enum class ConversionError : std::uint8_t { kBadStamp, kNotFinite, kWrongFrame, kMalformedCloud };
struct CloudStats { std::uint32_t points_in, points_non_finite; };
Result<lio::ImuInput, ConversionError> to_imu_input(const sensor_msgs::msg::Imu&);
Result<lio::ScanInput, ConversionError> to_scan_input(const sensor_msgs::msg::PointCloud2&, CloudStats& stats);
uavnav_interfaces::msg::LioState    to_msg(const LioStateOut&);    // covariances NaN, reset_delta_* latched
uavnav_interfaces::msg::LioHealth   to_msg(const lio::HealthOutput&);
uavnav_interfaces::msg::LioOdometry to_msg(const lio::OdometryOutput&);
class LioNode : public rclcpp::Node { public: explicit LioNode(const rclcpp::NodeOptions&); void shutdown() noexcept; };
```

- **`to_scan_input`:** requires FLOAT32 `x`,`y`,`z` fields, `point_step` ≥ 12, `data.size() == width·height·point_step`, and frame `kLidarFrameId`; drops non-finite points and counts them; sets `start = end = stamp` and `per_point_time = false`.
- **`LioNode` start:** (1) read the three paths; a missing or invalid file is one `RCLCPP_FATAL` and a non-zero exit, before any thread starts; (2) the recorder; (3) `LioIngest`; (4) one `MutuallyExclusive` group for both subscriptions.
- **Each callback:** (1) `SharedRosTime::store`, then one `rosutil::snapshot`; (2) convert; an error emits a rate-limited `LioInputRejected` with the `ConversionError`; (3) `LioIngest`; (4) publish.
- **`shutdown()`** follows the "Shutdown" row.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(LioRosConversion, ImuMessageToInput) { /* stamp, gyro, accel copied; orientation ignored */ }
  TEST(LioRosConversion, PointCloudDropsNonFinitePointsAndCountsThem) { /* 100 points, 7 inf -> 93, stats {100, 7} */ }
  TEST(LioRosConversion, RejectsGarbageMessages) {   // Review Focus 5
    // sec -1 or nanosec 1e9 -> kBadStamp; NaN accel -> kNotFinite; no "z" field, FLOAT64 x, point_step 8, data one byte short
    // -> kMalformedCloud; frame "base_link" -> kWrongFrame
  }
  TEST(LioRosConversion, OutputsMapToMessages) { /* LioState stamp, NaN covariances, reset_delta; LioHealth reason/reason_name;
     LioOdometry pose_covariance row-major == pose_cov, quality */ }
  TEST(LioNode, InProcessSyntheticRunPublishesAllTopics) {
    // test config paths; 10 s of room_scene IMU (200 Hz) and clouds (10 Hz) on /lidar/*; within 60 s: /lio/state 100/s +-5% over
    // the tracked span, /lio/health reaches TRACKING, >= 1 /lio/odometry, /events and the JSONL file carry StateTransition
  }
  TEST(LioNode, MultiThreadedExecutorKeepsCallbacksSerialised) {   // Review Focus 4
    // same node on a MultiThreadedExecutor (3 threads) for 5 s of streaming: zero LioIngestReentered, IMU states in order
  }
  TEST(LioNode, ShutdownJoinsTheBackendThenFlushesEvents) { /* shutdown() in std::async -> ready in 10 s; JSONL ends with IngestSummary */ }
  ```

  Run: `make build PKGS=uavnav_lio`. Expected: compile error, `ros_conversion.hpp` missing.

- [ ] **Step 2: Implement.** Then run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio`. Expected: all pass.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio
  git commit -m "feat(lio): add the lio ROS node over LioRunner with /lio/state, /lio/health, /lio/odometry and /events" \
    -m "SYSTEM_DESIGN §3.5/§6.1/§6.4/§6.5, O20, O21: the executor thread is the ingest thread, one callback group, garbage messages refused at the boundary, backend joined before the channels and the recorder." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 9: `Px4Bridge` core class: EV gate, alignment feeding, events (ROS-free)

Spec §4.1, D11, D21, D28, F13, F14; O22. Target `uavnav_px4_bridge_core`.

**Files:** under `src/px4/uavnav_px4_bridge/`:
- create `include/uavnav/px4bridge/bridge.hpp`, `src/bridge.cpp`, `test/test_bridge.cpp`;
- modify `include/uavnav/px4bridge/limits.hpp` (`kTickPeriod`, `kMaxStampAhead`, `kClockMode`) and `CMakeLists.txt`.

**Interfaces** (`uavnav::px4bridge`):

```cpp
enum class LioHealthCode : std::uint8_t { kInitializing = 0, kTracking, kDegraded, kLost, kRestarting };   // LioHealth.msg values
struct LioOdometryIn { time::SensorTime t; std::uint32_t epoch; Eigen::Vector3d p_flu_m; Eigen::Quaterniond q_flu;
                       Eigen::Vector3d v_flu_mps; Eigen::Matrix<double, 6, 6> pose_cov; Eigen::Matrix3d vel_cov; std::uint8_t quality; };
struct LioHealthIn { time::SensorTime t; LioHealthCode state; std::uint32_t epoch; };
struct Px4LocalPositionIn { Px4PoseSample sample; bool xy_valid; bool z_valid; };
enum class BridgeReason : std::uint8_t { kAccepted, kHealthNotTracking, kPx4PositionInvalid, kStampInFuture, kNotFinite };
struct BridgeOutputs { std::optional<EvSample> ev; std::optional<AlignmentOutput> alignment; };
class Px4Bridge {
 public:
  Px4Bridge(const AlignmentConfig&, events::EventRecorder&) noexcept;   // ClockMode = limits::kClockMode
  BridgeOutputs on_lio_odometry(const LioOdometryIn&, const time::TimeSnapshot& now);
  BridgeOutputs on_lio_health(const LioHealthIn&, const time::TimeSnapshot& now);
  BridgeOutputs on_px4_local_position(const Px4LocalPositionIn&, const time::TimeSnapshot& now);
  BridgeOutputs on_tick(time::SensorTime now_sensor, const time::TimeSnapshot& now);
};
```

- **Rules:** O22. The `/alignment` output rule is O21.
- **Future stamps:** a stamp after the newest tick time + `kMaxStampAhead` is refused; before the first tick nothing is refused as ahead.
- **Events** (component `kPx4Bridge`): rate-limited: `EvGated` (`kHealthNotTracking`), `EvRejected` (`EvReason`), `AlignmentPairRejected` (`kPairRejectedJump`/`kNoPx4Sample`/`kInputRejected`, with the residuals), `Px4SampleRejected` (`BridgeReason`), `LioInputRejected` (`kStampInFuture` or an out-of-order health); the rate limit keys on (input stream, reason): the first rejection of an episode is reported, later ones are counted, and a summary with the count is emitted when that stream's next input is accepted (so a second fault episode is never silent); one per change: `AlignmentTransition` (states and `AlignmentReason`); one per reset: `Px4ResetApplied` (the counters and deltas).

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Px4Bridge, EvOnlyFromTrackingOdometryWithItsOwnEpoch) { /* epoch 3 -> ev.reset_counter 3 (F14); quality 200 -> 0; FRD; us stamp */ }
  TEST(Px4Bridge, NewerNonTrackingHealthBlocksEv) { /* DEGRADED health (same epoch) at/after the odometry stamp -> no ev, EvGated;
     an older DEGRADED health does not block */ }
  TEST(Px4Bridge, LioLostStopsEvAndFreezesAlignment) {   // Review Focus 3
    // VALID after 20 pairs; PX4 streams at 50 Hz; health LOST -> FROZEN (kLioLost) in that call + AlignmentTransition; no ev for
    // later odometry of that epoch; after 10 s of ticks -> INVALID kFrozenTooLong; never kStale
  }
  TEST(Px4Bridge, DoubleResetDuringHoverKeepsTheVehiclePointContinuous) {   // Review Focus 2
    // VALID at p_lio (2, 1, -3); one PX4 sample: xy counter +2 (delta (1, 0)), heading counter +1 (dh 0.1) -> T' == G o T (1e-9),
    // Px4ResetApplied with both counters; the next 50 pairs in the new frame are all kPairAccepted
  }
  TEST(Px4Bridge, InvalidPx4PositionIsSkippedWithAnEvent) { /* xy_valid false -> not buffered, Px4SampleRejected; reset applied next valid */ }
  TEST(Px4Bridge, GarbageInputsAreRejectedWithEvents) {   // Review Focus 5
    // NaN odometry -> EvRejected kNonFiniteState, no pair; stamps now + 0.6 s -> kStampInFuture on each input;
    // regressing PX4 stamp -> AlignmentPairRejected kInputRejected; alignment state unchanged throughout
  }
  TEST(Px4Bridge, EveryAlignmentStateChangeIsOneEvent) { /* 5000 random mixed inputs: AlignmentTransition count == state changes */ }
  TEST(Px4Bridge, TickDrivesStaleTimeout) { /* VALID, 1.1 s of ticks without odometry -> INVALID kStale */ }
  ```

  Run: `make build PKGS=uavnav_px4_bridge`. Expected: compile error, `bridge.hpp` missing.

- [ ] **Step 2: Implement.** Then run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: all pass.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add src/px4/uavnav_px4_bridge
  git commit -m "feat(px4-bridge): add Px4Bridge: EV gate, alignment feeding and bridge events" \
    -m "SYSTEM_DESIGN §4.1, D11, D21, F14, O22: EV only from TRACKING odometry with its own epoch, non-TRACKING health freezes T, invalid PX4 samples skipped, one event per decision." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 10: `px4_bridge` node target

Spec §4.1, §6.4, §6.5; O19, O21, O22.

**Files:**
- Under `src/px4/uavnav_px4_bridge/`: create `include/uavnav/px4bridge/ros_conversion.hpp`, `src/ros_conversion.cpp`, `src/px4_bridge_node.cpp`, `test/test_ros_conversion.cpp`, `test/test_px4_bridge_node.cpp`, `test/test_core_target_has_no_ros.cpp`; modify `package.xml` (`rclcpp`, `px4_msgs`, `uavnav_interfaces`, `uavnav_ros`) and `CMakeLists.txt` (target `uavnav_px4_bridge_ros`, executable `px4_bridge_node`; the core target unchanged).
- Node rules from the T9 review: call `on_tick` once before creating the subscriptions (a future-stamped message before the first tick could hold the alignment in INIT); every bridge call comes from one mutually exclusive callback group (the core class has no overlap guard). Shutdown calls `Px4Bridge::flush(now)` before the recorder is destroyed so open rejection summaries are not lost. Deferred to O25 (S3/S4, when restart exists): odometry of an older epoch arriving after a newer-epoch health is not gated, so PX4's `reset_counter` could step backwards.
- Modify `src/core/uavnav_interfaces/msg/Alignment.msg`, comments only: `residual_*` is measured at the vehicle; `raw − filtered` includes the lever arm; `raw_*` is NaN when there is no pair.

**Interfaces:**

```cpp
Result<Px4LocalPositionIn, BridgeReason> to_px4_local_position(const px4_msgs::msg::VehicleLocalPosition&);  // from_px4_us
Result<LioOdometryIn, BridgeReason> to_lio_odometry_in(const uavnav_interfaces::msg::LioOdometry&);
Result<LioHealthIn, BridgeReason> to_lio_health_in(const uavnav_interfaces::msg::LioHealth&);
px4_msgs::msg::VehicleOdometry to_vehicle_odometry(const EvSample&, std::uint64_t timestamp_us);  // angular_velocity NaN
uavnav_interfaces::msg::Alignment to_msg(const AlignmentOutput&);                                // raw_* NaN when absent
```

The node class is declared in `include/uavnav/px4bridge_node/px4_bridge_node.hpp` (the executable's `main` names no ROS type). It reads `config_file` and `events_jsonl`. Known limit (next to O25): two PX4 resets inside one window of dropped (invalid) samples apply only the last delta; the alignment's jump gate is the backstop. It runs one callback group and a 100 ms ROS-clock timer that calls `on_tick`, with the topics and QoS of the runtime design.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Px4BridgeRos, LocalPositionConversion) { /* timestamp_sample 12'345'678 us -> 12.345678 s; counters, deltas, flags copied */ }
  TEST(Px4BridgeRos, RejectsGarbagePx4Messages) {   // Review Focus 5
    // timestamp_sample UINT64_MAX -> kNotFinite (from_px4_us kOutOfRange); NaN heading or x -> kNotFinite;
    // LioOdometry stamp nanosec 1e9 -> rejected; LioHealth state 9 -> rejected
  }
  TEST(Px4BridgeRos, VehicleOdometryFields) { /* frames 2/2, angular_velocity NaN, timestamp and timestamp_sample */ }
  TEST(Px4BridgeRos, AlignmentMessageRawIsNanWhenAbsent) { /* raw nullopt -> raw_* NaN; state value == enum value */ }
  TEST(Px4BridgeNode, PublishesEvAndAlignmentFromLioAndPx4Streams) {   // in process; plus MultiThreadedExecutorKeepsCallbacksSerialised and one child-process ExecutableStartsAndStopsOnSigintWithTheSummaryLast
    // /lio/health TRACKING, /lio/odometry 10 Hz, /fmu/out/vehicle_local_position_v1 50 Hz consistent with T = (1, 2, 0, 0.3) for 5 s:
    // /alignment VALID with |x - 1| < 0.05; /fmu/in/vehicle_visual_odometry at 10 Hz +-10%; waits <= 60 s
  }
  TEST(Px4BridgeLibrary, CoreTargetHasNoRosDependency) { /* core headers (not ros_conversion.hpp) and core sources contain no
     "rclcpp", "px4_msgs", "uavnav_interfaces/msg" */ }
  ```

  Run: `make build PKGS=uavnav_px4_bridge`. Expected: compile error, `ros_conversion.hpp` missing.

- [ ] **Step 2: Implement.** Then run: `make build PKGS="uavnav_px4_bridge uavnav_interfaces" && make test PKGS="uavnav_px4_bridge uavnav_interfaces"`. Expected: all pass.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add src/px4/uavnav_px4_bridge src/core/uavnav_interfaces/msg/Alignment.msg
  git commit -m "feat(px4-bridge): add the px4_bridge node publishing EV and /alignment" \
    -m "SYSTEM_DESIGN §4.1/§6.4/§6.5, O19, O21, O22: thin node over Px4Bridge; PX4 QoS best effort, versioned PX4 topic names, timestamp_sample through the range-checked converter." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 11: `uavnav_bringup` launch, the hover harness and the first S1 hover

Spec §7.2 (S1 check), §6.1, D14; D33 (O19, O24). It needs the owner's machine time with nothing else building. The probe (Task 3) has already fixed the hover mode: POSCTL with the stick stream, or the approved Hold fallback.

**Files:**
- Create: `src/uavnav_bringup/{package.xml,CMakeLists.txt}`; `src/uavnav_bringup/launch/s1_estimation.launch.py`; `tools/uavnav/sitl_hover.py` (rclpy driver and recorder).
- Modify: `tools/uavnav/sitl_hover.sh` (adds the launch and the hover mode); `tools/uavnav/sitl_checks.py` (`evaluate_hover`) and `tools/uavnav/tests/test_sitl_checks.py`; `Makefile`: `SITL_PKGS := uavnav_bringup uav_simulation uav_description uavnav_lio uavnav_px4_bridge`, target `sitl-build`, target `sitl-hover` (kept out of `test` and `gate`), and `uavnav_ros uavnav_lio` added to the `PKGS` default; `README.md` and `.agents/skills/build-and-test/SKILL.md`.

**Interfaces:**
- **Launch.** `s1_estimation.launch.py` takes four required arguments with no defaults: `lio_config`, `lio_node_config`, `bridge_config` and `events_dir`. It starts the two gz bridges of Task 3, `lio_node` and `px4_bridge_node`, all with `use_sim_time:=true`.
- **`make sitl-hover`** takes `PROFILE=gps|no_gps`, plus optional `[SPAWN_YAW_RAD=<rad>]` and `[PATTERN=hover|altitude_sweep]`. The script behaves as in Task 3, with these differences: it adds `ros2 launch uavnav_bringup s1_estimation.launch.py`; it runs `sitl_hover.py`; it is bounded at 300 s.
- **`sitl_hover.py` phases:** (1) `wait_ready` (≤ 90 s): LIO TRACKING, `/alignment` VALID, `cs_ev_pos` and `xy_valid`; (2) Mode and `arm`, per the probe result; (3) `climb` (≤ 20 s) to z ≤ −2.5 m; (4) `hover` for 60 s. This is the evaluation window; (5) `land`, with ≤ 30 s to disarm.
- **Recording.** JSONL of: `/lio/state` (arrival times); `/lio/health` (this includes the output tracking error, which S2 uses for the τ re-evaluation); `/lio/odometry` (receive time − stamp); `/alignment`; `/fmu/out/vehicle_local_position_v1`, `/fmu/out/estimator_status_flags` and `/fmu/out/vehicle_status_v1`; `/sim/ground_truth/odometry`.
- **Checker.** `sitl_checks.evaluate_hover(run_dir: pathlib.Path, profile: str, spawn_yaw_rad: float) -> dict` returns `{"result": "PASS"|"FAIL", "checks": {...}}` and writes `summary.json`. A check whose data is missing fails.
- **Criteria** (over the 60 s window unless noted):

  | Area | Pass when |
  |---|---|
  | EKF2 | `cs_ev_pos` in ≥ 95% of samples; `cs_ev_yaw` false throughout (gps) or true in ≥ 95% (no_gps, D20) |
  | LIO | TRACKING for 100% of the window; no LOST and at most 2 TRACKING entries in the run |
  | LIO flapping (owner 2026-10-13: accept the O15 flapping band, check the rate in SITL) | the number of DEGRADED→TRACKING transitions (from `StateTransition` events) is reported with the p50/p99 of the result age (`ScanResultTooOld.age_s` and the age of applied results); pass when ≤ 1 per 60 s of the window. A larger rate fails and stops for the owner (options: reset the DEGRADED confirmation run on a tick with gap > `gap_degraded`, or tie `gap_degraded` to the measured latency) |
  | Alignment | VALID throughout; `/alignment` ≥ 9 Hz; `PAIR_REJECTED_JUMP` ≤ 1% of pairs; no `STALE`; once the alignment has been VALID in the run, VALID after each `Px4ResetApplied` (a reset while still INIT is reported, not judged) |
  | Drift | PX4 horizontal drift against ground truth (ENU→NED, from the window start) ≤ 0.3 m |
  | `/lio/state` | 100 ± 5 Hz; inter-arrival p99 ≤ 20 ms (P7: not starved); the F23 step max \|Δp − v·Δt\| is reported |
  | EV age | ROS now − `timestamp_sample`, p99 ≤ 150 ms (< `EKF2_DELAY_MAX`) |
  | PX4 time | \|`vehicle_local_position.timestamp` − ROS now\| p99 ≤ 20 ms (D29) |
  | Events (whole run) | zero `LioQueueOverflow`, `LioBackendFailed`, `ImuRateTooHigh`, `EventsDropped`; `LioScanDropped` ≤ 1% of scans; the `ImuGap` count is reported; `LioNodeSummary` has `rejected_wrong_frame == 0` and `rejected_malformed_cloud == 0` (confirms the Gazebo cloud layout and the IMU frame); the count of `Px4SampleRejected` with reason NOT_FINITE is reported and expected 0 (if non-zero, SITL decides whether the NaN rule must mask by validity flag); the point rate received at the lio node is reported (the probe saw ≈ 9 of 10 Hz) |
  | F34 values (reported here, judged in Task 12) | `yaw_T` max − min over the window; `yaw_T` − (π/2 − spawn yaw) |

- [ ] **Step 1: Write the failing tests** (`SitlChecksTest`): `test_hover_pass_on_synthetic_records`; `test_fails_on_lio_lost` (`lio_tracking`); `test_fails_on_ev_fusion_gap` (`ekf2_ev_pos`); `test_fails_on_degraded_tracking_flapping` (`lio_flap_rate`: 3 DEGRADED→TRACKING transitions in 60 s fail; 0 pass; the rate and age percentiles are reported); `test_px4_time_identity_violation_fails` (50 ms offset); `test_hover_missing_topic_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `AttributeError` for `evaluate_hover`, `GATE_RESULT=FAIL`.

- [ ] **Step 2: Implement.** Then run:

  ```bash
  tools/uavnav/gate.sh python
  make sitl-build && source install/setup.bash && ros2 launch uavnav_bringup s1_estimation.launch.py --show-args
  ```

  Expected: `GATE_RESULT=PASS`; `--show-args` lists the four required arguments.

- [ ] **Step 3: First hover** (owner machine time, about 5 min). Run: `make sitl-hover PROFILE=gps`.
  - Expected: `summary.json` has `"result": "PASS"`.
  - On failure, read `python3 -m tools.uavnav.events log/sitl/<run>/events_lio.jsonl --timeline` (and `events_px4_bridge.jsonl`) before you change code.
  - Then make at most one fix-and-re-run cycle per root cause. A design deviation becomes a new O-entry; never patch around it.

- [ ] **Step 4: Commit** (the artifacts stay in `log/`; the summary goes into the body)

  ```bash
  git diff --cached --stat
  git add src/uavnav_bringup tools/uavnav/sitl_hover.sh tools/uavnav/sitl_hover.py tools/uavnav/sitl_checks.py \
          tools/uavnav/tests/test_sitl_checks.py Makefile README.md .agents/skills/build-and-test/SKILL.md
  git commit -m "test(sitl): add uavnav_bringup and the bounded S1 hover smoke" \
    -m "SYSTEM_DESIGN §7.2 (S1), D33 (O19, O24): hover in PX4 Position mode (or the approved Hold fallback), EKF2 fuses EV, T logged; first run gps: <paste the summary.json checks>." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 12: F34 check in SITL and report

Spec §4.1, §7.2 ("Kiểm giả thuyết F34"); F33, F34, D20; O24.

**Hypothesis (F34).** `main` sent EV labelled NED without rotating it, with EV yaw fused. But `lio_odom` points along the start heading, not north, so EV yaw competed with mag/GNSS, and the translation-only EV bias cannot absorb a rotation. That is a candidate root cause of the LIO–PX4 drift (P3).

**Decision rule.** F34 is closed by the FRD design when all four runs pass Task 11 **and** `T` absorbs the heading offset over the window:
- max − min of `yaw_T` ≤ 1°;
- gps profile, yawed runs: |`yaw_T` − (π/2 − spawn yaw)| ≤ 3°. π/2 − spawn yaw is the NED heading at LIO start for an ENU spawn yaw.
- no_gps profile (D35): EKF2 takes its heading from EV yaw, so |`yaw_T`| ≤ 3° with EV yaw fused in ≥ 95% of the window.

Otherwise F34 stays open: the report names the failing check, and it becomes an owner item.

**Files:** create the Vietnamese report docs/analysis/2026-10-s1b-f34-check.md: method, the four runs, the table of checks, the event counts and the conclusion.

- [ ] **Step 1: Four hovers** (owner machine time, about 20 min, one at a time)

  ```bash
  make sitl-hover PROFILE=gps
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2
  make sitl-hover PROFILE=no_gps
  make sitl-hover PROFILE=no_gps SPAWN_YAW_RAD=1.2
  ```

  Expected: four `summary.json` files with `"result": "PASS"`, the `yaw_T` spread and the offset values.

- [ ] **Step 2: Write the report.** Run: `python3 tools/check_documentation.py . docs`. Expected: `PASS`.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add docs/analysis/2026-10-s1b-f34-check.md
  git commit -m "docs(analysis): report the F34 SITL check" \
    -m "SYSTEM_DESIGN §4.1/§7.2, F34, D20: four hovers (gps, no_gps; spawn yaw 0 and 1.2 rad); EV FRD fused, T absorbs the start heading: <result>." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 13: Degeneracy threshold calibration from SITL logs (O6, §3.2)

Spec §3.2 ("lấy từ log trị riêng trên scene mở và scene suy biến (bay cao) trong SITL"), D20, D28, F36.

**Method:**
- **Flight:** `PATTERN=altitude_sweep`, gps profile only, because EV is lost when LIO degrades. The vehicle climbs in 2.5 m steps to 20 m, holds each step 10 s, then descends.
- **Why the information drops:** the Mid-360 sees nothing below −7° (F36), so the 5 m walls of `px4_lio_smoke` leave its view as it climbs.
- **Data:** per scan, the two minimum eigenvalues from the `LioScan` events; per 1 s window, the drift |Δp_lio − Δp_gt| after `T`.
- **Threshold:** for each block, the smallest eigenvalue level above which drift ≤ 0.05 m in 95% of windows. Scans after the first LOST are excluded.

**Files:**
- Create `tools/uavnav/lio_calibration.py`, `tools/uavnav/tests/test_lio_calibration.py` and the Vietnamese report docs/analysis/2026-10-s1b-degeneracy-calibration.md.
- Modify: `tools/uavnav/sitl_hover.py` (pattern `altitude_sweep`; its checker runs the data checks only); `config/lio/sim.yaml` (the two thresholds); `src/estimation/uavnav_lio_core/include/uavnav/lio/config.hpp` (the derivation comment cites the report instead of "S1b recalibrates").

**Interfaces:**

```python
def scan_eigenvalues(events) -> list[tuple[float, float, float]]          # (sensor_time_s, translation, rotation)
def drift_windows(lio, truth, window_s=1.0) -> list[tuple[float, float]]    # (window end, drift m)
def recommend_thresholds(scans, drift, max_drift_m=0.05, quantile=0.95) -> dict   # {"translation", "rotation", "support"}
```

- [ ] **Step 1: Write the failing tests and implement** (`LioCalibrationTest`): a synthetic sweep whose drift exceeds 0.05 m below 2e5 and 3e6 gives those values (±1 grid step); fewer than 50 windows per side raises `ValueError`.

  Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=PASS` after implementing.

- [ ] **Step 2: Sweep** (owner machine time, about 6 min). Run: `make sitl-hover PROFILE=gps PATTERN=altitude_sweep`.

- [ ] **Step 3: Report and config.**
  - The report covers: eigenvalue distributions per altitude step; the recommended pair; the D28 note (published odometry always has quality ≥ 50, so `EKF2_EV_QMIN` cannot matter at beta); `ImuGap` counts (the "an IMU gap makes the scan degenerate" policy); the floor-only eigenvalue.
  - Set the pair in `config/lio/sim.yaml`.
  - Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sitl-hover PROFILE=gps && python3 tools/check_documentation.py . docs`.
  - Expected: tests pass (a test that pins the old values is changed to read `config/lio/sim.yaml`); the hover passes; documentation `PASS`.

- [ ] **Step 4: Commit**

  ```bash
  git diff --cached --stat
  git add tools/uavnav/lio_calibration.py tools/uavnav/tests/test_lio_calibration.py tools/uavnav/sitl_hover.py config/lio/sim.yaml \
          src/estimation/uavnav_lio_core/include/uavnav/lio/config.hpp docs/analysis/2026-10-s1b-degeneracy-calibration.md
  git commit -m "feat(lio): calibrate the degeneracy thresholds from a SITL altitude sweep" \
    -m "SYSTEM_DESIGN §3.2, D20, O6: thresholds = smallest information with 1 s drift <= 0.05 m in 95% of windows: <values>; tier (b) change only." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 14: Close S1b: traceability, gate, carry-over to S2

**Files:** modify `docs/TRACEABILITY.md`, `README.md`, `.agents/skills/build-and-test/SKILL.md`, and this plan (append "Carry-over to S2").

- [ ] **Step 1: Run the full gate and a final hover.**

  Run: `tools/uavnav/gate.sh all && make sanitize && make sitl-hover PROFILE=gps`

  Expected: `GATE_RESULT=PASS`, no TSan warning, hover `PASS`.

- [ ] **Step 2: Update `docs/TRACEABILITY.md`.** The WP column is `S1b-T<n>`. The evidence is test names, commits and `summary.json` values.

  | Row | Status | Evidence |
  |---|---|---|
  | P2 | doing | `Px4ProfileTest.test_d20_ev_ctrl_per_profile`, hovers in both profiles (Task 12); mission frames are S2 |
  | P3 | doing | `Px4Bridge.DoubleResetDuringHoverKeepsTheVehiclePointContinuous`, SITL `T` VALID and drift ≤ 0.3 m; px4_mode is S2 |
  | P7 | done | `StateDecimator.OnePerTenMillisecondsOfSensorTime`, `LioNode.InProcessSyntheticRunPublishesAllTopics`, SITL `/lio/state` rate and p99 |
  | F14 | done | `Px4Bridge.EvOnlyFromTrackingOdometryWithItsOwnEpoch` |
  | F22 | done | add the SITL hover: no spurious LOST |
  | F23 | doing | the hover's reported F23 step; the τ re-evaluation is S2 |
  | F34 | per the Task 12 rule | the F34 report |
  | Event log, Config ba tầng, Message v2 | per evidence | `RosEventSink.PublishesEveryRecordFieldByField`, `LioNodeConfig.*`, `AlignmentConfig.SimYamlLoads`, the conversion tests |

  Also update the D33 code rows (added on 2026-10-12, status `todo`). Set each to `done` with its evidence.

  Run: `python3 tools/check_documentation.py . docs`

  Expected: `PASS`.

- [ ] **Step 3: Append "Carry-over to S2" to this plan.** Give each item with its file and task. It must include: the τ re-evaluation (O13), using `/lio/health` output tracking error from S2 flights; the seed helper `T⁻¹ × PX4 pose` (S4, with its caller).

  Then commit:

  ```bash
  git diff --cached --stat
  git add docs/TRACEABILITY.md README.md .agents/skills/build-and-test/SKILL.md docs/plans/2026-10-12-s1b-ros-shells-and-sitl.md
  git commit -m "docs(design): close S1b traceability and carry-over to S2" \
    -m "TRACEABILITY P2/P3/P7/F14/F22/F23/F34 and the D33 code rows updated with test and SITL evidence." \
    -m "Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

## Carry-over coverage

Every open item of both carry-over lists is scheduled here, or listed under "Not in this plan".

| Where | Carry-over items |
|---|---|
| D33 | D30 owner items 1–4 (= O15–O18), 6 and 8; the S1a-T8 and D30 interface additions |
| Task 1 | O15 |
| Task 2 | O16; `seed_age_s` > 0.3; `reset_to` with a stale snapshot rewinds; `reset_counter == epoch − 1`; the "clear map, then seed" comment |
| Task 3 | the SITL time-identity check (D29); the Gazebo cloud layout |
| Task 4 | the duplicated helpers (both lists); the `SpscQueue` `sizeof` note |
| Task 5 | the PX4 µs → SensorTime converter; `kBadQuaternion`; the non-unit quaternion test; the FROZEN→VALID reason; `config/px4_bridge/sim.yaml` |
| Task 7 | the bounded wait and drop; `StepOutputs` last wins; a non-blocking `LioClock`; measuring `kBackendRequestCapacity` and `kMaxScansInFlight` |
| Task 8 | joining the backend before the channels |
| Task 9 | EV only from TRACKING with its own epoch and quality; `tracking=false` from health; the PX4 validity gate; events from `AlignmentReason`; the `on_tick` and future-stamp guard |
| Task 10 | the `Alignment.msg` comment; `raw_*` when there is no raw pose |
| Task 11 | the `README.md` `PKGS` default; 100 Hz not starved; OpenMP and ikd latency; the IMU rate; dropped IMU copies; a stalled backend against `gap_degraded`; heading resets below jump/τ |
| Task 12 | F34 |
| Task 13 | the thresholds; the floor-only eigenvalue; the IMU-gap degenerate policy; D28 constant quality |

Already closed:
- the rebase covariance cap: `kRebaseMaxCovarianceEigenvalue` in `backend_math.cpp`;
- persistent `PredictorCorrectionRejected` degrading the state: D31, `LioFrontend.RejectedCorrectionDoesNotHoldTracking`.

## Not in this plan (moves to S2+ or owner)

| Goes to | Items | Reason |
|---|---|---|
| S2 | τ re-evaluation (O13; TRACEABILITY τ row) | Needs flights with real motion. S2 flies straight segments at 1–5 m/s, while S1 only hovers. Every S1b hover already records the output tracking error. |
| S3/S4 | `/lio/scan`; the restart caller, the seed helper `T⁻¹ × PX4 pose` and `T` in `LioRestart`; LOST recovery against a stale map; restart velocity and covariance; re-bootstrap after a degenerate bootstrap; logging the pending correction dropped at restart; the `LocalMapUpdate` event; the failed-registration reason; the SITL scenario with a PX4 reset during FROZEN plus a LiDAR gap; a reset lost in a dropped PX4 message | D33/O25 defers the consumer and the restart caller. The S4 fault injection exercises them. A helper without its caller would be dead code in S1b. |
| S4 lifecycle clean-up | `LioLifecycle::go()` silent on an unlisted edge; the reserved `kScanAccepted`/`kScanEmpty`; the `default` branch of `kind_of`; the `MathConfig` alias | S1b adds no lifecycle edge. |
| Hardware (P8) | the Livox `CustomMsg` adapter and its g scaling; the realtime PX4 time mode; `to_px4_us` returning `Result` | Out of the beta. `to_px4` already guarantees ns ≥ 0. |
| After the beta (O4) | per-IMU heap allocation; `notify_one` per push; the libgomp test baseline; the clamp on the 1e6 m bound of the reset deltas | These are measurements, not behaviour. |
| Architect / owner | spec sync of §3.2 hysteresis, §3.1 TRACKING→LOST on σ, and §4.1 INVALID→INIT; `vertical_velocity_mps()`, which needs a message field; the predictor's `kNoSampleAtStamp`, anti-windup and the dt > 300 ms drop (after the S2 τ work); extrinsic orthonormality in `create`, since the node builds rotations from roll, pitch and yaw; the `a39c806` message count, since history is kept; D32 marker coverage | Each item is a decision or a text change, not S1 code. |
| Hygiene WP | the moving, noisy scene with per-point time; the off-grid health-grid and `latest()` tests; the information-matrix tests; the quality-49 note; config-parsing edge cases; the event-recorder test items; the exactly-1e6 m boundary and the origin-gate counter; the hang-to-timeout guards; the cleaner test gaps; the `LioPipeline` restart error path; the float-overflow restart set-up; the 20 ms sleep check; the Release `assert` in `LioRunner::backend()`; fault injection for a result that never arrives; lines over 100 columns | Test quality only. There is no behaviour change. |
| Tooling / S2 | `make sanitize` for `fast_lio_core` (uninstrumented vendor math) and `uavnav_px4_bridge` (single-threaded under one executor); the aborted-restore note; the `MissionDefinition` comment (S2); the dead `fast_lio_core` headers, which are still referenced | S1b does not depend on them. |

## Model and execution notes

- **Model per task:**
  - most capable model: Tasks 1 and 2 (lifecycle and restart safety), 3 (the SITL probe decides the fallbacks), 7 (ingest concurrency and the wait), 8 (executor and shutdown), 9 (EV gate and alignment feeding), 11 (SITL harness and diagnosis) and 12 (the F34 judgement);
  - capable model: Tasks 10 and 13;
  - mid-tier model: Tasks 4, 5, 6 and 14.
- **Order:**
  - Run Tasks 1 → 14 strictly in order, one active task at a time (CLAUDE.local.md). Each task ends green and is reviewed before the next.
  - The probe (Task 3) runs early on purpose: a wrong SITL assumption stops the plan before any node is written. Hold is the approved fallback; any other failure needs a new O-entry.
  - The first S1 hover is Task 11, the earliest point at which both nodes exist.
  - Read-only reviewers may run alongside, at most 3 agents in total.
- **Machine limits:**
  - Builds run only through the Makefile (`-j3`, sanitize `-j2`).
  - SITL never overlaps a build, a test run or `make sanitize`.
  - Each SITL run takes about 5 min, about 50 min in total: the Task 3 probe (plus at most one re-run), the Task 11 hover (plus at most one re-run), four hovers in Task 12, one sweep and one re-check in Task 13, and one hover in Task 14.
  - These runs need the owner's machine time on an idle machine. Agents prepare the commands and evaluate the artifacts.
- **TSan:** Task 7 adds `uavnav_lio` (the ingest library and its tests only) to `make sanitize`; `setarch -R` stays.

## Carry-over to S2

Each item names where it lives and who closes it. 

| # | Item | Where | Closes in |
|---|---|---|---|
| 1 | τ re-evaluation (O13) from the output tracking error in `/lio/health`; the F23 step with real motion | every S1b hover records it (`log/sitl/*/records`); TRACEABILITY τ row | S2 flights at 1–5 m/s |
| 2 | Degeneracy thresholds: closed by D36 (degeneracy in an empty map is normal; beta pair kept). The tool `tools/uavnav/lio_calibration.py` and `PATTERN=altitude_sweep` stay as diagnostics only | `docs/analysis/2026-10-s1b-degeneracy-calibration.md` | closed |
| 3 | DV1 re-watch: the O15 age band (0.145–0.25 s) was never exercised (applied-result age p99 28 ms at hover) | TRACEABILITY DV1 (CLOSED, with this note) | S2 |
| 4 | `NO_PX4_SAMPLE` rejections vary per run (0 to about 19% of pairs) with the alignment still VALID; cause not investigated | `docs/analysis/2026-10-s1b-f34-check.md` §4 and §9, run `20261013T155144Z-no_gps-hover` | first S2 task touching the alignment |
| 5 | no_gps heading reset of about 20° when EV yaw starts (D35): it came before the alignment was VALID; the `T` reset handling (D30/O11) must absorb a step that large if it comes later | `docs/architecture/DECISIONS.md` D35 | S2 / S4 |
| 6 | Stale-epoch odometry gate and the two-resets-in-one-dropped-window limit | O25 | S3/S4 (restart) |
| 7 | `/lio/scan`, the restart caller, the seed helper `T⁻¹ × PX4 pose` with `T` in `LioRestart` | O25 | S3/S4 |
| 8 | Hardware: two lost scans in a row trip DEGRADED at `gap_degraded` = 0.25 s; the Fast DDS profile applies to SITL only; real Mid-360 noise and Ethernet transport | `config/ros/fastdds_sitl.xml`, README SITL section | hardware phase (P8) |
| 9 | Unverified in flight: `DO_SET_MODE` (accepted when disarmed only), whether POSCTL comes from the stick stream or PX4's default; the `ImuGap` policy never fired; floor-only scene | `docs/analysis/2026-10-s1b-f34-check.md` §10 | S2 |
| 10 | Deferred review minors: T11 M2/M3/M6/M8 (inter-arrival in /clock time, `ekf2_ev_pos` weighted by sample, private launch_ros attributes in a test, the probe re-run with the DDS profile); T8 M4 (diagnostic log of the first WRONG_FRAME / MALFORMED_CLOUD); T3 minors in `progress.md`; T7 deferred list in `progress.md`; `kEventSummaryEvery` duplicates the lio constant | `.superpowers/sdd/2026-10-12-s1b-ros-shells-and-sitl/progress.md` | when next touching the file |
| 11 | Governance: classify the legacy `navigation_*` packages in `tools/uavnav/arch_rules.yaml` (7 warnings); recalibrate `stress:` thresholds after S2; run `python3 -m tools.uavnav.arch_health` at slice end | `docs/CHANGE_CONTROL.md` | S2 close |

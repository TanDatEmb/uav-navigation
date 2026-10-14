# S1c v2: EV Fully Fused, Corrections by Consequence, LIO That Never Gives Up, Drift Benchmark

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Before S2, give the estimation stack the owner's decisions D40–D43, at the minimum that satisfies them under D44:
- **D42:** a compass always and EV fused in full in both profiles. Three SITL runs measure what PX4 does.
- **D40:** LIO never refuses a correction for its age. Only the predictor's technical refusal remains.
- **D38, D41:** LIO waits in LOST without limit and restarts itself after a fixed dwell. It seeds from its own predictor, and `T` re-initialises for the new epoch.
- **D39:** a drift benchmark at 1, 3 and 5 m/s, compared with the datasheets.

**Architecture:**
- No new state, message, thread or flag.
- The recovery chain is three explicit rules on existing machines:
  - `LioLifecycle`: LOST → RESTARTING after `restart_dwell_s` of scans that arrive and are not empty; INITIALIZING with a ready map but no confirmation after `restart_dwell_s` → LOST;
  - `LioFrontend`: while RESTARTING with no restart in flight, it issues one, seeded with the predictor's newest pose;
  - `AlignmentEstimator`: `T` belongs to one LIO epoch, and a pair of a newer epoch sends it to INIT.
- Design and the reasons for each cut: [critique](../analysis/2026-10-s1c-design-critique.md), AR4, O34 and O35.

**Tech Stack:** C++23, Eigen 3, ament_cmake, GoogleTest, rclcpp (Jazzy), px4_msgs v1.17.0, Gazebo Harmonic, PX4 v1.17 SITL, Python 3 (`unittest`, `rclpy`, numpy 1.26), GNU make.

**Spec:** [SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md):
- §0 (profiles, D42), §3.1 (lifecycle), §3.4 (restart, chain, budget table), §3.5 (stamp bound), §4.1 (EKF2 parameters, `T` per epoch), §7.2 (S1c).
- [DECISIONS.md](../architecture/DECISIONS.md): D4, D11, D20, D30, D31, D33, D36, D38–D45; AR4 (supersedes AR3); open O34, O35; F33, F36.
- Invariants in [CHANGE_CONTROL.md](../CHANGE_CONTROL.md) §4: LIO-1..LIO-4, PX4-1, PX4-2.
- Predecessor: the "Carry-over to S2" table of the [S1b plan](2026-10-12-s1b-ros-shells-and-sitl.md).

---

## Scope

**In this slice:**
- the EV-full profile and its heading measurement (D42);
- the removal of the age rule (D40);
- the self-restart chain (D41), and `T` per epoch;
- fault injection by LiDAR cut and climb;
- the drift benchmark (D39);
- the as-built drawings of what is built (D45).

**Not in this slice:**

| Item | Goes to | Why not now |
|---|---|---|
| PX4 seed offer, `LioRestartSeed`, seed `SYNC` (old O29) | dropped (O34) | a seed for a fresh map is a gauge; `T` INIT carries the PX4 information |
| Resync window, `alignment_resync_window_s` (old O33) | dropped (O34) | `T` per epoch replaces it |
| Scan self-information, innovation gate, six-value verdict, `kScanAppliedNotFresh` | dropped (AR4) | proxies without measured meaning; tuned guesses (D44.3, D44.7) |
| EV covariance inflation | dropped | below PX4's floor `EKF2_EVP_NOISE`; drift is not observable inside LIO (critique §1a) |
| `kClockMode` of the lio node, Livox clock, transport, per-point timing, IMU bias, dynamic objects | hardware (P8) | the simulation has one clock, no bias and no per-point time |
| Corridor or repetitive-structure world, compass fault injection | after S2 | not needed to decide anything in S1c |
| Supervisor give-up, `px4_mode`, handover reasons; `px4_mode` behaviour while `T` is INIT (critique C5) | S2 | no consumer of `T` before S2 |
| `/lio/scan`, Mapping, `ResetMap` | S3 | no consumer before Mapping (O25) |
| Removing the 100 ms scan wait | dropped | at 10 Hz "superseded" and "waited 100 ms" coincide; §3.5 states the consequence |

## Owner gates

| Gate | Recommended | Blocks |
|---|---|---|
| **AR4** (`Owner xác nhận` = `có`, result `APPROVED`) | AR4 as written | Tasks 2 and 3 (Level 3) |
| **O34**: LIO seeds itself; `T` per epoch | (a) | Task 3 Step 3 (the seed), Task 4 Steps 1–3 |
| **O35**: one owner for the LOST budget; `T` loses `frozen_max` | (a) | Task 4 Step 4 only |

A decision different from the recommendation is a plan change: edit the task before dispatching it.

## Global Constraints

- **Tree and commits:** single tree; `git add <path>` only; `git diff --cached --stat` before every commit; Makefile builds at `-j3` (sanitize `-j2`).
- **Trailers:**
  - A commit that touches a contract path or an invariant file ends with `Arch-Level:` and `Docs-Impact:`, then `Co-Authored-By: <model> <noreply@anthropic.com>`.
  - Tasks 2 and 3 are Level 3: they add `Review: AR4` and `Decision: D40` (Task 2) or `Decision: D41` (Task 3).
  - An executor never declares a level above what this plan states. It stops and reports.
- **Task report (D34, D45):** it names the invariants touched, the flags and states added (target: none), and the workarounds added. It ends with an `As-built:` line: the AS_BUILT section updated, or why none was needed.
- **As-built (D45):**
  - A task that finishes a product part redraws that package's section of [AS_BUILT.md](../architecture/AS_BUILT.md) in the same task: the Mermaid diagram, the state and edge tables copied from the code (`kLioTransitions`, `kAlignmentTransitions`), evidence, deviations, and `Verified at: <sha>`.
  - A TRACEABILITY row goes to `INTEGRATED` only after the AS_BUILT section of every package it names is current at or after the commit that the row's evidence cites.
- **SITL (D43):**
  - Agents run SITL themselves, one run at a time, never during a build, a test or `make sanitize`.
  - Headless; artifacts only under `log/sitl/`.
  - Bounds: 300 s per hover or recovery run; 600 s per drift run.
  - Every cited run id goes into docs/analysis/2026-10-s1c-sitl-runs.md (created by Task 1).
- **D40:** a refusal states the harm it prevents, in its doc comment and in its event reason. An age appears only as a measured value.
- **D41, D44:**
  - no attempt counter and no terminal state in `lio`;
  - every wait is a fixed dwell in sensor time;
  - no rule changes its own threshold.
- **D42:** no code path drops a valid EV sample.
- **Config (D22):** new tier (b) keys go in the spec tables with bounds; tier (a) constants go in `limits.hpp` with a derivation; profile values only in `config/px4/profiles/*.yaml`.

## Review Focus

1. **A saturated backend** (results aged 0.27 s, every 100 ms). LIO stays DEGRADED with a corrected output: no confirmation, no flapping, no LOST, no EV. Pinned by `LioFrontend.SaturatedBackendStaysDegradedWithACorrectedOutput` (Task 2).
2. **The edge of geometry** (non-empty but degenerate scans). Attempts are at least two dwells apart, and there is none while the scans are empty. Pinned by `LioRunner.EdgeOfGeometryAttemptsAreAtLeastTwoDwellsApart` (Task 3) and the `climb_loss` run (Task 5).
3. **A new epoch while `T` is FROZEN.** `T` goes to INIT and is never VALID on a pair of another epoch. Pinned by `Alignment.NewerEpochReinitialisesFromFrozen` (Task 4).
4. **Old-epoch odometry after new-epoch health.** It is refused, so `T` cannot alternate between epochs. Pinned by `Px4Bridge.OldEpochOdometryAfterNewEpochHealthIsRefused` (Task 4).
5. **One IMU stamp 0.4 s ahead of ROS time.** It is refused at ingest and LIO never reaches LOST. Pinned by `LioIngest.OneImuStampFourHundredMsAheadIsRefusedAndNeverReachesLost` (Task 2).

---

## File Structure

```text
config/px4/profiles/gps.yaml, tools/uavnav/{px4_profile,sitl_hover,sitl_checks}.py, tools/uavnav/sitl_hover.sh   T1, T5, T6
tools/simulation/run_px4_mid360.sh                                                                         T1, T6
src/estimation/uavnav_lio_core/   frontend.*, lifecycle.*, config.hpp, src/config.cpp, types.hpp           T2, T3
config/lio/sim.yaml                                                                                        T2, T3
src/estimation/uavnav_lio/        include/uavnav/lio_node/limits.hpp, src/ingest.cpp                        T2
src/px4/uavnav_px4_bridge/        alignment.*, bridge.*, config.*; config/px4_bridge/sim.yaml               T4
src/uavnav_bringup/launch/s1_estimation.launch.py                                                          T5
src/sim/uav_simulation/worlds/long_open_featured_speed.sdf, tools/uavnav/drift.py                          T6
docs/analysis/2026-10-s1c-{sitl-runs,ev-full,dv2,recovery,drift-benchmark}.md                              T1, T4, T5, T6
docs/architecture/AS_BUILT.md (sections uavnav_lio_core, uavnav_lio, uavnav_px4_bridge, uavnav_bringup)    T1–T5, T7
```

---

### Task 1: EV-full profiles and their SITL measurement (D42)

Spec §0, §4.1. There is no gate.

**Files:**
- `config/px4/profiles/gps.yaml` (`EKF2_EV_CTRL: 15`).
- `tools/uavnav/px4_profile.py` (`_DECIDED`: 15 in both profiles, citing D42) and its test.
- `tools/uavnav/sitl_hover.py`:
  - record from `estimator_status_flags`: `cs_mag_hdg`, `cs_mag_3d`, `cs_yaw_align`, `cs_ev_yaw`, `cs_gnss_pos`;
  - record `position_variance`, `velocity_variance` and `orientation_variance` of the EV sent (critique §1a).
- `tools/uavnav/sitl_hover.sh`: `GNSS_OUTAGE_S=<s>`, gps only. At 20 s of hover it sends `VEHICLE_CMD_INJECT_FAILURE` (unit GPS, `OFF`), then `OK` after the outage.
- `tools/simulation/run_px4_mid360.sh`: `SYS_FAILURE_EN=1` only when the harness asks for it.
- `tools/uavnav/sitl_checks.py` and its tests.
- Create docs/analysis/2026-10-s1c-ev-full.md and docs/analysis/2026-10-s1c-sitl-runs.md.

**Interfaces** (`sitl_checks`):
- `_check_f34_values(data, spawn_yaw_rad)`. PASS when:
  - the `yaw_T` spread is ≤ 1° in the window;
  - `alignment_valid` holds across every `Px4ResetApplied`.

  The absolute `yaw_T` is reported only.
- `_check_ekf2_heading(data)` is reported, not judged: the fraction of the window per heading source, and every heading reset with its time and size, before or after the first VALID.
- `_check_gnss_consistency(data, profile)`: gps only. `cs_gnss_pos` in ≥ 95 % of the window, with the outage window excluded.
- `_check_ev_variance(data)` is reported: p50 and max of the EV position σ, against `EKF2_EVP_NOISE`.
- The `events` check fails on `LioNodeFailed`, `BridgeNodeFailed`, `LioPublishFailed`, `BridgePublishFailed`, `LioIngestReentered` and a non-zero `LioNodeSummary.publish_failed`.
- Fold in the S1b minors T11 M2 and M3.

- [ ] **Step 1: Write the failing tests:**
  - `Px4ProfileTest.test_d42_ev_ctrl_is_full_in_both_profiles`;
  - `SitlChecksTest.test_f34_passes_on_any_stable_offset` (0° and 21° steady → PASS);
  - `SitlChecksTest.test_f34_fails_on_unstable_t` (1.5° spread → FAIL);
  - `SitlChecksTest.test_heading_sources_are_reported`;
  - `SitlChecksTest.test_gnss_dropout_outside_the_outage_fails_gps`;
  - `SitlChecksTest.test_ev_variance_is_reported`;
  - `SitlChecksTest.test_events_fail_on_publish_failures`.

  Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=FAIL` on the new tests.
- [ ] **Step 2: Implement.** Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=PASS`. Re-judge the four S1b F34 runs offline. Expected: PASS.
- [ ] **Step 3: Three runs**, one at a time:

  ```bash
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2
  make sitl-hover PROFILE=no_gps SPAWN_YAW_RAD=1.2
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2 GNSS_OUTAGE_S=10
  ```

  If PX4 refuses the injection, report it and leave out run 3.
- [ ] **Step 4: Report** (ev-full.md, in Vietnamese). Per run:
  - the heading source over time and every heading reset;
  - the `T` spread and states across the resets;
  - drift against ground truth;
  - the EV σ sent, against the PX4 floor.

  Decision rule:
  - The EV-full setting stands when all runs pass, and in run 3 `T` stays VALID with horizontal drift ≤ 0.5 m.
  - Otherwise, do not change the profile. Fly one comparison run with `EXPERIMENT_EV_CTRL=7` (harness-only, gps only, logged in the run name), and add an O-entry "exception to D42 for gps" with the data, for the owner.
- [ ] **Step 5: As-built and docs.**
  - AS_BUILT `uavnav_bringup`: the profile parameter table as applied.
  - Register the runs; the D42 row in TRACEABILITY.
  - Run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: 0 errors.
  - Commit with `Arch-Level: 1` and `Docs-Impact: docs/TRACEABILITY.md, docs/architecture/AS_BUILT.md`.

---

### Task 2: Corrections by consequence, no age rule (D40, AR4; Level 3)

Spec §3.1, §3.5. **Gate: AR4.**

**Files:**
- In `src/estimation/uavnav_lio_core/`:
  - `src/frontend.cpp`: remove the age branch (`ScanResultTooOld`). The only refusal left is the `Result<void, PredictorReason>` of `OutputPredictor::on_correction`: this is the single typed verdict that D40 asks for (AR4).
  - `include/uavnav/lio/lifecycle.hpp`, `src/lifecycle.cpp`: the freshness guard on confirmation.
  - `include/uavnav/lio/config.hpp`: the lower bound of `lifecycle_gap_degraded_s` becomes 0.2 s.
  - `types.hpp`: remove `kResultTooOld`.
  - Tests.
- In `src/estimation/uavnav_lio/`:
  - `include/uavnav/lio_node/limits.hpp`: `kMaxStampAhead` = 50 ms, and `static_assert(kMaxStampAhead < kGapDegradedMin − kScanPeriod)`;
  - `src/ingest.cpp` comment, tests.
- `docs/CHANGE_CONTROL.md` (LIO-4 text) and `tools/uavnav/arch_rules.yaml` (LIO-4 paths).

**Interfaces:**
- `LioLifecycle` keeps `last_imu_t_`, the newest `kImuTick` stamp it has seen (data, not a flag).
- In INITIALIZING (confirming) and DEGRADED, a `kScanGood` counts toward `confirm_scans` only when `last_imu_t_ − e.t ≤ gap_degraded`.
- A stale good scan still moves `last_scan_time()` and clears the degenerate run. The doc comment names the harm: confirmation on an output whose IMU-only horizon already breaks the gap rule (O15 flapping).
- `kMaxStampAhead` (tier a, 50 ms):
  - derivation: SITL skew 0, plus transport jitter; harm bound `kGapDegradedMin − kScanPeriod` = 0.1 s;
  - `kGapDegradedMin` = 0.2 s (two scan periods) is shared by `kLioSpecs` and the assert.

- [ ] **Step 1: Write the failing tests:**
  - `LioFrontend.ResultOlderThanGapDegradedIsAppliedToTheOutput` (age 0.27 s, coverable: applied, the output error shrinks);
  - `LioFrontend.SaturatedBackendStaysDegradedWithACorrectedOutput`: from TRACKING, 3 s of `kScanGood` aged 0.27 s every 100 ms. Expected: DEGRADED by the gap rule, then never TRACKING, never LOST, no odometry, output error < 0.05 m against the synthetic truth;
  - `LioLifecycle.StaleGoodScanMovesTheGapReferenceButNeverConfirms`;
  - `LioLifecycle.FreshGoodScansStillConfirm`;
  - `LioIngest.OneImuStampFourHundredMsAheadIsRefusedAndNeverReachesLost`;
  - `LioConfig.GapDegradedBelowTwoScanPeriodsIsRejected` (0.19 s refused).

  Replace the S1b age tests (`ResultOlderThanGapDegradedIsARejectedCorrection`, `ResultAgeEqualToGapDegradedCounts`, `ResultOneNanosecondOlderThanGapDegradedIsRejected`, `ResultAgeRuleTouchesOnlyGoodScans`, `SaturatedBackendReachesLostInsteadOfFlapping`) and list them in the commit body.

  Run: `make build PKGS="uavnav_lio_core uavnav_lio" && make test PKGS="uavnav_lio_core uavnav_lio"`. Expected: the new tests fail.
- [ ] **Step 2: Implement.** `frontend.cpp` gets shorter.
- [ ] **Step 3: Verify.** Run: `make build PKGS="uavnav_lio_core uavnav_lio" && make test PKGS="uavnav_lio_core uavnav_lio" && make sanitize`. Expected: all pass, no TSan warning.
- [ ] **Step 4: Docs and as-built.**
  - LIO-4 becomes: "Mọi lối vào TRACKING qua xác nhận; không có cạnh LOST → TRACKING; chỉ correction được predictor áp dụng mới tính là scan có correction, và xác nhận chỉ đếm scan đó khi luật gap còn giữ".
  - Remove "chờ AR4" from §3.1 and §3.5.
  - Update the D33/O15 TRACEABILITY row (superseded by D40, new test ids).
  - AS_BUILT `uavnav_lio_core` and `uavnav_lio`: the correction rule and the stamp bound.
- [ ] **Step 5: Commit** with `Arch-Level: 3`, `Review: AR4`, `Decision: D40`, and `Docs-Impact: docs/CHANGE_CONTROL.md, docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md, docs/architecture/AS_BUILT.md, tools/uavnav/arch_rules.yaml`.

---

### Task 3: LOST waits, LIO restarts itself (D38, D41, AR4, O34; Level 3)

Spec §3.1, §3.4. **Gates: AR4; O34 for Step 3.**

**Files:** in `src/estimation/uavnav_lio_core/`:
- `lifecycle.hpp/.cpp`;
- `frontend.hpp/.cpp` and `runner.hpp/.cpp` (`restart()` leaves the public API and becomes the internal effect);
- `config.hpp`, `src/config.cpp`, `config/lio/sim.yaml` (`lifecycle_restart_dwell_s`: [0.5, 30], beta 2.0);
- tests. The S1b tests that call `restart()` directly reach RESTARTING through LOST and the dwell instead.

**Interfaces** (`kLioTransitions`):
- Rename `kGeometryReturned` to `kStableDataDwell`: LOST → RESTARTING.
- Add `kReinitNotConfirmed`: INITIALIZING → LOST.
- LOST rule:
  - `stable_since_` is set by the first non-empty scan event (any scan kind except `kScanEmpty`) with no gap;
  - it is cleared by `kScanEmpty`, by a `kImuTick` with `t − last_scan_time() > gap_degraded`, and by entry into LOST;
  - at the first event with `t − stable_since_ ≥ restart_dwell` → RESTARTING.
  - The result of ICP against the frozen map is never read in LOST (critique §1).
- INITIALIZING rule:
  - `confirming_since_` is set at `kMapReady`;
  - at the first event with `t − confirming_since_ > restart_dwell` and no confirmation → LOST (`kReinitNotConfirmed`).
  - The same rule applies at boot; there is no "entered from" data.
- Frontend effect:
  - on every `push_imu`, after the lifecycle tick: if the state is RESTARTING and no restart is in flight, issue one;
  - seed = the predictor's newest position and yaw; roll, pitch and velocity from the predictor (O34);
  - a refused late answer (D33/O16) is re-issued by the same rule.
- The `LioRestart` event keeps its fields and adds `reason` `STABLE_DATA_DWELL`.

- [ ] **Step 1: Write the failing tests:**
  - `LioLifecycle.LostWaitsWithoutLimitOnEmptyScans` (600 s);
  - `LioLifecycle.NonEmptyScansForTheDwellRestart` (RESTARTING exactly at the boundary);
  - `LioLifecycle.AnEmptyScanOrAGapInsideTheDwellRestartsTheWait`;
  - `LioLifecycle.MapReadyButNotConfirmedWithinTheDwellReturnsToLost`;
  - `LioLifecycleTable.NoTerminalStateAndNoLostToTracking`;
  - `LioRunner.RestartsItselfAfterTheDwellWithoutAnyCall`;
  - `LioRunner.RefusedLateAnswerIsReissuedWithTheNewestPose`;
  - `LioRunner.EdgeOfGeometryAttemptsAreAtLeastTwoDwellsApart`.

  Run: `make build PKGS=uavnav_lio_core`. Expected: compile errors for the new names.
- [ ] **Step 2: Implement the lifecycle rules.**
- [ ] **Step 3 (O34): Implement the self-seeded restart effect.**
- [ ] **Step 4: Verify.** Run: `make build PKGS="uavnav_lio_core uavnav_lio" && make test PKGS="uavnav_lio_core uavnav_lio" && make sanitize`. Expected: all pass.
- [ ] **Step 5: Docs and as-built.**
  - §3.1 and §3.4 lose "chờ AR4 / O34".
  - AS_BUILT `uavnav_lio_core`: the lifecycle diagram and edge table from `kLioTransitions`, and the restart effect.
  - Commit with `Arch-Level: 3`, `Review: AR4`, `Decision: D41`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md, docs/architecture/AS_BUILT.md`.
- Risk to report: the predictor's velocity after a long IMU-only interval is the weakest part of the seed on hardware (critique §4, row 1). In SITL it cannot be seen.

---

### Task 4: `T` per epoch; stale-epoch odometry; `frozen_max` (O34, O35; DV2)

Spec §4.1. **Gates: O34 for Steps 1–3; O35 for Step 4.**

**Files:** in `src/px4/uavnav_px4_bridge/`:
- `alignment.hpp/.cpp`, `bridge.hpp/.cpp`, `config.*`, `config/px4_bridge/sim.yaml`;
- tests;
- docs/analysis/2026-10-s1c-dv2.md.

**Interfaces:**
- `LioPoseSample::epoch` (`std::uint32_t`). `AlignmentEstimator` keeps the epoch of its `T`, set when INIT → VALID.
- A tracking sample of a newer epoch:
  - VALID or FROZEN → INIT (`kNewLioEpoch`, two new edges);
  - in INIT the accumulation restarts;
  - INVALID → INIT as today.
- FROZEN → VALID (`kLioTracking`) only for a sample of `T`'s own epoch.
- Bridge: odometry or health with an epoch lower than the newest accepted health epoch is refused with `LioInputRejected` reason `STALE_EPOCH`: no EV and no pair.
- O35: remove the edge FROZEN → INVALID `kFrozenTooLong`, `AlignmentConfig::frozen_max` and the key `alignment_frozen_max_s`. FROZEN then ends by a PX4 reset (→ INVALID), the same epoch (→ VALID) or a newer epoch (→ INIT).

- [ ] **Step 1: Write the failing tests:**
  - `Alignment.NewerEpochReinitialisesFromFrozen`;
  - `Alignment.NewerEpochReinitialisesFromValid`;
  - `Alignment.SameEpochReturnFromFrozenKeepsT` (1e-6);
  - `Alignment.Px4ResetWhileFrozenStillInvalidates`;
  - `Px4Bridge.OldEpochOdometryAfterNewEpochHealthIsRefused`;
  - `Px4Bridge.LargeHeadingResetAfterReinitKeepsTheVehiclePointContinuous` (+20° with an xy delta; 1 mm);
  - the updated `Alignment.TransitionTableIsTheSpecDiagram` and `AlignmentFuzz.EveryStateChangeIsAListedEdge`.

  Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: the new tests fail.
- [ ] **Step 2: Implement.**
- [ ] **Step 3: Verify, update the §4.1 diagram, and commit** (`Arch-Level: 1`; PX4-1 touched and held, because the table and the spec change together).
- [ ] **Step 4 (O35): Remove `frozen_max`.**
  - Test `Alignment.FrozenStaysFrozenWithoutAPx4Reset` (600 s).
  - `AlignmentConfig.RemovedFrozenMaxKeyIsRejectedAsUnknown`.
  - Commit with `Arch-Level: 1`.
- [ ] **Step 5: DV2** from `log/sitl/20261013T155144Z-no_gps-hover`, offline. Measure:
  - the `vehicle_local_position` inter-arrival and lag around each `NO_PX4_SAMPLE`;
  - the pair's stamp against the newest buffered sample;
  - the buffer span.

  Then: close DV2 if the cause is PX4 rate or lag; write a failing test and fix it if the cause is the bridge; otherwise keep DV2 OPEN with Task 6 as the trigger.
- [ ] **Step 6: As-built.** AS_BUILT `uavnav_px4_bridge`: the `T` diagram and edge table from `kAlignmentTransitions`, and the epoch gate. Commit with `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md, docs/architecture/AS_BUILT.md`.

---

### Task 5: Fault injection and recovery runs (D38, D41)

Spec §3.4, §4.1, §7.2. There is no product change. It needs Tasks 2–4.

**Files:**
- `src/uavnav_bringup/launch/s1_estimation.launch.py`: required argument `start_lidar_bridge`; the docstring states the Fast DDS profile dependency.
- `tools/uavnav/sitl_hover.sh`:
  - `PATTERN=lidar_cut CUT_S=<s>` and `PATTERN=climb_loss`;
  - the LiDAR `ros_gz_bridge` runs as its own process group;
  - fold in the T3 harness minors.
- `tools/uavnav/sitl_hover.py`:
  - `lidar_cut`: hover 20 s, `SIGSTOP` for `CUT_S` of /clock, `SIGCONT` (also in `finally`), hover 40 s;
  - `climb_loss` (gps): 7.0 m for 15 s, 4.8 m for 20 s, 2.5 m for 30 s.
- `tools/uavnav/sitl_checks.py` (`evaluate_recovery`) and its tests.
- docs/analysis/2026-10-s1c-recovery.md.

**Criteria** (`evaluate_recovery(run_dir, profile) -> dict`):

| Check | Pass when |
|---|---|
| `lost_detected` | LOST within cut start + `gap_lost` + 0.2 s |
| `waits_while_unstable` | no `LioRestart` while the LiDAR is cut or the scans are empty; consecutive attempts ≥ 2 × `restart_dwell_s` apart |
| `restart` | `LioRestart` within `restart_dwell_s` + 0.5 s of non-empty scans returning; epoch +1 per applied restart |
| `retracked` | TRACKING of the new epoch within `restart_dwell_s` of `LioRestart` |
| `t_reinit` | `T` → INIT on the first pair of the new epoch, then VALID within 3 s; never VALID on a pair of another epoch |
| `ev_epochs` | no EV of the old epoch after the new epoch's first health; EV of the new epoch from its first TRACKING odometry |
| `px4_continuity` | gps: PX4 horizontal error ≤ 0.5 m against ground truth; no_gps: reported |
| `events` | as the hover check, plus counts per reason |

- [ ] **Step 1: Write the failing tests:**
  - `SitlChecksTest.test_recovery_pass_on_synthetic_records`;
  - `test_restart_during_cut_fails`;
  - `test_attempts_closer_than_two_dwells_fail`;
  - `test_t_valid_on_a_foreign_epoch_fails`;
  - `test_recovery_missing_cut_window_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Run: `tools/uavnav/gate.sh python && make sitl-build && make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2 DRY_RUN=1`. Expected: PASS; the dry run lists the separate LiDAR group.
- [ ] **Step 3: Three runs**, one at a time:

  ```bash
  make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2
  make sitl-hover PROFILE=no_gps PATTERN=lidar_cut CUT_S=1.5   # below EKF2_NOAID_TOUT; EKF2 resets on the new epoch's EV
  make sitl-hover PROFILE=gps PATTERN=climb_loss
  ```

  On failure, read the timelines first. At most one fix and re-run per root cause; a design gap becomes an O-entry.
- [ ] **Step 4: Report and docs.**
  - The report gives the chain timeline per run.
  - Register the runs.
  - P5, the D38/D41 row and F22 go to `INTEGRATED` only if the AS_BUILT sections of Tasks 3 and 4 are current.
  - Commit with `Arch-Level: 1` (launch argument) and `Docs-Impact: docs/TRACEABILITY.md`.

---

### Task 6: Drift benchmark (D39; DV1 and O13 data)

Spec §7.2, D39. There is no product change. It can run before Task 2 if the gates are late: the report then states which correction rules were active.

**Feasibility** (PX4 v1.17.0-14):
- `MPC_POS_MODE=0` maps the sticks to a velocity: `v = expo_deadzone(stick, 0.6, MAN_DEADZONE) × MPC_VEL_MANUAL` (`FlightTaskManualPosition.cpp`, `Sticks.hpp`).
- The stick stream sends `pitch = 0` today (`tools/uavnav/sitl_probe.py`).
- The world `long_open_featured_speed.sdf` lacks `<magnetic_field>` and `<atmosphere>`.

**Files:**
- the world (add both elements);
- `run_px4_mid360.sh` (`MPC_POS_MODE=0`, `MAN_DEADZONE=0.1` for `PATTERN=drift` only);
- `sitl_hover.sh` (`PATTERN=drift SPEED_MPS=<v>`, bound 600 s);
- `sitl_probe.py` (`set_sticks(throttle, pitch)`; the stick stream starts 3 s after the first /clock);
- `sitl_hover.py`;
- create `tools/uavnav/drift.py` and `tools/uavnav/tests/test_drift.py`;
- `sitl_checks.evaluate_drift`;
- docs/analysis/2026-10-s1c-drift-benchmark.md.

**Flight:**
1. Climb to 2.5 m, then 5 s stationary.
2. In-flight `DO_SET_MODE` Hold, then POSCTL; each must be reached within 1 s.
3. Legs along +x from 5 m to 105 m and back, until ≥ 200 m and ≥ 120 s of motion.
4. Land.

**Interfaces** (pure, numpy):

```python
def stick_for_speed(v_mps, vel_manual_mps=10.0, expo=0.6, deadzone=0.1) -> float   # inverse of PX4 expo_deadzone
def align_4dof(est, gt) -> tuple[float, np.ndarray]                               # over the stationary window
def relative_drift(est, gt, lengths_m=(10, 25, 50, 100)) -> dict                  # mean, p95 (m/m), n per length
def per_minute_drift(est, gt, window_s=60.0) -> dict                              # mean, p95 (m/min), n
def evaluate_drift(run_dir) -> dict   # LIO=/lio/odometry, PX4=vehicle_local_position (timestamp_sample), GT interpolated
```

- [ ] **Step 1: Write the failing tests** (`DriftTest`):
  - `test_stick_for_speed_inverts_px4_expo_deadzone`;
  - `test_align_recovers_a_known_4dof_offset`;
  - `test_constant_velocity_bias_gives_linear_drift`;
  - `test_relative_drift_needs_enough_segments`;
  - `test_drift_missing_topic_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `ModuleNotFoundError: drift`.
- [ ] **Step 2: Implement.** A dry run shows the world, 600 s and `MPC_POS_MODE=0`. Commit (`Arch-Level: 0`).
- [ ] **Step 3: Six runs:** `PROFILE=gps` and `PROFILE=no_gps`, `PATTERN=drift SPEED_MPS=<v>` for v = 1, 3, 5. A loss in flight is data, not a failure. In no_gps, a loss longer than 5 s ends in a PX4 failsafe: report it.
- [ ] **Step 4: Report** (in Vietnamese). It covers:
  - per profile and speed: LIO and EKF2 drift (m/m, m/min) and the yaw drift;
  - the benchmark table below, with every URL, and the sim-to-real gap;
  - DV1: close it when, in every run, DEGRADED → TRACKING happens ≤ 1 per 60 s and the result age p99 is < 0.145 s;
  - the τ observation (O13; a change is an owner item);
  - the rejected-correction counts per reason.

  Register the runs, update the D39, τ and DV1 rows, and commit.

**Benchmark reference** (the report re-checks every row):

| Quantity | Real (source) | SITL model | Gap to state |
|---|---|---|---|
| Mid-360 range precision (1σ) | ≤ 2 cm at 10 m ([Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs)) | σ 0.02 m at all ranges (`src/sim/uav_simulation/models/lidar_mid360/model.sdf`) | no range or reflectivity dependence |
| Angular precision (1σ) | < 0.15° (same) | none | about 5 cm lateral at 20 m missing |
| FOV, rates | 360° × (−7°..52°), 200,000 pts/s, 10 Hz (same) | same FOV (F36); 201,600 pts/s in 28 fixed rings | non-repetitive pattern missing |
| Per-point timing | spread over the 100 ms scan | one instant per scan (O20) | no motion distortion: up to 0.5 m at 5 m/s |
| Gyro, accel noise density | 4.5 mdps/√Hz, 100 µg/√Hz ([TDK ICM-40609-D](https://invensense.tdk.com/en-us/products/6-axis/icm-40609-d)) | ≈ 4.1 mdps/√Hz, ≈ 72 µg/√Hz | accel about 30 % quieter |
| IMU bias | temperature coefficients ±10 mdps/°C, ±0.15 mg/°C; instability **UNVERIFIED** | none | IMU-only intervals drift far less than on hardware |
| Simulated GNSS | | Gauss-Markov 0.8 m (`GZBridge.hpp`) | the gps EKF2 "drift" is GNSS noise |
| EKF2 assumed EV noise | `EKF2_EVP_NOISE` 0.1 m, `EKF2_EVV_NOISE` 0.1 m/s, `EKF2_EVA_NOISE` 0.1 rad (`params_external_vision.yaml`) | same | the EV accuracy PX4 assumes is these floors (critique §1a) |
| Magnetic declination | WMM −0.93° at the world's coordinates | +0.75° | 1.7° offset when the compass leads; `T` absorbs it |
| Real LIO drift, for scale | FAST-LIO2: 0.14 m over ≈ 650 m ([arXiv 2107.06829](https://arxiv.org/abs/2107.06829)) | | a loop error; a different sensor |

---

### Task 7: Close S1c, with the as-built drawing (D45)

**Files:**
- `docs/architecture/AS_BUILT.md`;
- `docs/TRACEABILITY.md`;
- `docs/architecture/SYSTEM_DESIGN.md`;
- `tools/uavnav/arch_rules.yaml`;
- this plan.

- [ ] **Step 1: Run the gate, sanitize and a final hover.** Run: `tools/uavnav/gate.sh all && make sanitize && make sitl-hover PROFILE=gps`. Expected: PASS, no TSan warning.
- [ ] **Step 2: As-built of the recovery chain.** One Mermaid diagram, exactly as coded:
  - the `LioLifecycle` and `AlignmentEstimator` machines, with edge tables generated from `kLioTransitions` and `kAlignmentTransitions`;
  - the epoch flow: `/lio/health` and `/lio/odometry` (epoch) → bridge (stale-epoch gate) → EV `reset_counter` → EKF2 reset → `T` INIT. There is no seed message;
  - the budget table as implemented, with values read from `config/lio/sim.yaml` and `config/px4_bridge/sim.yaml`;
  - deviations from §3.1, §3.4 and §4.1 cited as D, O or DV entries;
  - `Verified at: <sha>`.
- [ ] **Step 3: Spec sync.** Remove every "chờ" marker this slice closed. Check that §3.4's budget table matches the AS_BUILT table.
- [ ] **Step 4: Governance.**
  - Classify the 7 legacy packages in `arch_rules.yaml`.
  - Run `python3 -m tools.uavnav.arch_health` and record each trigger as an AR row (`Owner xác nhận` = `chưa`).
- [ ] **Step 5: Append "Carry-over to S2".** It includes:
  - the Supervisor give-up counted from the first LOST of the episode;
  - `px4_mode` with `T` per epoch (critique C5: commands use the `T` of their epoch; the frozen value serves old-epoch commands);
  - the seed velocity on hardware;
  - every O-entry opened in Tasks 1, 5 and 6.
- [ ] **Step 6: Commit** with `Arch-Level: 1` and the touched files in `Docs-Impact`.

---

## Carry-over coverage (S1b "Carry-over to S2", items 1–17)

| # | Item | Here |
|---|---|---|
| 1 | τ, F23 step with motion | Task 6 (data); a change is an owner item |
| 2 | Degeneracy thresholds | closed (D36); not used for any new proxy |
| 3 | DV1 | Task 6 (the drift flights are the motion flights); the band remains under D40 |
| 4, 17 | `NO_PX4_SAMPLE` (DV2) | Task 4 Step 5 |
| 5 | ≈ 20° heading reset absorbed by `T` | Task 1 runs; `Px4Bridge.LargeHeadingResetAfterReinitKeepsTheVehiclePointContinuous` (Task 4) |
| 6 | Stale-epoch odometry gate; two resets in one dropped window | gate: Task 4; the two-reset limit stays a documented limit |
| 7 | `/lio/scan`, restart caller, seed helper | the caller is LIO itself (Task 3); no PX4 seed (O34); `/lio/scan`: S3 |
| 8, 13, 14, 16 | Hardware items, lio `kClockMode`, Fast DDS on hardware, D36 scope | P8; the launch docstring in Task 5 |
| 9 | `DO_SET_MODE` in flight; POSCTL source | Task 6 flight step 2 and the delayed stick stream |
| 10 | Deferred review minors | Tasks 1, 2 and 5, when the file is touched |
| 11 | Governance | Task 7 |
| 12 | lio future-stamp tolerance | Task 2 (0.05 s, compile-time bound) |
| 15 | Unknown health code; zero-tolerance event list | event list: Task 1; the code stays refused before the core |

## Model and agent plan

- **Agents (at most 3, CLAUDE.local.md):** one implementer at a time in the tree, plus up to two read-only agents (a reviewer, and an offline analyst who reads `log/sitl/` while no build or SITL runs).
- **Models:**
  - most capable: Tasks 2, 3 and 4 (lifecycle and alignment), and the judgement in Tasks 1, 5 and 6;
  - capable: the harness code of Tasks 1, 5 and 6;
  - mid-tier: Task 7 Steps 3–6.
  - Reviewers: the most capable model for Tasks 2–4 and the final review.
- **Order:** 1 → 2 → 3 → 4 → 5 → 6 → 7. If AR4 or O34 is still open after Task 1, do Task 6 first.
- **Machine:** Makefile only, `-j3`. SITL never overlaps a build, a test or sanitize.
- **SITL time:** 3 + 3 (+ at most 2 re-runs) + 6 + 1 runs, about 95 min.
- **TSan:** Tasks 2 and 3 run `make sanitize`. No thread is added.

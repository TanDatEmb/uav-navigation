# S1c: Consequence-Based Verdict, LIO That Never Gives Up, EV Fully Fused, Drift Benchmark

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Before S2, give the estimation stack the owner's 2026-10-14 decisions:
- **D40:** LIO accepts or rejects data by its consequence, not by its age, through one typed verdict.
- **D38, D41:** LIO never gives up. LOST is an idle wait; LIO restarts itself after a dwell of stable data, with a seed from the sync or from itself; the sync decides how to resynchronise `T`.
- **D42:** a compass always, EV fused as fully as possible in both profiles. SITL measures what PX4 then does, and F34 is judged by the stability of `T`.
- **D39:** a drift benchmark of LIO and EKF2 against ground truth at 1–5 m/s, set against datasheets.

**Architecture:**
- No new state anywhere. The chain runs on `LioLifecycle` and `AlignmentEstimator` (spec §3.1, §3.4 chain table, §4.1 "Đồng bộ lại sau epoch mới").
- One pure verdict function, `judge_correction`, replaces the three places where the apply/reject rule lives today (AR3).
- LIO issues its own restart as the effect of the LOST → RESTARTING edge. The sync only offers seeds, as a latest-value stream (O29).
- All decisions stay in ROS-free cores; the nodes convert and publish.
- SITL stays in POSCTL; there is no `px4_mode` and no Supervisor. Fault injection and drift are new patterns of the bounded harness, judged by pure, unit-tested Python checkers.

**Tech Stack:** C++23, Eigen 3, ament_cmake, GoogleTest, rclcpp (Jazzy), px4_msgs v1.17.0, Gazebo Harmonic, PX4 v1.17 SITL, Python 3 (`unittest`, `rclpy`, numpy 1.26), GNU make.

**Spec:** [SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md):
- §0 (profiles, D42), §3.1 (target diagram, D41; the D40 rule), §3.3, §3.4 (self-restart, seed, chain table), §3.5 (future-stamp guard), §4.1 (EKF2 parameters; resync window), §7.2 (S1c).
- Decisions in [DECISIONS.md](../architecture/DECISIONS.md): D4, D11, D20, D30, D31, D33 (O15, O16, O20, O22, O25), D36, D38–D43; F22, F33, F36; AR1, AR3; open O29, O33.
- Invariants in [CHANGE_CONTROL.md](../CHANGE_CONTROL.md) §4: LIO-1..LIO-4, PX4-1, PX4-2.
- Predecessor: the "Carry-over to S2" table of the [S1b plan](2026-10-12-s1b-ros-shells-and-sitl.md).

---

## Slice order and why

The chosen order is S1c (this plan), then S2 (`px4_mode`, minimal Supervisor with the give-up guards, straight trajectories, Y1/Y3a), then S3 onward.

1. **EV-full profiles and their SITL measurement first (Tasks 2–3).** They gate nothing but change the PX4 behaviour that every later run sees. D42 also asks for the measurement early: if the full setting breaks GNSS fusion, the owner must see it before recovery and drift data are built on it.
2. **The AR3 gate (Task 1)** is an owner action and can close at any time. Tasks 4–7 (all `uavnav_lio_core` and `uavnav_lio` code under D40/D41) start only after it.
3. **Recovery (Tasks 4–11).** It is owner priority 1 (D38, D41), and it lives in `lio` and `px4_bridge`. S2 then builds the Supervisor's give-up guards on a chain that already works.
4. **Drift last (Tasks 12–13).** It needs no product code. With D40 and D41 in place, a saturated backend or a loss at 5 m/s no longer ends the run.
5. **If AR3 is not confirmed when Task 3 is done:** do the non-gated parts of Task 8 (offer helper, epoch gate), Task 9's DV2 step, then Tasks 12–13 (drift) under the S1b rules. Come back to Tasks 4–11 after the confirmation. The drift report then states which verdict rules were active.

**Rejected alternatives:**

| Alternative | Why rejected |
|---|---|
| Spec order: S2, S3, then recovery in S4 | Contradicts D38. Every S2 flight would end at the first LOST. |
| Supervisor as the restart caller | D41 makes LIO restart itself. The Supervisor owns only the give-up (D11). |
| Restart on request: bridge seeds only while RESTARTING | One more round trip, and the first attempt would usually go `SELF` (O29 (b)). |
| Patch the age rule again (a new margin) without the verdict function | Contradicts D40, and AR1's trigger has fired (a 5th change to the same rule). |
| A retry counter to stop repeated restarts | D41 forbids a count; the dwell of stable data spaces the attempts. |

## Owner gates

| Gate | Recommended | Blocks |
|---|---|---|
| **AR3** confirmed by the owner (`Owner xác nhận` = `có`, result `APPROVED`) | the AR3 design as written | Task 1 Step 2 onward, then Tasks 4–7 (Level 3: LIO-4 changes) |
| **O29**: how the sync's seed offer reaches LIO | (a) the latest-value stream `/lio/restart_seed` at 10 Hz while `T` is VALID or FROZEN | the stream part of Task 8. Without it, LIO still restarts with `SELF` seeds. |
| **O33**: the resync policy inside the window | (a) PX4 resets applied while FROZEN inside the window; new edge FROZEN → INIT after `alignment_resync_window_s` of gate rejections | Task 9 Steps 1–3 |

Closed and binding: O28 (→ D40), O30 (→ D41), O31 (→ D42: EV is never held back), O32 (→ D42: no `no_gps_ev_yaw` profile). A decision different from a recommendation is a plan change: edit the affected task before dispatching it.

## Global Constraints

- **Tree, staging and commits:** the S1b plan's Global Constraints apply: single tree, `git add <path>` only, `git diff --cached --stat` before every commit, Makefile-only builds at `-j3`, at most 3 agents.
- **Trailers:**
  - A commit that touches a contract path (`config/`, `msg/`, public headers) or an invariant file ends with `Arch-Level:` and `Docs-Impact:`, then `Co-Authored-By: <the model that did the work> <noreply@anthropic.com>`.
  - Tasks 1, 4, 5, 6 and 7 are Level 3. They also carry `Review: AR3` and `Decision: D40` (Tasks 1, 4, 5) or `Decision: D41` (Tasks 6, 7), and `check_change` requires AR3 to be owner-confirmed.
  - An executor never declares Level 2 or 3 beyond what this plan states: it stops and reports.
- **SITL (D43):**
  - Agents run SITL themselves, one run at a time. Never during a build, a test or `make sanitize`.
  - Headless; artifacts only under `log/sitl/`.
  - Bounds: 300 s per hover or recovery run; 600 s per drift run.
  - Every cited run id is registered in docs/analysis/2026-10-s1c-sitl-runs.md (created by Task 3).
- **D40 in every new guard:**
  - each acceptance or refusal states the harm it prevents, in its doc comment and in the event's reason;
  - an age may appear only as a measured value, never as the reason itself;
  - the technical bounds that remain are the 300 ms predictor buffer (`covers()`), the epoch or frame match, and the time base.
- **D41:** no attempt counter, no terminal state in `lio`. Every wait is a dwell measured in sensor time.
- **D42:** no code path drops a valid EV sample. PX4 arbitrates.
- **Config (D22):**
  - new tier (b) keys go in the existing spec tables with bounds;
  - new tier (a) constants go in `limits.hpp` with a derivation;
  - profile values go only in `config/px4/profiles/*.yaml`;
  - harness-only PX4 parameters (`MPC_POS_MODE`, `SYS_FAILURE_EN`) go only in `tools/simulation/run_px4_mid360.sh`.
- **Events:** one event per verdict, transition, restart, offer refusal and resync decision. Per-sample events are rate-limited like the existing bridge episodes.

## Review Focus

1. **A saturated backend (results aged 0.27 s, every 100 ms).** The results are applied, so the output keeps tracking. LIO sits in DEGRADED, never confirms TRACKING (no flapping) and does not go LOST. Pinned by `LioFrontend.SaturatedBackendStaysDegradedWithACorrectedOutput` (Task 4).
2. **Unstable data at the edge of geometry.** Restart attempts are never closer together than `restart_stable_s` after a failed attempt, and there are none while the scan self-information is low. Pinned by `LioLifecycle.UnstableDataNeverRestartsBeforeTheDwell` (Task 6), and by the SITL `climb_loss` boundary hold (Task 11).
3. **One wrong registration with a large innovation.** It is not applied: no output step and no odometry. It counts as a bad scan. Pinned by `LioFrontend.ImplausibleCorrectionIsNotApplied` (Task 4).
4. **A `SELF`-seeded restart while `T` is FROZEN.** The gate rejects the new-epoch pairs; `T` goes to INIT after the resync window instead of staying frozen for 10 s. Pinned by `Px4Bridge.SelfSeededEpochReinitialisesTAfterTheResyncWindow` (Task 9).
5. **Old-epoch odometry after new-epoch health, and a burst of old clouds after a LiDAR outage.** No EV and no pair for the old epoch; old clouds are superseded without a rewind. Pinned by `Px4Bridge.OldEpochOdometryAfterNewEpochHealthIsRefused` (Task 8) and `LioIngest.BurstOfOldScansAfterAnOutageIsDropped` (Task 5).

---

## File Structure

```text
docs/CHANGE_CONTROL.md, tools/uavnav/arch_rules.yaml                       T1 (after AR3 is confirmed)
config/px4/profiles/{gps,no_gps}.yaml, tools/uavnav/px4_profile.py          T2
tools/uavnav/{sitl_hover.sh,sitl_hover.py,sitl_probe.py,sitl_checks.py}     T2, T3, T10, T12
src/estimation/uavnav_lio_core/  T4 correction_verdict.hpp/.cpp, output_predictor, frontend; T6 lifecycle, backend self-information; T7 runner/frontend self-restart
src/estimation/uavnav_lio/       T5 ingest time base, scan wait; T8 seed-offer subscription
src/px4/uavnav_px4_bridge/       T8 seed_offer.hpp/.cpp, epoch gate, publisher; T9 resync window
src/core/uavnav_interfaces/msg/LioRestartSeed.msg (new) T8; src/core/uavnav_ros/qos.hpp T8
src/uavnav_bringup/launch/s1_estimation.launch.py T10; tools/simulation/run_px4_mid360.sh T3, T12
src/sim/uav_simulation/worlds/long_open_featured_speed.sdf T12; tools/uavnav/drift.py (new) T12
docs/analysis/2026-10-s1c-{sitl-runs,ev-full,dv2,recovery,drift-benchmark}.md (new) T3, T9, T11, T13
```

---

### Task 1: AR3 confirmation gate, then the invariant text (D40, D41; Level 3)

**Gate:** Step 1 asks the owner; nothing else in this task runs before the answer.

**Files:**
- Modify: `docs/architecture/DECISIONS.md` (AR3 row: `Owner xác nhận` `có`, result `APPROVED`);
- Modify: `docs/CHANGE_CONTROL.md` (§4.A, LIO-4);
- Modify: `tools/uavnav/arch_rules.yaml`;
- Modify: `docs/architecture/SYSTEM_DESIGN.md` (drop the "chờ AR3" wording in §3.1, §3.4, §3.5).

- [ ] **Step 1: Ask the owner to confirm AR3.** Give the owner the AR3 row and §3.1/§3.4 of the spec. Record the answer verbatim in the AR3 row. A change requested by the owner edits AR3 and this plan's Tasks 4–7 first.
- [ ] **Step 2: Invariant text.** Rewrite LIO-4 to read: "Mọi lối vào TRACKING qua xác nhận; không có cạnh LOST → TRACKING; chỉ kết luận `kApplied` của `judge_correction` mới tính vào xác nhận".
  - Add `src/estimation/uavnav_lio_core/include/uavnav/lio/correction_verdict.hpp` and `src/correction_verdict.cpp` to the LIO-4 `invariant_paths` in `arch_rules.yaml`, with the id matching CHANGE_CONTROL.
- [ ] **Step 3: Verify.** Run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check && python3 -m tools.uavnav.check_change`. Expected: 0 errors each.
- [ ] **Step 4: Commit** (`Arch-Level: 3`, `Review: AR3`, `Decision: D40`, `Docs-Impact: docs/CHANGE_CONTROL.md, docs/architecture/DECISIONS.md, docs/architecture/SYSTEM_DESIGN.md, tools/uavnav/arch_rules.yaml`).

---

### Task 2: EV-full profiles with a compass; F34 judged by the stability of `T` (D42)

Spec §0, §4.1, D42. There is no gate.

**Files:**
- Modify: `config/px4/profiles/gps.yaml` (`EKF2_EV_CTRL: 15`).
- Modify: `tools/uavnav/px4_profile.py` (`_DECIDED`: 15 for both, citing D42), `tools/uavnav/tests/test_px4_profile.py`.
- Modify: `tools/uavnav/sitl_hover.py`:
  - record from `estimator_status_flags`: `cs_mag_hdg`, `cs_mag_3d`, `cs_yaw_align`, `cs_ev_yaw`, `cs_gnss_pos`, `cs_gnss_vel`;
  - record `/fmu/out/estimator_status` `hdg_test_ratio` and `pos_test_ratio` if the topic is exported. Check `dds_topics.yaml` first; if it is not exported, say so in the Task 3 report.
- Modify: `tools/uavnav/sitl_checks.py`, `tools/uavnav/tests/test_sitl_checks.py`.

**Interfaces** (`sitl_checks`):
- `_check_f34_values(data, spawn_yaw_rad)`. PASS when:
  - the `yaw_T` spread is ≤ 1° over the window;
  - `alignment_valid` holds across every `Px4ResetApplied`.

  The values `yaw_T` and `yaw_T − wrap(π/2 − spawn)` are reported only. There is no profile-keyed absolute expectation (D42 replaces D35/D37).
- `_check_ekf2_heading(data)`: reported, not judged. It reports the fraction of the window per heading source (`cs_ev_yaw`, `cs_mag_hdg`, `cs_mag_3d`) and the heading resets (`vehicle_local_position.heading_reset_counter` steps) before and after the alignment first became VALID.
- `_check_gnss_consistency(data, profile)`: gps only. PASS when `cs_gnss_pos` is in ≥ 95% of the window and GNSS fusion never stops in the window.
- The `events` check fails on `LioNodeFailed`, `BridgeNodeFailed`, `LioPublishFailed`, `BridgePublishFailed`, `LioIngestReentered` and a non-zero `LioNodeSummary.publish_failed`, and reports `LioInputRejected` (carry-over 15).
- Fold in the S1b review minors T11 M2 (inter-arrival in /clock time) and M3 (`ekf2_ev_pos` weighted by sample).

- [ ] **Step 1: Write the failing tests:**
  - `Px4ProfileTest.test_d42_ev_ctrl_is_full_in_both_profiles`;
  - `SitlChecksTest.test_f34_passes_on_any_stable_offset` (`yaw_T` steady at 0° and at 21° → PASS);
  - `SitlChecksTest.test_f34_fails_on_unstable_t` (1.5° spread → FAIL);
  - `SitlChecksTest.test_heading_sources_are_reported`;
  - `SitlChecksTest.test_gnss_dropout_in_window_fails_gps`;
  - `SitlChecksTest.test_events_fail_on_publish_failures`.

  Run: `tools/uavnav/gate.sh python`. Expected: the new tests fail, and `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=PASS`.
- [ ] **Step 3: Re-judge the four S1b F34 runs offline** (`python3 tools/uavnav/sitl_checks.py --hover log/sitl/<run> --profile <p> --spawn-yaw-rad <y>`). Expected: PASS (spreads 0.03–0.08°).
- [ ] **Step 4: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/TRACEABILITY.md`; the spec already states D42).

---

### Task 3: SITL measurement of the EV-full setting in both profiles (D42)

Spec §0, §4.1, D42. The runs take about 35 min. There is no gate.

**What the PX4 v1.17 source predicts** (`ev_yaw_control.cpp`, `mag_control.cpp`, `estimator_interface.cpp`):
- An FRD EV yaw starts with a heading reset to EV and `yaw_align` = false.
- While GNSS fuses with `yaw_align`, FRD EV yaw neither starts nor continues.
- While EV yaw fuses, the compass is not fused.

So in gps the final source should be GNSS plus compass, with a reset if EV arrived first. In no_gps it should be EV yaw, as in the S1b runs, with a reset of up to the spawn heading at EV start. These are predictions; the runs decide.

**Files:**
- Modify: `tools/uavnav/sitl_hover.sh` and `sitl_hover.py`: `GNSS_OUTAGE_S=<s>` (gps only). After 20 s of hover the driver sends `VEHICLE_CMD_INJECT_FAILURE` (420, unit `FAILURE_UNIT_SENSOR_GPS`, type `FAILURE_TYPE_OFF`), then `OK` after the outage.
- Modify: `tools/simulation/run_px4_mid360.sh`: `SYS_FAILURE_EN=1` only when the harness asks for it.
- Create: docs/analysis/2026-10-s1c-ev-full.md (Vietnamese), docs/analysis/2026-10-s1c-sitl-runs.md (run register).

- [ ] **Step 1: Five runs**, one at a time:

  ```bash
  make sitl-hover PROFILE=gps
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2
  make sitl-hover PROFILE=no_gps
  make sitl-hover PROFILE=no_gps SPAWN_YAW_RAD=1.2
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2 GNSS_OUTAGE_S=10   # does FRD EV yaw take over, and what happens when GNSS returns
  ```

  If PX4 refuses the failure injection (ack ≠ ACCEPTED), report it and leave out run 5.
- [ ] **Step 2: Report**, per run:
  - which heading source PX4 used over time;
  - every heading reset (time, size, before or after the alignment's first VALID);
  - the resulting `yaw_T` and frame (north-referenced or LIO-start);
  - GNSS consistency (gps);
  - drift against ground truth;
  - the `T` spread and state across resets.
- [ ] **Step 3: Decision rule.** The EV-full setting stands when every run passes the hover checks of Task 2, and in run 5 `T` stays VALID or FROZEN-then-VALID with drift ≤ 0.5 m.

  Otherwise (for example GNSS fusion rejected, `T` INVALID after a reset, drift > 0.3 m), **do not change the profile**. Do this instead:
  - fly one comparison run with `EXPERIMENT_EV_CTRL=7` (a harness-only override, logged in the run name and `summary.json`, refused without `PROFILE=gps`);
  - write the data side by side;
  - add an O-entry "proposed exception to D42 for the gps profile" with that data, for the owner.
- [ ] **Step 4: Register the runs, update TRACEABILITY** (the D42 row: `INTEGRATED` if the rule holds), then run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 5: Commit.**

---

### Task 4: `CorrectionVerdict`: one consequence-based apply/reject rule (D40, AR3; Level 3)

Spec §3.1, §3.3, §3.5; AR3 points 1(a)–(d) and 2. **Gate: AR3 confirmed (Task 1).**

**Files:** in `src/estimation/uavnav_lio_core/`:
- Create `include/uavnav/lio/correction_verdict.hpp`, `src/correction_verdict.cpp`, `test/test_correction_verdict.cpp`.
- Modify `output_predictor.hpp/.cpp`: `innovation_at` replaces `covers` as the frontend's query.
- Modify `frontend.cpp/.hpp`: one call site; the age check and the restart match move into the verdict.
- Modify `lifecycle.hpp/.cpp`: input `kScanAppliedNotFresh`.
- Modify `frontend_config.hpp`, `config.hpp`, `src/config.cpp`, `config/lio/sim.yaml`.
- Modify `types.hpp` (event reasons), `test/test_frontend.cpp`, `test/test_lifecycle.cpp`.

**Interfaces:**

```cpp
namespace uavnav::lio {
struct Innovation { double position_m; double yaw_rad; };
/// Predictor output minus the snapshot at t, without changing anything; error = the covers() reason (buffer 300 ms).
Result<Innovation, PredictorReason> OutputPredictor::innovation_at(time::SensorTime t, const EstimatorSnapshot& s) const noexcept;

enum class CorrectionVerdict : std::uint8_t {
  kApplied,                 // apply; lifecycle kScanGood (counts toward confirmation)
  kAppliedNotFresh,         // apply; lifecycle kScanAppliedNotFresh: moves the gap reference, neutral for confirm/bad runs
  kRejectedNotCoverable,    // predictor cannot place it in time (technical); lifecycle kScanCorrectionRejected
  kRejectedImplausible,     // innovation above the gate (harm: an output/EV step); lifecycle kScanCorrectionRejected
  kRejectedStaleEpoch,      // result of another epoch (frame mismatch)
  kRejectedRestartPending   // a restart answer is pending; only that answer applies (D33/O16 rule)
};
struct CorrectionInput {
  std::uint32_t result_epoch, current_epoch;
  bool restart_pending;
  time::SensorTime scan_end, newest_imu;
  Result<Innovation, PredictorReason> innovation;
};
struct CorrectionLimits { time::Duration gap_degraded; double innovation_max_m; double innovation_max_rad; };
CorrectionVerdict judge_correction(const CorrectionInput& in, const CorrectionLimits& lim) noexcept;
}
```

- The check order is fixed: epoch → restart → coverable → plausible → fresh.
- "Fresh" means `newest_imu − scan_end ≤ gap_degraded`: the gap rule still holds right after applying. This is a consequence for the lifecycle, not a refusal.
- New tier (b) keys: `correction_innovation_max_m` [0.1, 10], beta 1.0; `correction_innovation_max_rad` [0.01, 1.0], beta 0.1745 (10°). The derivation comment names the harm (an output step like F23) and says the values are re-tuned from the Task 13 logs.
- Event `CorrectionVerdict`, one per non-`kApplied` result, rate-limited, with values `age_s`, `innovation_m`, `innovation_rad`, `buffer_span_s`.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(CorrectionVerdict, TableCoversEveryInputCombination) { /* exhaustive grid over epoch/restart/coverable/plausible/fresh */ }
  TEST(CorrectionVerdict, AgeAloneNeverRejects) { /* coverable, plausible, age 0.29 s -> kAppliedNotFresh, never a rejection */ }
  TEST(LioFrontend, SaturatedBackendStaysDegradedWithACorrectedOutput) {
    // DEGRADED; 3 s of kScanGood aged 0.27 s every 100 ms: never TRACKING, never LOST, output tracking error stays
    // below 0.05 m against the synthetic truth (it grows without corrections: the S1b test, now inverted)
  }
  TEST(LioFrontend, ImplausibleCorrectionIsNotApplied) { /* snapshot 2 m off -> no output step, no odometry, counted bad */ }
  TEST(LioLifecycle, AppliedNotFreshMovesTheGapReferenceButNeverConfirms) {}
  ```

  Also replace the S1b tests that pin the age rejection (`ResultOlderThanGapDegradedIsARejectedCorrection`, `SaturatedBackendReachesLostInsteadOfFlapping`, `ResultOneNanosecondOlderThanGapDegradedIsRejected`) with their D40 counterparts, and say so in the commit body.
- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio_core`. Expected: compile error, `correction_verdict.hpp` is missing.
- [ ] **Step 3: Implement.** `frontend.cpp` stays ≤ 450 lines; the verdict lives in its own file.
- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sanitize`. Expected: all pass, no TSan warning.
- [ ] **Step 5: Commit** (`Arch-Level: 3`, `Review: AR3`, `Decision: D40`, `Docs-Impact: docs/TRACEABILITY.md, docs/architecture/SYSTEM_DESIGN.md`). Update the TRACEABILITY D33/O15 row: it is superseded by D40, with the new test ids.

---

### Task 5: Ingest time base and scan wait by consequence (D40, AR3 1(e)–(f); carry-over 12, 13)

Spec §3.5. **Gate: AR3 confirmed.**

**Files:** in `src/estimation/uavnav_lio/`:
- `include/uavnav/lio_node/limits.hpp`: `enum class ClockMode { kSimulationIdentity }`, `kClockMode`, and `kMaxStampAhead` = 50 ms under `kSimulationIdentity`. The derivation: the largest stamping-to-receiving skew is 0 in SITL (probe: IMU p99 0 ms); 50 ms covers transport jitter; the harm bound is `gap_degraded − kScanPeriod`.
- Remove `kScanImuWaitMax`: a scan is dropped only when superseded.
- `src/ingest.cpp`, `src/lio_node.cpp` (start check), tests.

**Interfaces:**
- `StartError::kStampToleranceNotBelowGapMargin`: the start is refused unless `kMaxStampAhead < lifecycle_gap_degraded_s − 100 ms`.
- The refusal event reason is `STAMP_WOULD_CORRUPT_TIME_BASE`, with values `ahead_s` and `margin_s`.
- `IngestReason::kImuCoverageTimeout` is removed; `kSuperseded` stays.

- [ ] **Step 1: Write the failing tests:**
  - `LioIngest.ImuStampAheadBeyondTheSkewBoundIsRefusedWithItsHarm`;
  - `LioIngest.OneImuStampFourHundredMsAheadNeverReachesLost`;
  - `LioIngest.ScanWaitsForImuCoverageUntilSuperseded` (a 300 ms IMU stall, then IMU resumes: the scan is processed);
  - `LioIngest.BurstOfOldScansAfterAnOutageIsDropped`;
  - `LioNode.StampToleranceNotBelowTheGapMarginRefusesStart`.

  Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio`. Expected: these fail.
- [ ] **Step 2: Implement**, and fold in the S1b T7 deferred items in `ingest.hpp` (wording at :54-58 and :63; restore the 1e-12 tolerance in `test_ingest.cpp`) and T8 M4 (one diagnostic event for the first WRONG_FRAME / MALFORMED_CLOUD).
- [ ] **Step 3: Verify.** Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio && make sanitize`. Expected: all pass.
- [ ] **Step 4: Commit** (`Arch-Level: 3`, `Review: AR3`, `Decision: D40`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md`). The §3.5 future-stamp text loses "tới khi AR3 được xác nhận".

---

### Task 6: LOST as an idle wait; the restart dwell; failed re-init back to LOST (D41, AR3 3; Level 3)

Spec §3.1 target diagram, §3.4 rows 2, 2a, 2d. **Gate: AR3 confirmed.**

**Files:** in `src/estimation/uavnav_lio_core/`:
- `backend_math.hpp/.cpp`: `self_information`;
- `backend.cpp`: compute it for every scan while the map policy is `kFrozen`, with no ICP against the frozen map;
- `types.hpp`: `ScanResult::self_info`;
- `lifecycle.hpp/.cpp`: edges, reasons, `LioEvent` kinds;
- `frontend_config.hpp`, `config.hpp`, `src/config.cpp`, `config/lio/sim.yaml`;
- tests.

**Interfaces:**

```cpp
/// 6x6 information of the scan against planes fitted to its own k nearest neighbours (k = kSelfInfoNeighbours, tier a),
/// at the identity pose; split into blocks like §3.2. Pure; empty or too few points -> both blocks 0.
InformationBlocks backend_math::self_information(const PointCloudView& scan, const BackendConfig& cfg) noexcept;
```

- `LioEvent::Kind` gains `kScanSelfInfoSufficient` and `kScanSelfInfoInsufficient`. They are produced in LOST from `self_info` against the §3.2 thresholds.
- `LioReason` replaces `kGeometryReturned` with `kStableDataDwell` and gains `kReinitNotConfirmed`.
- Edges (the kLioTransitions table):
  - LOST → RESTARTING {`kStableDataDwell`};
  - INITIALIZING → LOST {`kReinitNotConfirmed`, `kLidarGapLost`, `kDegeneracyPersisted`}, only when INITIALIZING was entered by `kRestartSeeded`.
- State data, not states:
  - `stable_since_`: an optional SensorTime in LOST. Reset by a gap > `gap_degraded`, an insufficient or empty scan, or entry into LOST.
  - `reinit_since_`: set by entry from RESTARTING.
- New tier (b) key `lifecycle_restart_stable_s` [0.5, 30], beta 2.0. It is used both as the dwell and as the re-init confirmation limit, so the minimum interval between attempts is one dwell (D41).

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(BackendMath, SelfInformationOfAFloorAndTwoWallsIsFull) {}   // room_scene: both blocks above the §3.2 thresholds
  TEST(BackendMath, SelfInformationOfAFloorOnlyIsDegenerate) {}    // translation block below threshold (D36 case)
  TEST(LioLifecycle, LostWaitsWithoutLimitWhileDataIsUnstable) {}  // 600 s of insufficient scans: LOST, no edge
  TEST(LioLifecycle, StableDwellRestarts) {}                       // exactly restart_stable_s of sufficient scans -> RESTARTING at the boundary tick
  TEST(LioLifecycle, UnstableDataNeverRestartsBeforeTheDwell) {}   // one insufficient scan inside the dwell resets it
  TEST(LioLifecycle, FailedReinitReturnsToLostAndWaitsAFullDwell) {}
  TEST(LioLifecycle, FirstBootstrapHasNoReinitLimit) {}
  TEST(LioLifecycleTable, NoTerminalStateAndNoLostToTracking) {}   // every state has an outgoing edge; LIO-4
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio_core`. Expected: compile errors for the new names.
- [ ] **Step 3: Implement.** Backend files keep the D32 `#error` tail.
- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sanitize`. Expected: all pass. Also report the time of `self_information` on the room scene (p99), which must stay below the ICP time it replaces in LOST.
- [ ] **Step 5: Commit** (`Arch-Level: 3`, `Review: AR3`, `Decision: D41`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md`). Remove "chờ AR3" from §3.1.

---

### Task 7: LIO restarts itself; seed choice `SYNC` or `SELF` (D41, AR3 3; Level 3)

Spec §3.4 ("LIO tự khởi động lại", "Seed"). **Gate: AR3 confirmed.** This task does not need O29: offers can come from a test, and without any offer the seed is `SELF`.

**Files:** in `src/estimation/uavnav_lio_core/`:
- `runner.hpp/.cpp`: `offer_seed`; `restart()` leaves the public API;
- `frontend.hpp/.cpp`: the restart effect on LOST → RESTARTING; the automatic re-issue after a `LioRestartRejected`;
- `types.hpp`: `SeedOffer`, `SeedSource`;
- tests.

**Interfaces:**

```cpp
struct SeedOffer {                 ///< the sync's latest T^-1 x PX4 pose (§3.4)
  time::SensorTime t;              ///< SensorTime of the PX4 sample
  Eigen::Vector3d p_world_m;       ///< base_link in lio_odom (FLU)
  double yaw_world_rad;
  std::array<double, 4> t_used;    ///< x, y, z, yaw of the T used (logged in LioRestart)
};
enum class SeedSource : std::uint8_t { kSync, kSelf };
/// Ingest thread; latest value wins; never triggers a restart by itself.
void LioRunner::offer_seed(const SeedOffer& offer) noexcept;
/// Pure. kSync when the predictor covers offer.t: p, yaw moved forward by the predictor's motion from offer.t to `now`;
/// roll/pitch from the predictor at `now`. Otherwise kSelf: the predictor's newest pose.
std::pair<SeedPose, SeedSource> choose_seed(const std::optional<SeedOffer>& offer, const OutputPredictor& p, time::SensorTime now) noexcept;
```

- The `LioRestart` event carries: epoch old/new, reason `STABLE_DATA_DWELL`, `seed_source`, the seed, `offer_age_s` and `t_used`.

- [ ] **Step 1: Write the failing tests:**
  - `ChooseSeed.SyncOfferIsForwardedByThePredictorMotion`: offer 150 ms old while moving at 3 m/s; the seed lands within 1 mm of the truth at `now`.
  - `ChooseSeed.UncoverableOfferFallsBackToSelf`.
  - `LioRunner.RestartsItselfAfterTheDwellWithoutAnyCall`.
  - `LioRunner.RefusedLateAnswerIsReissuedWithTheNewestSeed`.
  - `LioRunner.OfferNeverTriggersARestartOutsideLost`.

  Run: `make build PKGS=uavnav_lio_core`. Expected: compile errors.
- [ ] **Step 2: Implement.** Keep `LioIngest` compiling without seeds (Task 8 adds the subscription).
- [ ] **Step 3: Verify.** Run: `make build PKGS="uavnav_lio_core uavnav_lio" && make test PKGS="uavnav_lio_core uavnav_lio" && make sanitize`. Expected: all pass.
- [ ] **Step 4: Commit** (`Arch-Level: 3`, `Review: AR3`, `Decision: D41`, `Docs-Impact: docs/TRACEABILITY.md`).

---

### Task 8: Seed offers from the sync; the stale-epoch odometry gate (O29, O25)

Spec §3.4, §4.1, §6.5. **Gate:** Steps 1–2 (pure offer helper, epoch gate) have none. Steps 3–4 (message, publisher, lio subscription) wait for O29.

**Files:**
- In `src/px4/uavnav_px4_bridge/`: create `include/uavnav/px4bridge/seed_offer.hpp`, `src/seed_offer.cpp`, `test/test_seed_offer.cpp`.
- Modify `bridge.hpp/.cpp`: `BridgeOutputs::seed_offer`; `BridgeReason::kStaleEpoch`.
- Modify `ros_conversion.*`, `px4_bridge_node.cpp`.
- Create `src/core/uavnav_interfaces/msg/LioRestartSeed.msg`.
- Modify `src/core/uavnav_ros/include/uavnav/ros/qos.hpp`: seed QoS reliable, depth 1; fix the stale `/livox/*` comment.
- Modify in `src/estimation/uavnav_lio/`: the subscription and `LioIngest::on_seed_offer` → `LioRunner::offer_seed`.
- Modify `uavnav_lio_core/include/uavnav/lio/limits.hpp` (`kEventSummaryEvery` from one place; carry-over 10).

**Interfaces:**

```cpp
enum class SeedOfferReason : std::uint8_t { kAlignmentNotUsable, kPx4PositionInvalid, kNotFinite };
/// Pure. Ok when state is VALID or FROZEN, xy_valid && z_valid, all finite. p = FLU(Rz(-yaw_T)(p_ned - t_T)),
/// yaw = wrap(-(yaw_px4 - yaw_T)); t = the PX4 sample's SensorTime. No freshness bound here: LIO judges it by covers().
Result<lio_msgs::SeedOffer, SeedOfferReason> make_seed_offer(AlignmentState state, const Pose4& t, const Px4LocalPositionIn& px4) noexcept;
```

- One offer per `on_tick` from the newest valid PX4 sample.
- The refusal event is rate-limited per reason, and only while the newest health is not TRACKING (otherwise offers are just not needed).
- Epoch gate: odometry with an epoch lower than the newest accepted health epoch is refused `STALE_EPOCH`: no EV, no pair.
- `LioRestartSeed.msg` fields: `stamp`, `position`, `yaw_rad`, `t_x_m`, `t_y_m`, `t_z_m`, `t_yaw_rad`, `alignment_state`.

- [ ] **Step 1: Write the failing tests:**
  - `SeedOffer.InvertsTFarFromTheOrigin` (T = (120, −40, 3, 0.7); p_lio_frd = (250, 30, −2.5); round trip within 1e-9 m);
  - `SeedOffer.RefusedUnlessValidOrFrozenAndPx4Valid`;
  - `Px4Bridge.OldEpochOdometryAfterNewEpochHealthIsRefused`;
  - `Px4Bridge.OneOfferPerTickFromTheNewestSample`.

  Run: `make build PKGS=uavnav_px4_bridge`. Expected: compile error.
- [ ] **Step 2: Implement and verify.** Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: all pass. Commit (`Arch-Level: 1`, `Docs-Impact: docs/TRACEABILITY.md`).
- [ ] **Step 3 (O29): Message, publisher, lio subscription.** Tests:
  - `Px4BridgeRos.SeedOfferMessageRoundTrips`;
  - `Px4BridgeNode.PublishesSeedOffersWhileTIsUsable`;
  - `LioNode.SeedOfferReachesTheRunnerAndARestartUsesIt` (synthetic room scene, gap → LOST → dwell → `LioRestart` with `seed_source=SYNC`).

  Run: `make build PKGS="uavnav_interfaces uavnav_ros uavnav_px4_bridge uavnav_lio" && make test PKGS="uavnav_interfaces uavnav_ros uavnav_px4_bridge uavnav_lio"`. Expected: all pass.
- [ ] **Step 4: Commit.** `Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/architecture/DECISIONS.md, docs/TRACEABILITY.md`. Also: §6.5 gets the message and QoS; drop the ★ from row 2b; turn O29 into a D-entry.

---

### Task 9: Resync of `T` after a new epoch; DV2 (O33; carry-over 4, 5, 17)

Spec §4.1 "Đồng bộ lại sau epoch mới". **Gate: O33** for Steps 1–3. Step 4 (DV2) has none: DV2's trigger is "the first task touching the alignment".

**Files:**
- In `src/px4/uavnav_px4_bridge/`: `alignment.hpp/.cpp` (resync window, edge FROZEN → INIT `kResyncDiscontinuity`, PX4 resets in the window), `config.hpp/.cpp`, `config/px4_bridge/sim.yaml` (`alignment_resync_window_s` [0.5, 10], beta 2.0), tests.
- Create docs/analysis/2026-10-s1c-dv2.md.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Alignment, SyncSeededEpochResyncsFrozenTWithTheFirstPair) {}      // T unchanged within 1e-6
  TEST(Alignment, Px4ResetInsideTheResyncWindowIsAppliedAndStaysFrozen) {}  // EV-induced EKF2 reset (F33c)
  TEST(Alignment, Px4ResetWhileLioIsStillLostStillInvalidates) {}       // outside the window: the kept rule
  TEST(Px4Bridge, SelfSeededEpochReinitialisesTAfterTheResyncWindow) {} // Review Focus 4
  TEST(Px4Bridge, LargeHeadingResetAfterResyncKeepsTheVehiclePointContinuous) {}  // +20 deg with xy delta; 1 mm
  TEST(AlignmentFuzz, EveryStateChangeIsAListedEdge) {}                 // extended with the new edge
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: the window tests fail.
- [ ] **Step 3: Implement.** Update the §4.1 diagram (FROZEN → INIT), drop the ★, and turn O33 into a D-entry. Commit (`Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/architecture/DECISIONS.md, docs/TRACEABILITY.md`). The report names PX4-1 as touched-and-held (the table and the spec change together).
- [ ] **Step 4: DV2.** From `log/sitl/20261013T155144Z-no_gps-hover` (113 `NO_PX4_SAMPLE` in the window), measure three things around each rejection:
  - the `vehicle_local_position` inter-arrival and `timestamp_sample` lag;
  - the pair's LIO stamp against the newest buffered PX4 sample;
  - the buffer span.

  Decision:
  - the cause is PX4's rate or lag, with `T` VALID: close DV2;
  - the cause is the bridge buffer or the time matching: write a failing test, fix it, re-judge the run offline;
  - otherwise: keep DV2 OPEN, with Task 13's data as the next trigger.

  Commit the report and the DV2 row.

---

### Task 10: Fault injection in the harness: LiDAR cut and climb loss

Spec §7.2 (S1c), §3.4. There is no product change.

**Files:**
- Modify: `src/uavnav_bringup/launch/s1_estimation.launch.py`:
  - required argument `start_lidar_bridge` (`true` | `false`);
  - the docstring states the Fast DDS profile dependency (carry-over 14).
- Modify: `tools/uavnav/sitl_hover.sh`:
  - `PATTERN=lidar_cut CUT_S=<s>` and `PATTERN=climb_loss`;
  - the harness starts the LiDAR `ros_gz_bridge` as its own process group and exports `UAVNAV_LIDAR_BRIDGE_PGID`;
  - fold in the S1b T3 harness minors (`PROFILE ?=` pickup, `commands.txt` quoting).
- Modify: `tools/uavnav/sitl_hover.py`:
  - `lidar_cut`: hover 20 s, `SIGSTOP` the group for `CUT_S` of /clock, `SIGCONT` (also in `finally`), hover 40 s;
  - `climb_loss` (gps): climb to 7.0 m and hold 15 s (no geometry); descend to 4.8 m and hold 20 s (the edge); descend to 2.5 m and hold 30 s;
  - windows into `hover_meta.json`;
  - record `/lio/restart_seed`.
- Modify: `tools/uavnav/sitl_checks.py` (`evaluate_recovery`), tests.

**Criteria** (`evaluate_recovery(run_dir, profile, spawn_yaw_rad) -> dict`):

| Check | Pass when |
|---|---|
| `lost_detected` | LOST within cut start + `gap_lost` + 0.2 s (F22 in SITL) |
| `waits_while_unstable` | no `LioRestart` while the LiDAR is cut or while the scan self-information is insufficient; consecutive attempts at least `restart_stable_s` apart after a failed one (D41) |
| `restart` | `LioRestart` within `restart_stable_s` + 0.5 s of stable data returning; epoch +1 per applied restart; `seed_source` reported |
| `retracked` | TRACKING of the new epoch within `restart_stable_s` of `LioRestart` |
| `resync` | `SYNC` seed: `T` FROZEN → VALID within 2.0 s of TRACKING. `SELF` seed: `T` → INIT, then VALID within `alignment_resync_window_s` + 3 s. Never stuck FROZEN for `frozen_max` after a TRACKING epoch |
| `ev_epochs` | no EV with the old epoch after the new epoch's first health; EV of the new epoch sent from its first TRACKING odometry (D42) |
| `px4_continuity` | gps: PX4 horizontal error against ground truth ≤ 0.5 m throughout; no_gps: reported |
| `events` | as for the hover, plus counts per reason of `CorrectionVerdict`, `LioStabilityReset`, `LioRestartRejected` and the seed-offer refusals |

- [ ] **Step 1: Write the failing tests** (`SitlChecksTest`): `test_recovery_pass_on_synthetic_records`; `test_restart_during_cut_fails`; `test_restarts_closer_than_the_dwell_fail`; `test_self_seed_expects_reinit`; `test_recovery_missing_cut_window_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Run: `tools/uavnav/gate.sh python && make sitl-build && make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2 DRY_RUN=1`. Expected: `GATE_RESULT=PASS`; the dry run lists the separate LiDAR bridge group.
- [ ] **Step 3: Commit** (`Arch-Level: 1` for the launch argument; `Docs-Impact: docs/TRACEABILITY.md`).

---

### Task 11: Recovery SITL runs and report (D38, D41)

Spec §3.4, §4.1, §7.2. The runs take about 30 min. It needs Tasks 4–10, with O29 and O33 decided. If O29 is still open, the runs show `SELF` seeds only; say so.

- [ ] **Step 1: Four runs**, one at a time:

  ```bash
  make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2      # SYNC seed, T resyncs FROZEN -> VALID
  make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=12     # T INVALID at 10 s (no offers), SELF seed, T re-INIT
  make sitl-hover PROFILE=gps PATTERN=climb_loss             # waits at 7 m, no rapid restarts at 4.8 m, recovers at 2.5 m
  make sitl-hover PROFILE=no_gps PATTERN=lidar_cut CUT_S=1.5 # EV of the new epoch makes EKF2 reset inside the resync window
  ```

  Expected: all PASS. On failure, read the timelines first (`python3 -m tools.uavnav.events log/sitl/<run>/events_*.jsonl --timeline`). Then at most one fix-and-re-run per root cause; a design gap becomes an O-entry.
- [ ] **Step 2: Report** docs/analysis/2026-10-s1c-recovery.md. Per run, the chain timeline: LOST, the stability resets, RESTARTING, `LioRestart` with its seed source, TRACKING, `T` transitions, EV epochs and PX4 resets. Also the PX4 continuity, and what stays for S2 (the give-up and the handover).
  - Register the runs.
  - Raise the TRACEABILITY rows: P5 and the D38/D41 row to `INTEGRATED`; F22 to `INTEGRATED` with the cut runs.
  - Run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 3: Commit.**

---

### Task 12: Drift harness and metrics (D39)

Spec §7.2 (S1c), D39. There is no product change.

**Feasibility (PX4 v1.17.0-14 source):**
- `MPC_POS_MODE=0` ("Direct velocity", `src/modules/mc_pos_control/multicopter_position_mode_params.c`) maps the sticks to a velocity setpoint: `v = expo_deadzone(stick, 0.6, MAN_DEADZONE) × MPC_VEL_MANUAL`, rotated into the heading frame (`FlightTaskManualPosition.cpp` lines 70–97, `lib/sticks/Sticks.hpp` lines 66–73).
- The defaults are `MPC_VEL_MANUAL` 10 m/s and `MAN_DEADZONE` 0.1.
- **What is missing:** the stick stream sends `pitch = 0` (`tools/uavnav/sitl_probe.py:203`).
- `long_open_featured_speed.sdf` (a 140 m leg, a free ±8 m band, features at ±8.5–12 m) lacks the `<magnetic_field>` and `<atmosphere>` of `px4_lio_smoke`.

**Files:**
- Modify the world (add the two elements).
- Modify `tools/simulation/run_px4_mid360.sh` (`MPC_POS_MODE=0`, `MAN_DEADZONE=0.1` for `PATTERN=drift` only).
- Modify `tools/uavnav/sitl_hover.sh` (`PATTERN=drift SPEED_MPS=<1..5> [WORLD=...]`, bound 600 s, `PX4_GZ_WORLD`).
- Modify `tools/uavnav/sitl_probe.py`:
  - `set_sticks(throttle, pitch)`;
  - the stick stream starts 3 s after the first /clock, so the record shows whether PX4 starts in POSCTL by itself (carry-over 9).
- Modify `tools/uavnav/sitl_hover.py`.
- Create `tools/uavnav/drift.py`, `tools/uavnav/tests/test_drift.py`.
- Modify `sitl_checks.evaluate_drift`.

**Flight:**
1. Climb to 2.5 m; 5 s stationary (the alignment window).
2. In-flight `DO_SET_MODE` Hold (1/4/3): expect `nav_state` 4 within 1 s; hold 3 s. Then POSCTL (1/3/0): expect `nav_state` 2 within 1 s (`in_flight_mode_switch`; carry-over 9).
3. Legs along world +x at `stick_for_speed(SPEED_MPS)`, from 5 m to 105 m and back, until the travelled distance is ≥ 200 m and the motion time is ≥ 120 s of /clock.
4. Hover 5 s, land.

**Interfaces** (`drift.py`, pure, numpy):

```python
def stick_for_speed(v_mps, vel_manual_mps=10.0, expo=0.6, deadzone=0.1) -> float   # inverse of PX4 expo_deadzone
def align_4dof(est, gt) -> tuple[float, np.ndarray]                               # yaw, translation over the stationary window
def drift_curve(est_aligned, gt) -> np.ndarray                                    # s_m, t_s, err_h_m, err_3d_m, err_yaw_rad
def relative_drift(est, gt, lengths_m=(10, 25, 50, 100)) -> dict                  # mean, p95 (m/m), n per length
def per_minute_drift(est, gt, window_s=60.0) -> dict                              # mean, p95 (m/min), n
def evaluate_drift(run_dir) -> dict   # LIO=/lio/odometry, PX4=vehicle_local_position (NED->ENU, timestamp_sample),
                                      # GT=/sim/ground_truth/odometry interpolated at each estimate stamp; writes drift.json
```

Also reported per run:
- the achieved cruise speed;
- DEGRADED→TRACKING per 60 s and the applied-result age p50/p99 (DV1);
- the verdict counts (D40);
- the output tracking error per speed (the τ data, O13);
- the `NO_PX4_SAMPLE` fraction (DV2);
- the heading sources and resets in motion (D42).

The data checks pass when:
- the speed is within ±10%;
- `in_flight_mode_switch` passes;
- every topic has data.

Losses are data, not failures.

- [ ] **Step 1: Write the failing tests** (`DriftTest`):
  - `test_stick_for_speed_inverts_px4_expo_deadzone`;
  - `test_align_recovers_a_known_4dof_offset`;
  - `test_constant_velocity_bias_gives_linear_drift` (a 1% scale error gives 0.01 m/m and 0.6·v m/min);
  - `test_relative_drift_needs_enough_segments`;
  - `test_drift_missing_topic_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `ModuleNotFoundError: drift`.
- [ ] **Step 2: Implement.** Run: `tools/uavnav/gate.sh python && make sitl-hover PROFILE=gps PATTERN=drift SPEED_MPS=3 DRY_RUN=1`. Expected: `GATE_RESULT=PASS`; the dry run shows the world, 600 s and `MPC_POS_MODE=0`.
- [ ] **Step 3: Commit** (`Arch-Level: 0`).

---

### Task 13: Drift runs and the benchmark report (D39; DV1, O13 data)

Spec §7.2 (S1c), D39. The runs take about 110 min: 12 runs of 8–10 min.

- [ ] **Step 1: Runs**, one at a time:
  - `PROFILE=gps` and `PROFILE=no_gps`, `PATTERN=drift SPEED_MPS=<v>`, for v = 1..5;
  - one repeat each of gps at 1 and at 5 m/s.
- [ ] **Step 2: Report** docs/analysis/2026-10-s1c-drift-benchmark.md (Vietnamese). It covers:
  - per profile and speed: LIO and EKF2 drift (m/m, m/min: mean and p95), the growth with distance, and the yaw drift;
  - the benchmark table below, with every URL;
  - the sim-to-real gap;
  - DV1: close it if, in every run, there is at most 1 DEGRADED→TRACKING per 60 s and the age p99 is below 0.145 s; otherwise propose the AR3 option (iv) data to the owner;
  - the τ observation (a τ change is an owner item);
  - the innovation distribution, to re-tune `correction_innovation_max_*` (tier (b), with the data in the commit).

  First try to verify the UNVERIFIED rows from the ICM-40609-D datasheet PDF.
- [ ] **Step 3: Register the runs, update TRACEABILITY** (D39 row, τ row, DV1), then run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 4: Commit.**

**Benchmark reference** (gathered 2026-10-14; the report re-checks it):

| Quantity | Real (source) | SITL model (source in repo) | Gap to state |
|---|---|---|---|
| Mid-360 range precision (1σ) | ≤ 2 cm at 10 m; ≤ 3 cm at 0.2 m ([Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs)) | Gaussian σ 0.02 m at every range (`src/sim/uav_simulation/models/lidar_mid360/model.sdf`) | matches at 10 m; no reflectivity or range dependence |
| Angular precision (1σ) | < 0.15° (same) | none | at 20 m about 5 cm lateral noise missing |
| FOV | 360° × (−7°..52°) (same) | 360° × (−7°..52°) (F36) | none |
| Point rate, frame rate | 200,000 pts/s first return, 10 Hz (same) | 720 × 28 × 10 Hz = 201,600 pts/s | same rate; 28 fixed rings (≈ 2.2°) against a non-repetitive pattern |
| Detection range | 40 m at 10 %, 70 m at 80 % reflectivity (same) | 40 m for all surfaces | sim conservative on bright surfaces |
| Per-point timing | spread over the 100 ms scan (needs deskew) | one instant per scan (O20) | **no motion distortion in sim**: up to 0.5 m smear at 5 m/s before deskew |
| Built-in IMU | ICM40609 (same) | 200 Hz | |
| Gyro noise density | 4.5 mdps/√Hz ([TDK ICM-40609-D](https://invensense.tdk.com/en-us/products/6-axis/icm-40609-d)) | σ 0.001 rad/s at 200 Hz ≈ 4.1 mdps/√Hz (σ = N·√f_s) | close; state the convention |
| Accel noise density | 100 µg/√Hz (same) | σ 0.01 m/s² at 200 Hz ≈ 72 µg/√Hz | sim about 30 % quieter |
| IMU bias | offset temperature coefficient ±10 mdps/°C, ±0.15 mg/°C (same); bias instability **UNVERIFIED** | none | **no bias in sim**: IMU-only intervals drift far less than on hardware |
| PX4 IMU, mag, baro (x500) | | gyro σ 8.7e-4 rad/s, accel σ 0.0064–0.0069 m/s² at 250 Hz, mag σ 1e-4 G, baro σ 3 Pa (`~/Dev/Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`) | |
| Simulated GNSS | | Gauss-Markov, amplitude 0.8 m, random walk 0.01, coefficient 0.95 (PX4 v1.17.0-14 `src/modules/simulation/gz_bridge/GZBridge.hpp`, constants `_pos_noise_amplitude`, `_pos_random_walk`, `_pos_markov_time`) | the gps EKF2 "drift" is GNSS noise |
| EKF2 assumed noise | defaults `EKF2_GYR_NOISE` 0.015 rad/s, `EKF2_ACC_NOISE` 0.35 m/s², `EKF2_GPS_P_NOISE` 0.5 m, `EKF2_EVP_NOISE` 0.1 m, `EKF2_MAG_NOISE` 0.05 G, `EKF2_HEAD_NOISE` 0.3 rad (PX4 v1.17.0-14 `src/modules/ekf2/module.yaml`, `params_*.yaml`) | same | PX4 publishes no EKF2 accuracy figure (none found: **UNVERIFIED**) |
| Magnetic declination | WMM at the world's coordinates −0.93° (EKF2 state; S1b F34 report §5) | +0.75° | 1.7° heading offset when the compass leads; `T` absorbs it (D42) |
| Real LIO drift, for scale | FAST-LIO2: 0.14 m end-to-end over ≈ 650 m; < 0.06 m over 81 m of fast handheld motion, Livox Avia ([Xu et al., arXiv 2107.06829](https://arxiv.org/abs/2107.06829)) | | end-to-end (loop) errors understate mid-trajectory drift; a different sensor |

---

### Task 14: Close S1c

**Files:** `docs/TRACEABILITY.md`, `docs/architecture/SYSTEM_DESIGN.md`, `tools/uavnav/arch_rules.yaml`, this plan.

- [ ] **Step 1: Run the gate, sanitize and a final hover.** Run: `tools/uavnav/gate.sh all && make sanitize && make sitl-hover PROFILE=gps`. Expected: `GATE_RESULT=PASS`, no TSan warning, hover `PASS`.
- [ ] **Step 2: Spec sync.**
  - §3.5: the frontend/backend text matches `judge_correction` (no more "`kScanDegenerate`" wording);
  - §3.1: the σ edge in the drawn diagram;
  - remove every "chờ" marker that this slice has closed.
- [ ] **Step 3: Governance** (carry-over 11):
  - classify the 7 legacy packages in `arch_rules.yaml`;
  - run `python3 -m tools.uavnav.arch_health` and record any trigger as an AR row (`Owner xác nhận` = `chưa`);
  - recalibrate the `stress:` thresholds, with the reason in the commit.
- [ ] **Step 4: Append "Carry-over to S2".** It must include:
  - the Supervisor's give-up, with the budget counted from the first LOST of the episode (§2.2);
  - REACQUIRING on a TRACKING epoch, plus `ResetMap` (S3);
  - `px4_mode`;
  - every O-entry opened in Tasks 3, 11 and 13 (for example a proposed D42 exception).
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact:` the files touched).

---

## Carry-over coverage (S1b "Carry-over to S2", items 1–17)

| # | Item | Here |
|---|---|---|
| 1 | τ re-evaluation, F23 step with motion | Task 13 (data and report); a τ change is an owner item |
| 2 | Degeneracy thresholds | closed (D36). The thresholds are reused for the self-information in Task 6. |
| 3 | DV1 (review 2026-11-30 or the first motion flight) | Task 13: the drift flights are the motion flights. The D40 verdict leaves the 0.145–0.25 s band as `kApplied`, so DV1 stays relevant |
| 4 | `NO_PX4_SAMPLE` cause | Task 9 Step 4 (DV2) |
| 5 | The ≈ 20° heading reset absorbed by `T` | measured in Task 3 (EV full, both profiles) and Task 11; pinned by `Px4Bridge.LargeHeadingResetAfterResyncKeepsTheVehiclePointContinuous` (Task 9) |
| 6 | Stale-epoch odometry gate; two resets in one dropped window | gate: Task 8. The two-reset limit stays documented: Not in this plan |
| 7 | `/lio/scan`; restart caller; seed helper; `T` in `LioRestart` | the caller is LIO itself (Task 7); offers: Task 8; `t_used` in `LioRestart`: Task 7; `/lio/scan`: S3 |
| 8 | Hardware (two lost scans at `gap_degraded`, DDS profile, Mid-360 transport) | Not in this plan (P8) |
| 9 | `DO_SET_MODE` in flight; POSCTL source; `ImuGap`; floor-only scene | Task 12 for the first two; the rest: Not in this plan |
| 10 | Deferred review minors | Task 2 (T11 M2/M3, events list), Task 5 (T7 deferred list, T8 M4), Task 10 (T3 harness minors), Task 8 (`kEventSummaryEvery`, `qos.hpp` comment) |
| 11 | Governance | Task 14 |
| 12 | lio future-stamp tolerance | Task 5 (D40: the time-base guard by clock mode) |
| 13 | lio `kClockMode` | Task 5 |
| 14 | Fast DDS profile not in the launch | Task 10 (docstring); the hardware check stays P8 |
| 15 | Unknown health code; zero-tolerance event list | the code is refused before the core (spec §4.1); event list: Task 2 |
| 16 | D36 scope (corridor, real noise) | Not in this plan (P8) |
| 17 | DV2 | Task 9 Step 4 |

DV1 and DV2 keep 2026-11-30 as a backstop. Their event triggers are met in Task 13 and Task 9.

## Not in this plan

| Goes to | Items | Reason |
|---|---|---|
| S2 | `px4_mode`; Supervisor NavAvailability with the D11 give-up counted from the episode's first LOST; handover reasons; straight trajectories; Y1/Y3a; a τ change if decided | S2 scope. LIO no longer gives up (D41), so the give-up lives only there. |
| S3 | `/lio/scan`, Mapping, `ResetMap` on a new epoch | No consumer before Mapping (O25). |
| Hardware (P8) | Livox `CustomMsg`, a real clock mode and its skew bound, transport, Mid-360 noise, corridor degeneracy, site `EKF2_MAG_DECL`, two lost scans at `gap_degraded`, the `ImuGap` policy, floor-only scene | Out of the beta. |
| Later, with data | the trend of the output tracking error as a health input (AR3 point 1(c)) | D40 asks for measurable consequences; Task 13 provides the data first. |
| Documented limit | Two PX4 resets in one dropped-sample window apply only the last delta | The jump gate is the backstop. |

## Model and execution notes

- **Model per task:**
  - most capable model: Tasks 4, 6, 7 (verdict, lifecycle, self-restart), 9 (resync), 11 (recovery diagnosis), 3 and 13 (judgement of PX4 behaviour and of the benchmark);
  - capable model: Tasks 5, 8, 10 and 12;
  - mid-tier model: Tasks 1, 2 and 14.

  Reviewers: the most capable model for Tasks 4–9 and the final review; the capable model otherwise.
- **Order:**
  - 2 → 3 → (1 when the owner confirms AR3) → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13 → 14.
  - If AR3 is not confirmed after Task 3: Task 8 Steps 1–2, Task 9 Step 4, then Tasks 12–13, and come back.
  - If O29 or O33 is still open at its task, do that task's non-gated steps and continue. Task 11 then runs with `SELF` seeds or with the old resync rules, and says so.
  - One active task at a time (CLAUDE.local.md). Read-only reviewers may run alongside, at most 3 agents in total.
- **Machine limits:** Makefile only (`-j3`, sanitize `-j2`). SITL never overlaps a build, a test or sanitize. Agents run SITL themselves (D43).
- **Estimated SITL time:**
  - Task 3: 5 runs, about 35 min;
  - Task 11: 4 runs plus at most 2 re-runs, about 40 min;
  - Task 13: 12 runs, about 110 min;
  - Task 14: 1 hover, 5 min.
- **TSan:** Tasks 4–7 run `make sanitize`. No new thread is added; `self_information` runs on the estimator thread.

# S1c: LIO Recovery Chain, Heading-Source Profiles, Drift Benchmark

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Before S2, give the estimation stack the owner's three 2026-10-14 directives:
- a LOST LIO recovers by itself: try tracking, then restart with the seed `T⁻¹ × PX4 pose`, then resynchronise `T` (D38);
- the heading source is a profile axis, compass first, EV yaw only without a compass (D37);
- a drift benchmark of LIO and EKF2 against ground truth at 1–5 m/s, set against the sensor datasheets (D39).

**Architecture:**
- No new state anywhere. The recovery chain runs on the existing `LioLifecycle` and `AlignmentEstimator` machines (spec §3.4 chain table).
- The new parts:
  - one message `/lio/restart_seed` (px4_bridge → lio, O29);
  - one retry edge INITIALIZING → LOST after a restart (O30);
  - one EV-gate rule (O31).
- Every decision stays in a ROS-free core: the seed helper in the bridge core, the seed composition in the `uavnav_lio` ingest library, the retry edge in `uavnav_lio_core`. The nodes only convert and publish.
- SITL stays in POSCTL: there is no `px4_mode` and no Supervisor. The fault injection and the drift flights are new patterns of the existing bounded harness. Pure, unit-tested Python checkers judge them.

**Tech Stack:** C++23, Eigen 3, ament_cmake, GoogleTest, rclcpp (Jazzy), px4_msgs v1.17.0, Gazebo Harmonic, PX4 v1.17 SITL, Python 3 (`unittest`, `rclpy`, numpy 1.26), GNU make.

**Spec:** [SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md):
- §0 (heading-source axis), §3.1, §3.4 (the recovery chain table), §3.5, §4.1, §6.1, §6.5, §7.2 (slice S1c).
- Decisions in [DECISIONS.md](../architecture/DECISIONS.md): D4, D11, D18–D21, D30, D31, D33 (O16, O20, O22, O25), D35–D39; facts F22, F33, F34, F36.
- Open entries this plan depends on: O28–O32 (see "Owner gates").
- Predecessor: the "Carry-over to S2" table of the [S1b plan](2026-10-12-s1b-ros-shells-and-sitl.md). Every item is mapped in "Carry-over coverage".

---

## Slice order and why

The chosen order is S1c (this plan), then S2 (`px4_mode`, minimal Supervisor, straight trajectories, Y1/Y3a), then S3 onward.

1. **Heading source first (Tasks 2–3).** It is small, decided (D37), and changes the PX4 profile that every later SITL run uses. Flying recovery or drift on the D35 profile would have to be repeated.
2. **Recovery next (Tasks 4–10).** Owner priority 1 (D38). The chain lives entirely in `lio` and `px4_bridge`, and it can be exercised in POSCTL by cutting the LiDAR. S2 then builds the Supervisor on a chain that already works, instead of on a LOST that never ends.
3. **Drift last (Tasks 11–12).** It needs no new product code, but its flights at 5 m/s may reach DEGRADED or LOST. With recovery in place, a run survives a loss and also measures the recovery under motion.
4. **Gate rule.** Tasks 5–7 wait for O29–O31. If the owner has not answered when Task 4 is done, run Tasks 11–12 next and come back. Task 1 waits for O28 and can run at any point. Nothing else waits.

**Rejected alternatives:**

| Alternative | Why rejected |
|---|---|
| Keep the spec order: S2, S3, then the recovery in S4 | Contradicts D38 ("kéo lên sớm"). Every S2 flight would end at the first LOST, and the drift benchmark would lose its runs. |
| Fold the recovery into S2, with the Supervisor as the restart caller | The Supervisor owns neither `T` nor the PX4 pose, so it would copy the bridge's knowledge (O29 (b)). Two risky subsystems in one slice. And the chain cannot be tested before a mission flies. |
| Drift benchmark only, then S2, with recovery after S2 | D38 is priority 1. A benchmark without recovery measures only flights that never lose LIO. |
| Restart driven by the `lio` node itself, which reads PX4 | Pulls PX4 into `lio` and breaks the layer rule (AGENTS §2.7, O29 (c)). |

## Owner gates

These entries are OPEN. A gated task does not start before the owner records the decision. The controller then turns the O-entry into a D-entry and updates the spec, in the same commit as the task's first change.

| Entry | Recommended | Gates |
|---|---|---|
| O28 | lio future-stamp tolerance 0.2 s (< `gap_degraded`) | Task 1, the tolerance part only |
| O29 | (a) the bridge computes the seed and publishes `/lio/restart_seed`; the lio node restarts on it | Tasks 6, 7 |
| O30 | (a) INITIALIZING (entered from RESTARTING) → LOST on `reinit_timeout_s` 3.0 s or gap; REACQUIRING → LOST keeps `since` | Task 5 (the Supervisor half goes to S2) |
| O31 | (a) EV of a new epoch is withheld while `T` is FROZEN | Task 6, the gate part only |
| O32 | (a) three profiles `gps`, `no_gps`, `no_gps_ev_yaw`; no compass emulated by `EKF2_MAG_TYPE=5`, `SYS_HAS_MAG=0` | Task 2, the `no_gps_ev_yaw` file only |

A decision different from the recommendation is a plan change: the controller edits the affected task before it is dispatched.

## Global Constraints

- **Tree, staging and commits:** the S1b plan's Global Constraints apply unchanged: single tree, `git add <path>` only, `git diff --cached --stat` before every commit, Makefile-only builds at `-j3`, at most 3 agents.
- **Trailers:** a commit that touches a contract path (`config/`, `msg/`, a public header) or an invariant file ends its body with `Arch-Level: 1` and `Docs-Impact: <paths>`, then `Co-Authored-By: <the model that did the work> <noreply@anthropic.com>`. An executor never declares Level 2 or 3: it stops and reports.
- **SITL:** one run at a time on an idle machine. Never while a build, a test or sanitize runs. Headless, artifacts only under `log/sitl/`.
  - Bounds: 300 s per hover or recovery run; 600 s per drift run (`PATTERN=drift` only).
  - Every run id that a doc cites is registered in docs/analysis/2026-10-s1c-sitl-runs.md, which Task 3 creates (`arch_check` requires it).
- **No ROS in the cores** (`uavnav_lio_core`, `uavnav_px4_bridge` core target, the `uavnav_lio` ingest target).
- **Events:** every seed offer, seed refusal, restart, retry and gated EV emits one event with a typed reason (AGENTS §2.3). Per-sample refusals are rate-limited like the existing bridge episodes.
- **Config (D22):**
  - new tier (b) keys go in the existing spec tables with bounds;
  - new tier (a) constants go in the package `limits.hpp` with a derivation comment;
  - profile values go only in `config/px4/profiles/*.yaml`;
  - harness-only PX4 parameters (`MPC_POS_MODE`) go only in `tools/simulation/run_px4_mid360.sh`.
- **Time:** the seed stamp is SensorTime; ages are computed in SensorTime (D29 identity in SITL).

## Review Focus

1. **A seed for the wrong epoch, or a stale one, after a refused restart answer (O16 retry).** The lio node refuses a seed whose `lio_epoch` is not the current RESTARTING epoch, or whose age exceeds 0.2 s, with an event, and calls nothing. Pinned by `LioIngest.SeedForAnotherEpochOrTooOldIsRefused` (Task 7).
2. **Old-epoch odometry arriving after new-epoch health (O25).** No EV is ever sent with the old epoch after the restart, and no alignment pair is formed from it. Pinned by `Px4Bridge.OldEpochOdometryAfterNewEpochHealthIsRefused` (Task 4).
3. **A large PX4 heading reset (≈ 20°) just after the resync**, in the ev_yaw profile. `T` takes the frame change G about the vehicle, and the vehicle point stays continuous. Pinned by `Px4Bridge.LargeHeadingResetAfterResyncKeepsTheVehiclePointContinuous` (Task 8).
4. **A burst of old clouds when the LiDAR returns.** After `SIGCONT`, the bridge may deliver queued clouds. Each cloud not newer than the pending or newest scan is dropped with an event: no rewind and no false confirmation. Pinned by `LioIngest.BurstOfOldScansAfterAnOutageIsDropped` (Task 7).
5. **A restart refused `kBackendBusy` or `STALE_ANSWER`.** The bridge keeps offering a seed every tick while RESTARTING, so the restart happens on a later seed without any retry logic in lio. Pinned by `LioNode.RestartHappensOnALaterSeedAfterARefusal` (Task 7).

---

## File Structure

```text
src/estimation/uavnav_lio/        (modify) T1 limits + start check; T7 seed subscription, compose_seed, events
src/estimation/uavnav_lio_core/   (modify) T5 retry edge, reinit_timeout_s, LioReason::kReinitTimeout; config/lio/sim.yaml
src/px4/uavnav_px4_bridge/        (modify) T4 restart_seed.hpp/.cpp, epoch gate; T6 seed offers, O31 gate, node publisher; T8 tests
src/core/uavnav_interfaces/msg/LioRestartSeed.msg (new) T6;  src/core/uavnav_ros/qos.hpp (modify) T6
config/px4/profiles/{gps,no_gps,no_gps_ev_yaw}.yaml, tools/uavnav/px4_profile.py (modify/new) T2
tools/uavnav/{sitl_hover.sh,sitl_hover.py,sitl_probe.py,sitl_checks.py} (modify) T2, T9, T11
tools/uavnav/drift.py (new) T11; src/uavnav_bringup/launch/s1_estimation.launch.py (modify) T9
src/sim/uav_simulation/worlds/long_open_featured_speed.sdf (modify) T11; tools/simulation/run_px4_mid360.sh (modify) T11
docs/analysis/2026-10-s1c-{sitl-runs,heading-source,recovery,drift-benchmark}.md (new) T3, T10, T12
```

---

### Task 1: lio future-stamp tolerance below `gap_degraded`, and a clock-mode guard (O28, carry-over 12–13)

Spec §3.5, D33/O20, O28. **Gate:** the tolerance value waits for O28. The `kClockMode` part does not wait: if O28 is open, do Steps 1–4 for `kClockMode` only.

**Files:**
- Modify: `src/estimation/uavnav_lio/include/uavnav/lio_node/limits.hpp` (`kMaxStampAhead` 500 → 200 ms with derivation; new `enum class ClockMode { kSimulationIdentity }` and `kClockMode`, as in the bridge's `limits.hpp:76`);
- Modify: `src/estimation/uavnav_lio/src/lio_node.cpp` (start check);
- Test: `test/test_ingest.cpp`, `test/test_lio_node.cpp`.

**Interfaces:**
- Produces: `limits::kMaxStampAhead == 200 ms`. A start check refuses a loaded `lifecycle_gap_degraded_s ≤ kMaxStampAhead` with `LioNodeStartFailed{StartError::kStampToleranceNotBelowGapDegraded}`. A runtime check is needed because `gap_degraded` is tier (b), and its lower bound of 0.05 s is below 0.2 s.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(LioIngest, ImuStampAheadByMoreThanTheToleranceIsRefused) { /* now.ros = 10 s; IMU at 10.25 s -> kStampInFuture, event, core untouched */ }
  TEST(LioIngest, OneImuStampFourHundredMsAheadNeverReachesLost) {
    // TRACKING at 200 Hz; one IMU at +0.40 s -> refused; the stream continues; lifecycle stays TRACKING
  }
  TEST(LioNode, GapDegradedNotAboveTheStampToleranceRefusesStart) { /* yaml lifecycle_gap_degraded_s 0.2 -> StartError */ }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio`. Expected: the first two fail (0.25 s and 0.40 s are accepted today); the third fails to compile (`kStampToleranceNotBelowGapDegraded` is missing).
- [ ] **Step 3: Implement.** Also update the spec §3.5 text "đi trước ROS > 0.5 s" → 0.2 s, and record D40 (O28 decided) in DECISIONS.md.
- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio && make sanitize`. Expected: all pass, and there is no TSan warning.
- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio docs/architecture/SYSTEM_DESIGN.md docs/architecture/DECISIONS.md docs/TRACEABILITY.md
  git commit -m "fix(lio): refuse sensor stamps more than 0.2 s ahead, below gap_degraded" \
    -m "SYSTEM_DESIGN §3.5, O20, O28 -> D40: one glitched IMU stamp can no longer drive LIO to LOST; start refused if gap_degraded <= the tolerance; kClockMode guards the SensorTime/RosTime comparison (simulation identity, D29)." \
    -m "Arch-Level: 1
  Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/architecture/DECISIONS.md, docs/TRACEABILITY.md

  Co-Authored-By: <model> <noreply@anthropic.com>"
  ```

---

### Task 2: Heading source as a profile axis; F34 rule by heading source (D37, O32)

Spec §0, §4.1, D35, D37. **Gate:** `no_gps_ev_yaw.yaml` waits for O32. Under O32 (b) it carries only `EKF2_EV_CTRL=15`. The rest of the task does not wait.

**Files:**
- Modify: `config/px4/profiles/gps.yaml`, `config/px4/profiles/no_gps.yaml`; create `config/px4/profiles/no_gps_ev_yaw.yaml`;
- Modify: `tools/uavnav/px4_profile.py`, `tools/uavnav/tests/test_px4_profile.py`;
- Modify: `tools/uavnav/sitl_hover.sh` (accept `PROFILE=no_gps_ev_yaw`), `tools/uavnav/sitl_hover.py` (record `cs_mag_hdg`, `cs_mag_3d`, `cs_yaw_align`, `cs_ev_yaw`; write `heading_source` into `hover_meta.json`);
- Modify: `tools/uavnav/sitl_checks.py`, `tools/uavnav/tests/test_sitl_checks.py`.

**Interfaces:**
- `Profile` gains `heading_source: str`. Every file has the top-level key `heading_source` (`compass` | `ev_yaw`).
- The allowed (localization, heading_source) pairs are `(gps, compass)`, `(no_gps, compass)` and `(no_gps, ev_yaw)`. Any other pair raises `ProfileError`.
- `_DECIDED` is keyed by the pair:
  - `EKF2_EV_CTRL` is 7 for compass and 15 for ev_yaw;
  - `(no_gps, ev_yaw)` adds `EKF2_MAG_TYPE: 5` and `SYS_HAS_MAG: 0` (O32 (a));
  - every O23 value is unchanged.
- `sitl_checks.expected_yaw_t(heading_source: str, spawn_yaw_rad: float) -> float` returns `wrap(π/2 − spawn)` for compass and `0.0` for ev_yaw. `_check_f34_values` and `_check_ekf2_ev_yaw` take `heading_source`, read from `hover_meta.json`. A missing key fails closed, unless the CLI option `--heading-source compass|ev_yaw` is given; it exists only to re-judge runs that predate the key, and `summary.json` records that it was used.
- New check `ekf2_heading_source`:
  - compass: (`cs_mag_hdg` ∨ `cs_mag_3d`) in ≥ 95% of the window, and `cs_ev_yaw` never;
  - ev_yaw: `cs_ev_yaw` in ≥ 95%, and mag never.
- The `events` check fails on `LioNodeFailed`, `BridgeNodeFailed`, `LioPublishFailed`, `BridgePublishFailed`, `LioIngestReentered` and a non-zero `LioNodeSummary.publish_failed`. It reports the `LioInputRejected` count (carry-over 15; S1b final review minor).

- [ ] **Step 1: Write the failing tests:**
  - `Px4ProfileTest`: `test_heading_source_pairs` (gps+ev_yaw raises; the three files load); `test_ev_ctrl_follows_heading_source`; `test_ev_yaw_profile_disables_the_compass`;
  - `SitlChecksTest`: `test_f34_compass_expects_north_reference` (no_gps compass, spawn 1.2, `yaw_T` 21.0° → PASS; 0.0° → FAIL); `test_f34_ev_yaw_expects_lio_start_frame`; `test_heading_source_missing_fails_closed`; `test_mag_dropped_in_compass_profile_fails`; `test_events_fail_on_publish_failures`.

  Run: `tools/uavnav/gate.sh python`. Expected: the new tests fail, and `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Also fold in the S1b review minors from `progress.md`: T11 M2 (inter-arrival in /clock time) and M3 (`ekf2_ev_pos` weighted by sample). Then run: `tools/uavnav/gate.sh python && make sitl-hover PROFILE=no_gps_ev_yaw DRY_RUN=1`. Expected: `GATE_RESULT=PASS`; the dry run prints the profile's `PX4_PARAM_*` lines.
- [ ] **Step 3: Re-judge the four S1b F34 runs** offline with the new checker: `python3 tools/uavnav/sitl_checks.py --hover log/sitl/<run> --profile <p> --spawn-yaw-rad <y>`. Expected: gps runs PASS; S1b no_gps runs judged with `heading_source=ev_yaw` (they flew `EKF2_EV_CTRL=15`) PASS. Their driver did not write `heading_source`, so pass `--heading-source`.
- [ ] **Step 4: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md`). The spec §0 table already names the axis; mark the TRACEABILITY D37 row `COMPONENT_VERIFIED` with the test names.

---

### Task 3: SITL check of the heading source, and the run register (D37)

Spec §0, §7.2 (S1c), D37. This task needs about 25 min of the owner's machine time.

**Files:** create docs/analysis/2026-10-s1c-heading-source.md (Vietnamese) and docs/analysis/2026-10-s1c-sitl-runs.md (the run register, in the format of the S1b register).

- [ ] **Step 1: Four hovers.** Fly one at a time:

  ```bash
  make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2        # regression
  make sitl-hover PROFILE=no_gps                       # compass, spawn 0
  make sitl-hover PROFILE=no_gps SPAWN_YAW_RAD=1.2     # compass, the decisive run
  make sitl-hover PROFILE=no_gps_ev_yaw SPAWN_YAW_RAD=1.2
  ```

  Expected: four `"result": "PASS"`. For the no_gps compass runs, `yaw_T` ≈ π/2 − spawn within 3°: the predicted offset is the simulated declination, +0.75°, because EKF2 fuses `EKF2_MAG_DECL` = 0 without GNSS (§4.1).
- [ ] **Step 2: Decision rule.** D37 holds in SITL when:
  - the no_gps compass runs PASS `ekf2_heading_source` and F34;
  - the ulog shows no heading reset after the alignment is VALID.

  If EKF2 drops the compass in flight (`cs_mag_hdg` and `cs_mag_3d` both false), stop. Report the ulog timeline (`estimator_status_flags`, `estimator_states` 16–17) and open an O-entry. Never change the profile around it.
- [ ] **Step 3: Report and register.** The report covers:
  - the per-run table and the `c_px4` / `c_lio` decomposition (method of the S1b F34 report §3);
  - the ulog flags;
  - the declination term;
  - whether the ev_yaw run still shows the ≈ 20° reset at EV start.

  Run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 4: Commit** the two docs and the TRACEABILITY D37 row (`INTEGRATED` with the `run:` ids). Trailer `Arch-Level: 1`, `Docs-Impact: docs/TRACEABILITY.md, docs/analysis/...`.

---

### Task 4: Seed helper `T⁻¹ × PX4 pose`, and the stale-epoch odometry gate (§3.4, O25)

Spec §3.4, §4.1, D20, D33/O25. There is no gate: the seed formula is decided (§3.4), and the epoch gate is O25 (a).

**Files:**
- Create: `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/restart_seed.hpp`, `src/restart_seed.cpp`, `test/test_restart_seed.cpp`;
- Modify: `include/uavnav/px4bridge/bridge.hpp`, `src/bridge.cpp` (epoch gate; `BridgeReason::kStaleEpoch` "STALE_EPOCH"), `test/test_bridge.cpp`, `CMakeLists.txt`;
- Modify: `src/estimation/uavnav_lio_core/include/uavnav/lio/limits.hpp` (reference the shared `kEventSummaryEvery` from one place: carry-over 10).

**Interfaces:**

```cpp
namespace uavnav::px4bridge {
struct RestartSeed {
  time::SensorTime t;            ///< SensorTime of the PX4 sample used
  std::uint32_t lio_epoch;       ///< the RESTARTING epoch this seed answers
  Eigen::Vector3d p_flu_m;       ///< base_link in lio_odom FLU = T^-1(p_px4_ned), then FRD -> FLU
  double yaw_flu_rad;            ///< wrap(-(yaw_px4 - yaw_T))
  Pose4 t_used;                  ///< the FROZEN T (for the LioRestart event)
};
enum class SeedReason : std::uint8_t { kLioNotRestarting, kAlignmentNotFrozen, kPx4PositionInvalid, kPx4SampleTooOld, kNotFinite };
/// Pure. Ok only when health.state == RESTARTING, state == FROZEN, xy_valid && z_valid,
/// now - px4.sample.t <= limits::kSeedMaxAge (200 ms: two bridge ticks), and every value finite.
Result<RestartSeed, SeedReason> make_restart_seed(const LioHealthIn& health, AlignmentState state, const Pose4& t,
                                                   const Px4LocalPositionIn& px4, time::SensorTime now) noexcept;
}
```

The epoch gate: odometry whose epoch is lower than the newest accepted health epoch is refused (`kStaleEpoch`, rate-limited event). No EV is sent and no pair is formed.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(RestartSeed, InvertsTForAPoseFarFromTheOrigin) { /* T = (120, -40, 3, 0.7 rad); p_lio_frd = (250, 30, -2.5), yaw 0.2:
     forward T then make_restart_seed returns p_flu == flu(p_lio_frd) within 1e-9 m, yaw within 1e-12 */ }
  TEST(RestartSeed, RefusesUnlessRestartingFrozenValidAndFresh) { /* one case per SeedReason, at the 200 ms boundary: equal ok, +1 ns refused */ }
  TEST(RestartSeed, NonFiniteInputIsRefused) {}
  TEST(Px4Bridge, OldEpochOdometryAfterNewEpochHealthIsRefused) { /* health epoch 2 INITIALIZING, then odometry epoch 1 TRACKING
     -> no EV, no pair, one Px4Bridge event STALE_EPOCH */ }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_px4_bridge`. Expected: compile error, `restart_seed.hpp` is missing.
- [ ] **Step 3: Implement.** Use `frames.hpp` for FRD↔FLU; there is no new frame helper.
- [ ] **Step 4: Verify.** Run: `make build PKGS="uavnav_px4_bridge uavnav_lio_core" && make test PKGS="uavnav_px4_bridge uavnav_lio_core"`. Expected: all pass.
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/TRACEABILITY.md`; the spec already states the formula).

---

### Task 5: Retry edge INITIALIZING → LOST after a restart (O30)

Spec §3.1 (the O30 note), §3.4 row 2d, LIO-1, LIO-4. **Gate: O30.**

**Files:** in `src/estimation/uavnav_lio_core/`:
- `include/uavnav/lio/lifecycle.hpp`: `LioReason::kReinitTimeout` ("REINIT_TIMEOUT"); edges INITIALIZING→LOST {`kReinitTimeout`, `kLidarGapLost`};
- `src/lifecycle.cpp`;
- `include/uavnav/lio/frontend_config.hpp`, `include/uavnav/lio/config.hpp`, `src/config.cpp`: tier (b) `lifecycle_reinit_timeout_s` in [1.0, 10.0], beta 3.0;
- `config/lio/sim.yaml`;
- `test/test_lifecycle.cpp`, `test/test_config.cpp`.

**Rule:**
- The timeout applies only to an INITIALIZING entered by `kRestartSeeded`. The lifecycle keeps `reinit_since_` (an optional SensorTime), set by that entry and cleared by `go()`. It is part of the INITIALIZING data, like `InitPhase`; it is not a state.
- First bootstrap is unchanged.
- No new state, and no LOST→TRACKING edge (LIO-4).

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(LioLifecycle, ReinitWithoutConfirmationGoesLostAfterTheTimeout) { /* RESTARTING -> kRestartSeeded at t0; only degenerate scans;
     IMU ticks: LOST REINIT_TIMEOUT at the first tick t > t0 + 3.0 s, not before */ }
  TEST(LioLifecycle, ReinitLidarGapGoesLost) { /* after kRestartSeeded, IMU only: LOST LIDAR_GAP_LOST after gap_lost */ }
  TEST(LioLifecycle, FirstBootstrapHasNoTimeout) { /* INITIALIZING from construction, 60 s of degenerate scans: stays INITIALIZING */ }
  TEST(LioLifecycleTable, ReinitEdgesAreListedAndLostNeverReachesTracking) {}
  TEST(LioConfig, ReinitTimeoutBoundsAreEnforced) { /* 0.5 and 11 refused; 3.0 loads */ }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio_core`. Expected: compile error, `kReinitTimeout` is missing.
- [ ] **Step 3: Implement.** Update the §3.1 diagram (it replaces the O30 note) and DECISIONS (O30 → D-entry).
- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core && make sanitize`. Expected: all pass, and there is no TSan warning.
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/architecture/DECISIONS.md, docs/TRACEABILITY.md`). The edge is new, the state set is unchanged, and `check_change` must not see a state enumerator. The report names LIO-1 and LIO-4 as touched-and-held.

---

### Task 6: `/lio/restart_seed` from the bridge; EV withheld while `T` is FROZEN (O29, O31)

Spec §3.4 rows 2b and 3b★, §4.1, §6.5. **Gate: O29** (the whole task) and **O31** (the gate part).

**Files:**
- Create: `src/core/uavnav_interfaces/msg/LioRestartSeed.msg`; modify the package's `CMakeLists.txt`.
- Modify: `src/core/uavnav_ros/include/uavnav/ros/qos.hpp`: the seed QoS is reliable, depth 1. Also fix the stale `/livox/*` comment.
- Modify in `src/px4/uavnav_px4_bridge/`:
  - `bridge.hpp/.cpp`: `BridgeOutputs::seed`; seed offers on `on_tick` and `on_px4_local_position`; the O31 gate with the new `BridgeReason::kAlignmentFrozen` ("ALIGNMENT_FROZEN");
  - `ros_conversion.hpp/.cpp`; `px4_bridge_node.cpp` (publisher);
  - tests.
- Update the spec §6.5 (message list, QoS) and §3.4 (drop ★ from rows 2b and 3b); DECISIONS O29/O31 → D-entries.

**Interfaces:**

```text
# LioRestartSeed.msg: px4_bridge -> lio on /lio/restart_seed (SYSTEM_DESIGN §3.4)
builtin_interfaces/Time stamp     # SensorTime of the PX4 sample
uint32 lio_epoch                  # the RESTARTING epoch it answers
geometry_msgs/Point position      # base_link in lio_odom, FLU, metres
float64 yaw_rad                   # base_link yaw in lio_odom, FLU
float64 t_x_m
float64 t_y_m
float64 t_z_m
float64 t_yaw_rad                 # the FROZEN T used (lio_odom FRD -> PX4 NED)
```

- `BridgeOutputs` gains `std::optional<RestartSeed> seed`. It is set at most once per `on_tick` (100 ms) while `make_restart_seed` is Ok.
  - Events: `RestartSeedOffered` (values: `lio_epoch`, `age_s`, the seed, `T`); `RestartSeedRefused` (`SeedReason`), rate-limited per reason, and only while health is RESTARTING.
- O31: while the alignment is FROZEN, an odometry sample whose epoch is greater than the epoch at which `T` froze is not encoded. The event is `EvGated` with reason `ALIGNMENT_FROZEN`. It is still fed to the alignment, so the first pair can take FROZEN→VALID. INIT and VALID are unchanged.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Px4Bridge, RestartingHealthWithFrozenTOffersOneSeedPerTick) {}
  TEST(Px4Bridge, NoSeedWhenTIsInvalidAndTheRefusalIsReportedOncePerEpisode) {}
  TEST(Px4Bridge, NewEpochEvIsWithheldUntilTheFirstNewEpochPairResyncsT) { /* FROZEN; epoch 2 odometry -> EvGated ALIGNMENT_FROZEN, pair
     accepted -> VALID; the next epoch 2 odometry is encoded with reset_counter 1 */ }
  TEST(Px4Bridge, StartupEvStillFlowsInInit) {}
  TEST(Px4BridgeRos, RestartSeedMessageRoundTrips) {}
  TEST(Px4BridgeNode, PublishesRestartSeedWhileLioIsRestarting) {}
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS="uavnav_interfaces uavnav_ros uavnav_px4_bridge"`. Expected: compile errors for the missing message and fields.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** Run: `make build PKGS="uavnav_interfaces uavnav_ros uavnav_px4_bridge" && make test PKGS="uavnav_interfaces uavnav_ros uavnav_px4_bridge"`. Expected: all pass.
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/architecture/DECISIONS.md, docs/TRACEABILITY.md`). The report names PX4-1 (unchanged edges) and PX4-2 (stricter gate) as touched-and-held.

---

### Task 7: The lio node restarts on a seed (O29)

Spec §3.4 row 2c, §3.5, D33/O16. **Gate: O29.**

**Files:** in `src/estimation/uavnav_lio/`:
- `include/uavnav/lio_node/ingest.hpp`, `src/ingest.cpp`: `LioIngest::on_seed`; `IngestReason::{kSeedWrongEpoch, kSeedTooOld, kNotRestarting}`;
- `ros_conversion.hpp/.cpp`, `lio_node.hpp`, `src/lio_node.cpp`: the subscription goes in the same mutually exclusive group;
- tests.

Also the S1b T7 deferred items in `ingest.hpp` (wording at :54-58 and :63; revert the 1e-9 tolerance to 1e-12 in `test_ingest.cpp`) and T8 M4 (one diagnostic event for the first WRONG_FRAME / MALFORMED_CLOUD).

**Interfaces:**

```cpp
/// The converted /lio/restart_seed (ros_conversion; non-finite fields refused there, kNotFinite).
struct SeedIn {
  time::SensorTime t;
  std::uint32_t lio_epoch;
  Eigen::Vector3d p_flu_m;
  double yaw_flu_rad;
  std::array<double, 4> t_used;   ///< x, y, z, yaw of the FROZEN T, logged in LioRestart (lio does not depend on px4bridge)
};
/// lio_node/limits.hpp: kSeedMaxAge = 200 ms (its own derivation: two bridge ticks; the bridge's limit of the same
/// value bounds the PX4 sample age at the offer, this one the seed age at application).
/// Pure: yaw and position from the seed, roll and pitch from the predictor's newest attitude (gravity-aligned,
/// IMU-propagated through the outage). Result is a unit quaternion.
lio::SeedPose compose_seed(const Eigen::Vector3d& p_flu_m, double yaw_flu_rad, const Eigen::Quaterniond& q_predictor) noexcept;
/// Ingest thread. Ok -> LioRunner::restart(compose_seed(...)) was queued. Refused (event "LioSeedRefused", reason):
/// not RESTARTING, seed.lio_epoch != current epoch, newest IMU time - seed.t > kSeedMaxAge (200 ms), non-finite.
Result<IngestOutputs, IngestReason> LioIngest::on_seed(const SeedIn& seed, const time::TimeSnapshot& now);
```

The `LioRestart` event gains the values `t_x_m`, `t_y_m`, `t_z_m`, `t_yaw_rad` from the seed (§3.4: "seed, `T`").

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(LioIngest, ComposeSeedKeepsPredictorTiltAndSeedYaw) {}
  TEST(LioIngest, SeedForAnotherEpochOrTooOldIsRefused) {}              // Review Focus 1
  TEST(LioIngest, SeedWhileTrackingIsRefusedAndChangesNothing) {}
  TEST(LioIngest, BurstOfOldScansAfterAnOutageIsDropped) {}             // Review Focus 4
  TEST(LioNode, RestartHappensOnALaterSeedAfterARefusal) {               // Review Focus 5
    // synthetic room scene: LOST by gap, scans back -> RESTARTING; first seed while a carried answer -> kBackendBusy;
    // next seed -> LioRestart epoch 2 with T values; INITIALIZING -> TRACKING; /lio/odometry epoch 2 reset_counter 1
  }
  ```

- [ ] **Step 2: Run them.** Run: `make build PKGS=uavnav_lio`. Expected: compile error, `on_seed` is missing.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Verify.** Run: `make build PKGS=uavnav_lio && make test PKGS=uavnav_lio && make sanitize`. Expected: all pass, and there is no TSan warning.
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/TRACEABILITY.md`).

---

### Task 8: `T` across a restart; DV2 investigation

Spec §4.1, D30/O11, D35 (carry-over 5); DV2 (carry-over 4 and 17). DV2's trigger is "the first task touching the alignment", which is this one.

**Files:**
- Modify: `src/px4/uavnav_px4_bridge/test/test_bridge.cpp`, `test/test_alignment.cpp`.
- Modify `src/alignment.cpp` only if a test fails or DV2 finds a buffer cause.
- Create: docs/analysis/2026-10-s1c-dv2.md (short, Vietnamese).

- [ ] **Step 1: Write the tests** (they may pass at once; they pin behaviour the chain relies on)

  ```cpp
  TEST(Px4Bridge, RestartResyncsFrozenTWithTheFirstNewEpochPair) { /* 3 s FROZEN; restart at seed; epoch 2 pairs within 0.1 m -> VALID,
     T unchanged within 1e-6 */ }
  TEST(Px4Bridge, LargeHeadingResetAfterResyncKeepsTheVehiclePointContinuous) { /* VALID after resync; PX4 heading reset +20 deg with
     xy delta; vehicle point continuous within 1 mm; no PAIR_REJECTED_JUMP afterwards */ }   // Review Focus 3
  TEST(Px4Bridge, PairJumpRejectedAfterRestartStaysFrozenUntilTimeout) { /* seed off by 1 m -> FROZEN until frozen_max -> INVALID ->
     INIT on epoch-2 TRACKING -> VALID after 20 pairs */ }
  ```

- [ ] **Step 2: DV2.** From the existing run `log/sitl/20261013T155144Z-no_gps-hover` (113 `NO_PX4_SAMPLE` in the window), measure three things around each rejection:
  - the `vehicle_local_position` inter-arrival and `timestamp_sample` lag;
  - the pair's LIO stamp against the newest buffered PX4 sample;
  - the buffer span.

  Decision:
  - the cause is the PX4 sample rate or lag, and `T` stays VALID: close DV2 with the analysis as evidence;
  - the cause is the bridge buffer or the time matching: write a failing test, fix it in this task, re-judge the run offline;
  - neither: keep DV2 OPEN with the new data and the drift runs (Task 12) as the next trigger.
- [ ] **Step 3: Verify.** Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`. Expected: all pass.
- [ ] **Step 4: Commit** the tests, any fix, the DV2 report and the DV2 row.

---

### Task 9: Fault injection in the harness: LiDAR cut and climb loss

Spec §7.2 (S1c), §3.4. There is no product change.

**Files:**
- Modify: `src/uavnav_bringup/launch/s1_estimation.launch.py`:
  - new required argument `start_lidar_bridge` (`true` | `false`);
  - the docstring states the Fast DDS profile dependency (S1b I3, carry-over 14).
- Modify: `tools/uavnav/sitl_hover.sh`:
  - `PATTERN=lidar_cut [CUT_S=<s>]` and `PATTERN=climb_loss`;
  - for both, the harness starts the LiDAR `ros_gz_bridge` itself, as its own process group, and exports `UAVNAV_LIDAR_BRIDGE_PGID` to the driver;
  - fold in the S1b T3 deferred harness minors (`PROFILE ?=` pickup, `commands.txt` quoting).
- Modify: `tools/uavnav/sitl_hover.py`:
  - `lidar_cut`: hover 20 s, then `os.killpg(pgid, SIGSTOP)` for `CUT_S` of /clock, then `SIGCONT`, then hover 40 s; `SIGCONT` is guaranteed in `finally`;
  - `climb_loss` (gps only): climb to 7.0 m, hold 5 s, descend to 2.5 m, hold 40 s;
  - the cut window and the steps go into `hover_meta.json`.
- Modify: `tools/uavnav/sitl_checks.py` (`evaluate_recovery`), `tools/uavnav/tests/test_sitl_checks.py`.
- Record `/lio/restart_seed`.

**Interfaces:** `sitl_checks.evaluate_recovery(run_dir, profile, spawn_yaw_rad) -> dict`, with the same shape as `evaluate_hover`. The criteria are measured from the cut end, or from the end of the climb-loss descent:

| Check | Pass when |
|---|---|
| `lost_detected` | LOST within cut start + `gap_lost` + 0.2 s (F22 in SITL) |
| `restart` | RESTARTING within 1.0 s of the LiDAR return; `LioRestart` within 0.5 s of RESTARTING; the epoch increments by one per restart |
| `retracked` | TRACKING of the new epoch within `reinit_timeout_s` (3.0 s) of `LioRestart` |
| `resync` | short cut (`CUT_S` < 5): `T` FROZEN→VALID within 2.0 s of TRACKING, never INVALID. Long cut (≥ 10): FROZEN→INVALID at `frozen_max` ± 0.2 s, no seed offered after INVALID, `RestartSeedRefused ALIGNMENT_NOT_FROZEN` reported |
| `ev_epochs` | no EV with the old epoch after the first new-epoch health; EV resumes after `T` is VALID (O31) |
| `px4_continuity` | gps: PX4 horizontal error against ground truth ≤ 0.5 m across the window; no_gps: reported |
| `events` | as for the hover, plus a count of each reason of `RestartSeedRefused`, `LioSeedRefused` and `LioRestartRejected` |

- [ ] **Step 1: Write the failing tests** (`SitlChecksTest`): `test_recovery_pass_on_synthetic_records`; `test_recovery_fails_without_restart`; `test_recovery_fails_on_old_epoch_ev`; `test_long_cut_expects_invalid_and_no_seed`; `test_recovery_missing_cut_window_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `AttributeError` for `evaluate_recovery`, and `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Then run: `tools/uavnav/gate.sh python && make sitl-build && make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2 DRY_RUN=1`. Expected: `GATE_RESULT=PASS`; the dry run lists the separate LiDAR bridge group.
- [ ] **Step 3: Commit** (harness and tests; `Arch-Level: 1` for the launch argument; `Docs-Impact: docs/TRACEABILITY.md`).

---

### Task 10: Recovery SITL runs and report (D38)

Spec §3.4, §7.2 (S1c). This task needs about 35 min of the owner's machine time.

- [ ] **Step 1: Five runs**, one at a time:

  ```bash
  make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=2
  make sitl-hover PROFILE=gps PATTERN=lidar_cut CUT_S=12        # chain exhausted: T INVALID, logged only (no Supervisor yet)
  make sitl-hover PROFILE=gps PATTERN=climb_loss                # D36 geometry loss, then recovery on the descent
  make sitl-hover PROFILE=no_gps PATTERN=lidar_cut CUT_S=1.5    # below EKF2_NOAID_TOUT (5 s)
  make sitl-hover PROFILE=no_gps_ev_yaw PATTERN=lidar_cut CUT_S=1.5
  ```

  Expected: all PASS. On failure, read the timelines first (`python3 -m tools.uavnav.events log/sitl/<run>/events_*.jsonl --timeline`). Then at most one fix-and-re-run per root cause. A design gap becomes an O-entry.
- [ ] **Step 2: Watch the drifted prior.** In `climb_loss`, LOST → RESTARTING needs one good scan against the frozen map with an IMU-only prior (§3.4 row 2a). If geometry is back (floor visible) but RESTARTING does not come within 3 s, stop and open an O-entry. The proposed option: judge "geometry returned" on the scan's own information, without the frozen map. Do not change the rule in this task.
- [ ] **Step 3: Report** docs/analysis/2026-10-s1c-recovery.md: the chain timeline per run (LOST, RESTARTING, seed, `LioRestart`, TRACKING, VALID, EV), the PX4 continuity, and what stays for S2 (the handover).
  - Register the runs.
  - Raise the TRACEABILITY rows: P5 and the D38 row to `INTEGRATED`; F22 to `INTEGRATED` with the cut runs (S1b I4).
  - Run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 4: Commit** the report, the register and TRACEABILITY.

---

### Task 11: Drift harness and metrics (D39)

Spec §7.2 (S1c), D39. There is no product change.

**Feasibility (checked in PX4 v1.17.0-14 source):**
- POSCTL with `MPC_POS_MODE=0` ("Direct velocity", `src/modules/mc_pos_control/multicopter_position_mode_params.c`) maps the sticks to a velocity setpoint: `v = expo_deadzone(stick, 0.6, MAN_DEADZONE) × MPC_VEL_MANUAL`, rotated into the heading frame (`FlightTaskManualPosition.cpp` lines 70–97, `lib/sticks/Sticks.hpp` lines 66–73, `lib/mathlib/math/Functions.hpp` `expo_deadzone`).
- The defaults are `MPC_VEL_MANUAL` 10 m/s and `MAN_DEADZONE` 0.1. The default `MPC_POS_MODE=4` maps sticks to acceleration, so a constant stick gives no exact speed.
- **What is missing:** `sitl_probe._stick_tick` sends `roll = pitch = yaw = 0` (`tools/uavnav/sitl_probe.py:203`), so pitch must become a field of the stick state.
- `long_open_featured_speed.sdf` has a 140 m leg with features at ±8.5–12 m and a free ±8 m band. It lacks the `<magnetic_field>` and `<atmosphere>` of `px4_lio_smoke`. Without them, compass and baro would differ between worlds.

**Files:**
- Modify: `src/sim/uav_simulation/worlds/long_open_featured_speed.sdf` (add the two elements, copied from `px4_lio_smoke.sdf`).
- Modify: `tools/simulation/run_px4_mid360.sh`: `UAVNAV_HARNESS_POS_MODE` sets `MPC_POS_MODE=0` and `MAN_DEADZONE=0.1`, for `PATTERN=drift` only. These are harness parameters, not profile values.
- Modify: `tools/uavnav/sitl_hover.sh`:
  - `PATTERN=drift SPEED_MPS=<1..5> [WORLD=long_open_featured_speed]`;
  - bound 600 s;
  - `WORLD` passed as `PX4_GZ_WORLD`.
- Modify: `tools/uavnav/sitl_probe.py`:
  - `set_sticks(throttle, pitch)`;
  - stick stream gated, off for the first 3 s of /clock, so the record shows whether PX4 starts in POSCTL by itself (carry-over 9).
- Modify: `tools/uavnav/sitl_hover.py` (pattern `drift`).
- Create: `tools/uavnav/drift.py`, `tools/uavnav/tests/test_drift.py`.
- Modify: `tools/uavnav/sitl_checks.py` (`evaluate_drift` = data checks plus `drift.py` metrics).

**Flight (`drift`):**
1. wait_ready, arm, climb to 2.5 m, then 5 s stationary: the alignment window.
2. In-flight mode check (carry-over 9): `DO_SET_MODE` Hold (1/4/3), expect `nav_state` 4 within 1 s, hold 3 s; then `DO_SET_MODE` POSCTL (1/3/0), expect `nav_state` 2 within 1 s. The check is `in_flight_mode_switch`.
3. Legs along world +x (spawn yaw 0): pitch stick `stick_for_speed(SPEED_MPS)`, forward from x ≈ 5 m to 105 m, stop, backward to 5 m. Repeat until both the travelled distance is ≥ 200 m and the motion time is ≥ 120 s of /clock.
4. Hover 5 s, land.

**Interfaces** (`drift.py`, pure, numpy):

```python
def stick_for_speed(v_mps: float, vel_manual_mps: float = 10.0, expo: float = 0.6, deadzone: float = 0.1) -> float
    # inverse of PX4 expo_deadzone: solve (1-e)y + e y^3 = v/vel_manual on [0,1], stick = dz + y(1-dz); ValueError if v > vel_manual
def align_4dof(est: np.ndarray, gt: np.ndarray) -> tuple[float, np.ndarray]   # yaw, translation; least squares over the stationary window
def drift_curve(est_aligned: np.ndarray, gt: np.ndarray) -> np.ndarray          # rows: s_m, t_s, err_h_m, err_3d_m, err_yaw_rad
def relative_drift(est: np.ndarray, gt: np.ndarray, lengths_m=(10, 25, 50, 100)) -> dict   # per length: mean, p95 (m per m), n
def per_minute_drift(est: np.ndarray, gt: np.ndarray, window_s: float = 60.0) -> dict      # mean, p95 (m per min), n
def evaluate_drift(run_dir: pathlib.Path) -> dict   # LIO = /lio/odometry (scan stamps), PX4 = vehicle_local_position (NED->ENU,
                                                    # timestamp_sample), GT = /sim/ground_truth/odometry; time-matched by linear
                                                    # interpolation of GT at each estimate stamp; writes drift.json
```

Also reported per run: the achieved cruise speed (ground truth, mean and sd over the cruise part of the legs); the DEGRADED→TRACKING count per 60 s and the applied-result age p50 and p99 (DV1); the `/lio/health` output tracking error per speed (the τ data, O13); and the `NO_PX4_SAMPLE` fraction (DV2).

The data checks pass when:
- the speed is within ±10% of `SPEED_MPS`;
- `in_flight_mode_switch` passes;
- every topic has data;
- LIO is TRACKING ≥ 95% of the motion window. Less is reported with the recovery events, not failed: losses are data here.

- [ ] **Step 1: Write the failing tests** (`DriftTest`):
  - `test_stick_for_speed_inverts_px4_expo_deadzone` (forward model at 1..5 m/s within 1e-6);
  - `test_align_recovers_a_known_4dof_offset`;
  - `test_constant_velocity_bias_gives_linear_drift` (a 1% scale error gives 0.01 m/m at every length and m/min = 0.6·v);
  - `test_relative_drift_needs_enough_segments` (ValueError);
  - `test_drift_missing_topic_fails_closed`.

  Run: `tools/uavnav/gate.sh python`. Expected: `ModuleNotFoundError: drift`, and `GATE_RESULT=FAIL`.
- [ ] **Step 2: Implement.** Then run: `tools/uavnav/gate.sh python && make sitl-hover PROFILE=gps PATTERN=drift SPEED_MPS=3 DRY_RUN=1`. Expected: `GATE_RESULT=PASS`; the dry run shows the world, the bound of 600 s and `MPC_POS_MODE=0`.
- [ ] **Step 3: Commit** (`Arch-Level: 0`; the world file is a sim asset).

---

### Task 12: Drift runs and the benchmark report (D39; DV1, O13 data)

Spec §7.2 (S1c), D39. This task needs about 110 min of the owner's machine time: 12 runs of 8–10 min.

- [ ] **Step 1: Runs.** One at a time:
  - `make sitl-hover PROFILE=gps PATTERN=drift SPEED_MPS=<v>` and `PROFILE=no_gps PATTERN=drift SPEED_MPS=<v>`, for v = 1, 2, 3, 4, 5;
  - then one repeat each of gps at 1 and at 5 m/s (repeatability).
- [ ] **Step 2: Report** docs/analysis/2026-10-s1c-drift-benchmark.md (Vietnamese). It covers:
  - per profile and speed: LIO and EKF2 drift in m/m and m/min (mean, p95), the growth curve against distance, and the yaw drift in degrees per minute;
  - the benchmark table below, with every source URL kept;
  - the sim-to-real gap analysis;
  - DV1: close it if, in every run, there is at most 1 DEGRADED→TRACKING per 60 s and the age p99 is below 0.145 s; otherwise open an AR for `lio`;
  - τ (O13): the output tracking error against speed. Any change to τ is an owner item.
  - Before writing, try to verify the UNVERIFIED rows from the ICM-40609-D datasheet PDF. Keep the mark if that fails.
- [ ] **Step 3: Register the runs, update TRACEABILITY** (the D39 row and the τ row with `run:` ids; DV1), then run: `python3 tools/check_documentation.py . docs && python3 -m tools.uavnav.arch_check`. Expected: `PASS`, 0 errors.
- [ ] **Step 4: Commit.**

**Benchmark reference** (gathered 2026-10-14 for this plan; the report re-checks it):

| Quantity | Real (source) | SITL model (source in repo) | Gap to state in the report |
|---|---|---|---|
| Mid-360 range precision (1σ) | ≤ 2 cm at 10 m; ≤ 3 cm at 0.2 m ([Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs)) | Gaussian σ 0.02 m at every range (`src/sim/uav_simulation/models/lidar_mid360/model.sdf`) | matches at 10 m; no reflectivity or range dependence |
| Angular precision (1σ) | < 0.15° (same) | none | at 20 m, 0.15° ≈ 5 cm lateral noise missing in sim |
| FOV | 360° × (−7°..52°) (same) | 360° × (−7°..52°) (F36) | none |
| Point rate, frame rate | 200,000 pts/s first return, 10 Hz typical (same) | 720 × 28 rays × 10 Hz = 201,600 pts/s | same rate; sim is 28 fixed rings (≈ 2.2° vertical), real is non-repetitive |
| Detection range | 40 m at 10 % reflectivity, 70 m at 80 % (same) | max 40 m for all surfaces | sim conservative on bright surfaces |
| Per-point timing | real scan is spread over 100 ms (needs deskew) | one instant per scan (O20) | **no motion distortion in sim**: at 5 m/s a real scan smears up to 0.5 m before deskew; the largest expected gap |
| IMU (Mid-360 built-in) | ICM40609 (same) | 200 Hz | |
| Gyro noise density | 4.5 mdps/√Hz ([TDK ICM-40609-D](https://invensense.tdk.com/en-us/products/6-axis/icm-40609-d)) | σ 0.001 rad/s per sample at 200 Hz ≈ 4.1 mdps/√Hz, assuming σ = N·√f_s | close; the conversion convention must be stated |
| Accel noise density | 100 µg/√Hz (same) | σ 0.01 m/s² at 200 Hz ≈ 72 µg/√Hz | sim ≈ 30 % quieter |
| Gyro and accel bias | offset temperature coefficient ±10 mdps/°C, ±0.15 mg/°C (same); bias instability **UNVERIFIED** (datasheet DS-000330 not read) | no bias, no random walk | **no bias in sim**: IMU-only intervals (outages, LOST) drift far less than on hardware |
| PX4 IMU, mag, baro (x500) | | gyro σ 8.7e-4 rad/s, accel σ 0.0064–0.0069 m/s² at 250 Hz, mag σ 1e-4 G, baro σ 3 Pa (`~/Dev/Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`) | |
| Simulated GNSS | | Gauss-Markov, amplitude 0.8 m, random walk 0.01, coefficient 0.95 (PX4 v1.17.0-14 `src/modules/simulation/gz_bridge/GZBridge.hpp`, constants `_pos_noise_amplitude`, `_pos_random_walk`, `_pos_markov_time`) | the gps EKF2 "drift" is GNSS noise, not dead-reckoning drift |
| EKF2 assumed noise | defaults `EKF2_GYR_NOISE` 0.015 rad/s, `EKF2_ACC_NOISE` 0.35 m/s², `EKF2_GPS_P_NOISE` 0.5 m, `EKF2_EVP_NOISE` 0.1 m, `EKF2_MAG_NOISE` 0.05 G, `EKF2_HEAD_NOISE` 0.3 rad (`src/modules/ekf2/module.yaml` and `params_*.yaml`, PX4 v1.17.0-14) | same parameters | PX4 publishes no EKF2 position-accuracy figure (none found: **UNVERIFIED**) |
| Magnetic declination | WMM at the world's coordinates: −0.93° (EKF2 state, S1b F34 report §5) | +0.75° | 1.7° heading offset in sim (gps); no_gps compass offset = the site declination |
| Real LIO drift, for scale | FAST-LIO2: 0.14 m end-to-end over ≈ 650 m outdoor–indoor; < 0.06 m over 81 m of fast handheld motion with a Livox Avia ([Xu et al., FAST-LIO2, arXiv 2107.06829](https://arxiv.org/abs/2107.06829), real-world experiments) | | end-to-end (loop) errors understate drift in mid-trajectory; a different sensor (Avia, not Mid-360) |

---

### Task 13: Close S1c: traceability, spec sync, governance, carry-over to S2

**Files:**
- Modify: `docs/TRACEABILITY.md`, `docs/architecture/SYSTEM_DESIGN.md`, `tools/uavnav/arch_rules.yaml`;
- Modify: this plan (append "Carry-over to S2").

- [ ] **Step 1: Run the gate, sanitize and a final hover.** Run: `tools/uavnav/gate.sh all && make sanitize && make sitl-hover PROFILE=gps`. Expected: `GATE_RESULT=PASS`, no TSan warning, hover `PASS`.
- [ ] **Step 2: Spec sync** (S1b final review minors):
  - §3.1: the age cause never fires in TRACKING (the gap rule fires first);
  - §3.5: a refused correction enters the lifecycle as `kScanCorrectionRejected`, not `kScanDegenerate`;
  - §3.1 diagram: TRACKING → LOST on σ.
- [ ] **Step 3: Governance** (carry-over 11):
  - classify the 7 legacy packages in `arch_rules.yaml`;
  - run `python3 -m tools.uavnav.arch_health` and act on any trigger (an AR row with `Owner xác nhận` = `chưa`);
  - recalibrate the `stress:` thresholds on the S1b + S1c history, with the reason given in the commit.
- [ ] **Step 4: Append "Carry-over to S2".** It must include:
  - the Supervisor half of the chain: REACQUIRING → LOST with the episode `since` (O30), the guards and the handover reasons;
  - `px4_mode`;
  - any O-entry opened in Tasks 3, 10 or 12.
- [ ] **Step 5: Commit** (`Arch-Level: 1`, `Docs-Impact: docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md, docs/CHANGE_CONTROL.md` if thresholds move).

---

## Carry-over coverage (S1b "Carry-over to S2", items 1–17)

| # | Item | Here |
|---|---|---|
| 1 | τ re-evaluation, F23 step with motion | Task 12 gives the data and the report; a τ change is an owner item, then S2 |
| 2 | Degeneracy thresholds | closed (D36); `altitude_sweep` stays a diagnostic |
| 3 | DV1 re-watch (review 2026-11-30 or the first motion flight) | Task 12: the drift flights are the first motion flights; closing rule as in the DV1 row |
| 4 | `NO_PX4_SAMPLE` cause | Task 8 (DV2 investigation) |
| 5 | no_gps ≈ 20° heading reset absorbed by `T` | Task 8 test; Task 3 (ev_yaw run) and Task 10 (ev_yaw cut run) in SITL; D37 removes it from the compass profiles |
| 6 | Stale-epoch odometry gate; two resets in one dropped window | gate: Task 4. The two-reset limit stays a documented limit (the jump gate is the backstop): Not in this plan |
| 7 | `/lio/scan`; restart caller; seed helper with `T` in `LioRestart` | caller: Tasks 6–7; seed: Task 4; `T` in `LioRestart`: Task 7; `/lio/scan`: S3 |
| 8 | Hardware (DEGRADED at two lost scans; DDS profile; Mid-360 transport) | Not in this plan (P8) |
| 9 | `DO_SET_MODE` in flight; POSCTL source; `ImuGap` never fired; floor-only scene | `DO_SET_MODE` and POSCTL source: Task 11. `ImuGap` and floor-only: Not in this plan |
| 10 | Deferred review minors | T11 M2/M3 and the events list: Task 2; T8 M4 and the T7 deferred list: Task 7; T3 harness minors: Task 9; `kEventSummaryEvery` duplicate: Task 4; `qos.hpp` comment: Task 6 |
| 11 | Governance (legacy packages, stress thresholds, `arch_health`) | Task 13 |
| 12 | lio future-stamp tolerance (O28) | Task 1 (owner gate) |
| 13 | lio `kClockMode` | Task 1 (no gate) |
| 14 | Fast DDS profile not carried by the launch | Task 9 states the dependency in the launch docstring; the hardware transport check stays P8 |
| 15 | Unknown health code; zero-tolerance event list | the code is refused before the core (spec §4.1, 6ed71af); event list: Task 2 |
| 16 | D36 scope (corridor, real noise) | Not in this plan (P8) |
| 17 | DV2 | Task 8 (its trigger) |

DV1 and DV2 review dates: both are 2026-11-30. DV1's event trigger is met in Task 12, and DV2's in Task 8. The date stays the backstop if those tasks slip.

## Not in this plan

| Goes to | Items | Reason |
|---|---|---|
| S2 | `px4_mode`; minimal Supervisor with NavAvailability (REACQUIRING → LOST, O30), guards, handover reasons; straight trajectories; Y1/Y3a; a τ change if the owner decides one | S2 scope. The chain here ends at "logged" where a handover would follow. |
| S3 | `/lio/scan`, Mapping, `ResetMap` on a new epoch | No consumer before Mapping (O25). |
| Hardware (P8) | Livox `CustomMsg`, real clock mode, the transport, Mid-360 noise, corridor degeneracy, site `EKF2_MAG_DECL`, a real no-compass airframe, two lost scans at `gap_degraded`, the `ImuGap` policy, floor-only scene | Out of the beta. |
| Documented limit | Two PX4 resets in one dropped-sample window apply only the last delta | The jump gate is the backstop (S1b T10). |
| Owner items | "geometry returned" judged without the frozen map, if Task 10 shows the drifted-prior problem | Only with evidence. |

## Model and execution notes

- **Model per task:**
  - most capable model: Tasks 5 (lifecycle edge), 6 (EV gate and seed offers), 7 (ingest restart path), 8 (resync and DV2), 10 (recovery diagnosis) and 12 (benchmark judgement);
  - capable model: Tasks 3, 9 and 11;
  - mid-tier model: Tasks 1, 2, 4 and 13.

  Reviewers: the most capable model for Tasks 5–8 and the final review; the capable model otherwise.
- **Order:**
  - 1 (whenever O28 is answered) → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13.
  - If O29–O31 are still open after Task 4, run 11 → 12 next, then 5–10.
  - One active task at a time (CLAUDE.local.md). Read-only reviewers may run alongside, at most 3 agents in total.
- **Machine limits:** Makefile only (`-j3`, sanitize `-j2`); SITL never overlaps a build, a test or sanitize.
- **SITL time** (owner machine, idle):
  - Task 3: 4 runs, about 25 min;
  - Task 10: 5 runs plus at most 2 re-runs, about 45 min;
  - Task 12: 12 runs, about 110 min;
  - Task 13: 1 hover, 5 min.

  In total about 3 h. Agents prepare the commands and evaluate the artifacts.
- **TSan:** Tasks 1, 5 and 7 run `make sanitize`; no new thread is added anywhere.

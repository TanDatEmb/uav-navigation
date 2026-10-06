# S1a: LIO and PX4-Bridge Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build every ROS-free piece of slice S1 so that it is unit-tested before any SITL run:
- close the S0 carry-over;
- apply the D28 message changes;
- reduce `fast_lio_core` to a pure math library that exposes the per-scan information matrix;
- add `uavnav_lio_core`: lifecycle state machine, degeneracy and quality, EKF2-style output predictor, and the `LioEstimator` facade;
- add the pure core of `uavnav_px4_bridge`: EV encoding in the FRD frame, PX4 time conversion, and the alignment estimator `T`.

**Architecture:** `fast_lio_core` stays a separate package that contains only math. D15 calls this "group-2 algorithms as a library with a new shell", so the math does not need to live inside `uavnav_lio_core`.
- `uavnav_lio_core` wraps the math and owns all lifecycle, health and output decisions (spec §3).
- `uavnav_px4_bridge` gets a pure library target; its ROS node arrives in S1b.
- All decisions return `Result<T, Reason>`, emit `EventRecord`s and take a `TimeSnapshot`, following S0's `uavnav_core`.

**Tech Stack:** C++23 (gcc 13.3), Eigen 3, ament_cmake, GoogleTest, yaml-cpp ≥ 0.8, and the reused `fast_lio_core` with `ikfom_vendor` and `ikd_tree_vendor`.

**Spec:** [SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md) §3, §4.1, §6, §7.2 (S1). Decisions: D15, D20, D21, D22, D25, D28 in [DECISIONS.md](../architecture/DECISIONS.md). S0 carry-over: [S0 plan, "Carry-over to S1"](2026-10-06-s0-foundations.md).

**What S1b will cover (not in this plan):**
- the `uavnav_lio` ROS node, the `uavnav_px4_bridge` node and the `/events` publisher;
- SITL bring-up script and PX4 profile parameters (`EKF2_EV_CTRL` per D20, `EKF2_EV_QMIN=0`);
- hover smoke test;
- the F34 check;
- calibration of the degeneracy thresholds from logs.

## Global Constraints

- Work in this tree on `rebuild/v2`. Stage with `git add <path>` only. **Run `git diff --cached --stat` before every commit**: other agents share this index, so unstage anything you did not intend.
- Build and test with the Makefile: `make build PKGS="<pkgs>"` and `make test PKGS="<pkgs>"`. They already apply `-j3`, two colcon workers and `nice`.
- **Deleting or moving tracked paths:** sweep references first, then let the owner run the deletion script (AGENTS.md §6). The first rebuild after a move uses `--cmake-clean-cache`, passed through `COLCON_EXTRA="--cmake-clean-cache"` (Task 4 adds this variable to the Makefile).
- New code lives in namespaces `uavnav::lio` and `uavnav::px4bridge`. The reused math keeps namespace `uav::nav::lio`.
- **No ROS dependency** in `fast_lio_core`, `uavnav_lio_core`, or the library target of `uavnav_px4_bridge`.
- **Time:** the new API uses `uavnav::time` types. Convert `uav::nav::lio::Timestamp` (int64 ns) only at the facade boundary, via `SensorTime{ts.nanoseconds()}`. External stamps go through the range-checked converters from Task 2.
- **Config:** each component has a typed `struct XConfig`, a `constexpr ParamSpec kXSpecs[]`, and `Result<XConfig, ConfigError> load_x_config(const ParamValues&)` (D22 tier b). Tier-(a) constants go in that package's `limits.hpp`, each with a derivation comment.
- **Tests:** no wall-clock assertion tighter than 1 s. Use synthetic data only; SITL belongs to S1b.
- **Commits:** format `type(scope): description`, a blank line, then `Co-Authored-By: <the model that actually did the work> <noreply@anthropic.com>`. Separate subject, body and trailer with blank lines.

## Review Focus

1. **Only the LiDAR stops while IMU keeps flowing** (the F22 regression). `LioEstimator` must reach DEGRADED after 0.25 s and LOST after 0.5 s of IMU sensor time, measured on IMU samples alone. Pinned in Task 8.
2. **A correction arrives for a time older than the predictor's buffer**, for example after a processing stall longer than 300 ms. The predictor must reject it with a reason and keep its output continuous: no jump and no stale rewind. Pinned in Task 7.
3. **The PX4 reset counter jumps by two or more in one sample** (F13). The alignment estimator applies the deltas from `vehicle_local_position` and stays VALID. Pinned in Task 10.
4. **An empty scan, or a scan whose points all lie on one plane** (flying high, the F20/P5 scenario). The estimator emits a degenerate result with a reason, never throws, and never enters TRACKING from it. Pinned in Tasks 6 and 8.
5. **External stamp with an out-of-range `nanosec`, or a negative `sec`** (signed-overflow carry-over). The converter returns an error; no `TimePoint` is built. Pinned in Task 2.

---

## File Structure

```text
src/core/uavnav_core/                  (modify)  event_builder.hpp, time_convert.hpp/.cpp, sanitizer option, golden test
src/core/uavnav_interfaces/msg/        (modify)  LioOdometry, LioHealth, Alignment, NavCommand, MissionDefinition (D28)
src/vendor/ikfom_vendor, src/vendor/ikd_tree_vendor   (moved from src/estimation/)
src/estimation/fast_lio_core/          (trim)    keep math; add information matrix to IkfomCorrectionResult
src/estimation/uavnav_lio_core/        (new)
  include/uavnav/lio/limits.hpp        tier-(a) constants
  include/uavnav/lio/config.hpp        LioConfig + kLioSpecs + load_lio_config
  include/uavnav/lio/lifecycle.hpp     LioState, LioEvent, LioReason, LioLifecycle (pure state machine)
  include/uavnav/lio/degeneracy.hpp    DegeneracyReport, evaluate_degeneracy
  include/uavnav/lio/output_predictor.hpp  OutputPredictor (EKF2-style)
  include/uavnav/lio/estimator.hpp     LioEstimator facade + output samples
  src/*.cpp, test/test_*.cpp, test/synthetic_scene.hpp (test-only scene and IMU generator)
src/px4/uavnav_px4_bridge/             (new, library target only in S1a)
  include/uavnav/px4bridge/frames.hpp       FLU/FRD conversions
  include/uavnav/px4bridge/ev_encoder.hpp   EvSample, encode_ev
  include/uavnav/px4bridge/px4_time.hpp     Px4TimeConverter
  include/uavnav/px4bridge/alignment.hpp    AlignmentEstimator
  src/*.cpp, test/test_*.cpp
```

---

### Task 1: `uavnav_core` event hardening (S0 carry-over)

**Files:**
- Modify (under `src/core/uavnav_core/`): `include/uavnav/core/event.hpp`, `include/uavnav/core/event_recorder.hpp`, `src/event_recorder.cpp`, `test/test_event_recorder.cpp`
- Create: `include/uavnav/core/event_builder.hpp`, `test/golden/event_v1.jsonl`, `test/test_golden_jsonl.cpp`
- Modify: `tools/uavnav/tests/test_events.py`

**Interfaces:**
- Produces:

  ```cpp
  // event_recorder.hpp
  enum class RecorderError : std::uint8_t { kZeroCapacity, kNoSink };
  constexpr std::string_view to_string(RecorderError);           // "ZERO_CAPACITY", "NO_SINK"
  static Result<std::unique_ptr<EventRecorder>, RecorderError>
      EventRecorder::create(std::unique_ptr<EventSink> sink, std::size_t capacity = limits::kEventRingCapacity);
  // The public constructor becomes private; all call sites use create().
  // event.hpp: add_value(key, v) returns false when key already present (record unchanged).
  // event_builder.hpp (namespace uavnav::events)
  class EventBuilder {
   public:
    EventBuilder(Component c, std::string_view event, const time::TimeSnapshot& now);
    template <ReasonEnum S> EventBuilder& states(S before, S after);   // stores to_string(before/after)
    template <ReasonEnum R> EventBuilder& reason(R r);                  // stores to_string(r)
    EventBuilder& identity(const EventIdentity&);
    EventBuilder& value(std::string_view key, double v);               // silently ignores the 17th/duplicate (record keeps first)
    EventRecord build() const;
  };
  ```

- Behaviour change: the writer emits the batch **first**, then the `EventsDropped` notice, so that log order matches time order.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(EventRecorder, CreateRejectsZeroCapacityAndNullSink) {
    EXPECT_EQ(EventRecorder::create(std::make_unique<FakeSink>(), 0).error(), RecorderError::kZeroCapacity);
    EXPECT_EQ(EventRecorder::create(nullptr, 4).error(), RecorderError::kNoSink);
  }
  TEST(EventRecorder, DroppedNoticeFollowsTheBatchItSummarises) { /* blocked sink, cap 2, emit A,B,C,D; release; flush:
     sink order == A,B,EventsDropped(count=2) */ }
  TEST(EventRecorder, ConcurrentEmittersAccountForEveryRecord) {
    // 4 threads x 10000 emits, capacity 1024, sink sleeps 1 ms per batch;
    // after flush: stats.emitted + stats.dropped == 40000 and stats.written == stats.emitted. No time bound.
  }
  TEST(EventRecord, AddValueRejectsDuplicateKey) { /* add "a"=1 true; add "a"=2 false; values[0].value==1 */ }
  TEST(EventBuilder, StoresTypedNames) { /* enum with to_string "RUNNING"/"STALE": built record.state_before=="RUNNING", reason=="STALE",
     t_steady_ns/t_ros_ns copied from the snapshot */ }
  TEST(GoldenJsonl, CppLineMatchesGoldenFile) { /* to_json_line(golden record) == first line of test/golden/event_v1.jsonl */ }
  ```

  Python, in `test_events.py`: `test_golden_file_round_trip` loads `src/core/uavnav_core/test/golden/event_v1.jsonl` and asserts every field of the golden record, including `values == {"prefix_s": 1.5, "nan_value": None}`.

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error on `EventRecorder::create` / `event_builder.hpp`.

- [ ] **Step 3: Implement the changes**

  Write the golden line to `event_v1.jsonl` once, by hand, matching the S0 key order. It must contain one non-finite value, which is encoded as `null`. Update every existing test that constructs `EventRecorder` directly so it calls `create`.

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core && tools/uavnav/gate.sh python`

  Expected: all pass.

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat   # must be empty before staging
  git add src/core/uavnav_core tools/uavnav/tests/test_events.py
  git commit -m "feat(core): typed event builder, recorder factory, ordered drop notice"
  ```

---

### Task 2: `uavnav_core` time and config hardening, plus the sanitizer target

**Files:**
- Create (under `src/core/uavnav_core/`): `include/uavnav/core/time_convert.hpp`, `src/time_convert.cpp`, `test/test_time_convert.cpp`
- Modify:
  - `include/uavnav/core/config.hpp`, `src/config.cpp`, `test/test_config.cpp`
  - `src/core/uavnav_core/CMakeLists.txt`
  - `Makefile`

**Interfaces:**
- Produces:

  ```cpp
  // time_convert.hpp (namespace uavnav::time)
  enum class TimeError : std::uint8_t { kNegativeSeconds, kNanosecondsOutOfRange, kNotFinite, kOutOfRange };
  constexpr std::string_view to_string(TimeError);
  template <class Tag> Result<TimePoint<Tag>, TimeError> from_stamp(std::int64_t sec, std::uint32_t nanosec);
      // sec in [0, 9'000'000'000], nanosec < 1'000'000'000
  Result<Duration, TimeError> duration_from_seconds(double s);   // finite, |s| <= 1e6
  // config.hpp (namespace uavnav::config)
  Result<double, ConfigError> value(const ParamValues& values, std::string_view key);  // kMissingKey, never throws
  // load_params_file: files larger than 1 MiB -> kFileUnreadable with detail "larger than 1048576 bytes"
  ```

- CMake changes:
  - `find_package(yaml-cpp 0.8 REQUIRED)`.
  - Option `UAVNAV_SANITIZE` (empty, `thread` or `address`). When set, it adds `-fsanitize=<value> -fno-omit-frame-pointer -g` to `uavnav_core` and its tests.
- Makefile target `sanitize`:
  1. builds `uavnav_core` with `--cmake-args -DUAVNAV_SANITIZE=thread` and `MAKEFLAGS=-j2`;
  2. runs `colcon test --packages-select uavnav_core`;
  3. greps the test logs (`log/latest_test/uavnav_core/`) for `WARNING: ThreadSanitizer`;
  4. rebuilds `uavnav_core` with `-DUAVNAV_SANITIZE=`, so that the shared `build/` returns to normal.
  - The target fails when a warning was found or a test failed. D28 approves this run.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(TimeConvert, AcceptsValidStamp) { EXPECT_EQ(from_stamp<RosTag>(2, 500).value().ns, 2'000'000'500); }
  TEST(TimeConvert, RejectsBadStamps) {
    EXPECT_EQ(from_stamp<RosTag>(-1, 0).error(), TimeError::kNegativeSeconds);
    EXPECT_EQ(from_stamp<RosTag>(1, 1'000'000'000).error(), TimeError::kNanosecondsOutOfRange);
    EXPECT_EQ(from_stamp<RosTag>(9'000'000'001, 0).error(), TimeError::kOutOfRange);
  }
  TEST(TimeConvert, DurationFromSeconds) { /* 0.25 -> 250'000'000 ns; NaN -> kNotFinite; 2e6 -> kOutOfRange */ }
  TEST(Config, ValueNeverThrows) { /* value(values,"absent_s").error().kind == kMissingKey */ }
  TEST(Config, RejectsOversizedFile) { /* write 1 MiB + 1 byte temp file -> kFileUnreadable */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error, `time_convert.hpp` missing.

- [ ] **Step 3: Implement the converters, the config changes, the CMake option and the `sanitize` target**

- [ ] **Step 4: Run the tests, then the sanitizer**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core && make sanitize`

  Expected: all tests pass, and `make sanitize` reports no ThreadSanitizer warnings.

  If TSan reports a race in `EventRecorder`, fix the race in this task. Do not suppress the warning.

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/core/uavnav_core Makefile
  git commit -m "feat(core): range-checked stamp conversion, config size cap, TSan target"
  ```

---

### Task 3: D28 message changes

**Files:**
- Modify (under `src/core/uavnav_interfaces/msg/`): `LioOdometry.msg`, `LioHealth.msg`, `Alignment.msg`, `NavCommand.msg`, `MissionDefinition.msg`

**Interfaces:**
- Produces:
  - `LioOdometry.msg`: add `uint8 quality  # 0-100 from LIO degeneracy margin (D28)`.
  - `LioHealth.msg`: comment `stamp: SensorTime of the newest IMU sample evaluated`.
  - `Alignment.msg`: comment `stamp: SensorTime of the LIO sample in the newest pair`.
  - `NavCommand.msg`: comment `stamp and sample_time_ns: RosTime`.
  - `MissionDefinition.msg`:
    - add `float64 yaw_rad  # Y1 lock heading; NaN = yaw at mission start`;
    - extend the frame comment: `FRAME_GPS: z = altitude above PX4 home [m]`.

- [ ] **Step 1: Edit the messages**

- [ ] **Step 2: Build and inspect**

  Run: `make build PKGS=uavnav_interfaces && source install/setup.bash && ros2 interface show uavnav_interfaces/msg/LioOdometry | grep quality && ros2 interface show uavnav_interfaces/msg/MissionDefinition | grep yaw_rad`

  Expected: both lines are printed.

- [ ] **Step 3: Commit**

  ```bash
  git diff --cached --stat
  git add src/core/uavnav_interfaces/msg
  git commit -m "feat(interfaces): add LIO quality, mission lock yaw and clock-domain notes (D28)"
  ```

---

### Task 4: Reduce `fast_lio_core` to math, move the vendors, expose the information matrix

Spec §3, D15, D20 (degeneracy needs the 6×6 `HᵀR⁻¹H`). The keep and drop lists below come from a read-only survey of the package. Confirm each path with `git ls-files` before it goes into the script.

**Files:**
- Delete (Step 2, owner-run), under `src/estimation/fast_lio_core/`:
  - `include/fast_lio_core/pipeline/` and `src/pipeline/`
  - `include/fast_lio_core/synchronization/` and `src/synchronization/`
  - `include/fast_lio_core/propagation/` and `src/propagation/`
  - `include/fast_lio_core/sensor/measurement_group.hpp`
  - `include/fast_lio_core/mapping/map_insertion_policy.hpp` and its `.cpp` if present
  - `include/fast_lio_core/initialization/initial_state_prior_policy.hpp` and its `.cpp` if present
  - tests: `test_fast_lio_pipeline.cpp`, `test_initial_state_prior_pipeline.cpp`, `test_measurement_buffer.cpp`, `test_measurement_synchronizer.cpp`, `test_imu_state_propagator.cpp`
- Keep: everything else, including:
  - `initialization/imu_initializer` (stationary gravity and bias initialisation is math);
  - `initial_state_prior` and `initial_state_prior_applicator` (base-link seed conversion for restart, §3.4).
- Move:
  - `src/estimation/ikfom_vendor` → `src/vendor/ikfom_vendor`
  - `src/estimation/ikd_tree_vendor` → `src/vendor/ikd_tree_vendor`
- Modify:
  - `src/estimation/fast_lio_core/CMakeLists.txt`: drop the deleted sources and tests.
  - `include/fast_lio_core/configuration/estimator_config.hpp`: remove `LifecycleConfig` and `TrackingConfig`, which move to `uavnav_lio_core`.
  - `include/fast_lio_core/estimation/ikfom_estimator.hpp` and `src/estimation/ikfom_estimator.cpp`.
  - `test/test_ikfom_estimator.cpp`.
  - `Makefile`: add `COLCON_EXTRA ?=`, forward it to `colcon build`, and add `fast_lio_core` to the default `PKGS`.
  - `README.md`: update the licensing paths of the moved vendors.

**Interfaces:**
- Produces, in `uav::nav::lio::IkfomCorrectionResult`:

  ```cpp
  std::optional<Eigen::Matrix<double, 6, 6>> information;   // sum over accepted rows of
      // H.leftCols<6>()^T * diag(1/variance) * H.leftCols<6>() from the final iteration;
      // block (0:3,0:3) = translation, (3:6,3:6) = rotation; nullopt when no rows
  ```

  It is computed inside `IkfomEstimator::buildMeasurement` (`src/estimation/ikfom_estimator.cpp`, near line 481), where the Jacobian and `variance_m2` of the measurement view are available.

- [ ] **Step 1: Reference sweep**

  Run:

  ```bash
  grep -rnE "fast_lio_pipeline|measurement_buffer|measurement_synchronizer|imu_state_propagator|measurement_group|map_insertion_policy|initial_state_prior_policy|LifecycleConfig|TrackingConfig|src/estimation/(ikfom|ikd_tree)_vendor" \
    --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=.git .
  ```

  Every hit must be either inside a file being deleted, or listed in this task's Modify list. Docs and plans are text-only and may stay.

- [ ] **Step 2: Write `tools/uavnav/s1_prune.sh` (one `git rm` per path, forwarding `-n`), dry-run it, then hand it to the owner (OWNER ACTION)**

  Run: `tools/uavnav/s1_prune.sh -n`

  Expected: only the listed files appear. Then **stop and ask the owner to run it.** After the owner confirms, delete the script in the same commit.

- [ ] **Step 3: Move the vendors and rebuild the math library with a clean cache**

  Run:

  ```bash
  git mv src/estimation/ikfom_vendor src/vendor/ikfom_vendor
  git mv src/estimation/ikd_tree_vendor src/vendor/ikd_tree_vendor
  make build PKGS=fast_lio_core COLCON_EXTRA="--cmake-clean-cache"
  ```

  Expected: the build succeeds.

- [ ] **Step 4: Write the failing information-matrix tests in `test_ikfom_estimator.cpp`**

  ```cpp
  TEST(IkfomEstimator, InformationMatrixFullRankForThreeOrthogonalPlanes) {
    // map + scan sampled on planes x=5, y=5, z=-2 around the origin; one correct()
    // -> information has value; min eigenvalue of (0:3,0:3) > 0 and of (3:6,3:6) > 0
  }
  TEST(IkfomEstimator, InformationMatrixDegenerateForSinglePlane) {
    // only plane z=-2 -> translation block min eigenvalue < 1e-6 * max eigenvalue
  }
  ```

- [ ] **Step 5: Run them and confirm they fail**

  Run: `make build PKGS=fast_lio_core`

  Expected: compile error, `information` is not a member.

- [ ] **Step 6: Implement the field and the accumulation**

- [ ] **Step 7: Run the whole math test suite**

  Run: `make build PKGS=fast_lio_core && make test PKGS=fast_lio_core`

  Expected: every remaining test passes, including `test_ikfom_compact_equivalence` and the two new tests.

- [ ] **Step 8: Commit**

  ```bash
  git diff --cached --stat
  git add -u src/estimation src/vendor Makefile README.md
  git add src/vendor
  git commit -m "refactor(lio)!: reduce fast_lio_core to math, expose per-scan information matrix"
  ```

---

### Task 5: `uavnav_lio_core` package, config and lifecycle state machine

Spec §3.1, D20, D12.1.

**Files:**
- Create (under `src/estimation/uavnav_lio_core/`):
  - `CMakeLists.txt`, `package.xml` (depends on `uavnav_core`, `fast_lio_core`, `eigen3_cmake_module`; `ament_cmake_gtest` for tests)
  - `include/uavnav/lio/limits.hpp`, `include/uavnav/lio/config.hpp`, `include/uavnav/lio/lifecycle.hpp`
  - `src/config.cpp`, `src/lifecycle.cpp`
  - `test/test_lio_config.cpp`, `test/test_lifecycle.cpp`
- Modify: `Makefile` (add `uavnav_lio_core` to the default `PKGS`)

**Interfaces:**
- Produces (namespace `uavnav::lio`):

  ```cpp
  enum class LioState : std::uint8_t { kInitializing, kTracking, kDegraded, kLost, kRestarting };   // values match LioHealth constants
  enum class LioReason : std::uint8_t { kNone, kScanAccepted, kConfirmationReached, kDegenerateScans, kScanEmpty,
      kLidarGapDegraded, kLidarGapLost, kDegeneracyPersisted, kCovarianceExceeded, kGeometryReturned, kRestartSeeded,
      kMapReady };
  constexpr std::string_view to_string(LioState);   // "INITIALIZING", "TRACKING", ...
  constexpr std::string_view to_string(LioReason);  // UPPER_SNAKE
  struct LioEvent {                       // one per scan result or IMU-time tick
    enum class Kind : std::uint8_t { kScanGood, kScanDegenerate, kScanEmpty, kImuTick, kMapReady, kRestartSeeded } kind;
    time::SensorTime t;                   // IMU or scan sensor time
    double position_sigma_m{0.0};         // sqrt(max diag of position covariance)
  };
  struct LifecycleConfig {                // tier (b), loaded from kLioSpecs
    std::uint32_t confirm_scans;          // 5
    std::uint32_t degenerate_scans;       // 3
    time::Duration gap_degraded;          // 0.25 s
    time::Duration gap_lost;              // 0.5 s
    time::Duration degeneracy_lost;       // 1.0 s
    double position_sigma_lost_m;         // 0.5
  };
  struct Transition { LioState before, after; LioReason reason; bool changed; };
  class LioLifecycle {
   public:
    explicit LioLifecycle(const LifecycleConfig&);
    Transition on(const LioEvent& e);     // the only writer of the state
    LioState state() const noexcept;
    time::SensorTime last_scan_time() const noexcept;
  };
  // config.hpp
  struct LioConfig { LifecycleConfig lifecycle; };   // Task 6 adds `DegeneracyConfig degeneracy`, Task 7 `PredictorConfig predictor`,
                                                     // Task 8 `EstimatorMathConfig math`; each task extends kLioSpecs and load_lio_config
  extern const std::array<config::ParamSpec, N> kLioSpecs;
  Result<LioConfig, config::ConfigError> load_lio_config(const config::ParamValues&);
  ```

  In this task `kLioSpecs` has the lifecycle keys only. Tasks 6–8 append their keys.

  | Key | Unit | Min | Max | Beta value |
  |---|---|---|---|---|
  | `lifecycle_confirm_scans` | kNone | 1 | 50 | 5 |
  | `lifecycle_degenerate_scans` | kNone | 1 | 50 | 3 |
  | `lifecycle_gap_degraded_s` | kSeconds | 0.05 | 2.0 | 0.25 |
  | `lifecycle_gap_lost_s` | kSeconds | 0.1 | 5.0 | 0.5 |
  | `lifecycle_degeneracy_lost_s` | kSeconds | 0.1 | 10.0 | 1.0 |
  | `lifecycle_position_sigma_lost_m` | kMeters | 0.05 | 5.0 | 0.5 |

  `load_lio_config` also rejects these cross-field errors with `kOutOfRange`, naming both keys:
  - `gap_lost <= gap_degraded`;
  - `degenerate_scans > confirm_scans * 10`.

- Transition rules, matching the diagram in spec §3.1:

  | From | Event / condition | To | Reason |
  |---|---|---|---|
  | INITIALIZING | `kMapReady` | INITIALIZING | `kMapReady` (internal counter starts) |
  | INITIALIZING | `confirm_scans` consecutive `kScanGood` after `kMapReady` | TRACKING | `kConfirmationReached` |
  | TRACKING | `degenerate_scans` consecutive `kScanDegenerate`/`kScanEmpty` | DEGRADED | `kDegenerateScans` |
  | TRACKING | `kImuTick` with `t - last_scan_time > gap_degraded` | DEGRADED | `kLidarGapDegraded` |
  | DEGRADED | `confirm_scans` consecutive `kScanGood` | TRACKING | `kConfirmationReached` |
  | DEGRADED | `kImuTick` with gap `> gap_lost` | LOST | `kLidarGapLost` |
  | DEGRADED | degenerate since `> degeneracy_lost` | LOST | `kDegeneracyPersisted` |
  | TRACKING/DEGRADED | any event with `position_sigma_m > position_sigma_lost_m` | LOST | `kCovarianceExceeded` |
  | LOST | `kScanGood` (geometry is back) | RESTARTING | `kGeometryReturned` |
  | RESTARTING | `kRestartSeeded` | INITIALIZING | `kRestartSeeded` |

  Any other (state, event) pair leaves the state unchanged, with `changed=false`. There is never a direct path from LOST to TRACKING.

- [ ] **Step 1: Write the failing tests**

  - `test_lio_config.cpp`:
    - the beta values load;
    - `gap_lost_s=0.2` together with `gap_degraded_s=0.25` → `kOutOfRange` naming both keys;
    - an unknown key is rejected.
  - `test_lifecycle.cpp`, one test per table row, plus:

  ```cpp
  TEST(LioLifecycle, LidarGapOnImuTicksAloneReachesLost) {   // F22
    // TRACKING at scan t=10.0 s; kImuTick every 5 ms with no scans:
    // DEGRADED at first tick with t > 10.25, LOST at first tick with t > 10.5
  }
  TEST(LioLifecycle, LostNeverJumpsToTracking) {             // F27
    // LOST + 20 x kScanGood -> state RESTARTING after the first, stays RESTARTING
  }
  TEST(LioLifecycle, EmptyScansCountAsDegenerate) { /* 3 x kScanEmpty in TRACKING -> DEGRADED, reason kDegenerateScans */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_lio_core`

  Expected: the package or headers are missing.

- [ ] **Step 3: Implement `limits.hpp`, `config`, and `lifecycle` as a table-driven `on()`**

  `limits.hpp` for now holds only `kMaxScanPoints = 200'000`, with the comment: "Mid-360 at 10 Hz ≈ 20k points per scan; 10× headroom, guards memory".

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core`

  Expected: all pass.

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core Makefile
  git commit -m "feat(lio): add LIO lifecycle state machine and typed config"
  ```

---

### Task 6: Degeneracy evaluation and quality

Spec §3.2, D20, D28.

**Files:**
- Create (under `src/estimation/uavnav_lio_core/`): `include/uavnav/lio/degeneracy.hpp`, `src/degeneracy.cpp`, `test/test_degeneracy.cpp`
- Modify: `include/uavnav/lio/config.hpp`, `src/config.cpp` (add the keys below)

**Interfaces:**
- Produces:

  ```cpp
  struct DegeneracyConfig { double translation_min_info; double rotation_min_info; };
  struct DegeneracyReport {
    double translation_min_eigenvalue; double rotation_min_eigenvalue;
    bool degenerate;                       // either block below its threshold
    std::uint8_t quality;                  // 0..100
  };
  DegeneracyReport evaluate_degeneracy(const std::optional<Eigen::Matrix<double,6,6>>& information,
                                       const DegeneracyConfig&);
  ```

- Quality rule: `quality = clamp(round(50 * min(λt / translation_min_info, λr / rotation_min_info)), 0, 100)`. So quality is 50 exactly at the threshold, 100 at twice the threshold, and 0 when there is no information.
- New keys: `degeneracy_translation_min_info` (kNone, range [1, 1e9]) and `degeneracy_rotation_min_info` (kNone, range [1, 1e12]).
- Beta starting values, with the derivation as a comment in the default YAML:
  - translation `1.1e5`: about 100 points at σ = 0.03 m along the weakest direction (100 / 0.03²);
  - rotation `2.8e6`: the same 100 points at a 5 m lever arm (100 · 5² / 0.03²).
  - S1b recalibrates both from SITL logs.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Degeneracy, NoInformationIsDegenerateWithZeroQuality) { /* nullopt -> degenerate, quality 0 */ }
  TEST(Degeneracy, QualityIsFiftyAtThreshold) { /* diag(1.1e5 x3, 2.8e6 x3) -> quality 50, degenerate false */ }
  TEST(Degeneracy, QualityCapsAtHundred) { /* 10x thresholds -> 100 */ }
  TEST(Degeneracy, WeakTranslationAxisIsDegenerate) { /* diag(1.1e5,1.1e5,10, 2.8e6 x3) -> degenerate, quality 0 */ }
  TEST(Degeneracy, RotationBlockUsedIndependently) { /* rotation min below threshold -> degenerate */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_lio_core`

  Expected: compile error, `degeneracy.hpp` missing.

- [ ] **Step 3: Implement**

  Use `Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>` on each diagonal block. A non-finite eigenvalue counts as degenerate.

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core`

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core
  git commit -m "feat(lio): add degeneracy report and EV quality"
  ```

---

### Task 7: EKF2-style output predictor

Spec §3.3, F35 (PX4 `output_predictor.cpp`), D20. Read `/home/letandat/Dev/Autopilot/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp` before implementing. Follow its structure, except for the two differences listed below.

**Files:**
- Create (under `src/estimation/uavnav_lio_core/`): `include/uavnav/lio/output_predictor.hpp`, `src/output_predictor.cpp`, `test/test_output_predictor.cpp`
- Modify: `include/uavnav/lio/limits.hpp`, `config.hpp`, `src/config.cpp`

**Interfaces:**
- Produces:

  ```cpp
  struct ImuDelta { time::SensorTime t; Eigen::Vector3d delta_angle_rad; Eigen::Vector3d delta_velocity_mps;
                    double dt_s; };                     // from consecutive IMU samples, body = IMU frame
  struct OutputSample { time::SensorTime t; Eigen::Quaterniond q_world_imu; Eigen::Vector3d v_world_mps;
                        Eigen::Vector3d p_world_m; std::uint32_t reset_counter; };
  struct EstimatorSnapshot { time::SensorTime t; Eigen::Quaterniond q_world_imu; Eigen::Vector3d v_world_mps;
                             Eigen::Vector3d p_world_m; Eigen::Vector3d gyro_bias; Eigen::Vector3d accel_bias;
                             Eigen::Vector3d gravity_world; };
  enum class PredictorReason : std::uint8_t { kApplied, kNotInitialized, kOlderThanBuffer, kNewerThanOutput,
                                              kNonFinite };
  constexpr std::string_view to_string(PredictorReason);
  struct ResetDelta { Eigen::Vector3d position_m; Eigen::Vector3d velocity_mps; double yaw_rad; };
  struct PredictorConfig { time::Duration tau_vel; time::Duration tau_pos; };   // 0.25 s, 0.25 s
  class OutputPredictor {
   public:
    explicit OutputPredictor(const PredictorConfig&);
    void align(const EstimatorSnapshot& s);                         // first init / epoch start; empties buffer
    std::optional<OutputSample> on_imu(const ImuDelta& d);          // nullopt until aligned
    Result<void, PredictorReason> on_correction(const EstimatorSnapshot& s);   // s.t = scan time
    ResetDelta reset_to(const EstimatorSnapshot& s);                // explicit reset; reset_counter++
    Eigen::Vector3d tracking_error() const noexcept;                // (att rad, vel m/s, pos m)
  };
  ```

- Tier-(a) constants in `limits.hpp`:
  - `kPredictorBufferSpan = milliseconds(300)` (spec §3.3);
  - `kPredictorBufferCapacity = 64`, which is ≥ 300 ms at 200 Hz;
  - `kPredictorDtMin = 0.0001 s` and `kPredictorDtMax = 0.03 s` (the PX4 clamp).
- New keys: `predictor_tau_vel_s` and `predictor_tau_pos_s`, both kSeconds, range [0.05, 5.0], beta value 0.25.
- **Differences from PX4:**
  - The delayed sample is looked up by **timestamp**. Take the buffered output whose time is the closest one ≤ `s.t`, and reject it when it is more than one IMU period away. PX4 instead takes the oldest sample (its own TODO).
  - A correction older than the buffer returns `kOlderThanBuffer` and changes nothing.
- **Kept from PX4:**
  - the attitude gain `0.5·dt/delay`;
  - PI correction of velocity and position (`dt/τ`, integral `0.1·gain²`) applied to the whole buffer;
  - averaged dt;
  - the separate vertical channel.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(OutputPredictor, IntegratesConstantVelocityWithoutCorrections) { /* align v=(1,0,0); 200 IMU deltas with gravity-compensating dv -> p.x ≈ 1.0 m after 1 s (tol 1e-3) */ }
  TEST(OutputPredictor, CorrectionConvergesWithoutJump) {
    // aligned at origin; correction says p=(0.5,0,0) at the delayed time;
    // keep feeding IMU + same correction each 100 ms: max |p_k - p_{k-1}| between consecutive outputs < 0.05 m,
    // and |p - truth| < 0.05 m after 2 s
  }
  TEST(OutputPredictor, CorrectionMatchedByTimestampNotOldest) { /* correction at t-0.15 s uses the sample nearest t-0.15, verified via tracking_error */ }
  TEST(OutputPredictor, RejectsCorrectionOlderThanBuffer) { /* s.t = now - 0.5 s -> kOlderThanBuffer, outputs unchanged */ }
  TEST(OutputPredictor, ResetReportsDeltaAndCounter) { /* reset_to shifted snapshot -> counter+1, delta.position == shift */ }
  TEST(OutputPredictor, JitteryDtIsClamped) { /* dt 0.0 and 0.1 inputs -> no NaN, dt clamped to [1e-4, 0.03] */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_lio_core`

  Expected: compile error, `output_predictor.hpp` missing.

- [ ] **Step 3: Implement, following PX4's `calculateOutputStates` and `correctOutputStates` structure with the differences above**

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core`

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core
  git commit -m "feat(lio): add EKF2-style output predictor with timestamp-matched corrections"
  ```

---

### Task 8: `LioEstimator` facade

Spec §3 (all subsections), D20. This is the single class the S1b ROS node calls. It is ROS-free and single-threaded; S1b's node owns the threads.

**Files:**
- Create (under `src/estimation/uavnav_lio_core/`):
  - `include/uavnav/lio/estimator.hpp`, `src/estimator.cpp`
  - `test/synthetic_scene.hpp` (test-only), `test/test_estimator.cpp`
- Modify: `config.hpp`, `src/config.cpp` (add the math keys), and `config/lio/sim.yaml` (create this file: the default tier-(b) values for SITL)

**Interfaces:**
- Consumes:
  - `LioLifecycle` (Task 5), `evaluate_degeneracy` (Task 6), `OutputPredictor` (Task 7);
  - from `fast_lio_core`: `IkfomEstimator`, `ImuInitializer`, `ScanDeskewer`, `PointCloudPreprocessor`, `IkdTreeRegistrationMap`, `LocalMapManager`, `InitialStatePriorApplicator`;
  - `EventRecorder` and `EventBuilder` (Task 1).
- Produces:

  ```cpp
  struct ImuInput { time::SensorTime t; Eigen::Vector3d gyro_rad_s; Eigen::Vector3d accel_mps2; };   // already scaled to m/s^2
  struct ScanInput { time::SensorTime start, end; std::vector<uav::nav::lio::LidarPoint> points; bool per_point_time; };
  struct SeedPose { Eigen::Vector3d p_world_m; Eigen::Quaterniond q_world_base; };   // T^-1 * PX4 pose (§3.4)
  struct StateOutput { OutputSample sample; std::uint32_t epoch; LioState state; };              // per IMU (S1b decimates to 100 Hz)
  struct OdometryOutput { time::SensorTime t; std::uint32_t epoch; std::uint32_t reset_counter;
                          Eigen::Vector3d p_world_m; Eigen::Quaterniond q_world_base; Eigen::Vector3d v_world_mps;
                          Eigen::Matrix<double,6,6> pose_cov; Eigen::Matrix3d vel_cov; std::uint8_t quality; };
  struct HealthOutput { time::SensorTime t; LioState state; std::uint32_t epoch; LioReason reason;
                        double translation_min_eigenvalue, rotation_min_eigenvalue, correction_age_s;
                        Eigen::Vector3d output_tracking_error; };
  struct StepOutputs { std::vector<StateOutput> states; std::optional<OdometryOutput> odometry;
                       std::optional<HealthOutput> health; };   // health set on every transition and every 100 ms of IMU time
  enum class EstimatorReason : std::uint8_t { kAccepted, kOutOfOrder, kTooManyPoints, kNotFinite, kWrongState };
  class LioEstimator {
   public:
    static Result<std::unique_ptr<LioEstimator>, config::ConfigError>
        create(const LioConfig&, const Eigen::Isometry3d& base_T_imu, const Eigen::Isometry3d& imu_T_lidar,
               events::EventRecorder& events);
    Result<StepOutputs, EstimatorReason> push_imu(const ImuInput&);    // runs predictor + gap check (kImuTick)
    Result<StepOutputs, EstimatorReason> push_scan(ScanInput&&);       // predict -> deskew -> correct -> degeneracy -> lifecycle
    Result<void, EstimatorReason> restart(const SeedPose& seed, const time::TimeSnapshot& now);  // only in RESTARTING
    LioState state() const noexcept; std::uint32_t epoch() const noexcept;
  };
  ```

- Rules:
  - Inputs must be non-decreasing in sensor time per stream. A regression returns `kOutOfOrder` and changes nothing.
  - **IMU-only gap check (F22):** every accepted IMU feeds `LioEvent{kImuTick, t}` to the lifecycle.
  - **Empty scan (F20/P5):** a scan with no points after preprocessing produces `kScanEmpty` and is never an exception.
  - **When `odometry` is produced:** only on a scan that leaves the state in TRACKING. It carries `quality` from the degeneracy report and the scan end time.
  - **Restart (§3.4):**
    1. clears the map;
    2. applies the seed through `InitialStatePriorApplicator`;
    3. increments `epoch` and the predictor's `reset_counter` (`reset_to`);
    4. feeds `kRestartSeeded`;
    5. emits a `LioRestart` event with values `old_epoch`, `new_epoch`, `seed_x_m`, `seed_y_m`, `seed_z_m`, `seed_yaw_rad`.
  - Every lifecycle transition emits an event built with `EventBuilder(kLio, "StateTransition", now).states(before, after).reason(r)`.
- Math keys added to `kLioSpecs`. Beta values come from `config/runtime/sim.yaml` (the survey of `main`):

  | Key | Beta value |
  |---|---|
  | `extrinsic_imu_lidar_x_m` / `_y_m` / `_z_m` | -0.011 / -0.02329 / 0.04412 |
  | `preprocess_min_range_m` / `preprocess_max_range_m` | 0.5 / 40 |
  | `preprocess_voxel_m` | 0.2 |
  | `map_voxel_m` | 0.3 |
  | `map_half_extent_x_m` / `_y_m` / `_z_m` | 30 / 30 / 15 |
  | `registration_max_iterations` | 4 |
  | `imu_init_min_samples` | 200 |
  | `imu_max_gap_s` | 0.02 |

  Each key has bounds in the spec table. Translate the agent-survey values into `kLioSpecs` with min/max chosen as roughly ¼× to 4× the beta value, with one exception: `registration_max_iterations` uses [1, 10].

- [ ] **Step 1: Write the test scene `synthetic_scene.hpp`**

  The scene is an axis-aligned box room, 20 × 20 × 6 m. The header provides:
  - `make_scan(pose, t_start, t_end)`, returning about 3000 points on the walls (no per-point time);
  - `make_imu(trajectory, t)`, returning a stationary or constant-velocity IMU sample with gravity;
  - `make_plane_only_scan(pose, t)`, returning floor points only (the degenerate case).

- [ ] **Step 2: Write the failing tests `test_estimator.cpp`**

  ```cpp
  TEST(LioEstimator, ReachesTrackingInStaticRoom) { /* 2 s stationary IMU at 200 Hz + scans at 10 Hz -> TRACKING, odometry present, quality > 50 */ }
  TEST(LioEstimator, ImuOnlyGapReachesLost) { /* TRACKING, then 1 s IMU only -> health reasons kLidarGapDegraded then kLidarGapLost; no odometry */ }
  TEST(LioEstimator, PlaneOnlyScansDegradeWithoutThrowing) { /* TRACKING, then 5 plane-only scans -> DEGRADED, no exception */ }
  TEST(LioEstimator, EmptyScanIsAnEventNotAnException) { /* points={} -> returns Ok, health reason kDegenerateScans after 3 */ }
  TEST(LioEstimator, RejectsOutOfOrderInput) { /* imu t=1.0 then t=0.9 -> kOutOfOrder */ }
  TEST(LioEstimator, RestartSeedsNewEpoch) {
    // drive to LOST, then a good scan -> RESTARTING; restart(seed) -> epoch 2, reset_counter +1,
    // recorder received "LioRestart" with old_epoch=1,new_epoch=2; then confirm scans -> TRACKING
  }
  TEST(LioEstimator, RestartRejectedOutsideRestarting) { /* restart in TRACKING -> kWrongState */ }
  ```

- [ ] **Step 3: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_lio_core`

  Expected: compile error, `estimator.hpp` missing.

- [ ] **Step 4: Implement**

  Use `git show main:src/estimation/fast_lio_core/src/pipeline/fast_lio_pipeline.cpp` only as a reference for call order: initialiser → predict → deskew → preprocess → correct → map insert/crop. Do not copy its lifecycle code.

- [ ] **Step 5: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_lio_core && make test PKGS=uavnav_lio_core`

  Expected: all `uavnav_lio_core` tests pass.

- [ ] **Step 6: Commit**

  ```bash
  git diff --cached --stat
  git add src/estimation/uavnav_lio_core config/lio/sim.yaml
  git commit -m "feat(lio): add LioEstimator facade over reused FAST-LIO math"
  ```

---

### Task 9: `uavnav_px4_bridge` library: frames, EV encoding, PX4 time

Spec §4.1, D20, D21, D28, F14, F34.

**Files:**
- Create (under `src/px4/uavnav_px4_bridge/`):
  - `CMakeLists.txt` with library target `uavnav_px4_bridge_core` (no ROS)
  - `package.xml` (depends on `uavnav_core`)
  - `include/uavnav/px4bridge/frames.hpp`, `ev_encoder.hpp`, `px4_time.hpp`
  - `src/ev_encoder.cpp`, `src/px4_time.cpp`
  - `test/test_frames.cpp`, `test/test_ev_encoder.cpp`, `test/test_px4_time.cpp`
- Modify: `Makefile` (add `uavnav_px4_bridge` to the default `PKGS`)

**Interfaces:**
- Produces (namespace `uavnav::px4bridge`):

  ```cpp
  // frames.hpp: lio_odom is a gravity-aligned FLU world with arbitrary heading; PX4 FRD-local = (x, -y, -z)
  Eigen::Vector3d flu_to_frd(const Eigen::Vector3d&);
  Eigen::Quaterniond flu_to_frd(const Eigen::Quaterniond& q_world_body_flu);   // world FLU->FRD and body FLU->FRD
  Eigen::Matrix3d flu_to_frd_cov(const Eigen::Matrix3d&);
  // px4_time.hpp
  enum class ClockMode : std::uint8_t { kSimulationIdentity, kRealtime };
  enum class Px4TimeError : std::uint8_t { kRealtimeNotSupported, kNegative };
  Result<time::Px4Time, Px4TimeError> to_px4(time::SensorTime t, ClockMode mode);   // identity in SITL (/clock); realtime -> error (beta, §0)
  std::uint64_t to_px4_us(time::Px4Time t);                                           // ns / 1000, t >= 0
  // ev_encoder.hpp
  struct EvSample {                       // maps 1:1 onto px4_msgs::msg::VehicleOdometry in S1b
    std::uint64_t timestamp_sample_us; std::uint8_t pose_frame;   // 2 = POSE_FRAME_FRD
    std::array<float,3> position; std::array<float,4> q_wxyz; std::uint8_t velocity_frame; // 2 = VELOCITY_FRAME_FRD
    std::array<float,3> velocity; std::array<float,3> position_variance, orientation_variance, velocity_variance;
    std::uint8_t reset_counter; std::int8_t quality;
  };
  enum class EvReason : std::uint8_t { kEncoded, kNotTracking, kNonFiniteState, kBadCovariance, kTime };
  constexpr std::string_view to_string(EvReason);
  struct EvInput { time::SensorTime t; std::uint32_t epoch; bool tracking; Eigen::Vector3d p_m; Eigen::Quaterniond q;
                   Eigen::Vector3d v_mps; Eigen::Matrix<double,6,6> pose_cov; Eigen::Matrix3d vel_cov; std::uint8_t quality; };
  Result<EvSample, EvReason> encode_ev(const EvInput&, ClockMode);
  ```

- Encoding rules:
  - `reset_counter = epoch % 256`. The counter comes from the sample's own epoch, never from a health stream (F14).
  - Frame labels are FRD for both pose and velocity, never NED (F34).
  - Position is reported in `lio_odom`, not in base_link.
  - Covariance blocks: position = `pose_cov(0:3,0:3)` rotated to FRD; orientation = `pose_cov(3:6,3:6)` in the body frame.
  - Every diagonal entry must be finite and > 0; otherwise `kBadCovariance`.
  - `quality` is copied from the input (D28).

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Frames, FluToFrdFlipsYZ) { /* (1,2,3) -> (1,-2,-3); identity quaternion stays identity */ }
  TEST(Frames, YawSignFlips) { /* q yaw +30 deg FLU -> yaw -30 deg FRD */ }
  TEST(Px4Time, IdentityInSimulationRealtimeRejected) { /* to_px4(SensorTime{5'000'000}, kSimulationIdentity).ns==5'000'000; kRealtime -> kRealtimeNotSupported */ }
  TEST(EvEncoder, UsesSampleEpochAsResetCounter) { /* epoch 257 -> reset_counter 1 */ }
  TEST(EvEncoder, LabelsFrdNotNed) { /* pose_frame == 2 && velocity_frame == 2 */ }
  TEST(EvEncoder, RejectsWhenNotTracking) { /* tracking=false -> kNotTracking */ }
  TEST(EvEncoder, RejectsNonPositiveVariance) { /* pose_cov(0,0)=0 -> kBadCovariance */ }
  TEST(EvEncoder, CopiesQuality) { /* quality 73 -> 73 */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_px4_bridge`

  Expected: the package is missing.

- [ ] **Step 3: Implement**

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/px4/uavnav_px4_bridge Makefile
  git commit -m "feat(px4-bridge): add FRD EV encoder and SITL PX4 time conversion"
  ```

---

### Task 10: Alignment estimator `T_px4←lio`

Spec §4.1, D7, D21, F13.

**Files:**
- Create (under `src/px4/uavnav_px4_bridge/`): `include/uavnav/px4bridge/alignment.hpp`, `src/alignment.cpp`, `test/test_alignment.cpp`, `include/uavnav/px4bridge/config.hpp`, `src/config.cpp`

**Interfaces:**
- Produces:

  ```cpp
  enum class AlignmentState : std::uint8_t { kInit, kValid, kFrozen, kInvalid };   // matches Alignment.msg constants
  enum class AlignmentReason : std::uint8_t { kNone, kPairsConsistent, kPairAccepted, kPairRejectedJump, kLioLost,
      kLioTracking, kPx4ResetApplied, kPx4ResetWhileFrozen, kStale, kFrozenTooLong, kNoPx4Sample };
  constexpr std::string_view to_string(AlignmentState); constexpr std::string_view to_string(AlignmentReason);
  struct Pose4 { double x_m, y_m, z_m, yaw_rad; };            // T maps FRD-converted lio_odom -> PX4 local NED
  struct Px4PoseSample { time::SensorTime t; Eigen::Vector3d p_ned_m; double yaw_rad;
                         std::uint8_t xy_reset_counter, z_reset_counter, heading_reset_counter;
                         Eigen::Vector2d delta_xy_m; double delta_z_m; double delta_heading_rad; };  // from vehicle_local_position
  struct LioPoseSample { time::SensorTime t; Eigen::Vector3d p_frd_m; double yaw_frd_rad; bool tracking; };
  struct AlignmentConfig { time::Duration tau; std::uint32_t consistent_pairs; double jump_position_m; double jump_yaw_rad;
                           double max_rate_mps; double max_yaw_rate_rad_s; time::Duration valid_stale;
                           time::Duration frozen_max; };
  struct AlignmentOutput { AlignmentState state; Pose4 filtered; std::optional<Pose4> raw; time::SensorTime stamp;
                           double age_s; double residual_position_m; double residual_yaw_rad; AlignmentReason reason; };
  class AlignmentEstimator {
   public:
    explicit AlignmentEstimator(const AlignmentConfig&);
    void on_px4(const Px4PoseSample& s);               // buffered (1 s); applies reset deltas deterministically
    AlignmentOutput on_lio(const LioPoseSample& s);    // pairs with PX4 interpolated at s.t
    AlignmentOutput on_tick(time::SensorTime now);     // staleness and frozen timeout
    AlignmentState state() const noexcept;
  };
  Result<AlignmentConfig, config::ConfigError> load_alignment_config(const config::ParamValues&);
  ```

- Config keys (tier b):

  | Key | Unit | Min | Max | Beta value |
  |---|---|---|---|---|
  | `alignment_tau_s` | kSeconds | 0.2 | 20 | 2.0 |
  | `alignment_consistent_pairs` | kNone | 1 | 200 | 20 |
  | `alignment_jump_position_m` | kMeters | 0.05 | 5 | 0.5 |
  | `alignment_jump_yaw_rad` | kRadians | 0.01 | 0.5 | 0.0873 (5°) |
  | `alignment_max_rate_mps` | kMetersPerSecond | 0.01 | 5 | 0.5 |
  | `alignment_max_yaw_rate_rad_s` | kRadiansPerSecond | 0.001 | 0.5 | 0.0873 |
  | `alignment_valid_stale_s` | kSeconds | 0.2 | 10 | 1.0 |
  | `alignment_frozen_max_s` | kSeconds | 1 | 120 | 10.0 (= `lio_recovery_timeout_s`, D18) |

- Rules (spec §4.1):
  - **Instant T:** `yaw = wrap(yaw_px4 - yaw_lio)`, `t = p_px4 - Rz(yaw)·p_lio`.
  - **INIT → VALID:** after `consistent_pairs` consecutive instant T values all lie within the jump thresholds of their running mean.
  - **Filter:** first order, `alpha = dt/τ`. Each step is then rate-limited by `max_rate·dt` (translation) and `max_yaw_rate·dt` (yaw).
  - **Jump rejection:** an instant T farther than the jump thresholds from the filtered T is rejected with `kPairRejectedJump`; the filtered T is unchanged.
  - **PX4 reset:** whenever any of `xy`/`z`/`heading_reset_counter` differs from the last seen value, by **any** step (F13), add `delta_xy`, `delta_z` and `delta_heading` to the filtered T and to the buffered PX4 samples, then emit `kPx4ResetApplied`. In FROZEN, the same event produces INVALID with `kPx4ResetWhileFrozen`.
  - **Transitions:**
    - `tracking=false` → FROZEN (`kLioLost`);
    - FROZEN with `tracking=true` → VALID (`kLioTracking`);
    - VALID with no accepted pair for `valid_stale` → INVALID (`kStale`);
    - FROZEN for `frozen_max` → INVALID (`kFrozenTooLong`);
    - INVALID → INIT on the next tracking pair, and a new INIT accumulation starts.

- [ ] **Step 1: Write the failing tests**

  ```cpp
  TEST(Alignment, ConvergesToTrueOffset) { /* truth T=(3,-2,0.5,0.6 rad); 30 noise-free pairs -> VALID, filtered within 1e-3 after 20 s */ }
  TEST(Alignment, RejectsGnssGlitchJump) { /* VALID, one PX4 sample offset +2 m -> kPairRejectedJump, filtered unchanged */ }
  TEST(Alignment, RateLimitsSlowDrift) { /* truth drifts 2 m/s -> filtered moves <= 0.5 m/s */ }
  TEST(Alignment, AppliesDoubleResetDeltas) {   // F13
    // VALID; PX4 sample with xy_reset_counter += 2 and delta_xy=(1,0) -> filtered.x += 1 exactly, state stays VALID
  }
  TEST(Alignment, FreezesOnLioLostAndInvalidatesOnResetWhileFrozen) { /* tracking=false -> FROZEN; reset -> INVALID kPx4ResetWhileFrozen */ }
  TEST(Alignment, FrozenTimesOut) { /* FROZEN, on_tick 10.1 s later -> INVALID kFrozenTooLong */ }
  TEST(Alignment, ValidGoesStale) { /* VALID, no pairs 1.1 s -> INVALID kStale */ }
  TEST(Alignment, NoPx4SampleNearPair) { /* empty buffer -> reason kNoPx4Sample, state unchanged */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_px4_bridge`

  Expected: compile error, `alignment.hpp` missing.

- [ ] **Step 3: Implement**

  Interpolate the PX4 pose linearly in position and use shortest-arc yaw interpolation between the two buffered samples around `s.t`. Return `kNoPx4Sample` when no sample lies within 50 ms on either side. 50 ms is a tier-(a) constant in `limits.hpp`, with the comment "PX4 odometry ≥ 50 Hz".

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_px4_bridge && make test PKGS=uavnav_px4_bridge`

- [ ] **Step 5: Commit**

  ```bash
  git diff --cached --stat
  git add src/px4/uavnav_px4_bridge
  git commit -m "feat(px4-bridge): add alignment estimator with deterministic PX4 reset handling"
  ```

---

### Task 11: Close S1a, gate, traceability, S1b carry-over

**Files:**
- Modify: `docs/TRACEABILITY.md`, and this plan (append "Carry-over to S1b")

- [ ] **Step 1: Run the full gate**

  Run: `tools/uavnav/gate.sh all`

  Expected: `GATE_RESULT=PASS`.

- [ ] **Step 2: Update `docs/TRACEABILITY.md`**

  Fill the WP column with `S1a-T<n>` and add test names as evidence:

  | Row | Status | Evidence |
  |---|---|---|
  | F13 | done | `Alignment.AppliesDoubleResetDeltas` |
  | F14 | done | `EvEncoder.UsesSampleEpochAsResetCounter` |
  | F22 | done | `LioLifecycle.LidarGapOnImuTicksAloneReachesLost`, `LioEstimator.ImuOnlyGapReachesLost` |
  | F23 | done | `OutputPredictor.CorrectionConvergesWithoutJump` |
  | F27 | done | `LioLifecycle.LostNeverJumpsToTracking` |
  | F34 | doing | `EvEncoder.LabelsFrdNotNed`; the SITL check is S1b |
  | P7 | doing | Predictor done; 100 Hz publishing is S1b |
  | P3 | doing | Alignment done; SITL in S1b/S2 |
  | Config ba tầng | doing | Typed `LioConfig` and `AlignmentConfig` |

  Run: `python3 tools/check_documentation.py . docs`

  Expected: `PASS`.

- [ ] **Step 3: Append "Carry-over to S1b" to this plan**

  List every obligation found during S1a that S1b must schedule. Then commit:

  ```bash
  git diff --cached --stat
  git add docs/TRACEABILITY.md docs/plans/2026-10-06-s1a-lio-and-bridge-core.md
  git commit -m "docs(design): close S1a traceability and carry-over to S1b"
  ```

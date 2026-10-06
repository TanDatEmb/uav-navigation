# S0 Foundations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clear the branch of `main`-only code. Build the shared foundation every later slice depends on:
- time types;
- typed results;
- the structured event recorder and its JSONL sink;
- the tier-(b) config loader;
- the v2 messages;
- a minimal gate and an event-log reader.

**Architecture:** `uavnav_core` is a ROS-free C++23 library that holds all decision-independent infrastructure (spec §6). `uavnav_interfaces` holds the v2 messages (§6.5). `tools/uavnav/` holds Python tooling and the new gate. Packages from `main` that later slices still mine for algorithms stay in the tree, but the default build does not include them.

**Tech Stack:**
- C++23 (gcc 13.3, `std::expected`)
- ROS 2 Jazzy: ament_cmake, rosidl, colcon
- GoogleTest via `ament_cmake_gtest`
- yaml-cpp
- Python 3 (`/usr/bin/python3`, `unittest`)

**Spec:** [docs/architecture/SYSTEM_DESIGN.md](../architecture/SYSTEM_DESIGN.md) §6, §7.1, §7.2 (S0). Decisions: D12, D13, D15, D16, D22, D25, D26 in [DECISIONS.md](../architecture/DECISIONS.md).

## Global Constraints

- Work only in this tree, on branch `rebuild/v2`. Stage files with `git add <path>` only. Never run `git add -A`, `git add .`, `git stash -u`, `git clean` or `git reset --hard` (CLAUDE.local.md).
- Machine limit: build with `MAKEFLAGS=-j3` and `colcon --parallel-workers 2`, under `nice`. Run at most 3 subagents at a time (AGENTS.md §5).
- Every new package name starts with `uavnav_`. Code identifiers, comments and commit messages are in English. Design docs stay in Vietnamese.
- Pure libraries have no ROS dependency (`uavnav_core`). Only ROS shells may depend on `rclcpp` (§7.1).
- Absolute time is a signed `int64` count of nanoseconds. Clock domains are distinct types and must not mix (§6.3).
- Every decision returns `Result<T, Reason>`, where `Reason` is an exhaustive enum. Never encode several meanings in a number or a bool (AGENTS.md §2.2).
- Do not add new state encoded as boolean flags. Do not hold a lock while publishing or while calling a callback (AGENTS.md §2.1, §2.6).
- Tier-(b) config: every key has a unit suffix and min/max bounds. A value outside its bounds rejects startup (§6.2, D22).
- Commits use conventional format `type(scope): description` and end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Before deleting or moving any tracked path, run a reference sweep across **all** file types, not just `package.xml`: CMake, YAML, launch, tests, scripts, docs. Resolve or list every hit. Mass deletions are prepared and dry-run by the agent and executed by the owner (AGENTS.md §6).
- Tests must not depend on tight wall-clock timing (≤ 1 s bounds), because the machine is shared with SITL.
- File-system and parsing APIs that can throw are used through their non-throwing overloads, or wrapped, so that failures surface as `Result` errors and never escape as exceptions.

## Review Focus

1. **Burst of events with a slow or blocked sink.** `emit()` must never block the caller. Overflow increments a drop counter, and an `EventsDropped` record reports the count later. Task 4 pins this.
2. **Sink write failure** (unwritable path, disk full). The recorder keeps running, counts `sink_failures`, and its destructor neither hangs nor throws. Task 4 pins this.
3. **Non-finite event values** (NaN, ±inf). Each JSONL line must still be valid JSON, so these values are written as `null`. Task 5 pins this.
4. **Strings with quotes, backslashes or newlines in event fields.** They must not break the one-record-per-line format. Task 5 pins this.
5. **Malformed config.** A string where a number is expected, `.nan`, `.inf`, a duplicate key, a nested map, or an unknown key must be rejected with the offending key named, never silently defaulted. Task 6 pins this.

---

## File Structure

```text
src/core/uavnav_core/
  CMakeLists.txt, package.xml
  include/uavnav/core/result.hpp          Result alias + ReasonEnum concept
  include/uavnav/core/time.hpp            Duration, TimePoint<Tag>, 4 domains, TimeSnapshot
  include/uavnav/core/event.hpp           Component, EventValue, EventIdentity, EventRecord, builder
  include/uavnav/core/event_recorder.hpp  EventSink, SinkError, RecorderStats, EventRecorder
  include/uavnav/core/jsonl_sink.hpp      JsonlSink (file sink, JSON encoding)
  include/uavnav/core/config.hpp          Unit, ParamSpec, ConfigError, ParamValues, load_params*
  include/uavnav/core/limits.hpp          tier-(a) constants owned by core
  src/time.cpp  src/event_recorder.cpp  src/jsonl_sink.cpp  src/config.cpp
  test/test_result.cpp  test/test_time.cpp  test/test_event_recorder.cpp
  test/test_jsonl_sink.cpp  test/test_config.cpp
src/core/uavnav_interfaces/
  CMakeLists.txt, package.xml, msg/*.msg (8 files)
tools/uavnav/
  __init__.py  events.py  gate.sh  tests/__init__.py  tests/test_events.py
Makefile                                  rewritten: help/build/test/gate/clean
```

---

### Task 1: Clear `main`-only code and set up the new build entry points

Spec §7.1, §7.2 S0, D13, D15. Delete only what no kept package depends on. These are kept, because S1 and S3 extract algorithms from them:
- `navigation_common`
- `navigation_mission`
- `navigation_world_model`
- `navigation_planning`
- `navigation_planning_backend`
- `navigation_mapping` (`navigation_planning_backend` depends on it)
- `fast_lio_core` and the three vendor packages
- `uav_simulation`, `uav_description`
- `config/`, `tools/simulation/`, `tools/datasets/`, `tools/setup.sh`

**Two lessons from the first execution attempt are now built into this task:**
- References between packages are not only `package.xml` dependencies. Configs, launch files, tests and scripts also refer to files by path. A reference sweep across all file types comes before any deletion (Step 1).
- Mass deletion is an irreversible local action. The permission classifier blocks it for agents. The agent prepares and dry-runs the deletion; the **owner** runs it (Step 2).

**Nothing is moved in S0.** The vendor and sim directories stay where they are. Moving them breaks path references (for example `tools/simulation/run_px4_mid360.sh` uses `src/uav_simulation/models`, and the `uav_description` test uses `ROOT.parent`). It also leaves stale source paths in the shared `build/` CMake caches. Each move happens in the slice that rebuilds the package (S1: `ikfom_vendor`, `ikd_tree_vendor`, `uav_simulation`, `uav_description`; S3: `rog_map_vendor`), using `--cmake-clean-cache` on its first rebuild.

**Files:**
- Delete (Step 2, owner-run):
  - `src/runtime/`, `src/execution/`, `src/px4/`
  - `src/estimation/fast_lio_ros/`, `src/estimation/fast_lio_tools/`
  - `src/contracts/navigation_contracts/`, `src/navigation_bringup/`
  - `tools/runtime/`, `tools/tests/`, `tools/refactor/`, `tools/benchmarks/`
  - `tools/gate.sh`, `tools/data.py`
  - `tools/validate_runtime_safety_ledger.py`, `tools/check_mission_authority_cut.py`
  - `tools/check_dependency_direction.py`, `tools/verify_baseline_migration.py`
  - the `evidence` folder under the `docs` directory
- Move: `src/runtime/navigation_runtime/config/planner.yaml` → `config/runtime/planner.yaml`. Two kept packages read this file.
- Create:
  - `tools/uavnav/__init__.py` (empty)
  - `tools/uavnav/tests/__init__.py` (empty)
  - `tools/uavnav/gate.sh`
  - the one-shot S0 prune script (since removed; see commit `f618909`)
- Modify:
  - `src/mapping/navigation_mapping/CMakeLists.txt`, `src/planning/navigation_planning_backend/CMakeLists.txt`: change every path to `planner.yaml` into `${CMAKE_CURRENT_SOURCE_DIR}/../../../config/runtime/planner.yaml`.
  - `src/uav_description/test/test_sensor_frames_contract.py`: remove the assertions that read `navigation_bringup/launch/fast_lio.launch.py`. Keep the standalone-launch assertions, and rename the test to `test_standalone_launch_mount_policy`.
  - `Makefile` (full rewrite).
  - `README.md`: remove the deleted `make` targets (`run`, `sim`, `replay`, `dataset-check`, …).
  - `.agents/skills/build-and-test/SKILL.md`: point it to `make build|test|gate`.

**Interfaces:**
- Produces:
  - `make build`, `make test`, `make gate`, `make clean`, with variable `PKGS` (default `uavnav_core uavnav_interfaces`)
  - `tools/uavnav/gate.sh [static|python|ros|all]`, whose last line is `GATE_RESULT=PASS|FAIL`

- [ ] **Step 1: Reference sweep, then resolve every hit before deleting anything**

  Run:

  ```bash
  DEL='src/runtime|src/execution|src/px4/|fast_lio_ros|fast_lio_tools|navigation_contracts|navigation_bringup|navigation_runtime|navigation_execution|px4_navigation_external_mode|px4_odometry_bridge|tools/runtime|tools/tests|tools/gate\.sh|tools/data\.py|validate_runtime_safety_ledger|check_mission_authority_cut|check_dependency_direction|verify_baseline_migration|docs/evidence'
  grep -rnE "$DEL" src/common src/contracts/navigation_mission src/mapping src/planning src/estimation/fast_lio_core \
    src/estimation/ikfom_vendor src/estimation/ikd_tree_vendor src/uav_simulation src/uav_description src/external/livox_ros_driver2 \
    config tools/simulation tools/datasets tools/setup.sh tools/check_documentation.py .agents README.md
  ```

  Resolve each hit as below, then rerun. The expected remaining hits are text-only (docs and comments) and are listed here:

  | Hit | Resolution |
  |---|---|
  | `navigation_mapping`/`navigation_planning_backend` CMake → `planner.yaml` | Move the file and edit the paths (see Files) |
  | `uav_description` test → `navigation_bringup` launch | Edit the test (see Files) |
  | `.agents/skills/build-and-test`, `README.md` | Rewrite (see Files) |
  | `config/runtime/{mapping,sim,external_mode}.yaml` keys named after old nodes | Keep. They are reference inputs for S1/S3 and no kept build reads their node keys |
  | `navigation_planning_backend/UPSTREAM.md`, `tools/simulation/README.md`, `tools/simulation/run_px4_mid360.sh` (echo text) | Keep. Text only; S1 rewrites the SITL launch |

  Then confirm no build or test file of a kept package reads a deleted path:

  `grep -rlE "$DEL" src --include=package.xml --include=CMakeLists.txt --include='*.py' --include='*.launch.py' | grep -vE '^src/(runtime|execution|px4|navigation_bringup|contracts/navigation_contracts|estimation/fast_lio_(ros|tools))/'`

  Expected: no output.

- [ ] **Step 2: Prepare the deletion script, dry-run it, and hand it to the owner (OWNER ACTION)**

  Write the one-shot S0 prune script (since removed; see commit `f618909`) as follows:
  - `set -euo pipefail`; run from the repo root;
  - one `git rm -r --quiet "$@" --` command per path from the Delete list;
  - it forwards its arguments, so passing `-n` performs a dry run.

  The agent runs only the dry run:

  Run the prune script with `-n`.

  Expected: one `rm '<path>'` line per tracked file under the listed paths, and nothing else. Then the agent **stops and asks the owner** to run the prune script. Every deleted file is tracked and stays recoverable from `main` and from `HEAD~`.

  After the owner confirms, the agent runs `git status --short | grep -c '^D '`. Expected: a non-zero count, and `git status` shows no deletion outside the listed paths.

- [ ] **Step 3: Rewrite `Makefile`**

  - Targets: `help`, `setup` (keeps `tools/setup.sh`), `build`, `test`, `gate`, `clean`.
  - `build` sources `/opt/ros/jazzy/setup.bash`, then runs:

    `nice -n 10 env MAKEFLAGS=-j3 colcon build --packages-up-to $(PKGS) --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
  - `test` runs `colcon test --packages-select $(PKGS) --parallel-workers 2`, then `colcon test-result --verbose`.
  - `gate` runs `tools/uavnav/gate.sh all`.
  - `clean` prints the paths it would remove (`log/`) and asks the user to delete them. It deletes nothing itself, because `build/` and `install/` are a shared incremental build (CLAUDE.local.md).

- [ ] **Step 4: Write `tools/uavnav/gate.sh`**

  Use `set -euo pipefail`, run from the repo root, and use the Python at `${PYTHON:-/usr/bin/python3}`. Three stages:
  - `static`: `git diff --check`, `git diff --cached --check`, `$PYTHON tools/check_documentation.py . docs`.
  - `python`: if `tools/uavnav/tests` contains no `test_*.py`, print `gate: python: no tests` and pass. Python 3.12 `unittest` exits with status 1 when it finds no tests. Otherwise run `$PYTHON -m unittest discover -s tools/uavnav/tests -t . -v`.
  - `ros`: `make build && make test`.

  `all` runs all three stages in order. The script prints `GATE_RESULT=PASS` only when every selected stage passes, and `GATE_RESULT=FAIL` otherwise (through an `EXIT` trap).

- [ ] **Step 5: Run the static and python stages**

  Run: `tools/uavnav/gate.sh static && tools/uavnav/gate.sh python`

  Expected: both stages end with `GATE_RESULT=PASS`, and the python stage prints `gate: python: no tests`.

- [ ] **Step 6: Commit**

  ```bash
  git add -u src tools docs Makefile README.md .agents/skills/build-and-test/SKILL.md
  git add config/runtime/planner.yaml tools/uavnav Makefile
  git commit -m "build!: drop main-only packages and tooling, add rebuild gate"
  ```

---

### Task 2: `uavnav_core` package with `Result` and the reason convention

Spec §6, AGENTS.md §2.2.

**Files:**
- Create: `src/core/uavnav_core/CMakeLists.txt`, `package.xml`, `include/uavnav/core/result.hpp`, `test/test_result.cpp`

**Interfaces:**
- Produces (namespace `uavnav`):

  ```cpp
  template <class T, class R> using Result = std::expected<T, R>;
  template <class E> concept ReasonEnum = std::is_enum_v<E> && requires(E e) {
      { to_string(e) } -> std::same_as<std::string_view>; };   // found by ADL
  ```

  - CMake target `uavnav_core::uavnav_core`. It is an INTERFACE library in this task; Task 3 turns it into a STATIC library when it adds the first source file.
  - It uses `cxx_std_23` and `-Wall -Wextra -Wpedantic -Werror`, and links `yaml-cpp::yaml-cpp` (`find_package(yaml-cpp REQUIRED)`; the target name was verified on this host).
  - `package.xml` dependencies: `ament_cmake`, `yaml-cpp`, and `ament_cmake_gtest` (test only).

- [ ] **Step 1: Write the failing test `test/test_result.cpp`**

  ```cpp
  namespace sample { enum class Reason : std::uint8_t { kOk, kStale };
    constexpr std::string_view to_string(Reason r) { return r == Reason::kOk ? "OK" : "STALE"; } }
  static_assert(uavnav::ReasonEnum<sample::Reason>);
  static_assert(!uavnav::ReasonEnum<int>);
  TEST(Result, CarriesValueOrReason) {
    uavnav::Result<int, sample::Reason> ok = 3;
    uavnav::Result<int, sample::Reason> bad = std::unexpected(sample::Reason::kStale);
    EXPECT_EQ(*ok, 3);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(to_string(bad.error()), "STALE");
  }
  ```

- [ ] **Step 2: Run it and confirm it fails**

  Run: `make build PKGS=uavnav_core`

  Expected: the build fails with `uavnav/core/result.hpp: No such file`.

- [ ] **Step 3: Write `package.xml`, `CMakeLists.txt` (INTERFACE target) and `result.hpp`**

- [ ] **Step 4: Run the test and confirm it passes**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core`

  Expected: `test_result` passes, and the summary reads `0 errors, 0 failures`.

- [ ] **Step 5: Commit**

  ```bash
  git add src/core/uavnav_core
  git commit -m "feat(core): add uavnav_core with Result and reason convention"
  ```

---

### Task 3: Clock-domain time types

Spec §6.3, D25.

**Files:**
- Create: `include/uavnav/core/time.hpp`, `src/time.cpp`, `test/test_time.cpp` (under `src/core/uavnav_core/`)
- Modify: `CMakeLists.txt` (make the library STATIC and add `src/time.cpp`)

**Interfaces:**
- Produces (namespace `uavnav::time`):

  ```cpp
  struct Duration { std::int64_t ns{0}; auto operator<=>(const Duration&) const = default; };
  constexpr Duration nanoseconds(std::int64_t), milliseconds(std::int64_t), seconds(std::int64_t);
  constexpr double to_seconds(Duration);                 // only at calculation boundaries
  Duration operator+(Duration, Duration); Duration operator-(Duration, Duration);
  template <class Tag> struct TimePoint {
    std::int64_t ns{0};
    auto operator<=>(const TimePoint&) const = default;
    friend Duration operator-(TimePoint a, TimePoint b);       // same Tag only
    friend TimePoint operator+(TimePoint t, Duration d);
    friend TimePoint operator-(TimePoint t, Duration d);
  };
  struct SensorTag; struct RosTag; struct SteadyTag; struct Px4Tag;
  using SensorTime = TimePoint<SensorTag>; using RosTime = TimePoint<RosTag>;
  using SteadyTime = TimePoint<SteadyTag>; using Px4Time = TimePoint<Px4Tag>;
  struct TimeSnapshot { SteadyTime steady; RosTime ros; };
  SteadyTime steady_now() noexcept;                      // std::chrono::steady_clock, in time.cpp
  ```

  There are no implicit conversions between domains, and none to or from a raw integer except through `.ns`.

- [ ] **Step 1: Write the failing test `test/test_time.cpp`**

  ```cpp
  using namespace uavnav::time;
  template <class A, class B> concept Subtractable = requires(A a, B b) { a - b; };
  static_assert(Subtractable<RosTime, RosTime>);
  static_assert(!Subtractable<RosTime, SensorTime>);
  static_assert(!Subtractable<SteadyTime, Px4Time>);
  static_assert(!std::is_convertible_v<SensorTime, RosTime>);
  static_assert(!std::is_convertible_v<std::int64_t, SteadyTime>);
  TEST(Time, ArithmeticWithinDomain) {
    const RosTime a{1'000'000'000}; const RosTime b = a + milliseconds(250);
    EXPECT_EQ((b - a).ns, 250'000'000);
    EXPECT_EQ((a - b).ns, -250'000'000);           // ordering preserved, no clamping
    EXPECT_DOUBLE_EQ(to_seconds(b - a), 0.25);
    EXPECT_LT(a, b);
  }
  TEST(Time, SteadyNowIsMonotonic) {
    const SteadyTime t0 = steady_now(); const SteadyTime t1 = steady_now();
    EXPECT_LE(t0, t1);
  }
  ```

- [ ] **Step 2: Run it and confirm it fails**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error, `uavnav/core/time.hpp: No such file`.

- [ ] **Step 3: Implement `time.hpp` and `time.cpp` with the signatures above**

- [ ] **Step 4: Run the test and confirm it passes**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core`

  Expected: `test_time` and `test_result` pass.

- [ ] **Step 5: Commit**

  ```bash
  git add src/core/uavnav_core
  git commit -m "feat(core): add clock-domain time types"
  ```

---

### Task 4: Event record and non-blocking `EventRecorder`

Spec §6.1, D25, AGENTS.md §2.3 and §2.6. Covers Review Focus items 1 and 2.

**Files:**
- Create (under `src/core/uavnav_core/`):
  - `include/uavnav/core/event.hpp`
  - `include/uavnav/core/event_recorder.hpp`
  - `include/uavnav/core/limits.hpp`
  - `src/event_recorder.cpp`
  - `test/test_event_recorder.cpp`
- Modify: `CMakeLists.txt` (add source and test; link `Threads::Threads`)

**Interfaces:**
- Consumes: `uavnav::Result`.
- Produces (namespace `uavnav::events`):

  ```cpp
  enum class Component : std::uint8_t { kCore, kLio, kPx4Bridge, kMapping, kPlanner, kSupervisor, kPx4Mode };
  constexpr std::string_view to_string(Component);       // "core","lio","px4_bridge","mapping","planner","supervisor","px4_mode"
  struct EventValue { std::string_view key; double value; };   // key: string literal only
  struct EventIdentity { std::uint64_t mission_id{0}; std::uint32_t lio_epoch{0};
                         std::uint64_t request_id{0}; std::uint64_t bundle_id{0}; std::uint64_t world_revision{0}; };
  struct EventRecord {
    std::int64_t t_steady_ns{0}; std::int64_t t_ros_ns{0};
    Component component{Component::kCore};
    std::string_view event, state_before, state_after, reason;   // static storage only
    EventIdentity identity;
    std::array<EventValue, limits::kMaxEventValues> values{}; std::uint8_t value_count{0};
    bool add_value(std::string_view key, double v) noexcept;   // false when full (record unchanged)
  };
  enum class SinkError : std::uint8_t { kIo };
  constexpr std::string_view to_string(SinkError);
  class EventSink { public: virtual ~EventSink() = default;
    virtual Result<void, SinkError> write(std::span<const EventRecord> batch) = 0; };
  struct RecorderStats { std::uint64_t emitted, written, dropped, sink_failures; };
  class EventRecorder {
   public:
    explicit EventRecorder(std::unique_ptr<EventSink> sink, std::size_t capacity = limits::kEventRingCapacity);
    ~EventRecorder();                       // stops the writer thread after draining; never throws
    bool emit(const EventRecord& record) noexcept;   // false = dropped; never waits on the sink
    void flush();                           // blocks until everything emitted so far reached the sink
    RecorderStats stats() const noexcept;
  };
  ```

- `limits.hpp` defines these tier-(a) constants in namespace `uavnav::limits`, each with a derivation comment:
  - `kMaxEventValues = 16` (spec §6.1);
  - `kEventRingCapacity = 4096` (about 4 s of 1 kHz bursts);
  - `kEventWriterPeriod = milliseconds(20)`.
- Behaviour:
  - `emit` copies the record into a bounded ring under a short mutex. When the ring is full it increments `dropped` and returns `false`.
  - The writer thread waits on a condition variable with timeout `kEventWriterPeriod` and swaps the ring out under the lock.
  - **Outside the lock**, the writer:
    - writes an `EventsDropped` record (component `kCore`, event `"EventsDropped"`, value `count`) if any drops happened since its last write;
    - then writes the batch.
  - A sink error increments `sink_failures`, discards that batch, and the writer continues.

- [ ] **Step 1: Write the failing tests `test/test_event_recorder.cpp`**

  Use a test sink, `FakeSink`, whose `write` can be told to block on a latch or to fail, and which records every batch.

  ```cpp
  TEST(EventRecorder, DeliversRecordsInOrder) { /* emit 3 records "A","B","C"; flush(); sink saw events A,B,C; stats.written==3 */ }
  TEST(EventRecorder, AddValueRejectsSeventeenth) { /* 16 add_value() true, 17th false, value_count==16 */ }
  TEST(EventRecorder, FullRingDropsWithoutBlockingAndReportsCount) {
    // capacity 4, sink blocked on a latch: run 10 emits inside std::async and require
    // future.wait_for(2s) == ready (detects blocking without a tight wall-clock bound);
    // at least 6 return false; stats.dropped == number of false returns;
    // release latch, flush(): sink received an "EventsDropped" record whose value "count" == stats.dropped
  }
  TEST(EventRecorder, SinkFailureIsCountedAndRecorderKeepsRunning) {
    // sink fails the first write, then succeeds: emit A, flush, emit B, flush;
    // stats.sink_failures == 1; sink saw B; destructor returns (test completes)
  }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error, `uavnav/core/event_recorder.hpp: No such file`.

- [ ] **Step 3: Implement `event.hpp`, `limits.hpp`, `event_recorder.hpp` and `event_recorder.cpp`**

- [ ] **Step 4: Run the tests and confirm they pass, five times in a row**

  Run: `make build PKGS=uavnav_core && for i in 1 2 3 4 5; do make test PKGS=uavnav_core || break; done`

  Expected: every run passes. The repeat checks that the thread timing is stable. No test may assert a wall-clock bound tighter than 1 s, because the machine is often loaded by SITL.

- [ ] **Step 5: Commit**

  ```bash
  git add src/core/uavnav_core
  git commit -m "feat(core): add structured event record and non-blocking recorder"
  ```

---

### Task 5: JSONL sink

Spec §6.1. Covers Review Focus items 3 and 4.

**Files:**
- Create (under `src/core/uavnav_core/`): `include/uavnav/core/jsonl_sink.hpp`, `src/jsonl_sink.cpp`, `test/test_jsonl_sink.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `EventSink`, `EventRecord`, `SinkError`, `Result` (Task 4).
- Produces:

  ```cpp
  namespace uavnav::events {
  class JsonlSink final : public EventSink {
   public:
    static Result<std::unique_ptr<JsonlSink>, SinkError> open(const std::filesystem::path& file);  // append mode, creates parent dirs;
                                     // uses the std::error_code overloads of <filesystem>, so no exception escapes (kIo instead)
    Result<void, SinkError> write(std::span<const EventRecord> batch) override;           // one line per record, flushed per batch
  };
  std::string to_json_line(const EventRecord& record);   // no trailing newline
  }  // namespace uavnav::events
  ```

- Line format: a single JSON object with keys in this order:
  - `t_steady_ns`, `t_ros_ns`, `component`, `event`, `state_before`, `state_after`, `reason`;
  - `mission_id`, `lio_epoch`, `request_id`, `bundle_id`, `world_revision`;
  - `values`, an object mapping each key to a number, or to `null` when the value is not finite.

  Strings are escaped per RFC 8259: `"`, `\`, and control characters (written as `\n`, `\t` or `\u00XX`). The Python reader in Task 8 parses this format.

- [ ] **Step 1: Write the failing tests `test/test_jsonl_sink.cpp`**

  ```cpp
  TEST(JsonlSink, EncodesAllFieldsInOrder) {
    // record: steady 5, ros 7, kSupervisor, event "Commit", before "RUNNING", after "RUNNING", reason "OK",
    // identity {1,2,3,4,5}, values {("prefix_s",1.5)}
    EXPECT_EQ(to_json_line(r), R"({"t_steady_ns":5,"t_ros_ns":7,"component":"supervisor","event":"Commit",)"
      R"("state_before":"RUNNING","state_after":"RUNNING","reason":"OK","mission_id":1,"lio_epoch":2,)"
      R"("request_id":3,"bundle_id":4,"world_revision":5,"values":{"prefix_s":1.5}})");
  }
  TEST(JsonlSink, NonFiniteValuesBecomeNull) { /* NaN and +inf -> "values":{"a":null,"b":null} */ }
  TEST(JsonlSink, EscapesQuotesBackslashAndNewline) { /* reason "a\"b\\c\nd" -> "reason":"a\"b\\c\nd" escaped; line has no raw '\n' */ }
  TEST(JsonlSink, WritesOneLinePerRecordAndAppends) { /* open tmp file, write 2, reopen, write 1 -> file has 3 lines */ }
  TEST(JsonlSink, OpenFailsOnUnwritablePath) { /* open("/proc/uavnav_forbidden/x.jsonl") -> error kIo */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error, `uavnav/core/jsonl_sink.hpp: No such file`.

- [ ] **Step 3: Implement `jsonl_sink.hpp` and `jsonl_sink.cpp`**

  Format numbers with `std::to_chars` using shortest round-trip form for doubles.

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core`

  Expected: all `uavnav_core` tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add src/core/uavnav_core
  git commit -m "feat(core): add JSONL event sink"
  ```

---

### Task 6: Tier-(b) config loader

Spec §6.2, D22. Covers Review Focus item 5.

**Files:**
- Create (under `src/core/uavnav_core/`): `include/uavnav/core/config.hpp`, `src/config.cpp`, `test/test_config.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Result`.
- Produces (namespace `uavnav::config`):

  ```cpp
  enum class Unit : std::uint8_t { kNone, kSeconds, kMeters, kMetersPerSecond, kMetersPerSecond2,
                                   kMetersPerSecond3, kRadians, kRadiansPerSecond, kHertz };
  constexpr std::string_view suffix(Unit);   // "", "_s", "_m", "_mps", "_mps2", "_mps3", "_rad", "_rad_s", "_hz"
  struct ParamSpec { std::string_view key; Unit unit; double min; double max; };
  struct ConfigError {
    enum class Kind : std::uint8_t { kFileUnreadable, kParse, kNotFlatMap, kDuplicateKey, kUnknownKey,
                                     kMissingKey, kWrongType, kNotFinite, kOutOfRange, kBadSpec };
    Kind kind; std::string key; std::string detail;   // detail names value and bounds for kOutOfRange
  };
  constexpr std::string_view to_string(ConfigError::Kind);
  using ParamValues = std::map<std::string, double, std::less<>>;
  Result<ParamValues, ConfigError> load_params(std::string_view yaml_text, std::span<const ParamSpec> specs);
  Result<ParamValues, ConfigError> load_params_file(const std::filesystem::path& file, std::span<const ParamSpec> specs);
  ```

- Rules, checked in this order:
  1. Each spec is valid: the key ends with `suffix(unit)` (for `kNone`, it must not end with any non-empty suffix), `min <= max`, and both bounds are finite. Otherwise `kBadSpec`.
  2. The YAML parses (`kParse`) and is a flat map of scalars (`kNotFlatMap`).
  3. There are no duplicate keys (`kDuplicateKey`). yaml-cpp accepts duplicates silently, so walk the map node and detect them yourself.
  4. Every YAML key is declared in the specs (`kUnknownKey`), and every spec key is present in the YAML (`kMissingKey`).
  5. Every value converts to `double` (`kWrongType`), is finite (`kNotFinite`, which also catches `.nan` and `.inf`), and lies inside `[min, max]` (`kOutOfRange`).

- [ ] **Step 1: Write the failing tests `test/test_config.cpp`**

  ```cpp
  constexpr ParamSpec kSpecs[] = {{"cruise_speed_mps", Unit::kMetersPerSecond, 0.5, 5.0},
                                  {"lio_recovery_timeout_s", Unit::kSeconds, 1.0, 60.0}};
  TEST(Config, LoadsValidFlatMap) { /* "cruise_speed_mps: 3.0\nlio_recovery_timeout_s: 10" -> values 3.0, 10.0 */ }
  TEST(Config, RejectsOutOfRangeNamingKeyAndBounds) { /* cruise 7.0 -> kOutOfRange, key "cruise_speed_mps", detail contains "7", "0.5", "5" */ }
  TEST(Config, RejectsMissingKey) { /* only cruise -> kMissingKey "lio_recovery_timeout_s" */ }
  TEST(Config, RejectsUnknownKey) { /* extra "cruise_sped_mps: 1" -> kUnknownKey "cruise_sped_mps" */ }
  TEST(Config, RejectsDuplicateKey) { /* cruise twice -> kDuplicateKey */ }
  TEST(Config, RejectsWrongTypeAndNonFinite) { /* "fast" -> kWrongType; ".nan" -> kNotFinite; ".inf" -> kNotFinite */ }
  TEST(Config, RejectsNestedMap) { /* "planner:\n  cruise_speed_mps: 1" -> kNotFlatMap */ }
  TEST(Config, RejectsSpecWithWrongUnitSuffix) { /* {"cruise_speed", kMetersPerSecond, 0, 1} -> kBadSpec */ }
  TEST(Config, FileUnreadable) { /* load_params_file("/nonexistent.yaml") -> kFileUnreadable */ }
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `make build PKGS=uavnav_core`

  Expected: compile error, `uavnav/core/config.hpp: No such file`.

- [ ] **Step 3: Implement `config.hpp` and `config.cpp`**

- [ ] **Step 4: Run the tests and confirm they pass**

  Run: `make build PKGS=uavnav_core && make test PKGS=uavnav_core`

  Expected: all `uavnav_core` tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add src/core/uavnav_core
  git commit -m "feat(core): add bounded tier-b config loader"
  ```

---

### Task 7: v2 messages, `uavnav_interfaces`

Spec §6.5, plus §2.2, §3, §4 and §2.6 for the enum values used here.

**Files:**
- Create under `src/core/uavnav_interfaces/`: `CMakeLists.txt`, `package.xml`, and `msg/`:
  - `LioState.msg`, `LioHealth.msg`, `LioOdometry.msg`, `Alignment.msg`
  - `NavCommand.msg`, `EventRecord.msg`, `Waypoint.msg`, `MissionDefinition.msg`

**Interfaces:**
- Produces: ROS messages `uavnav_interfaces/msg/*` with exactly the following fields. All `stamp` fields are `builtin_interfaces/Time`. The frame is given per message.

```text
LioState      stamp(sensor time) uint32 epoch uint32 reset_counter uint8 lio_state
              geometry_msgs/Pose pose  geometry_msgs/Vector3 velocity  geometry_msgs/Vector3 angular_velocity
              float64[36] pose_covariance  float64[9] velocity_covariance
              geometry_msgs/Vector3 reset_delta_position  geometry_msgs/Vector3 reset_delta_velocity  float64 reset_delta_yaw
              # frame: lio_odom (pose), base_link (velocity, angular_velocity)
LioHealth     stamp uint8 state uint32 epoch uint16 reason string reason_name
              float64 translation_min_eigenvalue float64 rotation_min_eigenvalue float64 correction_age_s
              float64 output_error_attitude_rad float64 output_error_velocity_mps float64 output_error_position_m
              constants: uint8 INITIALIZING=0 TRACKING=1 DEGRADED=2 LOST=3 RESTARTING=4
LioOdometry   stamp(scan time) uint32 epoch uint32 reset_counter geometry_msgs/Pose pose geometry_msgs/Vector3 velocity
              float64[36] pose_covariance float64[9] velocity_covariance          # frame: lio_odom
Alignment     stamp uint8 state float64 x_m float64 y_m float64 z_m float64 yaw_rad
              float64 raw_x_m float64 raw_y_m float64 raw_z_m float64 raw_yaw_rad
              float64 age_s float64 residual_position_m float64 residual_yaw_rad
              constants: uint8 INIT=0 VALID=1 FROZEN=2 INVALID=3              # T maps lio_odom -> PX4 local
NavCommand    stamp(ros time) int64 sample_time_ns uint32 lio_epoch uint64 bundle_id uint8 phase
              geometry_msgs/Vector3 position geometry_msgs/Vector3 velocity geometry_msgs/Vector3 acceleration
              float64 yaw_rad float64 yaw_rate_rad_s int64 lease_ns
              constants: uint8 MAIN=0 BACKUP=1 BRAKE=2 HOLD=3                 # frame: lio_odom
EventRecord   int64 t_steady_ns int64 t_ros_ns string component string event string state_before string state_after
              string reason uint64 mission_id uint32 lio_epoch uint64 request_id uint64 bundle_id uint64 world_revision
              string[] value_keys float64[] values
Waypoint      float64 x float64 y float64 z uint8 behavior float64 acceptance_radius_m float64 yaw_rad
              constants: uint8 STOP=0 PASS_THROUGH=1      # x,y,z = lat_deg,lon_deg,alt_m when frame GPS; NaN yaw_rad = unspecified
MissionDefinition uint64 mission_id uint8 frame uint8 yaw_mode float64 yaw_rate_rad_s Waypoint[] waypoints
              constants: uint8 FRAME_GPS=0 FRAME_LOCAL=1 YAW_LOCK=0 YAW_SEGMENT=1
```

- [ ] **Step 1: Write the eight `.msg` files, `package.xml` (`rosidl_default_generators`, `builtin_interfaces`, `geometry_msgs`, member of `rosidl_interface_packages`) and `CMakeLists.txt`**

- [ ] **Step 2: Build and inspect a message**

  Run: `make build PKGS=uavnav_interfaces && source install/setup.bash && ros2 interface show uavnav_interfaces/msg/NavCommand`

  Expected: the output lists `uint8 MAIN=0`, `int64 lease_ns`, `float64 yaw_rate_rad_s`.

- [ ] **Step 3: Build the default package set together**

  Run: `make build && make test`

  Expected: `uavnav_core` and `uavnav_interfaces` both build, and all tests pass.

- [ ] **Step 4: Commit**

  ```bash
  git add src/core/uavnav_interfaces
  git commit -m "feat(interfaces): add v2 message contracts"
  ```

---

### Task 8: Event-log reader in Python, gate wired end to end, traceability

Spec §6.1, §7.4. This task closes S0.

**Files:**
- Create: `tools/uavnav/events.py`, `tools/uavnav/tests/test_events.py`
- Modify: `docs/TRACEABILITY.md` (S0 rows)

**Interfaces:**
- Consumes: the JSONL line format from Task 5.
- Produces, in `tools/uavnav/events.py`:
  - `load(path) -> list[dict]`. It raises `ValueError(f"{path}:{line_no}: …")` on a malformed line.
  - `count_by_reason(records) -> dict[tuple[str, str, str], int]`, keyed by `(component, event, reason)`.
  - `timeline(records) -> list[str]`, one line per record, sorted by `t_steady_ns`, formatted as `"<t_steady_s:.3f> <component> <event> <state_before>-><state_after> <reason>"`.
  - A CLI: `python3 -m tools.uavnav.events <file.jsonl> [--counts|--timeline]`.

- [ ] **Step 1: Write the failing tests `tools/uavnav/tests/test_events.py`**

  ```python
  LINE = ('{"t_steady_ns":2000000000,"t_ros_ns":0,"component":"supervisor","event":"Handover",'
          '"state_before":"RUNNING","state_after":"HANDED_OVER","reason":"NO_PATH_TIMEOUT","mission_id":1,'
          '"lio_epoch":1,"request_id":0,"bundle_id":0,"world_revision":0,"values":{"waited_s":15.0,"x":null}}')
  def test_load_parses_null_values(self): # write LINE to tmp file; load()[0]["values"] == {"waited_s":15.0,"x":None}
  def test_load_reports_line_number_on_garbage(self): # second line "{oops" -> ValueError containing ":2:"
  def test_count_by_reason(self): # 2 identical records -> {("supervisor","Handover","NO_PATH_TIMEOUT"): 2}
  def test_timeline_sorted_and_formatted(self): # records at 2.0 s and 1.0 s -> first line starts "1.000 "
  ```

- [ ] **Step 2: Run them and confirm they fail**

  Run: `tools/uavnav/gate.sh python`

  Expected: `ModuleNotFoundError: tools.uavnav.events`, then `GATE_RESULT=FAIL`.

- [ ] **Step 3: Implement `events.py`, standard library only**

- [ ] **Step 4: Run the full gate**

  Run: `tools/uavnav/gate.sh all`

  Expected: the static, python and ros stages pass, and the last line is `GATE_RESULT=PASS`.

- [ ] **Step 5: Update `docs/TRACEABILITY.md`**

  - Set these "Hạ tầng" rows to `done`, with the test names as evidence:
    - "Thay `tools/gate.sh` …": `tools/uavnav/gate.sh all`
    - "Config ba tầng" (tier b): `test_config`
    - "Kiểu thời gian": `test_time`
    - "Message v2": `uavnav_interfaces` build
  - Set "Event log + script KPI" to `doing`, evidence `test_event_recorder`, `test_jsonl_sink`, `test_events`. The KPI part is S5.
  - Set P8 to `done`.
  - Fill the WP column with `S0-T<n>`.

  Run: `python3 tools/check_documentation.py . docs`

  Expected: `PASS`.

- [ ] **Step 6: Commit**

  ```bash
  git add tools/uavnav/events.py tools/uavnav/tests/test_events.py docs/TRACEABILITY.md
  git commit -m "feat(tools): add event log reader and close S0 traceability"
  ```

---

## After S0

The plans for S1 (`lio` and `px4_bridge`), S2, S3, S4 and S5 are written one at a time, each after the previous slice is done. Each new plan starts from the actual result of the slice before it, as D16 requires. Before an S1 plan is written, the S0 result is reviewed with the owner.

### Carry-over to S1

Obligations found by the S0 final whole-branch review that S0 deliberately did not close. The S1 plan must schedule each one or record why not.

- Range-check external stamps and config values before constructing `TimePoint`/`Duration`; signed overflow is UB (`src/core/uavnav_core/include/uavnav/core/time.hpp` contract). Applies to every S1 adapter.
- `EventsDropped` notice is written before the older buffered batch, so the log order is inverted; swap the order or document it (`src/core/uavnav_core/src/event_recorder.cpp`).
- `EventRecorder` accepts capacity 0 silently; reject it or assert (`src/core/uavnav_core/src/event_recorder.cpp`).
- Add an N-thread emit stress test (emitted + dropped == N·M, written == emitted, no time bound) and get an owner-approved TSan run before the first multi-threaded emitter (`src/core/uavnav_core/test/test_event_recorder.cpp`).
- `load_params_file` has no size cap; suggest 1 MiB, reported as `kFileUnreadable` (`src/core/uavnav_core/src/config.cpp`).
- The depth-guard test relies on yaml-cpp >= 0.8 behaviour (`src/core/uavnav_core/test/test_config.cpp`).
- Duplicate keys: `add_value` allows a repeated key while Python `json` keeps the last one; reject in `add_value` or in the reader (`src/core/uavnav_core/src/event_recorder.cpp`, `tools/uavnav/events.py`).
- Replace free-text `string_view` reasons with a typed builder (`set_reason(ReasonEnum auto)`, D25); the builder named in this plan's File Structure was never built (`src/core/uavnav_core/include/uavnav/core/event_recorder.hpp`).
- `ParamValues` is a string-keyed map whose `.at()` throws on a typo; S1 must bind it to a typed struct per §6.2 and D25 (`src/core/uavnav_core/include/uavnav/core/config.hpp`).
- Add a C++ to Python golden JSONL round-trip test; key order is currently pinned twice by hand (`src/core/uavnav_core/test/test_event_recorder.cpp`, `tools/uavnav/tests/test_events.py`).
- `LioOdometry` has no `quality` field although §4.1 says covariance and quality come from LIO; needs an owner decision or an open question (`src/core/uavnav_interfaces/msg/LioOdometry.msg`).
- The clock domain of `NavCommand.sample_time_ns` and of the `LioHealth`/`Alignment` stamps is unstated (D25); annotate the messages or open an O* item (`src/core/uavnav_interfaces/msg/NavCommand.msg`).
- `MissionDefinition` has no mission-level yaw for Y1 (§2.5) and the GPS altitude datum is unspecified; S2 needs both (`src/core/uavnav_interfaces/msg/MissionDefinition.msg`).
- `tools/simulation/README.md` (lines 4, 24 and 26) still mentions the removed `make sim`, `sim-check` and `tools/runtime`; S1 rewrites the SITL launch and this file.
- Commits `5ad488e` and `d8ef690` have a `Co-Authored-By` line glued to the subject; the owner may reword them.

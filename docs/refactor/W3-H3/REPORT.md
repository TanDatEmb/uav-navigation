# W3-H3 early fixes — implementation report

## Verdict

Six finding reviews were completed on branch `refactor/W3-H3` from `origin/main` at `432dc94630fbc76ca670138228f2f616f6840bb0`. Five findings are fixed and one is intentionally not fixed because its required authoritative parameter is absent. Branch is pushed for architecture review; nothing was merged. This is component/test evidence only, not qualification evidence.

## Finding status

| Finding | Status | Commit | Main files |
|---|---|---|---|
| H3.1 R4-01 | NOT_FIXED | `6557652` | `docs/safety/runtime_safety_current.md`, `docs/refactor/W3-H3/OPEN_QUESTIONS.md` |
| H3.2 R4-12 | FIXED | `b0fd5cf` | `src/estimation/fast_lio_core/src/pipeline/fast_lio_pipeline.cpp`, focused test, ledger |
| H3.3 R2-24 | FIXED | `e1a0905` | `src/planning/navigation_planning_backend/include/data_structure/base/polytope.h`, focused test, ledger |
| H3.4 O1-03 | FIXED | `761b95e` | `src/contracts/navigation_mission/include/navigation_mission/mission.hpp`, focused test, ledger |
| H3.5 O1-12 | FIXED | `dff3477` | `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp`, focused test, ledger |
| H3.6 O1-07 | FIXED | `235d3ad` | execution authority API plus runtime test fixtures/callers, ledger |

## Evidence

- H3.1 location `measurement_buffer.cpp:97-101` matches, but no authoritative IMU horizon was found; only `maximum_imu_samples` and `maximum_imu_gap_ns` exist. No value was inferred. See `OPEN_QUESTIONS.md`.
- H3.2 RED showed `state_time_` committed after a failed prediction and pre-tracking prediction failure did not consume the existing limit; GREEN focused tests passed 2/2.
- H3.3 RED minimal adjacent-transition case timed out at 3 s (`EXIT_CODE=124`); GREEN focused regression passed 4/4 and the prior safe adjacency cases remained passing.
- H3.4 RED loader/contract test failed before the terminal STOP guard; GREEN focused test passed 1/1.
- H3.5 RED equal non-retained admission returned true and cleared `kFailed`; GREEN focused test passed 1/1 for both `setAdmissionGoalEpoch` and `beginGoal`.
- H3.6 pre/post production grep found no `tryCommit`, `tryCommitAndFinalize`, or `stagePendingAndFinalize` legacy caller/API; execution authority passed 70/70 and runtime targeted tests passed 1/1, 14/14, 18/18, and terminal monitor 62/62 after sourcing the overlay.

## Build and verification outputs

- `colcon build --packages-select fast_lio_core ...`: PASS, 1 package finished; existing compiler stderr only.
- `colcon build --packages-select navigation_planning_backend ...`: PASS; direct focused test 4/4.
- `colcon build --packages-select navigation_execution ...`: PASS, 1 package finished [14.2 s].
- `colcon build --packages-select navigation_runtime ...`: PASS, 2 packages finished [13.0 s].
- `python3 tools/validate_runtime_safety_ledger.py`: PASS, `current=489 lines, gates=34, bypasses=1`.
- `git diff --check`: PASS.
- The repository CTest wrapper could not provide a gate in this environment because `ament_cmake_test` is unavailable (`ModuleNotFoundError`); direct built test binaries were used. No SITL/replay run was available for this branch.

## Baseline/deviation notes

`origin/main` is `432dc94630fbc76ca670138228f2f616f6840bb0`; W3-A2 is not merged, so the Wave 3 `all` gate is not available. The contract lineage command using `7e0b850` could not run because that object is absent in this clone (`fatal: Not a valid object name 7e0b850`); this is recorded rather than guessed around. No B1/B2/B3/B6 product files or prompt coordination documents were edited.

No qualification, flight readiness, SITL acceptance, or threshold claim is made. H3.1 needs an owner decision before implementation.

## Commit table

| SHA | Message |
|---|---|
| `761b95e` | `fix(mission): require stop terminal waypoint` |
| `b0fd5cf` | `fix(fast-lio): commit state time after prediction` |
| `e1a0905` | `fix(planning): bound nonrepresentable corridor retry` |
| `dff3477` | `fix(execution): preserve failure latch across equal admission` |
| `235d3ad` | `refactor(execution): remove legacy commit entry points` |
| `6557652` | `docs(h3.1): record missing imu horizon authority` |
| `852c2d5` | `docs(h3): record early-fix evidence` |

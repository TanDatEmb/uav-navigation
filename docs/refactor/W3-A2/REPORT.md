# W3-A2 report

Gate v3 tooling and documentation cleanup are implemented on baseline `432dc94` in branch `refactor/W3-A2`. Branch is pushed for architecture review; no merge was performed.

## Deliverable

- `tools/gate.sh {static|python|ros|all}` records HEAD/dirty count and fails closed on a failed stage.
- `tools/check_dependency_direction.py` checks package, CMake, and forbidden product subscriptions without ROS runtime.
- Provenance now fails clearly when a git submodule is present in the index but not initialized.
- `Makefile` exposes `make gate`; gate tests cover a passing current tree and a whitespace-error fixture.
- Architecture drift notes carry the historical banner; KB-08 Q2 points to `tools/gate.sh all`; the missing build skill is restored.

## Verification

| Command | Result |
|---|---|
| `git diff --check` | PASS |
| `./tools/check_dependency_direction.py` | PASS; 4 baseline exceptions allow-listed with finding/WP, including ADR-021 Q3 `/lio/diagnostics` |
| `tools/gate.sh static` | PASS; ledger PASS, mission authority PASS, citations `923 checked out_of_range=0`, dependency PASS, `GATE_V3_RESULT=PASS` |
| targeted gate/dependency tests | PASS (12/12) |
| B-1 failure-injection tests | PASS; each static validator and `colcon build` fails non-zero with `GATE_V3_RESULT=FAIL` |
| `tools/gate.sh python` | PASS; Python 3.12.3, tools/tests 19/19, runtime/tests 421/421, 2 skips |
| `tools/gate.sh all` | PASS on head `db6e9fb`; tools/tests 19/19, runtime/tests 421/421, 2 skips; ROS had no changed package; `GATE_V3_RESULT=PASS` |
| `tools/gate.sh ros` | NOT_MEASURED |

## Findings

| Finding | Status | Commit |
|---|---|---|
| S-02 missing `.agents/skills/build-and-test/SKILL.md` | FIXED | `5d5ea93` |
| S-03 missing consolidated gate | FIXED | `5d5ea93`, `47beb3d` |
| S-04 architecture-note drift banner and KB-08 Q2 | FIXED | `e9d3b51` |
| Review §3 F1/F2: swallowed gate errors and missing baseline diff check | FIXED | `9aaf7ff` |
| Review §3 F3/F4: CMake/package dependency parser coverage | FIXED | `9aaf7ff` |
| Review §3 F5: fresh selected-package test results and separate `px4_ros2_cpp` attachment | FIXED | `9aaf7ff` |
| Review §3 F6: certifier `rclcpp` rule and stale allow-list warning | FIXED | `9aaf7ff` |
| Review §4d: uninitialized-submodule provenance recursion | FIXED | `d43e4b0` |
| Review B-1: `if main` disabled fail-closed gate semantics | FIXED | `7afc5ad` |
| Review B-2: ADR-021 Q3 forbidden subscription guard | FIXED | `a9acba9` |
| Contract v3 §6 cleanup: consumed wave-1/wave-2/architecture docs and links | FIXED | final `docs(cleanup)` commit |

## Open questions / deviations

- `gate ros` package selection and `px4_ros2_cpp` skip are implemented, but full ROS execution remains unmeasured on this branch. Status: `CONDITIONAL`.
- ROS gate remains `NOT_MEASURED` because this W3-A2 write-set does not change a ROS package and no sourced ROS build was required for this branch. Status: `CONDITIONAL`.
- `navigation_execution -> navigation_world_model` remains an explicit baseline B3 allow-list entry (`V1/W3-B3`) and is still reported as a warning; this A2 change does not widen that exception. It must be removed when B3 resolves the dependency.
- The build parallelism default was not changed in A2. B3 evidence used the bounded override `PARALLEL_WORKERS=2 MAKE_JOBS=2`; this avoids reintroducing the prior high-parallelism crash risk while leaving build-policy ownership with the build/tooling workstream.
- The cleanup commit removes only the paths listed in `W3-A2_gate.md` and the corresponding stale links; the five-file architecture deletion list contained no tracked `px4_tracking_adapter_design20260909.md` in this baseline, so no untracked or unrelated file was removed.

## Commit table before transfer

| SHA | Message |
|---|---|
| `5d5ea93` | `build(wave3): add gate and dependency direction checks` |
| `e9d3b51` | `docs(wave3): mark historical architecture notes` |
| `91c1f51` | `docs(wave3): finalize A2 gate report` |
| `2ba95d0` | `docs(wave3): record A2 Python gate evidence` |
| `47beb3d` | `fix(gate): enforce wave3 dependency and package scopes` |
| `06f2b8c` | `docs(wave3): align A2 report with final gate` |
| `d6a2d81` | `docs(wave3): record A2 remote handoff` |
| `9aaf7ff` | `fix(gate): harden wave3 gate and dependency guard` |
| `d43e4b0` | `fix(tools): reject uninitialized provenance submodules` |
| `7afc5ad` | `fix(gate): preserve fail-closed stage errors` |
| `a9acba9` | `fix(gate): guard forbidden evidence subscriptions` |

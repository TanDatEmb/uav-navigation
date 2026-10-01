# W3-A2 report

Gate v3 tooling and documentation drift cleanup are implemented on baseline `432dc94` in branch `refactor/W3-A2`. No merge or push was performed.

## Deliverable

- `tools/gate.sh {static|python|ros|all}` records HEAD/dirty count and fails closed on a failed stage.
- `tools/check_dependency_direction.py` checks package and CMake dependency rules without ROS runtime.
- `Makefile` exposes `make gate`; gate tests cover a passing current tree and a whitespace-error fixture.
- Architecture drift notes carry the historical banner; KB-08 Q2 points to `tools/gate.sh all`; the missing build skill is restored.

## Verification

| Command | Result |
|---|---|
| `git diff --check` | PASS |
| `./tools/check_dependency_direction.py` | PASS; 3 baseline B3 violations allow-listed with finding/WP, no new violation |
| `tools/gate.sh static` | PASS; ledger PASS, mission authority PASS, citations `989 checked out_of_range=0`, dependency PASS |
| `tools/gate.sh python` | PASS; Python 3.12.3, tools/tests 12/12, runtime/tests 420/420, 2 skips |
| `tools/tests/test_gate.py` + `tools/tests/test_dependency_direction.py` | PASS (5/5 targeted) |
| `tools/gate.sh ros` | NOT_MEASURED |

## Findings

| Finding | Status | Commit |
|---|---|---|
| S-02 missing `.agents/skills/build-and-test/SKILL.md` | FIXED | `5d5ea93` |
| S-03 missing consolidated gate | FIXED | `5d5ea93`, `47beb3d` |
| S-04 architecture-note drift banner and KB-08 Q2 | FIXED | `e9d3b51` |

## Open questions / deviations

- `gate ros` package selection and `px4_ros2_cpp` skip are implemented, but full ROS execution remains unmeasured on this branch. Status: `CONDITIONAL`.
- ROS gate remains `NOT_MEASURED` because this W3-A2 write-set does not change a ROS package and no sourced ROS build was required for this branch. Status: `CONDITIONAL`.

## Commit table before transfer

| SHA | Message |
|---|---|
| `5d5ea93` | `build(wave3): add gate and dependency direction checks` |
| `e9d3b51` | `docs(wave3): mark historical architecture notes` |
| `91c1f51` | `docs(wave3): finalize A2 gate report` |
| `2ba95d0` | `docs(wave3): record A2 Python gate evidence` |
| `47beb3d` | `fix(gate): enforce wave3 dependency and package scopes` |

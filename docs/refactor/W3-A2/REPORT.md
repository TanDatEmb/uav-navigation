# W3-A2 report

Gate v3 tooling and documentation drift cleanup are implemented on baseline `432dc94` in coordinator branch `refactor/wave3-execution`. The owner must split or cherry-pick this work into `refactor/W3-A2` before opening the draft PR; no merge or push was performed here.

## Deliverable

- `tools/gate.sh {static|python|ros|all}` records HEAD/dirty count and fails closed on a failed stage.
- `tools/check_dependency_direction.py` checks package and CMake dependency rules without ROS runtime.
- `Makefile` exposes `make gate`; gate tests cover a passing current tree and a whitespace-error fixture.
- Architecture drift notes carry the historical banner; KB-08 Q2 points to `tools/gate.sh all`; the missing build skill is restored.

## Verification

| Command | Result |
|---|---|
| `git diff --check` | PASS |
| `./tools/check_dependency_direction.py` | PASS (21 package/CMake units checked) |
| `tools/gate.sh static` | PASS; ledger PASS, mission authority PASS, citations `989 checked out_of_range=0`, dependency PASS |
| `/usr/bin/python3 -m unittest tools.tests.test_gate -v` | PASS (2/2) |
| `tools/gate.sh python` | NOT_MEASURED |
| `tools/gate.sh ros` | NOT_MEASURED |

## Findings

| Finding | Status | Commit |
|---|---|---|
| S-02 missing `.agents/skills/build-and-test/SKILL.md` | FIXED | `5d5ea93` |
| S-03 missing consolidated gate | FIXED | `5d5ea93` |
| S-04 architecture-note drift banner and KB-08 Q2 | FIXED | `e9d3b51` |

## Open questions / deviations

- `gate ros` currently builds the package set returned by `colcon list` when `PACKAGES` is unset; reverse-dependency narrowing and the special `px4_ros2_cpp` attachment policy require validation on a sourced ROS environment. Status: `CONDITIONAL`.
- This coordinator branch is not the prompt-named `refactor/W3-A2`; the change must be transferred to that branch before push. Status: `CONDITIONAL`.

## Commit table before transfer

| SHA | Message |
|---|---|
| `5d5ea93` | `build(wave3): add gate and dependency direction checks` |
| `e9d3b51` | `docs(wave3): mark historical architecture notes` |

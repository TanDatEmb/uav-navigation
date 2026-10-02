# W3-A3 salvage after B6 — review request

## Verdict

**REVIEW REQUEST — not merge-ready.** The salvage patch applied cleanly after
B6 and the static/tooling checks pass, but ROS gate execution is blocked by a
missing canonical ROS Python dependency. The full C++ test run also retains
the known TB-003 planner regression; no threshold or product workaround was
introduced.

## Source and patch

- Branch: `refactor/W3-A3-after-B6`
- HEAD: `add2ecacdd2bcf99b7f62e557db9f0ca95d7bc81`
- Patch: `docs/refactor/wave3/salvage/P1A_rebased_main_after_B6.patch`
- Patch SHA-256: `1b457ce65ea5e52cd66d31b7d7afe7825c9e8b10ecb6201d2bd6c5945ee4cae9`
- `git apply --check`: PASS; patch applied and committed as `d4e32bd`.
- Main worktree was not checked out or modified.

## Verification

| Check | Result |
|---|---|
| Safety ledger validator | PASS — current=485 lines, gates=34, bypasses=1 |
| A6 semantic checker | PASS — 119 rows, 117 represented, 2 excluded, 0 open questions, 0 failures |
| Profile source/loader tests | PASS |
| Code generation `--check` | PASS |
| Release build | PASS — 24 packages, 41 minutes |
| Static gate | PASS |
| Python gate | PASS — 442 tests, 1 expected skip |
| `make test` | PARTIAL — 95 passed, 1 known TB-003 failure: `PlannerFacade.CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary` |
| ROS gate | BLOCKED before compile: `/usr/bin/python3` lacks `ament_package` |
| SITL | NOT RUN — must follow C1 and use the same binary cohort |

Logs:

- `/home/letandat/uavnav-w3-a3-after-b6-test-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-static-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-python-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-ros-20261002.log`

The ROS failure is an environment prerequisite failure, not evidence of a
passing or failing ROS test. The known TB-003 failure remains fail-closed and
is not converted into a tuning task from this single run.


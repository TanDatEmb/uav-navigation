# W3-A3 salvage after B6 — review request

## Verdict

**STOP / REVIEW REQUEST — not merge-ready.** Steps 1–4 remain complete, but
the three valid reruns for step 5 did not establish A/B parity. A3 run 3 had
valid infrastructure but `MISSION_TIMEOUT` with no accepted waypoint; runs 4–5
were `INFRASTRUCTURE_INVALID` because the simulation-clock wall-arrival lease
was exceeded. F1/B4/B5 SITL follow-up is blocked pending review of this
runtime failure and the infrastructure gaps. No threshold or product
workaround was introduced.

## Source and patch

- Branch: `refactor/W3-A3-after-B6`
- HEAD: `b6b617eaaa78ca9fb3f5589762fbb58a642f7592`
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
| SITL | STOPPED — 3 valid reruns below; 3 preflight runs excluded because monitor crashed |

Logs:

- `/home/letandat/uavnav-w3-a3-after-b6-test-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-static-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-python-20261002.log`
- `/home/letandat/uavnav-w3-a3-after-b6-ros-20261002.log`

The ROS failure is an environment prerequisite failure, not evidence of a
passing or failing ROS test. The known TB-003 failure remains fail-closed and
is not converted into a tuning task from this single run.

## Step 5 SITL A/B checkpoint

The A arm is C1 `M2/sanity_open/3 m/s/SAFE`, run indices 3–5. All A runs use
the C1 cohort SHA `c435a4f19da0455cd538cb7ae111c0ba00e2e644`, `tracking=off`,
`raycasting_on_backup_strict`, and the same scene/speed. A3 used a rebuilt
head with the measurement fix `c435a4f` cherry-picked as `b6b617e`.

| Arm/run | Session | Infrastructure | Verdict/outcome | Waypoints | Note |
|---|---|---|---|---:|---|
| A/3 | `external-mode-check-20261002T052818-167762` | VALID | BLOCKED / `CERTIFIED_COMMAND_PRESERVED` | 0/5 | C1 baseline |
| A/4 | `external-mode-check-20261002T052921-170928` | VALID | PASS / `EMERGENCY_COMMITTED` | 0/5 | C1 baseline |
| A/5 | `external-mode-check-20261002T053028-174085` | VALID | PASS / `SUPERSEDED_PENDING` | 0/5 | C1 baseline |
| B/3 | `external-mode-check-20261002T081031-33224` | VALID | FAIL / `MISSION_TIMEOUT` | 0/5 | no PVA/setpoint; PX4 hold not observed |
| B/4 | `external-mode-check-20261002T081400-36566` | INVALID | FAIL / `FAILED_COMPONENT` | 1/5 | 10 sim-clock gaps, max 918.577 ms |
| B/5 | `external-mode-check-20261002T081509-39666` | INVALID | FAIL / `FAILED_COMPONENT` | 1/5 | 10 sim-clock gaps, max 1973.947 ms |

The first A3 attempt (`080343`, `080403`, `080420`) is excluded from A/B:
all three were `INFRASTRUCTURE_INVALID` before readiness because the monitor
crashed on missing `_integer_value`; the cause was fixed by C1 tooling commit
`c435a4f`. B/3 is nevertheless a new product/runtime failure versus the
selected A arm, so the stop rule applies. Raw artifacts are under
`.artifacts/runtime/` with the session IDs above.

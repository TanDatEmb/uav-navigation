# W3-H3 early fixes — checkpoint report

## Summary

`REVIEW REQUEST`: implementation and component evidence are prepared; this
worktree does not assign PASS, merge readiness, flight readiness, or
qualification authority. The branch was rebased onto `origin/main` at
`24ec0fc8718bb8606e4e9e4a7eaa84773d384854`. No main-worktree files were
modified, no merge was performed, and SITL was not run.

## Deliverables

- H3.1: the measurement buffer uses the existing propagated-odometry IMU
  history duration, retains the oldest scan start bracket, and slides the
  no-scan IMU window while remaining bounded by `maximum_imu_samples`.
- H3.2: recovery prediction uses the second group epoch locally; `state_time_`
  is committed only after successful prediction. Existing initial-map failure
  accounting and thresholds are unchanged.
- H3.3: inherited bounded `SimplifySFC` retry returns false for the
  non-representable adjacent transition. The supplied transliteration reports
  `NO TERMINATION` for the old unbounded control flow.
- H3.4: C++ planner/trajectory fixtures end in STOP and the external-mode
  Python validator rejects an explicit pass-through final waypoint.
- H3.5: `beginGoal` computes `retain` before the equal-epoch guard; a failed
  latch is not cleared by an equal `retain_active=true` request.
- H3.6: legacy public commit/finalizer names have no production callers or
  declarations; tests use the predecessor-checked access path. The refactor
  commit is separate from the behavior/ledger commit.

## RED/GREEN evidence

- RED tests were committed in `623b886` before `34017ff` and `b47d9ef`:
  continuous IMU before first scan, second-epoch recovery prediction, equal
  retained request after `kFailed`, and terminal pass-through validator.
- H3.3 prior RED timeout and GREEN regression are retained in the existing
  history; current repro output is `NO TERMINATION (steps>10000, final size
  10001)` for the unbounded transliteration.
- H3.4 C++ fixture corrections cover `test_planner_facade.cpp` and
  `test_trajectory.cpp`; Python validator test is in the RED test commit.

## Verification

- `python3 tools/validate_runtime_safety_ledger.py`: PASS, current ledger
  490 lines, 34 gates, 1 bypass.
- `git diff --check origin/main...HEAD`: PASS.
- `tools/refactor/check_citations.py . docs/refactor`: PASS, 922 checked,
  0 out-of-range; dependency-direction check: PASS with 4 documented warnings.
- `tools/gate.sh python`: PASS, 19 tooling tests and 423 runtime tests,
  2 expected skips. The runtime report failure was the test's intentional
  cleanup/report fixture and the gate result was PASS.
- Full Release build, full backend CTest, and focused C++ GREEN outputs are
  recorded here after the queued `/tmp/uavnav-build.lock` verification run.
- Replay: `NOT_EVALUABLE`; no local recorded bag/cache exists. The catalog
  manifest is downloadable but no external 517 MB download was started.

## Safety and scope

The current safety ledger is authoritative. No threshold, UNKNOWN policy,
authority owner, lease, PX4 behavior, or safety gate was changed. H3 behavior
entries were updated with owner, scope, safety impact, evidence, removal
condition, and verification command. No prompt-listed cleanup path was found;
no unrelated file was deleted.

## Commit table

The final single commit table is generated from `git log --format='%h %s'
origin/main..HEAD` immediately before push.

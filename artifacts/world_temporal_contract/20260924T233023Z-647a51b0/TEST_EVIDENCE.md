# Test evidence

## Build and deterministic tests

- Authoritative Release build: PASS, 23 packages. Command: `python3 tools/runtime/build.py --mode release build`. The build wrote `install/.uav_navigation_build_manifest.json` for the pinned source and initialized submodules.
- Release manifest/data check: PASS. Command: `python3 tools/runtime/build.py --mode release check`; `colcon test-result` reported 91 tests, 0 errors, 0 failures, 0 skipped, and the tracked dataset catalog/blob guard passed.
- CTest: PASS, all 14 selected packages completed. `navigation_runtime` specifically passed 19/19 targets, including the new typed temporal assessment test, `test_world_snapshot_store`, and localization reset coverage.
- Python: PASS, `python3 -m pytest -q tools/runtime/tests tools/tests` reported 450 passed, 1 skipped.
- Static guards: PASS: mission authority, execution authority, desired intent, and fail-closed ownership fencing.
- Runtime safety ledger: PASS, 500 current-contract lines, 706 decisions, 36 gates, 34 active gates, 5 bypasses, 1 active bypass.
- `git diff --check`: PASS.

The typed `WorldTemporalAssessment` is ephemeral and wraps the existing timestamp freshness classifier. No freshness thresholds or decisions were changed. Unit tests cover absent/missing source stamps, current/stale/bounded-future/future timestamps, invalid clock/window inputs, and identity kept distinct from temporal assessment. Store tests cover same-source-stamp revision advance, timestamp restart under a new generation, and failed dependent finalization retaining the prior published world.

## Runtime and evidence limits

- No SITL was run for this branch. There is no isolated World Source Stale fault harness, nor an end-to-end runtime witness joining world transaction, execution disposition, command exposure, adapter admission, and recovery.
- C0-SW World transaction lifecycle events are not emitted/loss-accounted by the current evidence path. The C0-SW hardening success gate therefore remains unmet despite source, unit, and component evidence.
- These results are build/component evidence, not SITL, HITL, or flight qualification.

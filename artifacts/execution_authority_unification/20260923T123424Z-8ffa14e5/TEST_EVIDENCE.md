# Test evidence

## Pinned repaired baseline, before product refactor

- First fresh-worktree Release build compiled 23 packages, then the authoritative-build wrapper correctly rejected provenance because these audit documents were written while it ran. This was a source-fingerprint race in the audit workflow, not a compiler/product failure.
- Stable-source rerun: `PARALLEL_WORKERS=2 MAKE_JOBS=4 python3 tools/runtime/build.py --mode release build` completed 23 packages; `install/.uav_navigation_build_manifest.json` reports `authoritative=true`, `build_mode=release`, source HEAD `8ffa14e5` and pinned submodule SHAs. It includes these pre-mutation audit documents as dirty source; a clean artifact commit/build may follow.
- `python3 tools/runtime/build.py --mode release test --packages navigation_runtime navigation_mission navigation_execution navigation_contracts px4_navigation_external_mode`: 29 CTest targets, 0 errors, 0 failures, 0 skipped (`colcon test-result --test-result-base test-results --all`).
- `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`: 390 tests, OK, one skip. An earlier invocation before the build installed planner config had one expected missing-install assertion; the post-build rerun passed.
- `python3 tools/check_mission_authority_cut.py`: `MISSION_AUTHORITY_STATIC_CHECK: PASS`.
- `git diff --check`: PASS before product edits.

These are baseline component/static results. They are not refactor validation or flight qualification.

## Refactor component/static gates, before focused SITL

- Release selected-package build: `navigation_execution` and `navigation_runtime` passed after the final source changes. Full-workspace authoritative build remains a separate gate before SITL.
- `python3 tools/runtime/build.py --mode release test --packages navigation_runtime navigation_mission navigation_execution navigation_contracts px4_navigation_external_mode`: 29 CTest targets, 0 errors/failures/skips. Includes the new latch-based owner publication/activation, publication/fail-closed, world-recertification and suspended safety-ownership tests. `colcon test-result --test-result-base test-results --all` agrees: 29/29.
- `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`: 390 tests, OK, one skip.
- `python3 tools/check_execution_authority_cut.py`: PASS; `python3 tools/check_mission_authority_cut.py`: PASS; `python3 tools/validate_runtime_safety_ledger.py`: PASS; `git diff --check`: PASS.
- Logs: `/tmp/uav_execution_owner_races_build2.log`, `/tmp/uav_execution_owner_all_ctest.log`, `/tmp/uav_execution_owner_python.log`. Tests are component/static evidence. They do not prove flight behavior or timing parity.

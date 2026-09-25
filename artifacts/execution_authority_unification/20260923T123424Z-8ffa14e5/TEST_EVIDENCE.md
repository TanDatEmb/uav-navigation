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

## Final product-source checkpoint before SITL

- HEAD `2e4f7a0f1e09f4d72afed23686e26dc173150dd3`: authoritative Release build completed 23/23 packages; manifest `install/.uav_navigation_build_manifest.json` has `authoritative=true`, `build_mode=release`, `git_dirty=false`, matching HEAD and pinned submodules.
- Repeated after the diagnostic-only owner lock-wait observation was added: five relevant packages, 29/29 CTest targets passed; Python runtime contract suite 390 OK, one skip; execution and mission static guards PASS; runtime safety ledger validator PASS; `git diff --check` PASS. Logs `/tmp/uav_execution_owner_final_release_before_sitl.log`, `/tmp/uav_execution_owner_final_ctest.log`, `/tmp/uav_execution_owner_final_python.log`.
- Existing `candidateMatchesAnchor()` P/V/A/J rejection tests, owner sample-versus-activation/recertification/revocation and failClosed-versus-publication latch tests, stale-planner-result fences, localization reset tests, and mission/adapter seam tests ran within that CTest set. These retain actual continuity validation, not a replacement mock predicate.
- The cut did not change `docs/safety/runtime_safety_current.md`, runtime budgets, 100/200/500 ms leases, tracking threshold, certificate gates, UNKNOWN policy, or PX4 Hold policy. Pinned dependency full-workspace lint debt remains separate and is not labeled PASS.

## Focused boundary observations

- Nominal diagnostic SITL: 3/3 matched `long_featured` mission complete, 12 exact adapter request handoffs, no nominal stale-PVA/Hold, identity rejection or candidate continuity rejection. One pre-validation sample reject in run 3 is disclosed in `SITL_RESULTS.md`.
- Core pause fault: 350.112 ms process stop, adapter logged stale PVA and Hold request; report observed Hold. This is a lease-fence observation with a simultaneous mapping cloud gate, not world-source isolation.
- Existing repeated-replan diagnostic hook fired 28 times in one `long_featured` fault run; immutable BACKUP samples, safety ownership, analytic terminal hold, propagated measured stop and restart were observed. It did not exercise an emergency candidate.
- Two cut pillar attempts and a fresh exact-repair control all failed closed after the same high anchor-error/no-suffix mechanism at waypoint 0. These do not prove a cut-specific regression or isolated world-revision invalidation. The historical repair pillar revision-510 invalidation remains prior evidence; new owner world-recertification/revocation is covered by deterministic component tests.

The full SITL result and measurement method are in `SITL_RESULTS.md`, `PERFORMANCE.md`, `analyze_sitl.py`, and `HANDOFF_MEASUREMENTS.csv`. None is flight qualification.

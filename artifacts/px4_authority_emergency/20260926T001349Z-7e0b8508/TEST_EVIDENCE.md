# Test evidence

- Base source: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Pure handover regression source: `test/test_px4_authority_handover.cpp` (callback result never confirms Hold, status-before-callback fencing, stale activation/attempt rejection, retry deadline even when ModeCompleted is absent).
- Direct compile and run of the new reducer test translation unit with `g++ -std=c++17 -Wall -Wextra -Werror` passed after the request-timeout test was updated to the final helper signature: 6/6 tests.
- Mission, execution, desired-intent, fail-closed, exact OptimizationFailed, World evidence, NavigationCommand, runtime-config, state-transport guards, safety ledger, and `git diff --check`: PASS at the current worktree source state.
- Final-source authoritative Release build: PASS, 23 packages; manifest written to `install/.uav_navigation_build_manifest.json` after the request-timeout retry change.
- CTest: PASS, 94 tests, 0 errors/failures/skips across 15 test-result packages; this includes the handover helper tests and `test_navigation_mode_progression`.
- Python: PASS, 459 passed, 1 skipped.
- Final-source focused SITL: mapping-only 700 ms World stale injection passed its isolation gate, reached `PAUSED/SAFETY_STOP`, continued `velocity_hold` PX4 inputs, and the observer saw `nav_state=4`/`AUTO_LOITER` with `failsafe=false`. Run `external-mode-check-20260926T011400-401857`; raw checksums are in `H2_WORLD_STALE_HOLD.md`. The intentional fault run ended `PAUSED_SAFETY_STOP`, so runtime and C0-SW outcome are `BLOCKED`/`NOT_EVALUABLE`; callback phase is not producer-correlated.
- A preceding rerun (`external-mode-check-20260926T011209-398644`) failed setup before startup because evidence edits had invalidated the authoritative build fingerprint. It is excluded from runtime denominators. The source tree was rebuilt before the final-source run.
- H1 standalone Hold, H3 controlled adapter-state stale, H4 operator/failsafe ordering, H5 explicit reactivation, and the EMERGENCY end-to-end SITL have not been proven in this branch.

# Test evidence

Validation is recorded here against the final product source commit. A component/model test proves only its exercised seam; the static guard checks source shape; neither is flight qualification.

| Gate | Result | Evidence |
|---|---|---|
| Release build | PASS, 23 packages | `python3 tools/runtime/build.py --mode release build`; first attempt exposed a missing test include, fixed and rebuilt successfully. Authoritative clean-source manifest still pending commit/rebuild. |
| CTest | PASS, 14 packages; 89 tests, 0 errors/failures/skips | `python3 tools/runtime/build.py --mode release test`; `colcon test-result --test-result-base test-results`. |
| Runtime Python | PASS, 390 tests, 1 skipped | `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -q` after install. An earlier pre-install run failed solely on missing installed planner config and was superseded. |
| Mission authority guard | PASS | `python3 tools/check_mission_authority_cut.py` |
| Execution authority guard | PASS | `python3 tools/check_execution_authority_cut.py` |
| Desired intent guard | PASS | `python3 tools/check_desired_intent_cut.py` |
| Fail-closed ownership guard | PASS | `python3 tools/check_failclosed_ownership_fencing.py` |
| Safety ledger | PASS | `python3 tools/validate_runtime_safety_ledger.py` |
| Diff whitespace | PASS before final commit | `git diff --check`; rerun at final HEAD. |

Focused tests added or amended: exact active E1→E2 failure race; exact empty admission fencing; stale world-suspension rejection; exhaustive current planner-result classification; immutable lease L1→L2 barrier; pending P1→P2 barrier; solve witness lifetime; hot-retarget `OptimizationFailed` routing. Existing candidate continuity, STOP, BACKUP, reset and publication tests remain enabled.

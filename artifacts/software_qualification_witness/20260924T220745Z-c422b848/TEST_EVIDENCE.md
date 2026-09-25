# Test evidence

At source/evaluator HEAD `db43b608` before this documentation commit:

| Gate | Result |
| --- | --- |
| `python3 tools/runtime/build.py --mode release build` | PASS, 23 packages |
| `python3 tools/runtime/build.py --mode release test` | PASS, 14 selected packages |
| `colcon test-result --test-result-base test-results --all --verbose` | PASS, 90 reported CTest results; 0 errors/failures/skips |
| `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -q` | PASS, 444 tests, 1 skip |
| Mission, execution, desired-intent, failclosed, control-contract, config-truth, optimization-injection and C0-SW static guards | PASS |
| `python3 tools/validate_runtime_safety_ledger.py` | PASS |
| `git diff --check` | PASS |
| Raw evidence SHA256/size verification against `RAW_EVIDENCE_MANIFEST.csv` | PASS, 60/60 files |

`tools/runtime/tests/test_c0_sw_witness.py` has 17 focused tests covering exact producer ownership, missing/conflicting world/request/generation/localization identity, recorder loss, heading, emergency, retry/supersession/terminal monitor, adapter admission/rejection and no-execution signal. Earlier reduction false-pass and emergency monitor identity errors were corrected before the primary cohort. Final-HEAD validation is rerun after artifact commit and reported in delivery.

These are deterministic/component checks plus focused SITL observation. They do not qualify PX4 control performance or flight. The final-HEAD Release, CTest, Python, guards and ledger rerun returned PASS after the documentation commit; this row records the actual test-result base used by that rerun.

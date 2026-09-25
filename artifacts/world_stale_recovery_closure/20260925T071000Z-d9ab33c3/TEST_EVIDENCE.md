# Test evidence

- Release build: official `tools/runtime/build.py --mode release build`, 23 packages passed; authoritative build manifest emitted for the current dirty source tree.
- C++: fresh `colcon test --packages-select navigation_contracts navigation_execution navigation_mapping navigation_planning navigation_planning_backend navigation_runtime`; 6 packages, 37 CTest targets, 898 gtest cases, 0 failures, 0 skips. Includes R1–R4, unsafe-world no-resume, runtime reset, and mapping/world model coverage.
- Python runtime/tools suite: `python3 -m unittest discover -s tools/runtime/tests -q`; 448 passed, 1 skipped.
- Static guards: mission authority, execution authority, desired intent, fail-closed ownership, World evidence non-authority all PASS.
- Safety ledger: `python3 tools/validate_runtime_safety_ledger.py` PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1).
- `git diff --check`: PASS at last validation before final docs. Re-run at final HEAD.
- SITL: short stale recovery run mission COMPLETE and World C0 reducer RESOLVED. Long stale control reached the expected safety stop/Hold. Three clean nominal attempts include two COMPLETE and one expected fail-closed terminal recovery stop; see `NOMINAL_REGRESSION.md`.
- Final artifact validation: all 411 raw files listed in `RAW_SESSION_MANIFEST.csv` exist with recorded sizes; all six session `scenario.jsonl` streams were re-hashed against their manifest rows. The supplemental 420 ms session was re-evaluated with the corrected evaluator (`required=true`, `RESOLVED`, zero World unresolved/conflicts/reference gaps/drops); see `EVALUATOR_RECHECK_420.json`.
- Final static guards, runtime safety ledger validator, and `git diff --check`: PASS after evidence documentation/manifest updates. CTest result summary was re-read at final source state: 898 cases, zero errors/failures/skips. The official Release build manifest is authoritative, Release, HEAD `d9ab33c3e2321ac60794da4128863bc7933458ea`, source content SHA `d87cdabb103084659a30a3f166adb46d3151ba0fcbfcca48d6a03922adb44f76`.

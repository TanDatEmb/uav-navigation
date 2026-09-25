# Deterministic validation

Behavior-bearing source commit: `33ec33626c43e20b84029853925f402532517285`.

| Gate | Result | Evidence |
| --- | --- | --- |
| Authoritative Release build | 23 packages built | `MAKE_JOBS=2 python3 tools/runtime/build.py build`; build manifest `install/.uav_navigation_build_manifest.json`; local log `/tmp/uav_adapter_contract_build6.log` |
| CTest | 89/89 passed across 14 packages | `python3 tools/runtime/build.py test`; local log `/tmp/uav_adapter_full_ctest.log` |
| Runtime Python suite | 393 passed, 1 skipped | `python3 -m pytest tools/runtime/tests -q`; local log `/tmp/uav_adapter_python_tests2.log` |
| Static architecture guards | passed | `check_navigation_command_contract.py`, `check_mission_authority_cut.py`, `check_execution_authority_cut.py`, `check_desired_intent_cut.py`, `check_failclosed_ownership_fencing.py` |
| Safety ledger validator | passed | `python3 tools/validate_runtime_safety_ledger.py` |
| Whitespace | passed | `git diff --check` |

The typed contract tests compare each frozen old contract predicate outcome with the new reason-bearing assessment, including malformed frame/identity/status/PVA/continuation and temporal boundary cases. Adapter tests cover the pure session identity assessment and disposition mapping. Python tests cover the exact diagnostic join, missing, delayed, duplicate, conflicting, and wrong-key observer events. Existing runtime, mission, and execution suites remain unchanged in their safety assertions.

These are component/source checks. The SITL observations and their evaluator-owned qualification status are recorded separately.

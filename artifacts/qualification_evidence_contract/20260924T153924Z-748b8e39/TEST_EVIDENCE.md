# Test evidence

Focused evaluator/lifecycle tests cover exact PVA heartbeat collapse, changed-world same-stamp conflict, strict versus nondecreasing timestamp policy, missing versus conflicting bundle identity, orphan activation, explicit supersession, and recorder cache non-inheritance. The branch also runs the existing golden evaluator tests without weakening them.

The static mission, execution, desired-intent, failClosed, command-contract, runtime-config, state-transport and evidence-scope guards all passed. `python3 tools/validate_runtime_safety_ledger.py` passed. The evidence-scope guard verifies no `src/` byte difference from base `748b8e39` in committed, staged or working-tree changes.

The first isolated Release build compiled 23/23 packages but the build wrapper failed closed on provenance because artifact files changed during the run. The stable-tree rebuild at source commit `653fe5d0` passed and wrote an authoritative build manifest. A passing test suite is component evidence, not flight qualification.

| Gate | Result |
|---|---|
| Focused evaluator/lifecycle | Included in full Python runtime suite. |
| Release build | Stable-tree rerun PASS, 23/23 packages; first attempt compiled but provenance failed closed because artifact files changed concurrently. |
| CTest | PASS, 90 tests, 0 errors, 0 failures, 0 skipped (`colcon test-result --all`). |
| Python runtime suite | PASS, 427 tests, 1 skip (`python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -q`). |
| Static guards | PASS |
| Safety ledger validator | PASS |
| `git diff --check` | PASS before final documentation commit; rerun on final HEAD. |

The final evidence documentation commit changes no product `src/` bytes or evaluator code after the passing build and tests. The evidence-scope guard is rerun at final HEAD to verify that condition.

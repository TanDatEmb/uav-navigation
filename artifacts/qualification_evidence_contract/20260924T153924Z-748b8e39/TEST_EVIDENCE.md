# Test evidence

Focused evaluator/lifecycle tests cover exact PVA heartbeat collapse, changed-world same-stamp conflict, strict versus nondecreasing timestamp policy, missing versus conflicting bundle identity, orphan activation, explicit supersession, and recorder cache non-inheritance. The branch also runs the existing golden evaluator tests without weakening them.

The static mission, execution, desired-intent, failClosed, command-contract, runtime-config, state-transport and evidence-scope guards all passed. `python3 tools/validate_runtime_safety_ledger.py` passed. The evidence-scope guard verifies no `src/` byte difference from base `748b8e39` in committed, staged or working-tree changes.

The first isolated Release build compiled 23/23 packages but the build wrapper failed closed on provenance because artifact files changed during the run. A stable-tree rebuild is required and recorded below. A passing test suite is component evidence, not flight qualification.

| Gate | Result |
|---|---|
| Focused evaluator/lifecycle | 67 tests passed before the final versioned-policy test; rerun at final HEAD below. |
| Release build | First attempt: 23/23 compiled; provenance FAIL due concurrent artifact edits. Stable-tree rerun PENDING. |
| CTest | PENDING |
| Python runtime suite | PENDING |
| Static guards | PASS |
| Safety ledger validator | PASS |
| `git diff --check` | PENDING final staged check |

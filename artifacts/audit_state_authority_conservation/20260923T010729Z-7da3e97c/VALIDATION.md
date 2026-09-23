# Validation log and limits

Executed in isolated audit worktree at TARGET_SHA:

| Check | Result | Scope |
|---|---|---|
| `python3 -m py_compile tools/*.py tests/*.py` | PASS | syntax only |
| `python3 tests/target_model.py` | PASS: 1,867 states, 15,351 edges, depth 6 | abstract candidate transition model, not product equivalence |
| matrix schema/unique field check | PASS: 240 candidate rows, actions 22 KEEP / 3 MOVE / 215 UNRESOLVED | sixteen selected types, not whole product |
| command schema register | PASS: 73 non-constant fields | preliminary classification, consumer proof pending |
| `python3 tools/metrics.py` | PASS: reproducible scoped counts in `metrics.json` | proxy counts only; comparison/owner totals unresolved |
| Graphviz render and XML parse | PASS: 5 SVGs | structural parse, not visual UX or formal verification |

Not executed: TARGET C++/ROS tests, paired runtime/PX4 process, SITL, recorded sensor replay, hardware, queue latency/WCET. Existing audit test results are labeled `FACT_FROM_AUDIT_ARTIFACT` and not counted as this audit's executed tests. The audit worktree submodules were not initialized; TARGET gitlinks are pinned in `BASELINE.md`.

Structural validation does not prove field exhaustiveness, transition equivalence, flight safety or qualification. The scanner is declaration-based and does not expand nested aggregates or resolve aliases/writers. This is why `AUDIT_DELIVERY` remains `PARTIAL`.

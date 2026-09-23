# Validation log and limits

Executed in isolated audit worktree at TARGET_SHA:

| Check | Result | Scope |
|---|---|---|
| `python3 -m py_compile tools/*.py tests/*.py` | PASS | syntax only |
| `PYTHONDONTWRITEBYTECODE=1 python3 tools/extract_state.py` after audit commit | PASS: 368 declarations, all input blobs equal pinned TARGET | regeneration proof; scanner still partial |
| `python3 tests/target_model.py` | PASS: 10,699 states, 89,408 edges, depth 6 | abstract candidate transition model, nine target invariants, not product equivalence |
| `PYTHONDONTWRITEBYTECODE=1 python3 tools/build_evidence.py` after audit commit | PASS: TARGET blobs and PX4 library gitlink/blob resolved | source provenance, not runtime behavior |
| matrix schema/unique field check | PASS: 239 candidate rows, actions 22 KEEP / 3 MOVE / 214 UNRESOLVED; 68 selected-path writer/reader rows | sixteen selected types, not whole product |
| command schema register | PASS: 73 non-constant fields | preliminary classification, consumer proof pending |
| `python3 tools/metrics.py` | PASS: reproducible scoped counts in `metrics.json` | proxy counts only; comparison/owner totals unresolved |
| Graphviz render and XML parse | PASS: 5 SVGs | structural parse, not visual UX or formal verification |
| Python syntax compile; `git diff --check` | PASS | audit artifact hygiene only |

Not executed: TARGET C++/ROS tests, paired runtime/PX4 process, SITL, recorded sensor replay, hardware, queue latency/WCET. Existing audit test results are labeled `FACT_FROM_AUDIT_ARTIFACT` and not counted as this audit's executed tests. The audit worktree submodules were not initialized; TARGET gitlinks are pinned in `BASELINE.md`.

Structural validation does not prove field exhaustiveness, transition equivalence, flight safety or qualification. The scanner is declaration-based and does not expand nested aggregates or resolve aliases/writers. This is why `AUDIT_DELIVERY` remains `PARTIAL`.

# Rules against state proliferation

| Review rule | Static check candidate | Invariant/test |
|---|---|---|
| New persistent `*_pending/valid/confirmed/latched` needs State Admission Test and independent protocol/physical fact | AST diff of member declarations; require matrix row ID | impossible-combination model for each sum type |
| No `object.valid()` mirror | flag bool names matching another object's validity accessor | mutate object/clear mirror fault test; require same decision |
| Epochs are semantic reset fences, not synchronization for duplicate owners | inventory every new epoch writer/consumer | delayed result across reset cannot activate |
| No manager mirrors `CoreState` | compare new member types/fields against authority schema | mutation observed only through one transition API |
| PX4 adapter does not read planner recovery internals | dependency boundary check and `rg` rule | adapter rejects stale reference without planner availability |
| MissionProgress does not read planner internals | include/dependency boundary check | measured crossing and acceptance work with delayed successor |
| Worker cannot commit Store/Core authority | forbidden call/namespace check in worker sources | stale worker result is inert |
| Diagnostic fields cannot authorize | generated consumer map for `.msg` fields | perturb diagnostics, admission unchanged |
| Identity tuple copied only as immutable event/payload | AST comparison of repeated tuple members | one context-key stale test plus raw provenance checks |

The declaration extractor included here is an initial inventory, not an enforcement implementation. A compiler-aware CI check with explicit allowlist and source annotations is required before claiming these rules are enforced.

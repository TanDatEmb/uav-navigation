# State conservation coverage

`STATE_CONSERVATION_MATRIX.csv` is regenerated from the pinned TARGET by `tools/extract_state.py`. It covers member declarations in sixteen selected owner types, including runtime, Episode, Store, mission, PX4, world, state-input, derivative, quality receipt, mapping/planning/heading workers and trace store. `DECLARATION_TRIAGE.csv` preserves infrastructure/config and diagnostic candidates separately.

The scanner found **368 declarations**, triaged as **240 behavioral candidates** and **78 diagnostic/test candidates**; the rest are infrastructure/config. Of the 240, **22 KEEP, 3 MOVE, 215 UNRESOLVED**. `DERIVE/MERGE/DELETE` are zero because exact information conservation is not yet proved. These are **scoped candidate counts**, not a whole-product behavioral total. Aggregates such as `RouteProgress.state_`, `CandidateBundle`, execution-state lease payload, mapping model state, mission data, and nested adapter caches must be expanded before claiming 100%.

Every row includes declaration path/line and Git blob hash. Selected-path writer/reader/lifetime evidence is filled for 44 rows; other rows remain `UNRESOLVED` until a complete alias/call graph and transition proof exists. Even filled rows do not assert exhaustive writer coverage or target equivalence. A named field is not a duplicate solely because its name resembles another.

## Current disposition by fact cluster

| Cluster | Conserved fact | Current disposition |
|---|---|---|
| Desired and executing | New waypoint can be desired while previous certified command executes | KEEP both facts; move desired epoch/request out of Episode only after atomic transition proof |
| Active and staged | Future activation and predecessor remain independently observable | KEEP active/pending pointers and activation stamp |
| Physical progress | Previous measured sample and route cursor differ from accepted waypoint | KEEP separately; source time of prior sample needs stronger definition |
| World | Latest snapshot and certified active bundle world may differ | KEEP both identities; establish expiry semantics |
| PX4 | API in flight and VehicleStatus Hold confirmed differ | KEEP independent protocol observations |
| Episode phase/recovery/booleans | May represent orthogonal policy, temporal memory, and mirrors | UNRESOLVED pending reachable-state table and equivalence proof |
| Adapter planner recovery/suffix/mission terminal | May be protocol timing or replicated mission/execution policy | UNRESOLVED; do not delete by name |
| One-shot baseline refinement | Planning-worker quality receipt, described as unpromoted experiment in current safety contract | UNRESOLVED for target; do not promote by migration |

## What remains before coverage can close

1. Expand all nested aggregate fields, candidate/certificate objects and stateful mapping/estimator/bridge/adapter owners.
2. For every row, identify all writes (including aliases, atomics and callbacks), reads, lifetime, invalidation and lock/transaction.
3. Mark independent physical/protocol information and temporal memory separately from caches, diagnostics and accidental copies.
4. Admit any proposed replacement through `TARGET_STATE_MODEL.md` State Admission Test and a runnable equivalence/fault invariant.
5. Reconcile static scanner false positives and exclusions against a compiler-aware AST inventory; until then the matrix is not exhaustive.

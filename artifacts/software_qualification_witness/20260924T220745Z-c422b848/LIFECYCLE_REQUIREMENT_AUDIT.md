# Lifecycle requirement audit

The C0-SW required producer classes are: `PLANNING_CYCLE`, `HEADING_REBIND`, `EMERGENCY_BRAKE`, `TERMINAL_MONITOR`, and `NO_EXECUTION_SIGNAL`. Planning request/result, retained decision, export/activation/publication, supersession, and recovery retry are required phases when that path occurs. The evaluator derives transaction obligations from the actual producer class rather than demanding planner phases for a heading rebind or emergency generation.

| Observed class | Classification | Why |
| --- | --- | --- |
| Planner request/result/retained/commit/export/activation/publication | `C0_SW_REQUIRED` | Establishes exact candidate and execution causality |
| Heading rebind and emergency generation | `C0_SW_REQUIRED` | These create executable bundles without a normal solve owner |
| Recovery retry and supersession/stale discard | `C0_SW_REQUIRED` when emitted | Closes asynchronous work without borrowing a later consumer |
| Terminal monitor | `C0_SW_REQUIRED` when emitted | Explains a safety/terminal decision against its captured generation |
| Core no-execution authorization plus typed adapter rejection | `C0_SW_REQUIRED` when emitted | Proves fail-closed delivery for an intentionally absent command |
| Generic `execution_diagnostics` repetition and planner timing detail | `OPTIONAL_DIAGNOSTIC` | Descriptive; no independent authority transition |
| Physical tracking/ground-truth residual | `C0_IFP_ONLY` | Outside C0-SW performance acceptance |
| Deprecated causal inference from cached prior activation | `DEPRECATED` | Cannot supply a valid owner witness |

`mode_activate`/`deactivate` are external-mode boundary observations, not a planning/bundle lifecycle transaction in this reducer. Their safety implications remain a separate mode protocol; they are not silently attached to the nearest planning cycle. Among reducer-owned fresh lifecycle transactions, no optional transaction class is retained: optional unresolved/conflicts = 0 by empty classification, not a claim of complete generic diagnostics. Historical optional transaction counts cannot be reconstructed with the old event taxonomy; report them as unavailable rather than subtracting them from the 140 required-gap total.

Before this branch, the historical cohort had 140 unresolved lifecycle transactions: 87 missing bundle owner, 30 missing export owner, 23 missing consumer or supersession witness; reported conflicts were 0. Those old records are not reinterpreted as complete. In both fresh cohorts, required unresolved and conflicts are 0 for every run, including the three ineligible verification runs. This is a fresh-witness result, not a repair of old raw data.

The reducer checks transaction status on every exit path. An earlier monitor outcome path failed to contribute to the unresolved count; commit `5a77f5e0` fixed that false-pass risk. Safety-suffix `final_bridge_usable=0` is a valid explicit disposition, not an absent bridge. The emergency terminal monitor outcome uses its own producer event sequence and created generation; it is not silently inherited from a nearby planner transaction.

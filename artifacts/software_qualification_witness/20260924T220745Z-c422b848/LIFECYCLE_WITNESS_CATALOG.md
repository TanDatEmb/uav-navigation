# Lifecycle witness catalog

| Producer class | Originating witness | Required consumer/outcome |
| --- | --- | --- |
| `PLANNING_CYCLE` | Planning cycle, desired revision/request, localization and candidate owner | Exact result, retention/commit or explicit failure/supersession |
| `HEADING_REBIND` | Source type and newly created bundle generation | `heading_admitted` plus exact activation/publication or supersession |
| `EMERGENCY_BRAKE` | Emergency producer source and generation | Emergency planner outcome, safety ownership and explicit terminal monitor disposition |
| `TERMINAL_MONITOR` | Captured bundle generation and producer event sequence | Final-bridge decision and explicit retained disposition |
| `NO_EXECUTION_SIGNAL` | Exact Core rejected authorization sample | Typed adapter rejection of that command identity |

The producer emits the identity; the recorder carries it unchanged. CandidateBundle's immutable `producer_planning_cycle_id` is a diagnostic witness. `NavigationExecutionDiagnostics` includes bundle owner cycle/source. Core also emits heading, supersession, activation, retry, retained-decision and rejected-authorization events at their causal transition. No witness grants flight authority.

`tools/runtime/evaluation.py::reduce_lifecycle` owns evidence consistency checks. A complete transaction is not inferred from temporal adjacency. Conflicting payloads for the same identity remain conflicts rather than last-writer-wins.

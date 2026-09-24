# Open findings

| ID | Class | Finding | Risk | Required evidence | Target |
|---|---|---|---|---|---|
| A-001 | FIX_NOW / CLOSED | Runner requested `tracking=off` while Core/adapter enabled the zero-coefficient bypass under `use_sim_time=true`. | False configuration provenance and non-evaluable nominal evidence. | Explicit policy, live witnesses, C++/Python tests, three tracking-off SITLs. | Phase A |
| A-002 | BLOCKING_FINDING | True tracking-off nominal run A2 accepted `[0,1,2]`, then adapter rejected a command because navigation odometry receive age was 208.583 ms against the existing 200 ms boundary; it requested Hold. A1/A3 completed. | Three consecutive nominal completions cannot be claimed; receive-time tail cause is unresolved. | Correlate odometry producer, DDS callback scheduling, and steady-clock receive timestamps under representative load. Do not relax the boundary. | Phase A follow-up |
| A-003 | RECORD_ONLY | All three evaluator reports are `FAIL/NOT_EVALUABLE` for lifecycle attribution, motion acceptance policy, reference lineage, and tracking coverage policy; A2/A3 also report source timestamp duplication. | Mission completion cannot be promoted to qualification evidence. | Resolve evidence lineage/policy separately; retain raw evaluator reports. | Phase F foundation |
| B-001 | RECORD_ONLY | Isolated world-source stale evidence remains open. | Unknown temporal boundary outcome. | Source-only fault SITL. | Phase B |
| C-001 | RECORD_ONLY | PX4 Hold callback/status ordering and emergency runtime path remain separate debts. | Authority handover uncertainty. | Pinned dependency trace, deterministic protocol tests, focused SITL. | Phase C |

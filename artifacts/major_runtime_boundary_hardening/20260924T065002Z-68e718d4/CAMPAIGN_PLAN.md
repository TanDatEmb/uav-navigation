# Campaign sequence and gates

1. A: establish requested/effective/source runtime configuration truth and run three true tracking-off nominal SITLs.
2. B: define and test the world source/receive/certificate temporal contract; isolate source stale without pausing Core.
3. C: define PX4 Hold handover observations and retry; obtain focused status-order and emergency evidence.
4. D: extract bounded pure policy from RuntimeNode only after B/C semantics close.
5. E: measure command, diagnostic, DDS and PX4 boundary tails with exact identities.
6. F: assemble a reproducible evidence matrix, retaining UNIT/COMPONENT/SITL/HITL/FLIGHT distinctions.

Each phase must pass source audit, invariant, deterministic test, focused SITL,
evidence, and verdict. A partial or regressed phase stops this campaign before
the next phase. No threshold or planner algorithm changes are authorized here.

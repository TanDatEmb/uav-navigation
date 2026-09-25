# World reducer contract

`reduce_world_transactions` is a diagnostic/evidence reducer. It joins only on producer runtime identity, producer sequence, and the declared transaction key. It does not join by nearest timestamp and contains no runtime authority APIs.

For required World sessions it checks:

- event identity and exact transaction-key recomputation;
- active/pending identities when the corresponding generation is nonzero;
- duplicate sequence conflicts and missing internal sequence ranges;
- producer counter monotonicity, runtime-instance continuity, and event-tail agreement;
- exact expected event classes and minimum required transaction count;
- event and producer-counter writer category submitted/accepted/written/dropped accounting;
- event artifacts against writer counts, queue drops, serialization errors, and write errors.

Initial event sequence values may be greater than one because the runtime exists before the scenario observer starts; observed gaps after the first witnessed producer event and a missing final tail are incomplete. C0-SW event classes are required only when the scenario explicitly declares `world_evidence_required`.

Statuses summarize evidence only: `RESOLVED`, `SUPERSEDED`, `SUSPENDED`, `RECERTIFIED`, `RESUMED`, or `INVALIDATED`. Missing identities, sequence gaps, writer errors, and missing required classes make required evidence incomplete. Conflicting identities produce a conflicting result.

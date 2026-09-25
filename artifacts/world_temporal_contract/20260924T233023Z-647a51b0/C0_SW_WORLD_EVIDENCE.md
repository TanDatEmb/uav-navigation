# C0-SW world evidence status

Existing C0-SW lifecycle contract has exact producer-owned identities for planner/result/retention/commit/export/activation/publication and external adapter receipt. Command authorization/publish evidence carries world generation, revision and observation source stamp. Mapping lifecycle observer only reports mutable-map update stamp and shutdown observation-accounting snapshot; mapping diagnostics expose counters and latest world generation/revision/stamp.

The baseline does **not** emit a dedicated producer-owned event chain for `WORLD_PUBLISHED`, recertification start, disjoint fast-path retention, full validation retention, active/pending invalidation, stale callback supersession, freshness suspension/resume or publication failure. The current periodic/latest mapping status is not a transaction identity and cannot fill those lifecycle facts without timestamp inference. No dedicated world event reducer or C0-SW world scenario counts exist.

This is the primary evidence-contract blocker. Do not call existing command world identity sufficient to prove world transaction lifecycle. Required world C0 unresolved/conflict/reference/drop/eligibility metrics are unavailable, not zero.

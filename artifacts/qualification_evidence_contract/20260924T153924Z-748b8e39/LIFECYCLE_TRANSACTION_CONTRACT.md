# Lifecycle transaction contract

One planning cycle can export a candidate without making it executable. One active bundle can then be authorized for many command samples, including retained-command renewals whose current validation cycle differs from the bundle-owner cycle. Thus `causal_planning_cycle_id` on a sample is not its bundle-owner identity.

Required joins:

1. `request → export`: same runtime/session, localization epoch, desired epoch/request, and producer planning cycle.
2. `export → activate`: exact bundle generation and admission context, plus explicit immediate-commit or delayed-activation witness. A later export alone is not proof of prior supersession.
3. `active → authorize`: exact active execution identity, sample ID, authorization boundary/steady stamp and world certificate.
4. `authorize → publish`: exact request, bundle generation, sample ID and world identity; PX4 trace sequence is downstream evidence.

Terminal states are `RESOLVED`, `SUPERSEDED` with an explicit owner transition witness, `INTENTIONALLY_ABSENT` with an explicit rejection/cancellation witness, `MISSING_EVIDENCE`, and `CONFLICTING_EVIDENCE`. The current reducer's `VALID_REJECT` and `INCOMPLETE` vocabulary is narrower. It must not reclassify historical incomplete transactions as superseded merely because a later event has a larger timestamp.

Equal duplicate observations may be deduplicated; contradictory payloads retain a conflict. No action in this branch changes flight authority.

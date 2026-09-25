# Open findings

- `WF-01` CLOSED: isolated mapping-only stale suspension and exact fresh-world resume are observed in 430 ms SITL, with E generation retained.
- `WF-02` CLOSED for long-stale behavior: 700 ms gate isolation, Core suspension and downstream Hold are observed. PX4 re-entry remains NEXT_PHASE and is not claimed.
- `WF-03` CLOSED at deterministic component level: unsafe fresh world cannot resume suspended execution. No geometry-backed SITL was required or fabricated.
- `WF-04` CLOSED for C0-SW World evidence: session scenario requirements now survive evaluator config loading; exact reducer resolves World transactions with zero loss/conflict/missing references. Overall C0-SW remains NOT_EVALUABLE for unrelated policy/lineage blockers.
- `WF-05` retained: runtime event producer CPU cost and writer queue high-water are not exposed; no loss occurred in counted sessions.
- `WF-06` deferred/out of scope: N2 nominal trial took terminal/recovery fail-closed after `final_bridge_usable=false`; this was not a World stale transition. Do not change terminal policy in this branch.
- `WF-07` retained: historical same-source-timestamp higher revision remains unreachable through ordinary live MappingActor; A3 stays `STORE_CONTRACT_PERMITS / LIVE_PRODUCER_DOES_NOT_EMIT`, historical raw proof NOT_EVALUABLE.
- `WF-08` closed by raw-session re-evaluation: the 420 ms configured run also crossed the effective World source-age boundary (producer events show suspension on W2/rev191 followed by W2/rev192 recertification/resume). The original report had `required=false`; current evaluator honors the session manifest. It is supplemental because its source-content digest differs from the primary run.

# Qualification scope output contract

The report has five independent concepts. None is inferred from another.

| Output | Meaning | Owner |
| --- | --- | --- |
| `runtime_verdict` | Runtime/mission runner outcome, including runtime stream and scenario gates. | Runtime report builder |
| `software_qualification` | Single-session C0-SW software contract assessment (`C0_SW`). | Versioned software evaluator |
| `integrated_flight_qualification` | C0-IFP integrated performance assessment (`C0_IFP`). | Versioned integrated evaluator |
| `single_session_evidence_completeness` | Whether this session contains complete required single-session evidence. | Versioned evaluator evidence dimension |
| `multi_run_qualification` | Cross-run qualification status. A single-session report cannot establish it. | Multi-run aggregator; `NOT_EVALUABLE` here |

The legacy top-level `verdict` remains for schema compatibility and is an alias of `runtime_verdict`. It is not a qualification verdict. The legacy `evaluation` remains the aggregate C0-IFP assessment; its scope is explicit in `evaluation.qualification_scope`. The legacy top-level `qualification_scope` remains the requested/selected run scope and does not relabel `evaluation`.

The report boundary validates evaluator schema and records failures in `evaluation_contract_errors`. Malformed or non-PASS qualification output may make qualification ineligible, but it cannot rewrite a runtime PASS into a runtime FAIL. A real runtime failure, such as a measured input-stream freshness violation, remains in `runtime_verdict` and `reasons`.

C0-SW defers the following C0-IFP-only conditions and must not inherit them as C0-SW blockers: unavailable motion acceptance policy, unavailable tracking acceptance policy, absolute tracking quality outside an unapproved C0-IFP threshold, and tracking ground-truth pair lineage mismatch. C0-SW continues to require its own exact lifecycle and reference evidence; `C0_SW_REQUIRED_REFERENCE_LINEAGE_MISSING` and `C0_SW_REQUIRED_REFERENCE_LINEAGE_CONFLICT` remain blocking.

A `PAUSED_SAFETY_STOP` can be assessed as C0-SW PASS only with a producer witness joining the exact current planner result and retained-decision request to the active execution, a current monitor window, recent measured state, valid anchor/body facts, an unusable command bridge, and the resulting fail-closed state. A status label or PX4 Hold without this exact witness remains `NOT_EVALUABLE`. This means the software decision path was correctly exercised; it does not claim a completed mission or acceptable physical performance.

The single-session result never claims multi-run eligibility. It emits `multi_run_qualification.assessment_status=NOT_EVALUABLE` with `MULTI_RUN_AGGREGATION_NOT_PERFORMED` until a separate cohort aggregator evaluates the required runs.

No raw historical session is rewritten to apply this contract. Re-evaluations are emitted into the campaign artifact directory.

# Qualification scope reconciliation

The report bug was confirmed in `tools/runtime/report.py`: the report builder treated the top-level C0-IFP aggregate as a runtime verdict gate, mutating runtime PASS to FAIL when C0-IFP was NOT_EVALUABLE. It then labeled the report's `qualification_scope` from the nested C0-SW assessment while retaining the C0-IFP object under `evaluation`. That made one report appear to have one scope while showing a different scope's assessment.

The report builder now keeps the runtime verdict unchanged and publishes independent `software_qualification` (C0-SW), `integrated_flight_qualification` (C0-IFP), `single_session_evidence_completeness`, and `multi_run_qualification` objects. The legacy `verdict` aliases `runtime_verdict`; legacy `evaluation` remains C0-IFP; legacy `qualification_scope` reflects the requested scope. Evaluator schema issues are reported under `evaluation_contract_errors`, not converted into runtime failures.

The C0-SW evaluator continues to reject its own missing/conflicting lifecycle and producer-owned reference evidence. It does not inherit C0-IFP-only tracking policy, motion policy, or tracking/ground-truth `REFERENCE_LINEAGE_MISMATCH`. Exact producer-backed retained fail-close evidence may make a safety-stop product decision assessable; a safety-stop label alone remains NOT_EVALUABLE. The no-execution lifecycle reducer now recognizes an exact rejected authorization with zero bundle/cycle even when the current desired goal epoch is nonzero, and still requires exact typed adapter rejection evidence.

Focused golden coverage is Q1 in report projection tests, Q2 exact fail-close safety-stop witness tests (plus the status-only negative test), Q3 missing C0-SW reference lineage tests, and Q4 C0-IFP tracking-lineage isolation tests. These tests do not make C0-IFP eligible or change any motion/tracking acceptance policy.

Fresh evaluator output for retained World/N sessions is in `WORLD_SESSION_REEVALUATION.md`; raw sessions were not modified.

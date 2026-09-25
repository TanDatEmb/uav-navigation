# Evidence index and limits

| Claim | Evidence | Class | Limit |
|---|---|---|---|
| Incoming verdict and provenance | `INPUT_DELIVERY.md`; incoming `artifacts/qualification_evidence_contract/20260924T153924Z-748b8e39/VERDICT.md`, `BASE_PROVENANCE.md`; live Git/PX4 SHA checks | Source and provenance | Incoming PX4 checkout has unrelated pre-existing dirty paths. |
| Lifecycle 140, lineage 5,957/32,745, 0/10 eligible | Incoming `LIFECYCLE_UNRESOLVED_CLASSES.md`, `REFERENCE_LINEAGE_RESULTS.md`, `RAW_SESSION_REEVALUATION.csv`, `QUALIFICATION_ELIGIBILITY.md` and raw replay script | Raw-session reevaluation | Missing historical producer IDs cannot be retroactively supplied. |
| Timestamp semantics and A3 conflict | Incoming `STREAM_TIMESTAMP_POLICY.md`, `WORLD_EVIDENCE_CONFLICT.md`; `tools/runtime/evaluation.py` | Source, unit, historical replay | No proof of a World product defect or resolution of A3. |
| Policy fields and fail-closed outcome | `tools/runtime/evaluation.py` functions `_tracking_coverage`, `_tracking_acceptance_status`, `evaluate_session`; incoming `TRACKING_POLICY_CONTRACT.md`, `MOTION_POLICY_CONTRACT.md`, `POLICY_PROVENANCE.md` | Source and policy-provenance audit | Missing approved values cannot be inferred from test fixtures or cohort maxima. |
| Safety versus qualification boundary | `docs/safety/runtime_safety_current.md` non-negotiable invariants, HG-004/HG-007; targeted history `DEC-20260828-001`, `DEC-20260916-009` | Current contract and targeted history | Command/planner gates do not become independent-truth acceptance policy. |
| Ten-run descriptive ranges and calculation | Incoming tracking/motion contract reports; `OPTIONS_ANALYSIS.md` | Descriptive analysis | No independent holdout or risk allocation; no numeric policy approved. |
| Deferred simulator/PX4 findings | Incoming `SCOPE_AND_DEFERRED_FINDINGS.md`, prior product-stability terminal/temporal artifacts | Historical SITL observation | No new causal attribution or runtime reproduction. |

All incoming artifact names above reside under `artifacts/qualification_evidence_contract/20260924T153924Z-748b8e39/` unless otherwise specified. Large raw session data and PX4 logs remain outside Git with paths and hashes in the incoming evidence manifest.

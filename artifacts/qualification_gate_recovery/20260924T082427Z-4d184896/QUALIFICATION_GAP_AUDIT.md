# Initial evaluator gap audit

All A1/A2/A3 reports are `FAIL/NOT_EVALUABLE`; none may be promoted to
qualification evidence. The following findings come from the versioned
`evaluation` object and source inspection, not guessed thresholds.

| Reason | Initial class | Concrete evidence / action |
| --- | --- | --- |
| `LIFECYCLE_ATTRIBUTION_INCOMPLETE` | PRODUCT_EVIDENCE_MISSING | A1 has 10 unresolved transactions. Repeated sample lifecycle records include missing activate phases and authorize/publish groups with no export owner; audit exact causal keys before reducer changes. |
| `MOTION_ACCEPTANCE_POLICY_UNAVAILABLE` | SCENARIO_POLICY_MISSING | No versioned motion acceptance policy is recorded in the nominal scenario. A numeric policy must be approved, not inferred from completion. |
| `REFERENCE_LINEAGE_MISMATCH` | REFERENCE_LINEAGE_MISSING | `_reference_lineage_status` rejects the current reference provenance. Inspect exact mission/route/frame/source fields and producer mapping. |
| `TRACKING_COVERAGE_POLICY_UNAVAILABLE` | SCENARIO_POLICY_MISSING | The scenario has no `min_coverage_ratio`, `max_uncovered_interval_s`, or `max_pairing_gap_s`. Do not copy test fixture values into production. |
| `SOURCE_TIMESTAMP_DUPLICATE` | QUALIFICATION_EVIDENCE_PIPELINE | A2's 15 duplicate rows are exact PVA heartbeats and can be accounted for without losing raw evidence. A3 includes two same-stamp world-revision changes, so its duplicate reason remains after the focused evaluator repair; see `SOURCE_TIMESTAMP_DUPLICATES.md`. |
| `REQUIRED_DIMENSION_NOT_PASS` | DERIVED_RESULT | Follows unresolved required dimensions above; not a root cause to suppress. |

The A2 mission failure itself is `EXPECTED_SAFETY_RESPONSE` to a stale
accepted odometry receive lease, with the upstream timing origin still under
investigation. A1/A3 mission COMPLETE remains functionally distinct from
evaluator eligibility.

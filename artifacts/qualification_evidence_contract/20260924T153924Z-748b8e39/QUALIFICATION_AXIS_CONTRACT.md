# Qualification axes and scope

| Axis | Required witness | Valid outcome vocabulary | Present status |
|---|---|---|---|
| PRODUCT_LOGIC | Exact mission/execution/adapter lifecycle and correct safety disposition | PASS / FAIL / NOT_EVALUABLE | NOT_EVALUABLE from lineage gap |
| TEMPORAL_SAFETY | Source/receive clocks, admission/lease and complete critical capture | PASS / FAIL / NOT_EVALUABLE | Can be assessed per observed fault, not whole mission from missing lifecycle |
| EVIDENCE_COMPLETENESS | Finalized writer counts, zero critical drops, exact identities and frame/clock witnesses | COMPLETE / INCOMPLETE / CONFLICTING | INCOMPLETE in all ten pinned runs |
| FLIGHT_PERFORMANCE | Approved tracking and measured-motion acceptance policy plus independent truth | PASS / FAIL / NOT_EVALUABLE / DEFERRED | NOT_EVALUABLE; D-002 deferred |
| ENVIRONMENT_VALIDITY | Simulation/clock/sensor progress witness | VALID / INVALID / DEFERRED / NOT_EVALUABLE | D-001 deferred for historical stall |

`qualification_eligible` in the current evaluator is the existing integrated navigation-quality boolean. It is **false** for all ten pinned runs and is not silently repurposed to software-only qualification. A future software-specific eligibility field may be added only after exact lifecycle and scope criteria are implemented; an eligible FAIL would then remain FAIL. Correct rejection of a 0.762 m terminal anchor can be software safety PASS while integrated flight performance remains failed/deferred, but that judgment requires the exact gate and measurement witness, not merely mission outcome.

Blocker taxonomy: `MISSING_EVIDENCE`, `CONFLICTING_EVIDENCE`, `UNKNOWN_CLOCK`, `INVALID_LINEAGE`, `MISSING_POLICY`, `INSUFFICIENT_COVERAGE`, `ENVIRONMENT_DEFERRED`, `FLIGHT_PERFORMANCE_DEFERRED`, `PRODUCT_FAILURE`. Existing free-text reasons should map to these without discarding their specific diagnostic codes.

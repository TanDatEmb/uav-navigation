# Eligibility result

The frozen ten-run product-stability cohort remains 8 mission COMPLETE / 2 PAUSED_SAFETY_STOP and **0/10 integrated qualification eligible** under re-evaluation from raw JSONL. Each run has `LIFECYCLE_ATTRIBUTION_INCOMPLETE`, `REFERENCE_LINEAGE_MISMATCH`, `TRACKING_COVERAGE_POLICY_UNAVAILABLE`, and `MOTION_ACCEPTANCE_POLICY_UNAVAILABLE`; `REQUIRED_DIMENSION_NOT_PASS` follows. Of 32,745 raw executable references, 5,957 lack exact qualification lineage. This is not a report-format artifact: it is derived by `load_evaluation_inputs()` and `evaluate_session()` on the raw session.

No five-run new nominal cohort is used to claim closure while these independent blockers persist. The target of three consecutive software-eligible runs is unmet. The two terminal safety stops remain in the original denominator and retain their original mission outcomes.

The prior ten scenario writer summaries each report zero dropped, snapshot-rejected, serialization-error and write-error records. This narrows the known blockers to attribution/lineage and policy provenance; it does not prove every producer emitted its required event.

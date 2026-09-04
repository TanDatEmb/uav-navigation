# E5 causal analysis

This analysis uses only the retained pre-telemetry E5 artifact. Missing fields are explicitly marked `NOT_RECORDED`; no causal value is inferred from waypoint geometry or from a later command sample.

## Measured boundaries

- T0: sample_id=40, timestamp_ns=20468000000, generation=1, role=MAIN, analytic_role=MAIN, time_to_backup=1.8774401711603277 s.
- T1: injected solve start timestamp is `NOT_RECORDED`; cycle=5, solve_generation=2.
- T2: injected failure is recorded as candidate_result=4, failure_stage=1, failure_reason=15; exact return timestamp_ns is `NOT_RECORDED`.
- T3: emergency decision log timestamp=1788513732.263620111; exact ROS timestamp_ns and predicate result are `NOT_RECORDED`.
- T4: sample_id=41, timestamp_ns=20484000000, generation=2, role=EMERGENCY, analytic_role=EMERGENCY.

## C1–C5

- C1: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The old command stream has no anchor_error_m or retained_tracking_limit_m.
- C2: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The old command stream has no projected_anchor_error_m.
- C3: measured T0 `backup_available=True`; the retained MAIN command reports backup_start_time_s=2.517440171160328 and time_to_backup_start_s=1.8774401711603277.
- C4: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. `committedSafetySuffixIsUsable` and `sampled_path_clear` were not serialized. The log proves that the exceeded-anchor emergency path was entered, but cannot distinguish the individual suffix-usability inputs.
- C5: `NOT_IDENTIFIABLE_FROM_OLD_ARTIFACT`. The exact authorization predicate was not serialized. The source code contains the predicate, but this artifact alone is insufficient to claim its runtime branch without the new telemetry.

## Required next evidence

Add the causal fields to DECISION_TRACE and NavigationCommand, then rerun the controlled failure with the retained MAIN margin preconditions. H4 root cause remains unresolved until that run records the authorization reason.

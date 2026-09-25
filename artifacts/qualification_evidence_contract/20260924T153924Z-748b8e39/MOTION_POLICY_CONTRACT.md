# Motion policy contract

`evaluate_motion_quality()` reports measured vector acceleration/jerk, stop-go, chattering, role switches, completion time and mission guidance deviation. The current evaluator deliberately leaves motion descriptive and emits `MOTION_ACCEPTANCE_POLICY_UNAVAILABLE`; source comment attributes this to the unresolved W9 acceptance contract. The mission guidance polyline deviation is explicitly `descriptive_only`, so it cannot be promoted to a qualification failure by this branch.

The existing MAIN command envelope 5/5/8 and physical/BACKUP envelope 12/12/30 bound candidate generation/certification, not a post-hoc measured acceleration/jerk distribution or stop-go count. A measured motion acceptance policy needs a declared flight-performance objective, filtering/time basis, segment exclusions and independent validation cohort. Picking the maximum of this cohort would be circular. Motion qualification remains `POLICY_DECISION_REQUIRED` and `NOT_EVALUABLE`; structural candidate certification is reported separately as software evidence.

Ten-run characterization from the prior canonical reports (minimum / median / maximum of each run's statistic):

| Metric | Per-run p95 range and median | Per-run maximum range and median |
|---|---:|---:|
| Measured vector acceleration, m/s² | 1.087 / 1.521 / 4.140 | 2.883 / 6.929 / 11.379 |
| Sampled command acceleration, m/s² | 0.746 / 1.037 / 2.175 | 1.887 / 4.749 / 4.962 |
| Sampled command jerk, m/s³ | 1.584 / 4.565 / 13.531 | 6.846 / 23.088 / 28.790 |
| Speed-change rate, m/s² | 1.046 / 1.427 / 3.534 | 2.786 / 6.439 / 10.953 |

The sampled-command jerk aggregate mixes candidate roles: comparing its 28.790 maximum directly to MAIN 8 would be a category error; physical/BACKUP permits a different envelope. A policy must segment by role and use exact active execution lineage before applying either bound.

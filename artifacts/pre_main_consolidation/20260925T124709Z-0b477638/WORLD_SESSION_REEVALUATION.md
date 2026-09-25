# Incoming World sessions reevaluated with current qualification scope code

Raw source sessions are read only. The report below is generated from current evaluator code and does not overwrite their report.json files. Runtime outcome is recomputed with _sim_report; C0-SW and C0-IFP are emitted independently.

## 430ms primary — external-mode-check-20260925T091243-356218

- Runtime: **PASS**; mission COMPLETE; runtime reasons: [].
- C0-SW: **PASS**, eligible True, lifecycle unresolved/conflicting 0/0, stop witness False, blockers [].
- C0-IFP: **NOT_EVALUABLE**, eligible False, blockers ['MOTION_ACCEPTANCE_POLICY_UNAVAILABLE', 'REFERENCE_LINEAGE_MISMATCH', 'REQUIRED_DIMENSION_NOT_PASS', 'TRACKING_COVERAGE_POLICY_UNAVAILABLE'].

## 700ms long stale — external-mode-check-20260925T092009-360731

- Runtime: **BLOCKED**; mission PAUSED_SAFETY_STOP; runtime reasons: ['External Mode exited before mission completion', 'mission completion event was not observed', 'waypoint acceptance coverage incomplete: expected [0, 1, 2, 3, 4], got []', 'mission did not reach COMPLETE outcome: PAUSED_SAFETY_STOP'].
- C0-SW: **NOT_EVALUABLE**, eligible False, lifecycle unresolved/conflicting 0/0, stop witness False, blockers ['PRODUCT_OUTCOME_UNATTRIBUTED', 'SAFETY_STOP_GATE_DECISION_WITNESS_NOT_ASSESSED'].
- C0-IFP: **NOT_EVALUABLE**, eligible False, blockers ['MOTION_ACCEPTANCE_POLICY_UNAVAILABLE', 'REQUIRED_DIMENSION_NOT_PASS', 'TRACKING_COVERAGE_POLICY_UNAVAILABLE'].

## N1 — external-mode-check-20260925T092305-366082

- Runtime: **FAIL**; mission COMPLETE; runtime reasons: ['lidar timestamp/freshness/validity violation'].
- C0-SW: **PASS**, eligible True, lifecycle unresolved/conflicting 0/0, stop witness False, blockers [].
- C0-IFP: **NOT_EVALUABLE**, eligible False, blockers ['MOTION_ACCEPTANCE_POLICY_UNAVAILABLE', 'REFERENCE_LINEAGE_MISMATCH', 'REQUIRED_DIMENSION_NOT_PASS', 'TRACKING_COVERAGE_POLICY_UNAVAILABLE'].
- LiDAR: source stale events 1, maximum source gap 600.0 ms, max wall arrival gap 751.851232 ms.

## N2 — external-mode-check-20260925T092630-369436

- Runtime: **BLOCKED**; mission PAUSED_SAFETY_STOP; runtime reasons: ['External Mode exited before mission completion', 'mission completion event was not observed', 'waypoint acceptance coverage incomplete: expected [0, 1, 2, 3, 4], got [0, 1, 2, 3]', 'mission did not reach COMPLETE outcome: PAUSED_SAFETY_STOP'].
- C0-SW: **PASS**, eligible True, lifecycle unresolved/conflicting 0/0, stop witness True, blockers [].
- C0-IFP: **NOT_EVALUABLE**, eligible False, blockers ['MOTION_ACCEPTANCE_POLICY_UNAVAILABLE', 'REFERENCE_LINEAGE_MISMATCH', 'REQUIRED_DIMENSION_NOT_PASS', 'TRACKING_COVERAGE_POLICY_UNAVAILABLE', 'tracking.position_error_m source incomplete', 'tracking.velocity_error_mps source incomplete'].

## N3 — external-mode-check-20260925T092909-373055

- Runtime: **PASS**; mission COMPLETE; runtime reasons: [].
- C0-SW: **PASS**, eligible True, lifecycle unresolved/conflicting 0/0, stop witness False, blockers [].
- C0-IFP: **NOT_EVALUABLE**, eligible False, blockers ['MOTION_ACCEPTANCE_POLICY_UNAVAILABLE', 'REFERENCE_LINEAGE_MISMATCH', 'REQUIRED_DIMENSION_NOT_PASS', 'TRACKING_COVERAGE_POLICY_UNAVAILABLE'].

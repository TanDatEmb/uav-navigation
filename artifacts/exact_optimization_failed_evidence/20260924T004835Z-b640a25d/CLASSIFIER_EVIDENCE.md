# Classifier evidence

`PlannerStatus::kOptimizationFailed` has the exact existing wire enum code 6. `classifyPlannerResult()` has an explicit case: available incumbent → `RetainCommittedCommand` (disposition code 4); no incumbent → `FailClosed`. The hook changes `PlannerStatus` before this call and does not patch disposition. Runtime `planner_trace` now records original status, actual post-hook status, actual disposition and one-shot flag. The focused analyzer requires code 6 and disposition 4 in the same planning cycle as `FAULT_INJECTION_APPLIED`.

Existing `PlannerFsm.HotRetargetOptimizationFailureDoesNotRevokePredecessor` and status table tests cover the source/component classifier. The new tests cover semantic one-shot selection and invalid-retention fallback. Neither substitutes for SITL evidence.

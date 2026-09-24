# Semantic trigger contract

Explicit test parameters: `inject_exact_optimization_failed_once=true`, `inject_exact_optimization_predecessor_request=2`, `inject_exact_optimization_successor_request=3`. The runner writes them only when requested. Invalid IDs or combining with older failed-replan hooks rejects startup.

The one-shot hook requires a real completed backend solve; a current `PlanningKey` with committed-future-state start mode; same-route waypoint advance and PASS_THROUGH desired goal; desired request 3 against active request 2 in the same mission/localization/route; exact captured `ExecutionAuthoritySnapshot` still current; available, valid, nonterminal MAIN predecessor with backup certificate and unexpired bundle interval; current source-fresh state and world; no PlanFromRest, terminal hold, watchdog timeout or existing diagnostic failure. It replaces only `PlannerStatus` after all stale-result checks and before the real classifier.

The trigger is a selection filter, **not** HG-023 authorization. The unchanged validator rechecks current world, measured state, anchor, safety suffix, bundle interval and owner identity. A failed HG-023 gate remains a safety fallback. `FAULT_INJECTION_ARMED` and `FAULT_INJECTION_APPLIED` are emitted only when the semantic trigger fires; a focused harness run without exactly one of each fails `INJECTION_NOT_ARMED`.

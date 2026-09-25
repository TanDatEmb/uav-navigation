# Product behavior diff

Product C++ changes are confined to commit `d62319fd` and add diagnostic-only immutable provenance (`CandidateBundle.producer_planning_cycle_id`, execution-diagnostic message fields), exact lifecycle event emission and retained-decision observation. They do not change `NavigationCommand.msg`, command content/admission, `ExecutionAuthority`, PX4 setpoint updates, safety gates, planner algorithm or threshold constants. Later commits change recorder, evaluator, runner/report and tests. `tools/check_software_qualification_scope.py` enforces the allowlisted product-source diff; it is a static guard, not a timing proof.

Source review and deterministic tests support a behavior-neutral intent. The primary five missions completed at the instrumented source. Diagnostic emission occurs on product paths, and its exact latency overhead versus the uninstrumented base was **not independently measured**. This residual risk is recorded in `PERFORMANCE.md`; the branch does not claim byte-for-byte execution timing equivalence.

The independent owner domains remain MissionProgress, DesiredPlanningIntent, ExecutionAuthority, WorldModel and PX4 adapter. This branch creates no flight authority owner or mutable control state mirror.

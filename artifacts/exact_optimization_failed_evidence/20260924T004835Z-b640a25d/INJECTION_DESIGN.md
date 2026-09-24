# Injection design

A single explicitly configured diagnostic hook observes the immutable `PlanningKey`, desired goal revision and captured `ExecutionAuthoritySnapshot` after a real backend solve. Only the `PlannerStatus` delivered to the existing `classifyPlannerResult()` is replaced with `kOptimizationFailed`; the backend outcome is preserved as original evidence. The staged candidate is discarded by the normal non-CommandReady path. The unchanged HG-023 retained-command validator then runs.

Default is off. The runtime rejects explicit enablement outside the `sitl` deployment profile. The runner writes injection parameters only for an explicit test invocation; production YAML has no enabling parameter. The hook never supplies a candidate, certificate, freshness witness, command authority, or retained-validation result. Its one-shot `consumed` bit is test-only bookkeeping. Normal disabled execution skips the eligibility work. The status substitution adds no persistent product authority field.

Structured `FAULT_INJECTION_ARMED` and `FAULT_INJECTION_APPLIED` diagnostics contain the exact cycle, desired/active identities, owner version/lineage, world identity, original status, substituted status and times. `planner_trace` records the actual status and classifier disposition. Absence of the two events is an explicit `INJECTION_NOT_ARMED` result in the evidence analyzer.

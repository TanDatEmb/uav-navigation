# Source path

1. `NavigationRuntimeNode::runCycle` captures immutable `PlanningKey` and `ExecutionAuthoritySnapshot` before `planner_->plan(planning_request)`.
2. Backend returns a real `PlanningOutcome`. Runtime maps it to `PlannerStatus`, preserving `planner_result_before_injection` and `planner_backend_outcome` for diagnostics.
3. After watchdog, localization, desired and terminal-hold stale checks, the explicitly enabled one-shot hook verifies the exact current hot handoff and substitutes only `result=kOptimizationFailed`.
4. Product `classifyPlannerResult()` maps `kOptimizationFailed` plus an available command to `RetainCommittedCommand`; no available command maps to `FailClosed`.
5. The existing non-CommandReady path discards the backend's private staged candidate. The `RetainCommittedCommand` path calls `validateRetainedCommand()` with the captured `execution_at_solve` witness; its diagnostics identify actual validation and delivery.
6. Command publication and adapter admission remain independent. Bag `/navigation/navigation_command` and `/navigation/command_admission` are matched by exact mission/request/generation/sample; bag `/fmu/in/trajectory_setpoint` is only a recorder observation, not firmware consumption proof.

The code path is source proof only. Per-run proof requires the event, planner trace, retained-decision trace, command/admission continuity, successor cutover and absence of unexpected Hold.

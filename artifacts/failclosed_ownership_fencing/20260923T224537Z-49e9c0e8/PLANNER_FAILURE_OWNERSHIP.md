# Planner failure ownership

## Provenance and policy

Target source is base `49e9c0e8`; HG-023 in `docs/safety/runtime_safety_current.md` and targeted archive line 3384 is the controlling retained-command contract. A failed replacement solve does not revoke a still-certified incumbent merely because the desired successor still matches. Retention continues only through the existing latest-world, anchor, suffix, identity and lease validator. This repair changes result routing and callback ownership, not those gates.

`PlanningKey` already carries localization epoch, desired goal revision, request, route revision, predecessor generation, world generation/revision, start mode, anchor timestamp and dynamics hash. The solve additionally captures an ephemeral `ExecutionAuthoritySnapshot` and exact pending-goal pointer. The snapshot is checked before planner-result fail-close, stopped timeout and retained validation; the candidate commit path retains its existing `PlanningKey`/commit-token fences.

`classifyPlannerResult()` now explicitly names every current `PlannerStatus`. `OptimizationFailed` and `Success`/`Finished` without a committed candidate route a still-available active command into `RetainCommittedCommand`. An unknown future status cannot receive destructive authority over an available incumbent through a default return. Without a command, current no-command fail-close/retry behavior remains.

The async result mutates execution only after the desired revision/localization check **and** `ExecutionAuthority::failClosedIfCurrentSnapshot()` accepts the exact captured solve snapshot. A newer active generation, admission revision, world transaction, lifecycle transition or reset makes the result stale. The owner method returns applied/stale/already-failed; RuntimeNode clears completion witnesses only for an applied mutation.

`RestartFromRest` is also fenced by the captured solve execution before setting a completion witness or restart request. This prevents a late local-boundary result from attaching recovery state to a newer command with the same desired goal.

The retained-command validator checks the solve's expected execution at entry, then rechecks its own captured owner/version, desired request and exact final state lease before delivery. A newer execution must not be reinterpreted as the incumbent of the old solve. A failed certificate for the exact current incumbent still takes the existing brake/fail-close path.

## Deliberate limits

The small deterministic classifier/owner tests are not evidence that a runtime optimizer actually emitted `OptimizationFailed` during an airborne hot handoff. That requires the focused injected scenario or is reported as a runtime gap in `SITL_RESULTS.md`.

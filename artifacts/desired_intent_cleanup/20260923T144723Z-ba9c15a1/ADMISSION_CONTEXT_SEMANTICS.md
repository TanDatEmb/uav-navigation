# Admission Context Semantics

## Source pin

Source: `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`.

## Desired revision and admission fence

`active_goal_epoch_` is a monotonically advanced desired-planning revision, despite its name. RuntimeNode advances it when accepting a new desired goal (`navigation_runtime_node.cpp:2253-2271`). It is copied into the planning request key alongside request, route, predecessor generation, localization, and world context (`3384-3395`).

ExecutionAuthority stores `admission_goal_epoch_` and `admission_localization_epoch_` as its consumer-side stale-result fence. `beginGoal` advances those admission values and can retain the active predecessor without relabeling its identity (`committed_bundle_store.hpp:121-141`). Candidate stage/commit rejects when the captured goal epoch differs from the current admission fence (`committed_bundle_store.hpp:447-466`, `488-504`, `620-632`). The admission values are exposed in an authority snapshot as admission fields (`340-345`).

These values may be numerically equal after a desired transition and still have different roles:

- Desired owner produces the planning revision and attaches it to an immutable request/key.
- ExecutionAuthority stores the latest allowed admission revision and rejects results outside that fence.
- The active record keeps the goal epoch of the command actually admitted earlier. While successor N+1 is desired, active predecessor N retains its old bundle/goal identity until cutover.

This is producer context plus consumer fence, not two active-execution identities. Never call the admission value an active execution epoch. Active execution is identified by ExecutionAuthority's active record/token, including its bundle generation, goal, and localization identity.

## Naming recommendation

Rename RuntimeNode's desired producer revision to `PlanningIntentRevision` or `desired_intent_revision`. Preserve explicit `admission_*` names in ExecutionAuthority. Do not rename admission revision to an active/executing epoch, and do not expose its read-only fence copy as authority over desired state.

No dependency from the desired-intent value to ExecutionAuthority is needed. RuntimeNode passes typed goal/revision/context to the owner; ExecutionAuthority need not inspect MissionProgress internals.

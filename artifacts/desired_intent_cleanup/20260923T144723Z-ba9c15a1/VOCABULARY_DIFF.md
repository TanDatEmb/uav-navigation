# Desired and Identity Vocabulary Diff

## Provenance and scope

Baseline: `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`. After counts are pinned to source/test commit `c9006768b3f75180613319aa93fd8de62af4dae5`. They describe RuntimeNode `.hpp`/`.cpp` source only; tests, docs, and historical artifacts are excluded.

## Legacy desired names and identity helpers

Counts below are exact identifier-token matches across `navigation_runtime_node.hpp` and `navigation_runtime_node.cpp`. `active_goal_` is counted as an exact identifier, excluding the separate `active_goal_epoch_` identifier.

| Product-source identifier | Before | After (`c9006768`) |
|---|---:|---:|
| `active_goal_` | 96 | 0 |
| `active_goal_epoch_` | 27 | 0 |
| `new_goal_` | 16 | 0 |
| `hot_goal_transition_` | 22 | 0 |
| `desiredGoalIdentityMatchesLocked` | 16 lexical occurrences (declaration + definition + 14 calls) | 0 |
| `executingCommandIdentityMatchesLocked` | 16 lexical occurrences (declaration + definition + 14 calls) | 0 |

Current desired identity decisions use `desired_intent_.matches(...)`, for example at `navigation_runtime_node.cpp:2692,4435,4576,4923,5648,5853,8539,8586`. Active execution identity is queried through `execution_authority_.matchesActive(...)` or owner snapshots, for example at `:1252,2387,2705,4540,5606,5781,8411,8478`. The two roles remain separate when a condition needs both.

## Raw comparison counts versus helper migration

For a reproducible narrow count, the raw operator metric counts each source expression in RuntimeNode `.hpp`/`.cpp` where `==` directly touches one of `mission_id`, `waypoint_index`, `request_id`, `goal_epoch`, `bundle_generation`, or `route_revision` on either side:

| Metric | Before | After (`c9006768`) |
|---|---:|---:|
| Direct `==` expressions over the six named fields | 99 | 88 |
| `desiredGoalIdentityMatchesLocked` calls | 14 | 0 |
| `executingCommandIdentityMatchesLocked` calls | 14 | 0 |

The equality-expression count is a syntactic inventory, not a count of redundant or semantically equivalent identity predicates. Remaining comparisons include independent mission progression, external protocol checks, route checks, owner invariants, and legitimate desired-plus-execution completion conditions. The earlier audit's 102 was a broader search upper bound; this 99 → 88 metric uses only direct `==` expressions touching the six listed fields. Helper call-site elimination is reported separately so owner/API centralization is not confused with the residual raw equality count.

## Vocabulary rule after this cut

- **Desired planning intent** is the goal and revision currently requested by mission/planning control.
- **Admission fence** is ExecutionAuthority's last admitted goal/localization context used to reject stale results.
- **Active execution** is the exact active command/goal record owned by ExecutionAuthority.

Names containing `active` remain appropriate for active execution records and bundles. They must not describe desired planning intent. The `c9006768` RuntimeNode source has zero occurrences of the four removed desired-state identifiers and both removed RuntimeNode identity helpers.

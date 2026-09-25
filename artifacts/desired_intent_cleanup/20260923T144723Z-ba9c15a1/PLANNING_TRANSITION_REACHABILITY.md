# Planning Transition Reachability

## Source pin and write inventory

Source: `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`.

Product writes to the persistent pair are:

- Initialization in the header sets both false: `navigation_runtime_node.hpp:508,512`.
- Localization reset derives the new-goal flag from whether a desired goal remains and clears hot: `navigation_runtime_node.cpp:1841-1843`.
- Foreign-mission transition and terminal/cancel clearing set both false: `2112-2117`, `2485-2494`.
- Accepted new goal assigns complementary values in one locked transition: `2305-2307` (`hot_goal_transition_ = effective_hot_retarget`, `new_goal_ = !effective_hot_retarget`).
- Other writers clear hot alone, clear new alone after successful commit, or clear both after terminal transfer/activation: `1171`, `4140`, `4243`, `4260`, `4733`, `4786`, `5694`, `5939-5944`, `8129-8133`, `4658-4660`.

The remaining reads copy the flags under the existing input lock or use them in already serialized control decisions: `2753`, `3313-3314`, `3437-3447`, `3716-3718`, `4815`, `5040`, `5426`, `7071-7072`, `7814`. Planner-worker request state is captured separately as a `PlanningKey`; the worker does not read these live flags (`3448-3455`, `3384-3395`).

## Reachable logical combinations

| `new_goal` | `hot_goal_transition` | Product meaning | Reachable? |
|---:|---:|---|---|
| 0 | 0 | No outstanding desired-goal transition | Yes: initialization, clear, or consumed transition |
| 1 | 0 | Desired goal needs measured-state/PlanFromRest handling | Yes: accepted non-hot goal and localization reset with a retained goal |
| 0 | 1 | Desired successor is eligible for hot-retarget handling | Yes: accepted transition where `effective_hot_retarget` is true |
| 1 | 1 | No defined meaning; would request both mutually exclusive dispositions | No product write produces this logical state |

The only product write that can set either flag true assigns the complementary values at `2305-2306`. The localization-reset write sets hot false before deriving new-goal at `1841-1842`; all other writes either clear a flag or clear both. Callers reading the pair use the same `input_mutex_`-protected domain, so they cannot observe an intermediate state across the paired accepted-goal mutation. Direct test-fixture assignment can construct arbitrary combinations (`test_navigation_runtime_terminal_monitor.cpp:273-274`); that is test setup, not a product transition.

Therefore the 1/1 combination has no legitimate product meaning and is a type-level invalid state. Replace the pair with one transition enum/disposition. Keep the two independent desired facts (goal and revision) as separate members of the aggregate. Do not encode worker planning/running/result state in that enum.

## Relation to `GoalTransitionKind`

`GoalTransitionKind` describes the relationship between two goal messages: steady, initial, same-route waypoint advance, route replacement, mission replacement, or cancel/localization reset (`runtime_boundaries.hpp:20-26,72-96`). The boolean pair instead persists the selected planning disposition after consulting lifecycle facts.

These concepts do not map one-to-one. A same-route waypoint advance may or may not qualify for hot retarget depending on command availability, failure latch, and suffix ownership (`planner_fsm.hpp:93-103`). A route replacement can also be hot eligible under those conditions, while the classifier still calls it `kRouteReplacement`. Thus `GoalTransitionKind` alone cannot reproduce `new_goal_`/`hot_goal_transition_` consumers. Do not add an overlapping identity enum; retain `GoalTransitionKind` for relationship classification and use one coarse typed planning disposition only where persistence is required.

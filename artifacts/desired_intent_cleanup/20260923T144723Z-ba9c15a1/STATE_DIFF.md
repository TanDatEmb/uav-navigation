# Desired State Diff

## Provenance and counting scope

Baseline is commit `ba9c15a1214ca8ba54ec8f301f1e4056a3322848` (tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`). The after column is the source/test checkpoint `c9006768b3f75180613319aa93fd8de62af4dae5`; no later product edit is included in these counts.

Counts cover the persistent desired-intent fields in RuntimeNode and their semantic representation, not comments, tests, or wrapper member count alone. Base fields were read with `git show ba9c15a1:<path>`.

## Persistent state

| Measure | Before (`ba9c15a1`) | After (`c9006768`) | Change |
|---|---:|---:|---:|
| Independent desired-intent facts | 4 | 3 | 4 → 3 |
| Desired goal payload | 1 `active_goal_` | 1 optional goal in `desired_intent_` | same fact, corrected vocabulary |
| Desired revision | 1 `active_goal_epoch_` | 1 atomic revision in `desired_intent_` | same fact, corrected vocabulary |
| Planning disposition | 2 booleans (`new_goal_`, `hot_goal_transition_`) | 1 `PlanningIntentTransition` enum | two boolean dimensions → one typed state |
| RuntimeNode member slots for this cluster | 4 fields | 1 `DesiredPlanningIntent` value | aggregate replaces fields; not counted as 4 → 1 semantic reduction |
| RuntimeNode declared persistent members, all categories | 157 | 154 | 4 removed, 1 added; includes diagnostics and ROS handles |
| Invalid simultaneous new+hot combination | Representable (`1,1`) | Unrepresentable | eliminated |
| New mutex for desired intent | 0 | 0 | unchanged |

The aggregate has three independent facts: optional goal, monotonically advancing revision, and transition disposition. The enum admits `kNone`, `kNewIntent`, or `kHotRetarget`; it is not planner-worker state. RuntimeNode stores the value at `navigation_runtime_node.hpp:464`; its transition methods use the existing `input_mutex_` domain (`:441`, `:278-297`). The revision remains atomic for the existing lock-free readers; no new mutex or owner was introduced (`desired_planning_intent.hpp:23-25, 36-55, 120-123`).

The total-member count scans declaration identifiers ending in `_` from `registered_scan_topic_` through the end of `NavigationRuntimeNode`, excluding comments. The four removed and one added member are the only set difference. This is a declaration count, not a claim that all 157/154 members are independent behavioral authority.

`ExecutionAuthority` remains the active/staged execution owner (`navigation_runtime_node.hpp:601`). The desired-intent replacement does not transfer, mirror, or mutate execution ownership. The MissionProgress writer and the independent WorldModel and PX4 boundaries are unchanged by this state representation.

## Interpretation

This cut removes one independent boolean fact while making the mutually exclusive planning dispositions a single enum. The goal and revision remain independent facts inside the value. The four source fields were not honestly a single semantic fact, so the result is reported as **4 → 3**, not **4 → 1**. The wrapper is a typed value under existing RuntimeNode locking, not a new manager.

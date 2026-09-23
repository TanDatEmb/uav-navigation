# Target Desired Planning Intent

## Ownership and shape

Keep desired state inside RuntimeNode's existing mission/planning control domain, protected by its existing canonical runtime lock. Do not add a manager or mutex. MissionProgress remains the sole mission-progress writer and emits the goal decision; RuntimeNode installs that goal as desired planning intent. ExecutionAuthority remains the sole active/staged execution owner.

Target value shape:

```cpp
struct DesiredPlanningIntent {
  std::optional<NavigationGoal> goal;
  PlanningIntentRevision revision{0};
  PlanningIntentTransition transition{PlanningIntentTransition::kNone};
};
```

`PlanningIntentTransition` replaces the mutually exclusive booleans with one disposition: `kNone`, `kNewIntent`, or `kHotRetarget`. It answers why the current desired goal requires handling; it is not a planner-worker state machine. Keep `GoalTransitionKind` separately for classifying desired-versus-executing relationship, since same-route/route-replacement classification does not itself prove hot-retarget eligibility (`runtime_boundaries.hpp:20-26,72-96`; `planner_fsm.hpp:93-103`).

Provide bounded mutation/query operations on the value: install/begin, clear, consume transition, and `matches(goal, revision, ...)`. Do not expose mutable field references to callers. Mutation remains serialized by RuntimeNode's current lock.

## Lifecycle preserved from the pinned source

1. Accepted MissionProgress/goal decision installs desired goal and advances its monotonic revision (`navigation_runtime_node.cpp:2147-2271`). The current new-versus-hot disposition is selected using existing command, failure, suffix, and anchor facts.
2. Planning captures immutable desired context in the existing `PlanningKey` (`3384-3395`); the worker does not read live mutable desired state (`3448-3455`). Reuse this key rather than adding a duplicate request identity.
3. A result is applicable only while its desired revision remains current and ExecutionAuthority's admission fence still accepts that revision. A candidate is staged/committed through ExecutionAuthority.
4. Consume the desired transition at the existing semantic points: new-intent after successful admission (`5939`); hot-retarget after successful immediate commit when applicable (`5940-5944`); staged successor at actual activation (`8123-8133`). Preserve existing explicit restart/handover consumption and reset behavior.
5. Cancel/reset clears the desired goal or outstanding disposition as current behavior dictates (`1841-1843`, `2112-2117`, `2485-2494`). A pending goal behind a moving safety suffix remains pending until measured-stop promotion (`4191-4205`).

Do not consume transition merely because a planner request was submitted: the current fields survive across planning cycles until commit/activation or an explicit reset/handover point.

## Identity separation invariant

Desired planning intent is what Core currently wants solved. Admission is the last desired revision ExecutionAuthority permits to enter. Active execution is the exact bundle/goal currently authorized to fly. For a hot retarget, desired N+1 and active N coexist. A late result for an older revision is discard-only; it cannot commit, stage, revoke, or change active lifecycle. Publication checks the exact active token and may continue predecessor N until valid successor cutover, subject to current certificates.

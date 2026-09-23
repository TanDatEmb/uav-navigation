# As-Is Desired State

## Pin and owner

Source: `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`, tree `6fbd807010aa7a88da247e77cab7a71984b7f33d`. RuntimeNode owns `active_goal_`, `active_goal_epoch_`, `new_goal_`, and `hot_goal_transition_` in its existing input/control domain (`src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp:504-514`). There is no separate desired-intent owner. Desired state is read and written while holding the existing lifecycle/input locks.

The names `active_goal_` and `active_goal_epoch_` describe desired planning intent, not active execution. Accepted incoming goal transitions assign the goal and advance the desired epoch before setting transition disposition (`navigation_runtime_node.cpp:2253-2271`). During an eligible hot retarget, RuntimeNode explicitly keeps the predecessor in ExecutionAuthority while installing the successor desired goal (`navigation_runtime_node.cpp:2303-2323`).

MissionProgress remains the mission-progress writer. RuntimeNode turns its `Goal` decision into a `NavigationGoal` and passes it to the validated-goal transition (`navigation_runtime_node.cpp:2503-2534`); MissionProgress activation and measured observations produce those decisions at `2603-2606`. Desired state does not advance waypoint progress itself.

## Current semantic facts

The cluster has four stored fields, but only three independent semantic facts:

1. Desired goal message: `active_goal_`.
2. Monotonic desired/admission revision: `active_goal_epoch_`.
3. One transition disposition encoded redundantly by the mutually exclusive pair `new_goal_` and `hot_goal_transition_`.

`new_goal_` makes `plan_from_rest = new_goal || restart_from_rest` true (`navigation_runtime_node.cpp:4861`). `hot_goal_transition_` participates in handoff/scheduling eligibility and allows a hot committed-future retarget only while a usable MAIN command and anchor remain (`planner_fsm.hpp:93-103`, `navigation_runtime_node.cpp:4980-4985`). Neither flag means that the desired goal is already executing.

## Planner request and consumption

`PlanningKey` captures localization epoch, the desired goal epoch, request, route revision, committed predecessor generation, and pinned world identity (`navigation_runtime_node.cpp:3384-3395`). A result must still match current desired identity before it is admitted. A successful command path consumes the new-goal bit at `navigation_runtime_node.cpp:5939`; hot transition clears after a successful immediate commit when applicable (`5940-5944`) or at staged successor activation (`8123-8133`). Reset/cancel paths clear the transition (`2112-2117`, `2485-2494`); localization reset marks an extant goal for new planning and clears hot disposition (`1841-1843`).

The pending safety-suffix handoff has an explicit cycle-local PlanFromRest disposition after measured stop (`navigation_runtime_node.cpp:4191-4205`, `4373-4385`). This local choice is not a new persistent worker state.

## Valid coexistence

Desired successor N+1 may coexist with ExecutionAuthority active predecessor N. The predecessor remains publishable until successor activation when its lifecycle/world/lease certificates remain valid; desired identity must not be substituted for the active execution witness. The source documents the retained predecessor behavior at `navigation_runtime_node.cpp:2303-2307` and the publication path samples ExecutionAuthority's active bundle at `8249-8294`.

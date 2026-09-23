# AS-IS authority and callback model at TARGET

| Owner | Fact/decision it writes | Invalidation and transaction | Evidence |
|---|---|---|---|
| `MissionController` | accepted waypoint/request, route cursor, STOP confirmation and mission phase | per mission activation/deactivation; `mutex_` | `mission_controller.hpp:110-130`; `mission_controller.cpp:340-590` |
| `NavigationMode` | last admitted command, independent state/health/frame/lease admission, local safety and handover latches | mode lifecycle and command/state callbacks; `trajectory_mutex_` | `navigation_mode.hpp:135-261`; `navigation_mode_node.cpp:1339-1410` |
| `NavigationModeExecutor` | Hold scheduling, retry and VehicleStatus confirmation | activation/deactivation; callback/timer state | `navigation_mode.hpp:281-290`; `navigation_mode_node.cpp:2846-2937` |
| `NavigationRuntimeNode` | desired/executing goal, goal/localization epochs, world suspension, publication decision, recovery orchestration | ingress→localization→input→command transition lock order; separate callbacks | `navigation_runtime_node.hpp:429-560`; `.cpp:2983-3004,7704-7725,7935-7950` |
| `ExecutionEpisode` | physical execution phase/recovery/availability/failure/suffix bits, copied active identity | internal mutex plus caller transition locks; goal/reset/commit/sample/failure | `execution_episode.hpp:41-260` |
| `ExecutionTimelineStore` | active/staged immutable bundle, activation time, world and transaction versions | store mutex; conditional admission/activation and exact predecessor check | `committed_bundle_store.hpp:30-35,821-829` |
| `WorldSnapshotStore` | latest immutable world snapshot | publication gate/atomic pointer; strict identity advance | `world_snapshot_store.hpp:131-178` |
| `PendingGoalHandoffOwner` | one newer same-mission deferred goal, exact consume | `goal_mutex_` plus caller transition transaction | `planner_fsm.hpp:23-90` |
| `RouteProgress` | measured projection/cursor, no waypoint acceptance | mission caller mutex; reset per route/session | `route_progress.cpp:254-345` |
| `PlanningWorker` | one active solve, one latest pending job, cancellation/fatal state | worker mutex and backend access mutex; no direct store authority | `planning_worker.hpp:268-350` |
| `ExecutionStateStore` | source/receive-stamped state and epoch ingress sequence | internal mutex; reset on epoch | `execution_state_store.hpp:12-48` |
| `KinematicDerivativeEstimator` | prior velocity/acceleration/time for derivative estimate | runtime derivative mutex; reset on epoch | `kinematic_derivative_estimator.hpp:20-91` |
| `HeadingRebindWorker` | pending heading job and worker lifecycle | worker mutex/condition; result returned to command clock | `heading_rebind_worker.hpp:84-121` |
| `BaselineRefinementOpportunity` | one-shot quality scheduling receipt | serial planning worker only; consume at backend entry | `baseline_refinement.hpp:30-103` |
| `MappingWorker` | latest ready observation and worker lifecycle/order | worker mutex/condition; cannot commit execution authority | `mapping_worker.hpp:23-293` |
| `ExecutionTraceStore` | diagnostic snapshot and minimum epoch | internal mutex; no admission consumer intended | `execution_trace_snapshot.hpp:135-177` |

These are **sixteen scoped state owner types**: seven decision/protocol owners (MissionController, NavigationMode, ModeExecutor, RuntimeNode, Episode, Store and PendingGoalHandoffOwner), plus independent world, route measurement, execution-state input, derivative history, worker scheduling/quality and diagnostic owners. This is not an exhaustive whole-system count. Nested estimator/mapping/bridge and planner cache ownership remains to inventory. `NavigationRuntimeNode`, Episode and Store participate in one execution decision through multiple writes and rechecks. Treating all sixteen as redundant authority would discard independent PX4/world/worker facts.

## Callback flow, selected path

| Callback/event | Current write or lock | Authority affected | Candidate target behavior |
|---|---|---|---|
| registered scan / mapping completion | mapping worker and latest world publication; world recertification can update Store/Episode | world/active lease | immutable observation and worker result; reducer decides recertification |
| propagated odometry | execution state store, localization/reset and derivative state | source/receive witness | ingress validates and queues compact immutable state; safety heartbeat retains direct bounded access |
| estimator health | runtime and PX4 independently cache freshness/epoch | command admission | preserve separate receiver witness, do not mirror planner policy |
| goal | runtime mutates desired, pending handoff and Episode identity under transition locks | intent | reducer owns acceptance/intent revision |
| mode status | runtime recovery/failure path | execution policy | typed external event, provenance-bound |
| planning timer | forms key, submits/cancels worker and checks world | worker scheduling and possible authority transition | bounded scheduling event, solve on worker |
| planner completion | exact key/world/predecessor rechecks then Store stage/commit | active/staged execution | immutable result to reducer; worker cannot commit |
| command timer | activates due staged bundle, samples and publishes under nested locks | command exposure | high-priority bounded decision; no unbounded compute or queue wait |
| PX4 command callback | validates/cache command; may call `updateMission()` immediately | local admission and mission progression | adapter preserves independent admission; mission fact event to mission owner |
| PX4 state / VehicleStatus | frame reset/alignment, status and Hold confirmation | PX4 boundary | independent safety observations and protocol events |

The target `callback → immutable event → bounded queue → reducer` is a **proposal**, not a source fact. It must define queue capacity, overflow disposition, priority, epoch reset ordering, shutdown and command deadline. Mapping, polynomial planning, swept collision validation, expensive recertification and diagnostic serialization stay on workers. The reducer alone would write mission/execution decisions; PX4 remains an independent local veto.

## Identity and manual comparison

Semantic identities include mission/session and route revision, goal/intent revision, localization epoch, world generation/revision, bundle generation, request ID, transaction ID, sample ID and dynamics hash. `PlanningKey` is already a typed key alias (`planning_key.hpp:7`) and a candidate for one stale-result comparison at the reducer, but raw identities must remain in provenance, reset and recertification records. TARGET still performs multi-field final publication rechecks (`navigation_runtime_node.cpp:8595-8615`). The number of eliminable comparisons has not been proved; do not collapse them to a generic ID.

## Control protocol observation

`NavigationCommand.msg` carries authority/identity/lease/frame-adjacent witnesses, PVA reference and many diagnostic fields. The message itself marks a diagnostic section and later temporal-alignment evidence as non-admission. A split into `ControlReference` plus `NavigationDecisionTrace` is plausible only after a field-by-field consumer audit. The receiver still needs source and receive freshness, localization epoch, mission/request/bundle identity, sample order, world stamp, frame, validity interval, role/status, continuation witness, PVA/yaw and independent execution authorization. Jerk is diagnostic/reference provenance in this PX4 path, not a PX4 executed setpoint (current safety contract line 46).

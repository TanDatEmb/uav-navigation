# Authority transition width

`W_t` counts independently owned mutable objects that **must change to complete** a transition. A single object's many fields count once; read-only messages and diagnostics do not count. For asynchronous transitions with unclear completion point, report a lower bound and unknown upper bound. Target values are conditional architecture expectations, not measured reductions.

| Transition and completion boundary | AS-IS W_t; minimum witnessed owners | TARGET expected W_t | Qualification gap |
|---|---|---|---|
| new goal / desired intent accepted by runtime | ≥3, upper ? — MissionController, NavigationRuntimeNode, ExecutionEpisode or PendingGoalHandoffOwner | 1 mission decision + 1 execution admission = 2 | exact hot-retarget writer graph |
| PASS crossing / accepted waypoint | ≥2, upper ? — RouteProgress, MissionController | 1 Mission record with separate measured/accepted facts | callback order and observation retention |
| successor activation / active command switches | ≥2, upper ? — ExecutionTimelineStore, ExecutionEpisode; runtime mirrors may add | 1 Execution transaction | atomic admission under current locks |
| recoverable brake / certified moving replacement | **not established in product call graph**; `onTrajectory` API/test path alone has W_t=1 MissionController object | 1 Execution policy + 1 PX4 observation = 2 **only if approved** | no product call to `onTrajectory`; future policy and planner capability |
| committed safety stop / one-way disposition | ≥2, upper ? — bundle store, Episode; adapter STOP callback can add | 1 Execution transaction + 1 adapter receiver = 2 | policy commitment point |
| measured stop / restart allowed | ≥2, upper ? — Episode and MissionController timers/status | 1 measured witness + 1 execution decision = 2 | source-time stationary confirmation |
| world recertification / active bundle retained | ≥2, upper ? — WorldSnapshotStore and ExecutionTimelineStore; Episode may resume | 2 orthogonal owners (WorldModel + Execution) | async/expiry timing |
| PX4 Hold request / API scheduled | 1 local owner + pinned library ScheduledMode = 2; runtime initiation may add | 1 Core transfer decision + 1 PX4 protocol owner = 2 | request/ACK order |
| PX4 Hold confirmation / authority transferred | ≥2, upper ? — adapter status cache and executor protocol; PX4 itself external owner | 1 PX4 boundary + external PX4 = 2 | exact fresh status/charge witness |

`FACT_FROM_TARGET_CODE`: MissionController update and PublishGoal (`mission_controller.cpp:451-613`), runtime Episode/timeline (`execution_episode.hpp:86-230`, `navigation_runtime_node.cpp:1150-1305,8180-8235`), adapter Hold (`navigation_mode_node.cpp:2846-2937`), pinned library ScheduledMode (`mode_executor.cpp:225-260,484-519`). `INFERENCE`: lower bounds from independently mutable objects and target expectations. `SPECIFICATION_GAP`: a full alias/writer graph and exact transition linearization; no exact whole-product W_t is claimed. **Highest observed lower bound = 3** (new goal); no proven global maximum.

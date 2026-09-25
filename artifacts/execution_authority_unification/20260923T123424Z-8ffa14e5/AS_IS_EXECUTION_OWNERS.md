# AS-IS execution owners (pinned repair commit)

Evidence: `src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp:472,484,562,622`; `src/runtime/navigation_runtime/include/navigation_runtime/execution_episode.hpp:41-268`; `src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp:24-834`.

| Owner | Mutable execution fact | Mutex / transition |
|---|---|---|
| RuntimeNode `active_goal_`, `active_goal_epoch_` | Desired planning scope; may advance before execution | `input_mutex_` plus outer transition locks |
| RuntimeNode `executing_goal_`, `command_goal_epoch_` | Full active goal and epoch copied from execution | Outer transition locks; epoch is atomic |
| `ExecutionEpisode` | Phase, recovery, exposure, safety suffix, restart, copied desired and active identities | Its own `mutex_` |
| `ExecutionTimelineStore` | Active/pending bundle, pending activation, world transaction, admission epoch, versions and transaction watermark | Its own `mutex_` |
| `ExecutionStateFailureLatch` | Independent command lease failure boundary | Its transition mutex; remains a separate safety boundary |

`setActiveGoalEpoch(next, true)` advances *candidate admission scope* while preserving the predecessor bundle. It is not the active command epoch. `ExecutionEpisode::beginGoal(..., retain=true)` similarly updates its desired identity and retains the active identity. The repair's `publishCommand()` accepts the still-owned predecessor until successor cutover. The producer of the full active goal is currently `executing_goal_`, not `CandidateBundle` (bundle has no mission/waypoint/route payload).

Current immediate commit mutates the timeline, then writes `command_goal_epoch_`, swaps `executing_goal_`, and calls `ExecutionEpisode::commandCommitted()` under the outer locks (`navigation_runtime_node.cpp:2778-2805`). Pending activation similarly calls the timeline finalizer, then updates the three mirrors (`navigation_runtime_node.cpp:8111-8153`). This gives multiple mutable representations and two execution-level mutexes even though outer locks currently narrow the race.

The store's `last_transaction_id_` is a stale-result fence; `active_lineage_version_` protects predecessor anchor identity; `timeline_version_` protects optimistic world/staged transactions. These are distinct from bundle generation and from the desired mission gate.

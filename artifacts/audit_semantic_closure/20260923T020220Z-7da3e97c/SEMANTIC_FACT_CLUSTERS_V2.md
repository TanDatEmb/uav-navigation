# Semantic fact clusters V2

The companion `FIELD_CLUSTER_MAP.csv` maps **all 239 prior behavioral candidates** into 30 named clusters before any new disposition. The deterministic lexical/source-owner mapping is deliberately marked `LEXICAL_TENTATIVE` for 191 rows; 48 rows from five heavily reviewed owner types are marked `SOURCE_REVIEWED`. Cluster membership is not a proof that similarly named fields have the same semantics, nor a resolution of the 214 prior UNRESOLVED dispositions. `tools/cluster_inventory.py` regenerates the map from the pinned prior CSV and asserts 239 rows. Nested aggregates outside the prior scanner remain out of scope.

`D_f` counts independently mutable **authoritative representations of the same semantic fact**. A read-only wire observation in the adapter is not an extra product decision writer. `?` means no safe count without the writer/alias graph. Target numbers below are *design expectations*, not implementation evidence.

| Fact ID / meaning | Current representation and writers | AS-IS D_f | Target representation / sole owner | Target D_f |
|---|---|---:|---|---:|
| FACT-MISSION-DESIRED: requested waypoint/intent | MissionController `active_waypoint_index_`,`request_id_`; runtime `active_goal_`, Episode `goal_epoch/request_id` receive and react through separate callbacks; these may instead be distinct accepted/desired/executing facts | ? | `DesiredMissionIntent` / Mission domain; runtime receives immutable intent | 1 intended |
| FACT-MISSION-ACCEPTED: measured gate accepted | MissionController index/request; adapter completion fields | ≥1; exact ? | `AcceptedMissionProgress` / Mission domain | 1 intended |
| FACT-ROUTE-CURSOR: ordered physical progress | `RouteProgress.state_` (cursor); MissionController previous sample/time is a distinct measurement-history fact | 1 for cursor; history separate | `MeasuredRouteState` / Mission measurement owner | 1 owner, multiple independent fields |
| FACT-CROSSING-OBS: physical waypoint crossing at t | ephemeral local result from `measuredWaypointCrossingError` | 0 persistent; callback local | bounded typed `WaypointCrossingObservation` / Mission measurement owner | 1 intended |
| FACT-EXEC-ACTIVE-ID: active bundle generation | `ExecutionTimelineStore.committed_` bundle generation and `ExecutionEpisodeSnapshot.active_generation`, independently mutated by store/Episode transitions | **2 proven** | immutable active bundle/timeline record / Execution domain | 1 intended |
| FACT-EXEC-STAGED: future splice | timeline `pending_`, `pending_activation_ns_` | 1 owner, distinct fields | `StagedExecution` / Execution domain | 1 intended |
| FACT-EXEC-SAFETY: safety commitment | Episode recovery state/phase and runtime sampled role are independent; adapter `Braking` is reachable only through an API with no repository product call site | ? | typed lifecycle plus orthogonal sample role / Execution and PX4 boundary | 1 policy writer intended |
| FACT-CERT-WORLD: world validated for active bundle | bundle certificate and timeline world identity; latest world differs | ≥1; exact ? | certificate carried by immutable active execution / Execution domain | 1 intended |
| FACT-WORLD-LATEST: newest world | `WorldSnapshotStore.latest_` | 1 | latest immutable world / WorldModel | 1 |
| FACT-COMMAND-LEASE: last admitted command and receive age | runtime command valid_until; adapter `navigation_command_`, `last_command_receive_ns_` encode distinct source/receive facts | **not a duplicate count** | source lease / Execution; receive lease / PX4 boundary | 1 per fact |
| FACT-PX4-REQUEST: Hold command attempt | executor pending/inflight/retry fields | 1 owner, orthogonal facts | `HoldTransfer` / PX4 boundary | 1 |
| FACT-PX4-COMPLETION: ModeCompleted/cancel | callback result currently folded into pending | 1 transient callback | typed completion witness / PX4 boundary | 1 intended |
| FACT-PX4-STATUS: AUTO_LOITER observation | `px4_hold_confirmed_` cached bool from `VehicleStatus` | 1, freshness missing | source/receive-stamped `VehicleStatusWitness` / PX4 boundary | 1 intended |
| FACT-LOCAL-EPOCH: localization reset fence | runtime epoch, Episode epoch, adapter epoch; process boundary copies and local gates | ? | authoritative localization source + immutable received witness | 1 per domain/observation |
| FACT-STOP-CONFIRM: measured stationary duration | MissionController braking/stopped timers, runtime recovery stop gate; different decisions | ? | explicit measured-stop witness and policy timer | 1 decision owner intended |
| FACT-WORKER-JOB: pending computation | planning/mapping/heading worker records | distinct jobs, not duplicate authority | worker-local job state | 1 per worker |
| FACT-DIAGNOSTIC: trace/accounting | runtime telemetry/counters | not authority | diagnostic only | n/a |

`FACT_FROM_TARGET_CODE` high-criticality source anchors: `mission_controller.cpp:367-613`, `route_progress.cpp:254-345,430-490`, `execution_episode.hpp:49-230`, `execution_timeline_store.hpp` active/pending records, `navigation_runtime_node.cpp:1150-1305,7696-7720`, `navigation_mode_node.cpp:2846-2937`. `INFERENCE` target ownership and D_f expectations. `SPECIFICATION_GAP` exact writer graph for candidate mirrors and the semantic relation between runtime commitment and adapter Braking.

# NavigationCommand consumer graph at 2b9279b4

```text
Core ExecutionAuthority + MissionProgress
  └─ NavigationRuntimeNode::publishCommand
       ├─ /navigation/navigation_command (reliable)
       │    ├─ PX4 NavigationMode::onNavigationCommand
       │    │    ├─ commandContractValid + commandValidAt
       │    │    ├─ session/world/health ordering
       │    │    ├─ odometry freshness + sample ordering
       │    │    ├─ tracking envelope + terminal/recovery
       │    │    ├─ atomic navigation_command_ commit
       │    │    └─ /navigation/command_admission (success only)
       │    ├─ runtime scenario/evaluator (read-only evidence)
       │    └─ rosbag/replay/report tools (read-only evidence)
       └─ local rememberMissionCommandIssued + admission receipt
            └─ MissionProgress continuation witness
```

The adapter additionally copies `causal_planning_cycle_id` from the command into `Px4InputTraceRecord`; that path is diagnostic, not an admission or PX4 setpoint predicate. The adapter reads `emergency_authorization_reason` and `emergency_candidate_commit_result` in its velocity-only experiment output gate: both remain control fields. The contract helper reads the continuation pair; Core uses it with an exact adapter admission receipt. `execution_authorization` is Core-local issuance authority plus runner provenance; the adapter never reads it as authorization.

Outside-repository ROS consumers are not enumerated by source. No versioned external consumer contract for this topic was found in the pinned repository; this is a compatibility limitation to state in the final verdict. The message schema change is atomic for in-repository producer/adapter/runner/replay consumers and requires a clean rebuilt overlay. Historical bags retain the old schema and must be decoded with their matching old overlay or a version-aware reader.

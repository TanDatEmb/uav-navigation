# Decisions requiring evidence

1. Is `Braking → ExecutingWaypoint` on a newly certified nominal trajectory before measured stop allowed? TARGET does allow it (`mission_controller.cpp:150-165`). A one-way target would be a behavior change.
2. Should a measured PASS_THROUGH crossing be retained across a later continuation witness, and for how long/which route, epoch, source-time and odometry-gap conditions? Current code couples the checks (`mission_controller.cpp:363-371,569-579`).
3. What is the cross-process budget between world freshness expiry, recertification, 100 ms stream lease and PX4 local command/state expiry? No representative latency distribution was produced here.
4. Which adapter planner-recovery/suffix/mission-terminal fields are independent local safety information versus producer policy mirrors? Require field-level receiver negative tests before removal.
5. What is the reducer queue priority/overflow/shutdown contract, including map burst and command heartbeat deadline? No WCET/CPU affinity evidence exists.
6. What does `ModeCompleted(Success)` before a locally observed AUTO_LOITER `VehicleStatus` mean, and when may Hold retries stop? Current code clears pending on `Success`/`Deactivated` (`navigation_mode_node.cpp:2907-2918`) while only VehicleStatus sets confirmed (`:2846-2854`). The pinned library routes the completion callback separately from command ACK and VehicleStatus (`mode_executor.cpp:225-260,484-519`). A target-specific ordering trace and explicit operator-takeover disposition are required.
7. Is deterministic serialization/hashing of `CandidateBundle` required? If yes, callable captures become a target blocker; if no, defer cleanup.

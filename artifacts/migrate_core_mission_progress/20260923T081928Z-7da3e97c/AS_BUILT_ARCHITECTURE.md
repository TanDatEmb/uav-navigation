# As-built product architecture

Baseline `7da3e97cb399c2e39d62cfe60213a45e8a92300e`; implementation commit `ccb373422642f27a8740b32ea4ab6062e433e0be`. This is product behavior, not a shadow selector.

```text
mission YAML -> Core loadMission() -> immutable Mission definition
                                   -> MissionProgress (single policy writer)
propagated odometry -> ExecutionStateStore -> Core measured route cursor/gate
Core authorized command -> PX4 adapter local admission -> exact admission receipt
                                                |                 |
                                                |                 v
                                                |         Core continuation witness
                                                v                 |
                                         PX4 P/V/A setpoint       v
                                             and Hold      Core acceptance decision
                                                              |
                                        internal validated-goal transition
                                                              |
                                                 Core mission progress receipt
                                                              |
                                              adapter terminal mode handover
```

Core serializes mission policy under `localization_transition_mutex_` then `input_mutex_`. Its 50 ms mission tick consumes immutable execution state and a fresh adapter ACTIVE/airborne observation. The same owner transaction creates a validated initial/successor `NavigationGoal` internally. No Core→PX4→ROS goal roundtrip remains. The adapter neither parses mission YAML nor constructs `MissionController` in its product path.

Core records only commands that pass the final execution-store exposure gate. The adapter emits `NavigationCommandAdmission` after its local freshness, health, session, monotonic sample, frame, lease and tracking admission. Core matches the receipt to the exact issued command, current route/localization/gate and PX4 activation before creating a continuation witness. Publication alone cannot grant waypoint acceptance.

The adapter remains the sole owner of PX4-local state and control: estimator/frame/reset witnesses, 100 ms receive lease, local tracking response, accepted command cache, setpoint publication, planner recovery hold and PX4 Hold/takeover. WorldModel remains latest-world owner; active execution retains its certificate. This cut does not redesign braking or Hold ordering.

The legacy `MissionController` library remains installed for source/ABI compatibility. Product adapter CMake no longer links it and the adapter call graph has no constructor, `onTrajectory()`, native trajectory callback, waypoint update or goal publisher. `onTrajectory()` is **INTERNAL_DEAD/LEGACY** in the in-repository product call graph; external clients of the public library remain unproven boundary debt.

This architecture has component/source proof. It has no flight qualification claim.

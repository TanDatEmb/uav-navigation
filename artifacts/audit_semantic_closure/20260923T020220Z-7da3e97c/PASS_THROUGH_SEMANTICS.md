# Blocker A — measured PASS_THROUGH

**Verdict: PARTIAL.** The current mechanism and an information-conserving candidate contract are source-derived; retention policy at delayed readiness and real producer→consumer reachability remain unproved.

## Current behavior

`FACT_FROM_TARGET_CODE` `navigation_mode_node.cpp:1355-1464` obtains odometry and a current MAIN continuation, then calls `MissionController::update`. The continuation is also tested on new command arrival at `:1328-1335`; thus a timer-only race assertion would be false. `mission_controller.cpp:367-371` saves the old sample locally and replaces `previous_position_` with the current position. The timestamp stored there is callback `now_s`, not odometry source time. `:451-477` updates `route_progress_` and passes old/current positions plus callback gap to `measuredWaypointCrossingError`, bounded by 0.25 s (`mission_controller.hpp:103`). `route_progress.cpp:254-345,430-490` projects in 3-D on ordered route segments, uses a monotone arc and tie handling, and admits either current position in the acceptance ball or a forward crossing segment whose closest approach is inside radius. It rejects a stale gap and reverse tangent. A scalar arc alone does not identify which leg of a loop/self-intersection was traversed.

`FACT_FROM_TARGET_CODE` `mission_controller.cpp:517-613`: measured acceptance, finite velocity, and a current exact MAIN continuation (or certified safety suffix stop, coincident terminal hold, or initial waypoint exception) are conjunctive in the same update. The safety suffix exception additionally checks bundle/mission identity, completed BACKUP, measured low speed and position in `navigation_mode_node.cpp:1410-1436`; the coincident PASS→STOP case waits for measured settling. Sharp corners do not require outgoing velocity alignment. Acceptance advances the waypoint and emits `PublishGoal`; that event alone does not activate an executable successor.

`FACT_FROM_EXISTING_TEST` `test_mission.cpp:369-415,500-652,653-745,746-~800,1301-1382` covers per-update continuation, high-speed skipped ball, near self-crossing branch, stale gap, corner, coincident STOP and suffix behavior. These tests were read, not rerun as product tests here.

`INFERENCE` Counterexample to the local conjunction: at t0 crossing=true/readiness=false; at t1 crossing=false/readiness=true. No update accepts although t0 physically crossed. A new command callback can narrow this window; neither the counterexample nor the abstract test proves that the full producer→consumer scheduling can reach it in SITL.

## Independent facts and candidate contract

1. **MeasuredRouteState:** route revision, localization epoch, source-stamped ordered cursor `(segment_index, segment_u, arc_length_m)`, previous measured sample. It must survive successive measurements; scalar `route_progress_m` is insufficient on loops, nearly parallel or self-intersecting 3-D segments, and cannot alone detect reverse motion. Segment index plus local parameter is minimum route-order witness; a tie policy and uncertainty tolerance are still required.
2. **WaypointCrossingObservation:** `(route revision, localization epoch, waypoint index/request identity, before/after source stamps, before/after ordered cursors, 3-D closest approach/error, acceptance radius used)`. The measurement/mission domain creates and owns it after validating two source-stamped samples, bounded source gap, forward ordered crossing and geometry. A current-in-ball observation may be represented with one sample and explicit kind; do not fabricate a prior crossing.
3. **AcceptedMissionProgress:** monotone accepted waypoint/request; only mission decision writer consumes valid physical evidence. This is not the measured cursor.
4. **SuccessorExecutionReadiness:** exact admitted MAIN continuation or explicitly qualified suffix/coincident STOP witness. It can arrive before or after crossing; it cannot manufacture crossing.
5. **ActiveExecutionIdentity:** current bundle/generation/goal/epoch. Successor activation occurs at its own certified timeline boundary, after mission acceptance; it is not equivalent to `PublishGoal`.

`INFERENCE` A candidate crossing may be consumed by a later matching continuation **only while** route/localization/waypoint/request identity remains exact, source gap/age policy remains satisfied, no reverse evidence invalidates it, and no safety transition requires a different acceptance path. World revision does not erase a physical observation by itself; it may revoke execution readiness, so consumption must recheck the current certified command. The observation is invalidated by route revision, localization reset, mission reset, accepted advance, contradictory reverse movement, or odometry discontinuity. It must not persist indefinitely: a maximum source-time retention and spatial departure/reentry rule require representative motion evidence and policy sign-off. Cached callback time is not a replacement for source time.

## Required answers

- **Q-A1:** Yes, an ordered measured route cursor must persist across samples if crossing is to be accepted after a callback and route ambiguity is to be detected. A full `RouteProgress` object already stores ordered projection, but the target must carry exact route/epoch/source identities.
- **Q-A2:** No. A scalar arc cannot encode segment choice, reverse evidence, 3-D crossing geometry, or a loop. Minimum candidate: segment index, segment parameter, arc, route revision, localization epoch, source stamp, plus measured position/uncertainty in the observation.
- **Current behavior:** same-update physical evidence/readiness conjunction; source time is not used for the crossing gap.
- **Desired target behavior:** typed bounded crossing observation consumed only by matching current readiness and mission gate.
- **Information preserved:** physical crossing, order, source time, localization/route identity, accepted progress and execution identity separately.
- **Information derivable:** `crossing_pending` and generic `progression_ready` booleans from observation/readiness and gates.
- **Behavior intentionally lost:** none approved; delayed consumption is a policy change until retention bounds are agreed.
- **New risk:** stale observation could advance a later waypoint; exact invalidation and age bound are mandatory.
- **Required product tests:** two callback orderings with real command producer, route revision/reset/reverse/gap, high-speed ball skip, loop/self-intersection, sharp corner, coincident PASS→STOP, safety suffix, and source-time jitter.

`SPECIFICATION_GAP`: maximum retention age and departure rule; route projection tie tolerance under localization uncertainty. `RUNTIME_UNVERIFIED`: reachability of the t0/t1 sequence and behavior at realistic callback/transport latency.

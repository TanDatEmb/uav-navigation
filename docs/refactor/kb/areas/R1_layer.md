# R1 — Planner core layer

## Role
Planner core (`navigation_planning_backend::Planner`, planner_core/*) is the PLANNING layer's solver: given one immutable `PlanningRequest` (route snapshot, pinned `WorldModelView`, start KinematicState or execution anchor, dynamics, steady deadline, cancellation token) it produces at most one complete, certified MAIN(+BACKUP) `CandidateBundle` or a typed failure (`PlanningOutcome`). It owns guide construction (A*), safe-flight-corridor (CIRI) generation, MINCO nominal optimisation, yaw optimisation, backup braking synthesis, and the producer-side certificates (yaw rate/accel, flatness, route regression, swept world validation) before handing the bundle to the runtime/execution layer. It does NOT own activation: execution ACKs promote staged candidates back into warm-start history (`onExecutionTimelineActivated`).

## Components
- `Planner` (planner.hpp/planner.cpp, 6.2k LOC) -> request admission, stage orchestration, candidate staging/export, heading rebind, emergency brake; ~80 mutable members, 5 mutexes.
- `Planner::generateExpTraj` (planner.cpp:2878-4651) -> hot-splice compatibility, guide prefix, A* route, pass-through corner window/lookahead, endpoint policy, SFC, evidence speed governor, MINCO, yaw, flatness.
- `Planner::generateBackupTrajectory` (4653-5703) -> visibility scan, backward switch-time search, braking seed, braking-aligned CIRI polytope, KNOWN_FREE sweep, optional L-BFGS refinement + shape gate, backup yaw.
- `Planner::authorizeAndStage` (630-852) -> final yaw certs, route-regression certificate, latest-world swept validation, `WorldCommitAuthorizer::commitIfCurrentOrUnaffected`, generation reservation, staging.
- `Planner::exportStagedCommandCandidate` (966-1342) -> CandidateBundle with evaluator/world_validator closures, role schedule, route boundary constraint/event.
- `Planner::resolveGoalForPlanning` (265-559) -> STOP acceptance-ball endpoint projection + lease.
- `Planner::PathSearch` (5724-6156) -> A* orchestration (escape, preferred-altitude, unrestricted, prob-map fallback), altitude projection, shortcutting, continuous-edge invariant.
- `Planner::commitEmergencyBrake` (2726-2864) -> jerk-limited measured-state stop, altitude-preserving, emergency candidate.
- `Planner::stageImmediateHeadingRebind` / `buildImmediateHeadingRebindCandidate` (1365-1694) -> yaw-only successor on retained position suffix (duplicated, R1-08).
- `Planner::tryStageMeasuredTerminalStopHold` (1696-1841) -> zero-progress STOP hold inside acceptance ball on UNKNOWN voxel with body witness.
- `validateExecutableCandidate` & helpers (trajectory_world_validator.hpp) -> swept certificate: adaptive dt 2-50 ms, chord <= 0.5*res, curve deviation A*dt^2/8 <= 0.25*res, AABB tube enumeration (<=4096 cells/segment), per-role UNKNOWN policy.
- `AbsoluteDeadline` (absolute_deadline.hpp) -> dual sim/steady deadline, refinement reserve.
- `commandTrajectoryTime` / `hotReplanWindow` / `inheritedBackupInterval` (command_time.hpp) -> wall/ns -> trajectory-time conversion.
- `path_search::Astar` (astar.cpp) -> 26-connected weighted A* on inflated or evidence layer, virtual start shell, direct-segment fast path, frontier bookkeeping, escape search.
- `CorridorGenerator` (corridor_generator.cpp/.h) -> greedy seed-line extension on the guide, CIRI polytopes per seed, overlap checks, route-boundary gate cell (inner cube of acceptance ball).
- `CIRI` (ciri.cpp/.h, vendored FIRI) -> convex decomposition around seed line with robot_r tangent planes + MVIE.
- `backup_braking.hpp` -> minimum-snap free-end stop polynomial, steady-cruise duration, altitude smoothstep, Bezier hull, stop reachability.
- `evidence_speed_governor.hpp` -> closed-form cruise cap from support length; verifies stop polynomial fits support.
- `route_regression_certificate.hpp` -> MAIN progress high-water certificate along active segment(s); emergency-correction exception.
- `pass_through_terminal_velocity.hpp`, `route_boundary_timing.hpp`, `route_backbone.hpp`, `guide_vertical_envelope.hpp`, `guide_endpoint.hpp`, `altitude_route_projection.hpp` -> guide/endpoint shaping helpers (pure functions).
- `route_yaw_reference.*` -> leg-bearing yaw target + bounded heading step.
- `Config` (config.hpp) -> YAML load + derived safety envelope (robot_r, visibility horizon, deadline ordering).
- `replan_contract.hpp` / `planner_result.hpp` / `planning_stage.hpp` -> result code -> typed outcome mapping, stage names.
- `deterministic_nominal_seed.hpp`, `optimized_nominal_candidate.hpp`, `corridor_bezier_seed.hpp`, `corridor_plane_validation.hpp` -> nominal certificates used by traj_opt (R2 boundary).
- `FOVChecker` (fov_checker.h) -> optional OMNI FOV/sensing-horizon polytope cut.
- `NominalProblemSnapshotWriter` -> bounded background JSON dump (env-gated diagnostics).

## Processes & threads
- No own threads/timers. All solve entry points (`plan`, `planInitialFromStoppedState`, `planSuccessorFromExecutionAnchor`, `commitEmergencyBrake`) run on the runtime's planning worker; `replan_lock_` serialises stopped/successor solves (not taken by `plan()` preamble or `commitEmergencyBrake`).
- `buildImmediateHeadingRebindCandidate` is designed to run on a separate heading worker concurrently with the position solve (local YawTrajOpt, CmdTraj::snapshot under its own lock, `solve_commit_mutex_` for registration).
- Execution thread calls `onExecutionTimelineActivated`, `discard*`, `validateStagedCommandCandidate`, `exportCommandCandidate` (all take `solve_commit_mutex_`).
- Cancellation: `std::stop_callback` on request token -> `solve_cancelled_` atomic polled by stages/optimizers; `cancelActiveSolve()`.
- Diagnostics accessors (`solveStage`, `latestCandidateMaximumYawRateRadS`, `requiredLookaheadMeters`, guide bounds...) read plain members without locks (only timeline/time_consuming snapshot are locked) -> R1-12.
- `NominalProblemSnapshotWriter` (env UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR) owns one worker thread (mutex+cv, capacity 4, drop on full).
- Exported CandidateBundle closures (evaluator 1088, world_validator 1137) run on execution/runtime threads after export: they capture `shared_ptr<const CandidateCommandBundle>` + certificate/generation/policies by value, no `this` -> lifetime-safe (R1-46).

## Flows
`PlanningRequest` -> `plan()` (2349): validate limits == back_traj_cfg, policy == cfg; set nominal max vel (R1-13); `setRouteSnapshot`, `setGoalAcceptanceRadius`, `setWorldModelView`, `setCurrentBodySupport`, `setState` (+ anchor overwrite for kCommittedFutureState), `setCommandIdentity` ->
  (a) kStoppedMeasuredState -> `planInitialFromStoppedStateImpl` (1850): snapshot robot_state_ -> resolveGoal -> route yaw -> start shift (<=3 m, R1-15) -> terminal STOP hold? -> clear history (R1-10) -> generateExpTraj(baseline) -> generateBackupTrajectory -> buildCandidate -> authorizeAndStage -> stageCommandHistory.
  (b) kCommittedFutureState -> `planSuccessorFromExecutionAnchorImpl` (2078): same, successor splice at `requested_activation_stamp_ns_`, retains NO_NEED.
  (c) kMeasuredEmergencyBrake -> commitEmergencyBrake (dead, R1-11).
-> `exportCommandCandidateDetailed` -> `exportStagedCommandCandidate` -> `PlanningOutcome{candidate, outcome kind}`.
Inside generateExpTraj: committed CmdTraj snapshot -> hot splice compatibility -> guide prefix sampled from committed command (classify per 0.8*res) -> A* (`PathSearch`) to route backbone target within `localRouteSearchHorizon` -> bounded prefix certification -> time allocation -> pass-through lookahead/corner window -> `resolveGuideEndpoint` -> `CorridorGenerator::SearchPolytopeOnPath` -> `applyGuideVerticalEnvelope` -> `evidenceAwareSpeedLimit` -> terminal velocity policy -> `ExpTrajOpt::solve` (baseline/refine, deadline reserve) -> `YawTrajOpt::optimizeToTarget(route_yaw_reference_)` -> flatness.
Outputs to runtime: CandidateBundle (evaluator/world_validator closures, role_schedule, route_boundary_constraint/event, protected_region, world identities), diagnostics (solveStage, timeline, backup certificate diagnostics, LogOneReplan for viz).
**Planner state mutated per request (not read-only from the request):** route_snapshot_, acceptance lease, pass_through_next_target_, terminal_stop_required_, goal acceptance radius, map_ptr_ (+A*/CG views), current body support, robot_state_/solve_state_, command_identity_ (discarding staged candidate), exp_traj_opt_ max velocity (ambient, R1-13), gi_.goal_p / planning_goal_p_ / goal_endpoint_adjusted_ (rewritten mid-generateExpTraj 3233/3357/3930), latest_guide_*, lookahead fields, planner_previous_exp_ (cleared at PlanFromRest start, R1-10), backup diagnostics, latest_replan, time_consuming_, route_yaw_reference_/last_route_yaw_target_rad_.
**Clock domains:** trajectory start_WT / authorization times are double seconds of `planner_context_->getSimTime()`; request deadlines are steady ns; activation/anchor stamps int ns converted with *1e-9 (2922) and back with secondsToNanoseconds (1010) (N3/R1-44); evaluator uses int ns deltas (command_time.hpp:32-46); validator uses double tt.

## State machines
- `solve_stage_` int (planner.hpp:225-227): 0 idle,1 setup,2 A*,3 corridor (30+cg stage),4 MINCO,5 backup; set via `setPlannerStage` (planner.cpp:68-94) at 1863/2094 (1), 3258/3726 (2), 4052 (3), 4404 (4), 1968/2172 (5); never reset to 0 in planner.cpp (finishPlannerTimeline closes the timeline only).
- `CorridorGenerator::solve_stage_` (corridor_generator.h:70-74): 0 idle,1 ray checks,2 line box,3 line CIRI,4 overlap LP,5 point box,6 point CIRI; set at corridor_generator.cpp:84-412, 435-475, 548-586; names in planning_stage.hpp disagree (R1-38).
- Candidate slots: `staged_planner_candidate_` / `retained_heading_candidate_` : empty -> staged (authorizeAndStage 837 / build rebind 1691) -> promoted (onExecutionTimelineActivated 876-908) | discarded (discardCommandCandidate 910, discardRetainedPositionHeadingCandidate 918, setCommandIdentity 493). Generation monotone via `reserveCandidateGenerationLocked` (854-863).
- Body witness: `current_body_support_matches_start_`/`admission_pending_`: set at plan() 2468-2469, consumed at 850, cleared in finish/cleanup 2363-2381.
- `candidate_terminal_stop_active_`: false at solve start (1862/2093) -> true only when connected to STOP (4370-4371) or STOP hold (1831).
- `baseline_candidate_ready_for_refinement_`: false on new goal / PlanFromRest -> true after staged candidate; selects baseline_only for next MINCO (2128).
- Acceptance endpoint lease (request_id, route_revision, waypoint_index) : reset on route identity change (setRouteSnapshot 532-542) -> set in resolveGoalForPlanning 442/537 -> reset if invalid 312.
- A* node state (GridNode::OPENSET/CLOSEDSET + rounds_ stamp, astar.cpp:681, 771, 913) with lazy-deletion open set; frontier queue with sequence stamps (897).
- RET_CODE result of solves {SUCCESS, FAILED, NEW_TRAJ, EMER, NO_NEED, FINISH, OPT_FAILED...} mapped to CompletePlanningOutcome in plan() 2519-2565 and to PlanningFailureStage/Reason via classifyPlannerFailure (replan_contract.hpp:58-116).

## Behavior per branch
**plan()** (2349-2569): cancelled -> kInvalidInput-stage outcome; request invalid / limits != back_traj_cfg / policy mismatch / nominal speed invalid -> finish() (kInvalidInput, no log); route invalid -> finish; setState false -> finish; anchor missing for future mode -> finish; result NO_NEED -> kRetainedCommittedBundle; result != SUCCESS/FINISH -> classifyPlannerFailure -> kNoCompleteBundle; export fails -> kCommitRecertification/kCandidateExportInvalid; else kDeadlineWithCompleteBundle | kBaselineCompleteBundle | kRefinedCompleteBundle. Exceptions (setCommandIdentity/setWorldModelView throw) escape plan(); ScopeExit cleans witness.
**planInitialFromStoppedStateImpl** (1850-2068): no odom -> NO_ODOM; route yaw invalid -> INVALID_ROUTE; start not traversable -> shift <=3 m or NO_START_POINT; STOP hold applicable -> hold result; EXP FAILED -> classified; deadline before/after backup -> TIMEOUT; backup not executable -> classifyBackupResult; SUCCESS -> MAIN+BACKUP staged; FINISH/NO_NEED -> main-only staged; stage-history failure -> discard + CANDIDATE_REJECTED.
**planSuccessorFromExecutionAnchorImpl** (2078-2347): as above plus EXP NEW_TRAJ/EMER passthrough, EXP NO_NEED -> retain; replan_dt > cfg budget -> TIMEOUT (R1-09); backup NO_NEED with backup-capable committed command -> retain (NO_NEED); SUCCESS/NO_NEED/FINISH -> stage; else backup-required failure.
**generateExpTraj** (2878-4651): future activation not in future -> FAILED; hot splice outside envelope -> FAILED (retain) or fail-closed if measured start not traversable; command end reached -> FAILED; invalid guide state/prefix -> FAILED; A* fail -> FAILED; bounded prefix not certified -> FAILED; allocation invalid -> FAILED; SFC fail -> FAILED; vertical envelope fail -> FAILED; speed governor insufficient -> STOP_OUTSIDE_RECOVERY_ENVELOPE/STOP_SYNTHESIS_FAILED/MAIN_KNOWN_FREE_INSUFFICIENT/timeout; MINCO no candidate -> FAILED; successor over forward time -> FAILED; boundary mismatch >2 cm -> WARN only (R1-18); yaw fail -> FAILED; flatness fail -> FAILED; prefix duration invalid -> FAILED; else SUCCESS.
**generateBackupTrajectory** (4653-5703): abort -> FAILED; start beyond end -> NO_NEED; command boundary not BACKUP-admitted -> FAILED; all visible & rest & not STOP -> FINISH (main-only); STOP rest within robot_r -> FINISH; visible otherwise -> still require backup; <=1 visible sample -> FAILED; no switch window -> OPT_FAILED; backward search finds no KNOWN_FREE hull -> OPT_FAILED (first hull test always fails, R1-20); refinement fail/out-of-window/shape-deviation/UNKNOWN -> fallback to seed; seed not KNOWN_FREE -> OPT_FAILED; yaw opt fail -> minimum-snap yaw stop (R1-22) or OPT_FAILED; flatness fail -> OPT_FAILED; else SUCCESS.
**authorizeAndStage** (630-852): no authorizer -> reject; yaw rate/accel over limit -> reject; identity invalid -> reject; route regression invalid (not rebind) -> reject; no published world -> kNoPublishedWorld; latest-world sweep invalid -> reject; commit lambda: cancelled / canCommit false / rebind occupying slot / generation overflow -> not committed; else staged, body witness consumed. All rejections surface as PLANNER_CANDIDATE_REJECTED -> kWorldChanged (R1-40).
**exportStagedCommandCandidate** (966-1342): zero identity/generation -> invalid_input_identity; bad window -> invalid_time_window; zero world identity -> invalid_world_identity; identity mismatch -> staged_identity_mismatch; empty/non-finite -> invalid_trajectory; ns conversion fail -> invalid_endpoint_metadata; pass-through boundary witness missing -> incomplete_route_boundary; role schedule invalid; protected region invalid; else candidate.
**validateExecutableCandidate** (validator 525-827): non-finite/finished -> kInvalidTimeWindow; role partition not exact -> kInvalidRoleSchedule (R1-06/R1-07); bad geometry; initial point blocked; per step: piece lookup, chord halving to 0.5*res (floor 2 ms), curvature halving to 0.25*res, tube enumeration (limit 4096 -> kEnumerationLimit), endpoint classify, segment oracle; valid when t reaches duration.
**PathSearch / A*** (5724-6156, astar.cpp:354-975): AABB support insufficient -> false (partial allowed for lookahead); start obstacle -> false; escape fails -> false; inf-map P2P (preferred altitude, then unrestricted) -> NO_PATH -> prob-map retries; INIT_ERROR -> goal_valid=false; path trimmed to corridor box; shortcut (24-lookahead); blocked continuous edge -> invariant failure false.
**SearchPolytopeOnPath** (corridor_generator.cpp:73-414): empty path / invalid subdivision / gate not interior -> false; path starts occupied -> empty box prefix; blocked adjacent edge -> false; seed too long -> false; CIRI fail -> false; low overlap -> point-seed retry, overlap <= 0.01 -> false; route gate cell excludes waypoint / low overlap -> false; 1000 iterations -> false.
**resolveGoalForPlanning** (265-559): lease valid -> leased endpoint; STOP & known-free & point certifiable -> exact goal; STOP & free but clearance deficit -> best lattice point in xy-disc (O(n^2 x N_occ)); state not OCCUPIED/OUT_OF_MAP -> exact goal; nearestNotOccupied fails -> exact goal; candidate not traversable -> exact goal; else projected + walked inward, lease recorded.
**commitEmergencyBrake** (2726-2864): non-finite -> false; seed infeasible or altitude not preserved -> false; 24x1.15 duration growth until yaw+flatness pass else false; emergency candidate authorizeAndStage.

## Information needs
| input | source | frame | clock | freshness bound | authority |
|---|---|---|---|---|---|
| start KinematicState (p,v,a,j,q,yaw_rad, estimated flags) | runtime request.start_state (odometry bridge) | ENU world, q world<-body; yaw_rad independent of q (R1-23) | source_stamp_ns (int ns) -> rcv_time double s | none checked in planner (runtime owns) | measured; a/j estimated bounded by min(exp,back) limits (setState 6159) |
| execution anchor (PVAJ, yaw, yaw_rate, activation_stamp_ns) | runtime ExecutionAuthority | ENU | int ns; compared to getSimTime() double s (2920-2937) | must be future at solve start | execution-owned |
| WorldModelView (inflated/evidence layers, identity) | mapping via request.world; latest via WorldCommitAuthorizer | ENU AABB rolling | observation_stamp_ns | pinned per request; latest at authorisation | world model |
| observed occupied points (CIRI) | WorldModelView::observedOccupiedPoints(box) | ENU | same snapshot | per corridor seed | world model; UNKNOWN not represented (R1-28) |
| route snapshot (waypoints, segments, active index, acceptance radius) | mission via request | ENU | revision/request_id | immutable per request | mission |
| mission start position | request.mission_start_position_world | ENU | - | per request | mission (used only for route yaw, not route certificate R1-24) |
| dynamics (V/A/J, unknown policy, cruise) | request.dynamics vs cfg_ | - | - | must equal back_traj_cfg | cfg_ is authoritative; request only lowers cruise (R1-13, R1-36) |
| steady deadline, cancellation token | runtime transaction | - | steady ns | hard | runtime |
| sim time | planner_context_->getSimTime() | - | ROS/sim double s | - | trajectory timestamps, sim deadline (R1-03) |
| committed command snapshot | CmdTraj planner_warm_start_ | ENU | start_WT double s | activation-synchronised | execution ACK |
| current body support witness | request.current_body_support | ENU | source_stamp_ns + world identity | same world snapshot | single-use per stopped request |
| emergency history endpoint | request.history.emergencyEndpointFor | ENU | generation/epoch | per request | runtime (route-regression exception) |

## Bottlenecks
- Backward backup switch search: one braking seed + CIRI polytope + swept validation per sample_traj_dt_s step (5175-5334, R1-21).
- Swept validation `validateExecutableCandidate` runs on every candidate at least 3x (backup seed, backup final, authorizeAndStage) plus runtime recertification closures per world revision; each step enumerates up to 4096 cells and calls isSegmentTraversable.
- A*: heap allocation + hash insert per visited node, ray query per edge, up to 6 searches per PathSearch (R1-31); usleep in visual_process mode (R1-32).
- Guide prefix sampling scans the whole remaining committed command with per-sample classify (3132-3147) though only `receding_distance_m` is kept.
- resolveGoalForPlanning clearance lattice O((2e+1)^2 x N_occupied) (363-417), only when lease invalid.
- `main_acceptance_entry_t` scan up to 1e7 samples guard (5082-5100).
- Heavy fmt logging on hot path (info per corner/stretch/backup), fmt::print / std::cout to stdout in planner/corridor/CIRI/A* (R1-29).
- LogOneReplan copies full SFC vectors and trajectories every solve (setExpCondition/setExpTraj/setBackup*), returned by value.
- solve_commit_mutex_ held while copying staged candidate (full trajectories) in export/validate (2666-2671, 2691-2701).
- Timer before replan_lock_ + second budget (R1-09).

## Notes for target architecture
Keep (good, load-bearing):
- One immutable request -> one typed outcome; execution-owned activation with staged/retained slots and monotone generations; history promoted only on ACK.
- Continuous swept validator (tube enumeration + curvature bound) and analytic polynomial certificates (stop polynomial extrema, route regression by derivative roots, corridor plane violation by roots) — mathematically sound (verified, see coverage notes).
- Dual deadline with steady-clock authority, cancellation token polling in every long loop, finalization reserve for refinement.
- Deterministic minimum-snap BACKUP seed as the safety artefact; optional refinement can only fall back to the seed.
- Evaluator closures capture immutable shared state by value (no dangling).
Over-engineered / to remove or restructure:
- Planner is a 6.2k-line god object with ~80 mutable members mutated mid-solve; generateExpTraj ~1.8k lines (R1-17). Split into pure stages with typed inputs/outputs (guide -> endpoint policy -> corridor -> nominal -> backup -> certify -> export).
- Multiple sources of truth: nominal speed (cfg vs request, R1-13), dynamic limits (R1-36), role-at-time (R1-01/R1-06), acceptance geometry (sphere vs box, R1-02), corner threshold (R1-42), braking altitude policy (R1-19), boundary state for governor (R1-14), plane normalization (R1-43), stage names (R1-38).
- Duplicated heading-rebind builders (R1-08), dead branches/flags (R1-11, R1-16, R1-20, R1-35, cut_first_poly R1-41), tautological config checks and locked-constant "parameters" (R1-37).
- Corridor generator should be a sound certificate (query box vs boundary box, UNKNOWN policy) or explicitly demoted to optimizer hint (R1-27/R1-28).
- Replace int-coded stages, RET_CODE and PlannerResultCode chains with one typed rejection reason carried from authorizeAndStage to PlanningOutcome (R1-40).
- Time: one integer-ns type through planner, bundle and evaluator (N3/R1-44); derive sim deadline once (R1-03).

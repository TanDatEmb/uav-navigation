## Role

R7 is the SITL/replay harness and judge: `tools/runtime/runner.py` orchestrates PX4/Gazebo/ROS processes and generates per-session parameter files; `monitor.py` samples streams; `external_mode_scenario.py`/`offboard_scenario.py` drive arm/takeoff/mode activation and record flight evidence; `evaluation.py`/`report.py` reduce evidence to PASS/FAIL/NOT_EVALUABLE; `html_report.py`/`flight_review_report.py` render. Static guards in `tools/check_*.py` are CI source-text contracts. `config/runtime/*` is the YAML layer for both product nodes (via runner-generated param files) and the judge. The judge is NOT a flight authority but it is the only authority for "PASS" evidence, so its math and its re-derivations of C++ semantics (V7) matter.


## Components

| File / class | Responsibility |
|---|---|
| tools/runtime/runner.py (`_run_sim_unlocked`, `run_dataset`, `RuntimeLock`, `_RunnerSignalGuard`) | CLI entry; scene→profile→world/mission resolution; per-session param generation (fast_lio, navigation_runtime, planner.yaml, external_mode, scenario_config, resolved_mission); process orchestration; effective-config witness checks; stop+report |
| tools/runtime/process_group.py (`Session`) | process-group ownership registry, PID-reuse guard, staged SIGINT/TERM/KILL shutdown, state.json |
| tools/runtime/monitor.py (`StreamStats`, monitor node) | rclpy sampler of all streams; rate/gap/stale/timestamp statistics (wall arrival); monitor.json + samples.jsonl |
| tools/runtime/external_mode_scenario.py (`ExternalModeScenario`) | PX4 arm/takeoff/External-Mode activation FSM, GT collision/clearance, LIO-vs-GT watchdog (can request handover), truth frame witness, evidence recording, scenario verdict |
| tools/runtime/offboard_scenario.py / closed_loop_characterization.py | offboard segment follower / analytic NED reference harness |
| tools/runtime/evidence_writer.py (`EvidenceWriter`) | bounded async JSONL writer with drop accounting |
| tools/runtime/evaluation.py | offline evaluator: lifecycle/world-transaction reducers, tracking vs truth/LIO, motion quality, localization ATE/RPE, C0_IFP + C0_SW assessments |
| tools/runtime/report.py | runtime verdict (PASS/FAIL/BLOCKED), stream/provenance/mapping/acceptance reasons, integrates evaluation, writes report.json + REPORT.html |
| tools/runtime/flight_review_report.py / html_report.py | HTML rendering; re-derive their own gates/overall (R7-41) |
| tools/runtime/world_observation_gate.py | SITL-only RegisteredScan relay that drops mapping input for fault tests |
| tools/runtime/evidence_contract.py, planner_trace.py, command_diagnostics.py | evidence schema/joins used by evaluation |
| tools/data.py | dataset catalog/prepared-dataset resolution and bag topic accounting |
| tools/check_*.py, validate_runtime_safety_ledger.py, pre_main_gate.py | source-text static guards and ledger validator run by CI/pre-main |
| src/uav_simulation (gz_lidar_bridge, visibility_bridge, visibility_cloud) | Gazebo PointCloud → ROS (latest-only worker thread); organized cloud → free-space endpoints (explicit no-return rays) |
| src/navigation_bringup/launch/* | fast_lio(+external odometry bridge, RSP), navigation_runtime, px4_external_mode, avoidance_mission composite |
| config/runtime/*.yaml | common (judge thresholds/timeouts), sim/dataset (fast_lio + bridges), mapping (runtime node), external_mode (adapter), scenario templates, map_profiles registry, missions |


## Processes & threads

Launch order in `runner._run_sim_unlocked` (runner.py:3617-3868), each `session.start(role, bash -lc 'source …; exec …')` in its own process group:
1. `monitor` (monitor.py --workflow sim --config sim.yaml) — rclpy sampler, writes monitor.json/samples.jsonl.
2. `px4_gazebo` (tools/simulation/run_px4_mid360.sh; env PX4_GZ_WORLD, SITL profile, COM_RC_IN_MODE) → wait `/world/<w>/clock` (startup_s=90).
3. optional `gazebo_native_observer` (diagnostic).
4. `xrce_agent` MicroXRCEAgent udp4 -p <port 8892>.
5. `bridge` ros_gz parameter_bridge (control bridge yaml with world clock rewritten), `bridge_lidar` gz_lidar_bridge (/sim/mid360/scan/points→/lidar/points).
6. `rosbag` ros2 bag record RUNTIME_EVIDENCE_TOPICS (runner.py:132-160).
7. `visibility_bridge` (gz_visibility_bridge 720×28 beams, range 40/60 m) → /lidar/free_space_endpoints.
8. `px4_ingress` px4_odometry_bridge_node (fast_lio_params.yaml).
9. optional `world_observation_gate` (fault injection on /lio/mapping_observation).
10. `mapping` = navigation_runtime.launch.py (navigation_runtime_params.yaml + resolved_mission.yaml) → log witnesses checked (tracking, planner_fault, dynamics).
11. `lio` = fast_lio.launch.py (fast_lio + px4_external_odometry_bridge + robot_state_publisher; mount 0 0 0.28 hard-coded).
12. `rviz` (GUI). Waits: imu&lidar>0 (90 s) → LIO TRACKING (90 s) → external_odometry>0 (45 s) → mapping_ready (180 s).
13. `external_mode` px4_external_mode.launch.py → wait EXTERNAL_MODE_READY_MARKER (15 s) → tracking witness check.
14. `external_mode_scenario` (or offboard / closed_loop_characterization) → wait wall_timeout_s+10 → sleep post_flight_monitor_s → `_stop_and_report` → report.build.
Dataset (runner.run_dataset 2684-2850): monitor → mapping → lio (no external odom) → wait diagnostics → `replay` ros2 bag play --clock → optional dataset_shadow_planning (45 s) → wait drain.
Locks: BuildRuntimeLock(shared) + RuntimeLock(flock .runtime-sim.lock / .runtime-dataset.lock) + XRCE port probe; ROS_DOMAIN_ID 42 (SITL) / 43 (dataset); GZ_PARTITION per domain/port. Signal guard converts SIGINT/SIGTERM to KeyboardInterrupt (can interrupt a file write, R7-21).


## Flows

**Config layering (who overrides whom)**
- Judge/runner runtime settings: workflow YAML (sim/dataset) + `common.yaml.runtime` shallow-merged by `load_config` (runner.py:365-375) and again by `report._build_complete_report` (report.py:3669-3676); `runtime_overrides` shallow (R7-22).
- fast_lio + px4_external_odometry_bridge + px4_odometry_bridge: `sim.yaml`/`dataset.yaml` → `_ros_params` keeps only those node blocks and overrides `preprocessing.maximum_range_m` = visibility range (runner.py:1271-1296) → `fast_lio_params.yaml` (shared by 3 nodes) → launch adds `use_sim_time`.
- navigation_runtime_node: `mapping.yaml` → `_mapping_params` overwrites tracking_experiment (runner default *relaxed*, R7-16), mission_file, fault-injection flags, registered_scan_topic (fault runs); planner.yaml copied with backup_allow_unknown/raycasting/control_envelope overrides → session planner.yaml → `navigation_runtime_params.yaml`; launch then sets `navigation_runtime.mission_file` again from `mission_file:=` (launch default "" would clobber it; runner always passes it).
- px4_navigation_external_mode: `external_mode.yaml` → `_external_mode_params` overwrites tracking_experiment (+trace flag).
- Scenario: `external_mode_scenario.yaml`/`offboard.yaml` → runner overlays ~40 keys (profile, mission, collision obstacles from SDF, pillar table, timeouts, acceptance, expected_outcome from registry, speed contract) → `scenario_config.yaml`; in-code defaults disagree (R7-46).
- Mission: `missions/<registry mission>.yaml` → `_resolved_mission_file` (+speed cap) → `resolved_mission.yaml` consumed by runtime, adapter(no), scenario, report, evaluation (good single source).
- Keys read by >1 process with different values/defaults: propagated-odometry freshness (0.20 adapter / 0.5 runtime / 0.50 judge / 0.10 scenario), external odometry (0.5 bridge / 0.20 judge) (R7-08); tracking_experiment.mode (YAML off vs runner relaxed) (R7-16); cross-track limit (registry vs runner vs renderer) (R7-04/R7-41); expected_outcome vocabulary (mission_complete vs complete) and fail-closed set (R7-19); vehicle collision radius 0.35 (R-08); lidar grid (R7-42); timing metadata (R7-20).

**Evidence flow**: product topics → monitor (samples.jsonl, monitor.json) + scenario (scenario.jsonl via EvidenceWriter, scenario.json) + rosbag → report.build → evaluation.load_evaluation_inputs (normalize, reducers) → evaluate_session → report.json → flight_review_report.render → REPORT.html.

**V7 — places where the judge re-derives C++ semantics**
| Judge site | Mirrors |
|---|---|
| runner.py:3185-3188 waypoint default behavior "stop" | navigation_mission/src/mission.cpp:110-114 (pass_through unless hold/last) — divergent (R7-17) |
| runner.py:1383-1408 `_mission_planning` allowed keys/unknown_policy | mission.cpp:59-147 |
| runner.py:1168-1172 planner/command rates | planner.yaml + PlanningTimingContract::kPlannerRateHz (R7-20) |
| runner.py:644-673 suppressed gate list for tracking modes; report.py:441-518 re-infers mode/suppression | tracking_experiment.hpp loader |
| evaluation.py:290-292,351-369,443,646-718,2807,2845 raw ordinals | CandidateSource, RetainedValidationPurpose, PlannerStatus, PlannerResultDisposition, retained disposition codes (R7-31/R-05) |
| external_mode_scenario.py:293-306 mode status names | NavigationModeStatus.msg:13-24 (R7-36) |
| external_mode_scenario.py:1383-1418 divergence thresholds 0.45/0.5 m, 0.75/1.0 m/s, 0.5 s | product localization watchdog |
| external_mode_scenario.py:1118,runner.py:3161 vehicle radius 0.35 | planner vehicle_radius_m (R-08) |
| report.py/evaluation.py/html_report waypoint acceptance parsers | mission_progress.cpp acceptance (R-07, R7-28) |
| html_report.py:382-405 NED↔ENU, report.py _C_NED_FROM_ENU/_C_FRD_FROM_FLU | px4_odometry_bridge conversions |


## State machines

**Run lifecycle (runner._run_sim_unlocked, runner.py:3021-3942)** — implicit, not an enum: VALIDATE_ARGS → RESOLVE_SCENE/PROFILE (1766-1814) → SESSION_CREATE (3310) → CONFIG_GENERATE (3403-3607) → PREREQ (3499; fail → `_stop_and_report` FAIL, rc 1) → PROCESS_START (3617-3783) → READINESS waits (3797-3804; TimeoutError/early exit → failures) → EXTERNAL_MODE_READY (3841-3857; CONFIGURATION_MISMATCH → setup_status) → SCENARIO (3868-3886) → POST_FLIGHT sleep → STOP_AND_REPORT (2555-2582) → rc: 0 PASS/OBSERVATION_COMPLETE, 2 ABORTED_OPERATOR/PAUSED_SAFETY_STOP (unless expected fail-closed pause), else 1 (3930-3942).

**Scenario tick FSM (external_mode_scenario._tick, 2497-2856, mission mode)** — boolean-flag state, no enum:
WAIT_SIM_CLOCK → DISCOVER_MODE (external_mode_id from can_set_nav_states_mask, 1424-1433; timeout ACTIVATION_TIMEOUT) → PRE_ARM_ACTIVATE (VEHICLE_CMD_SET_NAV_STATE, 2557) → ARM (retry arm_retry_period_s; ARM_TIMEOUT) → [amsl] PREPARE_HOLD → SETTLE (takeoff_mode_settle_s) → NAV_TAKEOFF (global AMSL + alt; GLOBAL_ALTITUDE_TIMEOUT) | [local_ned] PRESTREAM setpoints → OFFBOARD → TAKEOFF_WAIT (z ≤ −0.9·alt & |v|<0.5; TAKEOFF_TIMEOUT) → STABLE_WINDOW (|v|≤0.15 for takeoff_stable_s; odometry age ∈[−0.05,0.10] for 1.0 s) → ACTIVATE_EXTERNAL (retry; activation timeout → handover request airborne_activation_timeout) → MISSION (mission_timeout_s from mode entry → MISSION_TIMEOUT) → terminal: FAILED_COMPONENT (localization divergence or ModeStatus FAILED), COMPLETE (mission_complete & nav_state==AUTO_LOITER; else HOLD_HANDOVER_FAILED after hold_handover_timeout_s), PAUSED_SAFETY_STOP (ModeStatus PAUSED+SAFETY_STOP, or PAUSED & Hold), ABORTED_OPERATOR (mode exit & Hold). Global: WALL_TIMEOUT (wall clock). Exit code: 0 COMPLETE w/o failures, 2 ABORTED/PAUSED, 1 otherwise (3172-3178).
Localization watchdog (1255-1418): arms after post-takeoff mode entry; FAIL if (expected fail_closed: pos>0.45) or (pos>0.5 & vel>0.75) or vel>1.0 sustained 0.5 s → handover request + FAILED_COMPONENT.

**Report verdict (report._sim_report 3523-3588)**: reasons = provenance + per-stream samples/staleness/validity (+R-01 mask) + external odom rate + LIO TRACKING + mapping integrity + scenario failures + process failures + mission acceptance → PASS iff no reasons; overrides for external-mode: PAUSED_SAFETY_STOP → BLOCKED (not expected fail-closed) / FAIL (no structured evidence) / PASS-after-removing-handover-reason; ABORTED_OPERATOR → BLOCKED; FAILED_COMPONENT → FAIL (R7-37). Build crash → durable FAIL report (3796-3825).

**Evaluation dimensions (evaluation.evaluate_session 3201-3408)**: mission {PASS, FAIL, NOT_EVALUABLE}, safety (collision_count==0), tracking (truth-only; requires 5 qualification checks then policy p95/max), motion (always NOT_EVALUABLE), evidence (completeness). assessment = FAIL if any FAIL, PASS if all PASS (unreachable, R7-30), else NOT_EVALUABLE. C0_SW axes PRODUCT_LOGIC/AUTHORITY_IDENTITY/TEMPORAL_SAFETY/EVIDENCE_COMPLETENESS (2852-3165). The evaluation never changes report.verdict (3695-3701).


## Behavior per branch

| Function | Branch → outcome |
|---|---|
| runner._resolve_scene_profile (1766) | MAP_PROFILE given → used verbatim (legacy escape); scene missing/smoke → "smoke"; testcase → motion preset → nominal/positive fallback; none → ValueError |
| runner._resolve_map_descriptor (1858) | profile in registry → descriptor w/ mission, truth, outcome, benchmark; not in registry (open/speed/smoke) → synthetic descriptor, expected_outcome hard-coded mission_complete (R7-02) |
| runner._collision_obstacles (1530) | SDF parse OK → all models with box/cylinder (rpy ignored R-03); parse error → swallowed → hard-coded fallback; unknown profile → UnboundLocalError (R7-03) |
| runner._check_effective_*_configuration (2319/2398) | witness missing/timeout → CONFIGURATION_MISMATCH RuntimeError; count≠1 → mismatch; value ≠ requested (exact, %.17g round-trip) → mismatch; ok → metadata.runtime_configuration |
| runner._wait_until (2229) | predicate true → return; any core role pgid dead → RuntimeError "exited before readiness"; deadline → TimeoutError |
| external_mode_scenario._ground_truth (1085) | non-finite pos → ignore; per obstacle signed distance − 0.35 → min clearance; ≤0 → collision_count++; pillar 2D clearance if pillar_waypoint_index≥0 |
| external_mode_scenario.finish (2857) | common checks (mode id, PVA, setpoints finite/yaw, Hold, exit, failsafe, RTL, divergence); mission-only: waypoint acceptance order (+initial skip), fail-closed contract, planned pillar clearance, GT clearance ≤0.05, < configured margin, collisions, map observability, speed contract, takeoff, terminal outcome (R7-33) |
| report._mission_acceptance (673) | disabled or fail_closed → no reasons; missing completion → reason; acceptance indices ≠ expected (± initial skip) → reason; cross-track computed but never gated (R7-04); outcome≠COMPLETE → reason |
| report._sim_report (3458) | see State machines; stale masked for external/propagated odometry on terminal handover (R-01) |
| evaluation.evaluate_tracking (2227) | frame witness invalid → NOT_EVALUABLE metric; else bracketed errors (≤ pairing gap, same identity); coverage policy/window missing → NOT_EVALUABLE; velocity error frame bug (R7-25) |
| evaluation._tracking_acceptance_status (3168) | policy missing/provenance/version/invalid → NOT_EVALUABLE; metric missing → NOT_EVALUABLE; any limit exceeded → FAIL |
| evaluation.evaluate_software_qualification (2852) | product_logic: COMPLETE+complete ordered acceptance → PASS; PAUSED_SAFETY_STOP with exact retained/result witness (raw ordinals, R7-31) → PASS else NOT_EVALUABLE; COMPLETE w/o lineage → NOT_EVALUABLE; COMPLETE/FAILED otherwise → FAIL |
| monitor.StreamStats.update/check_stale (140/238) | stamp>upper bound → epoch discard; wall gap > stale_after → arrival-gap event; duplicate/regression counters; timer: age>stale_after after ≥2 samples → one stale event per outage (R7-39) |


## Information needs

| Input (consumer) | Source | Frame | Clock | Freshness bound | Authority |
|---|---|---|---|---|---|
| /sim/ground_truth/odometry (scenario collision, watchdog; evaluation truth) | gz OdometryPublisher via ros_gz bridge | pose: Gazebo world ENU; twist: child base_link (body FLU) | sim time | none in scenario (only 40 ms pairing for watchdog) | evaluation-only truth |
| /lio/odometry_propagated (scenario pre-activation, watchdog) | fast_lio | lio_odom; twist body | sim time | 0.10 s pre-activation; judge 0.50 s; adapter 0.20 s (R7-08) | product state |
| /navigation/navigation_command (evaluation reference) | navigation_runtime | lio_odom ENU (velocity world) | sim time source stamp | pairing ≤ max_pairing_gap_s / 0.15 s | product command |
| /fmu/out/vehicle_status_v1, vehicle_local_position_v1 (scenario FSM) | PX4 via XRCE | NED local | PX4 µs timestamps (not used for age) | none (latest value) | PX4 |
| /fmu/in/trajectory_setpoint (scenario speed/yaw counters) | adapter AND scenario itself (local takeoff) | NED | PX4 µs | n/a | observation, unattributed |
| monitor streams (report freshness) | all topics | various | wall arrival (time.time_ns) vs stale_after_s (R7-39) | common.yaml streams | judge |
| truth_frame_witness T_L_G (evaluation) | scenario, first post-takeoff valid LIO/GT pair | lio_odom ← gazebo world | sim time | validity_scope localization_epoch | evaluation transform |
| map_profiles.yaml registry (runner, report html) | repo | world ENU | n/a | n/a | scene/world/mission/outcome (partially bypassed: R7-02/R7-04/R7-19) |
| resolved_mission.yaml (runtime, adapter, scenario, report, evaluation) | runner copy of mission YAML + speed cap | lio_odom | n/a | n/a | single mission input (good) |
| RUNTIME_CONFIG_EFFECTIVE log witnesses (runner) | mapping.log / external_mode.log | n/a | wall | 15 s wait | requested-vs-effective check |


## Bottlenecks

- Offline evaluator `_bracket` sorts the full measured stream per reference sample: O(N·M log M); measured 4.1 s for 3k samples, ~100 s for a 5-min run (R7-24).
- Monitor: single-threaded rclpy spin with per-message list copies (R7-40); its wall-arrival timing is itself the freshness evidence, so monitor lag becomes false staleness (R7-39).
- Scenario node: single-threaded, 50 ms tick, decodes every /lidar/points in Python (`sensor_msgs_py.read_points`, ~20k points/scan at 10 Hz) in the same executor as GT/odometry callbacks → tick jitter delays FSM transitions and collision sampling under load.
- Runner readiness polling reads monitor.json + scans /proc per role every 0.25 s (R7-45); sequential shutdown up to 8 s per process group.
- world_observation_gate inserts a Python relay into the mapping input during fault runs (latency confound for world-freshness evidence).


## Notes for target architecture

- Keep: session-owned generated parameter files + effective-config log witnesses (runner.py:2319-2458) — a good "requested vs effective" pattern; resolved_mission.yaml as single mission input for all processes; process-group session model; DDS/GZ isolation; lidar→imu extrinsic inversion (runner.py:2181-2215, verified correct).
- Over-engineered / to remove: three profile→world alias maps + hard-coded fallback collision geometry (R7-01/R7-03); per-profile acceptance/timeout/pillar tables in runner that duplicate map_profiles.yaml (R7-04/R7-18/R7-19); substring-based static guards (R7-12/R7-13) — replace with AST/unit tests.
- Config layering target: one declared registry (map_profiles.yaml) owning world, mission, expected outcome, route obstacles, acceptance limits; one freshness contract per stream shared by product and judge (R7-08).

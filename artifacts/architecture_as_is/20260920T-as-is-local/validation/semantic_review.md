# Semantic review độc lập với model lint

Đây là rà soát code đọc bằng tay trên local working-tree snapshot, không phải checker tự xác nhận model. Checker chỉ chứng minh ref/ID/path/hash/link nhất quán.

## Đối chiếu owner và boundary

- Runtime active/pending timeline store là authority cho candidate thực thi; worker queue là scheduler; sampler là reader. Candidate result/diagnostics không nâng quyền của bundle. Evidence: [worker](../evidence/refs.html#E_WORKER), [candidate](../evidence/refs.html#E_CANDIDATE), [timeline](../evidence/refs.html#E_TIMELINE), [publish gate](../evidence/refs.html#E_PUBLISH_FINAL).
- `ExecutionEpisode` contains recovery enum alongside phase/availability/active generation; `transitionExecutionRecovery` is a pure next-state helper. Mutators/callers own actual transition. Test `CommitUpdatesLifecycleAndRecoveryInOneSnapshot` checks episode component behavior, not all callers. Evidence: [snapshot](../evidence/refs.html#E_EPISODE_SNAPSHOT), [helper](../evidence/refs.html#E_RECOVERY_HELPER), [test](../evidence/refs.html#E_EPISODE_TEST).
- `NavigationMode` cache, `MissionController` waypoint/request and `NavigationModeExecutor` Hold flags are distinct owners. `NavigationCommand` acceptance, mission advancement, `TrajectorySetpointType::update`, mode API result and `VehicleStatus.AUTO_LOITER` are separate boundaries. Evidence: [mode admission](../evidence/refs.html#E_MODE_ADMISSION), [mission update](../evidence/refs.html#E_MISSION_UPDATE_END), [setpoint](../evidence/refs.html#E_SETPOINT), [Hold executor](../evidence/refs.html#E_HOLD).

## Source/comment/config conflicts retained

1. `NavigationMode::checkArmingAndRunConditions` comment around lines 810-812 says first goal is published from `onActivate`; actual activation calls `updateMission`, whose `MissionController::update` returns `None` while `!airborne`; test `WaitsForAirborneBeforePublishingMissionGoal` asserts no goal before airborne. More exact semantic: activate calls an immediate update; the immediate goal occurs only if armed and local z > 0.5 m already. The test does not establish full mode behavior. [Comment excerpt](../evidence/refs.html#E_ACTIVATE_COMMENT) · [update](../evidence/refs.html#E_MISSION_UPDATE) · [test](../evidence/refs.html#E_MISSION_TEST).
2. Config comments call zero tracking envelope a disabled tracking/health-response gate. Shared loader sets `enabled` from `use_sim_time`; zero coefficients make `trackingGateEnabled=false`, and then simulated time sets suppress flags true. `avoidance_mission.launch.py` defaults `use_sim_time=false`, while a simulation invocation may pass true. Live launch parameters are unresolved. [YAML](../evidence/refs.html#E_TRACKING_CONFIG) · [policy](../evidence/refs.html#E_TRACKING_POLICY) · [launch default](../evidence/refs.html#E_LAUNCH_DEFAULT) · [runtime gate](../evidence/refs.html#E_RUNTIME_TRACKING_GATE) · [mode bypass](../evidence/refs.html#E_TRACKING_MODE_BYPASS).

These conflicts are not modified or reinterpreted as design intent.

## Concurrency and profile limits

The local runtime uses two executor threads plus workers; PX4 state inputs have a separate thread. Runtime transition locks are documented as ingress -> localization -> input -> command transition. Mode command and mission timer are grouped/serialized by `trajectory_mutex_`; no total order spans separate ROS processes. Current source adds identity/version rechecks around stale-result, stage, activation and exposure paths, but this review is not a proof of all indirect writer paths.

Compile database provenance is not a current runtime profile, and test/build binaries are not tied to the captured local source. No build/test was run. These constraints are recorded in [baseline metadata](../baseline/environment.json) and [coverage](../tables/coverage.md).

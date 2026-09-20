# Commands, provenance, exit codes

All artifact paths below are under artifacts/audit_claim_validation/20260920T-claim-validation-local. Source A is the source_snapshot at artifact commit f2bd3f46f9936d622377ea4761f733f988273b66. No source/config/product test was edited.

## Tool/build provenance

- dot - graphviz version 2.43.0 (0); all six DOT source files rendered successfully to SVG.
- mmdc --version: command not found. node and npm are not installed; Mermaid rendering was blocked without installing a global or external runtime.
- GCC/G++ 13.3.0, CMake 3.28.3, ROS Jazzy / ament_cmake 2.5.6, Release builds. H5 helper uses compile flags and dependency link list from the isolated build compile database/target link file.
- Audit submodule revisions: px4_msgs 86d8239e962f6939e05c3737784f60c02fa884db; px4_ros2_interface_lib 4a3370f084ac6f1ef001a4afa2b007845ffd0837.
- Existing B compile database was not used for runtime source claims. Isolated Release compile database is validation/build/compile_commands.json, SHA-256 ca8a45783552c708b56226014fed26b09a5c06bd47b1ef891fed791e62b7733d; it references source A and isolated install paths.

## Isolated builds

1. Backend/dependency slice, exit 0; 7 packages finished in 2m18s. Command:

    source /opt/ros/jazzy/setup.bash
    colcon build --base-paths A_SOURCE_SNAPSHOT/src --build-base OUTPUT/validation/build --install-base OUTPUT/validation/install --packages-up-to navigation_planning_backend --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+

    Raw log: validation/logs/colcon_build_backend.log

2. Runtime slice, exit 0; 10 packages finished in 1m40s. Same base/build/install paths, packages-up-to navigation_runtime, plus:

    --log-base OUTPUT/validation/colcon-log

    Raw log: validation/logs/colcon_build_runtime.log

Replace A_SOURCE_SNAPSHOT and OUTPUT with the paths in the local workspace; do not substitute current checkout source for A. A and initial B were byte-identical over 596 manifest paths at audit start. Checkout C later moved to 8eaab3d33db36e9e636ad4005ed91b8f055f4d68; the isolated builds and tests below were built from frozen A, not C.

## Executed focused tests

| Test | Command/filter | Exit | Result/log |
|---|---|---:|---|
| H0 policy loader fixture | bash tools/run_h0_loader_truth_table.sh | 0 | Five loader rows printed; validation/logs/h0_compile.log and h0_run.log |
| H1 production MissionController fixture | bash tools/run_h1_mission_counterexample.sh | 0 | Positive progression, late-witness local counterexample, wrong-identity negative control; validation/logs/h1_compile.log and h1_run.log |
| Existing mission contract | test_mission_contract (isolated runtime install sourced) | 0 | 19 passed; validation/logs/h1_mission_contract_gtest.log |
| Continuation helper boundary | test_certified_continuation --gtest_filter=CertifiedContinuation.ExactPlannedEntryReserveDoesNotAuthorizeLaterMeasuredHandoff | 0 | 1 passed; validation/logs/h1_helper_boundary_gtest.log |
| Runtime recovery helper | test_planner_fsm --gtest_filter=PlannerFsm.ResumesOnlyExactFreshWorldRecertifiedGeneration:PlannerFsm.FiveSecondTimeoutExistsOnlyWhileStationary:PlannerFsm.WatchdogKeepsBoundedStoppedRecoveryHoldForRetry | 0 | 3 passed; validation/logs/fsm_focused_gtest.log |
| Execution episode lifecycle | test_execution_episode --gtest_filter=ExecutionEpisode.SuspendAndClearDoNotRetainCommandIdentity:ExecutionEpisode.RecoveryEventsRemainOneWayInsideTheLifecycleRecord | 0 | 2 passed; validation/logs/h2_execution_episode_gtest.log |
| Store lock and world generation | test_committed_bundle_store --gtest_filter=CommittedBundleStore.ExposureMustRecheckFreshnessAfterWaitingForStoreLock:ExecutionTimelineStore.WorldAdvanceInvalidatesPendingSuccessor | 0 | 2 passed; validation/logs/store_focused_gtest.log |
| Planning admission/deadline | test_planning_contracts --gtest_filter=PlanningCandidate.AdmissionRequiresDerivedMainReserve:PlanningBudget.UsesSteadyClockAndCancellation | 0 | 2 passed; validation/logs/admission_focused_gtest.log |
| Speed governor existing tests | test_planner_config --gtest_filter=PlannerSpeedGovernor.* | 0 | 3 passed; validation/logs/h5_planner_speed_governor_gtest.log |
| Audit fine-speed sweep | python3 tools/run_h5_speed_grid_oracle.py | 0 | 24 helper inputs; zero grid rejects with fine-sweep witness; validation/logs/h5_speed_grid_oracle_compile.log, link.log and run.log |

Each GTest was run after sourcing /opt/ros/jazzy/setup.bash and OUTPUT/validation/install/setup.bash. Tests are native binaries from the isolated A build; none starts a ROS node or publisher.

## Not run / constraints

- No target workload trace or matched runtime profile was available; no no-fault latency distribution was measured.
- No paired ROS process, PX4 simulator, SITL/Gazebo, flight stack, real aircraft, arming, mode change or control-topic publish was used.
- No Mermaid renderer or Node/npm was present. The app browser blocked opening the local file URL, so visual inspection of index.html/SVG was not completed. DOT rendering succeeded; the artifact checker verified SVG XML roots/viewBoxes and resolved local index links. No external visualization service was used.
- A bounded search under artifacts/ for *.ulg, *.db3, *.mcap, *trace*.jsonl, *safety*.log and *px4*.log found no matched target workload evidence.

# Evidence index

All source facts below are from TARGET `7da3e97cb399c2e39d62cfe60213a45e8a92300e`. Blob IDs allow verification after branch movement. Line spans are source locations, not executed behavior.

| Fact | TARGET path and lines | Git blob |
|---|---|---|
| Safety contract | `docs/safety/runtime_safety_current.md:40-69,84-130` | `25719f3f17127a7003d42db3e2ab332f83167350` |
| Safety index targeted | `docs/safety/runtime_safety_index.md:37,43,55,77,766` | `2aab08c33e6c905521babe0937dd2057a8006255` |
| Safety archive targeted gate rows | `docs/safety/archive/runtime_safety_legacy_full.md:3378,3384,3396` | `6453c7587a9780ab393febe7f696e4fb9e3e4139` |
| Safety archive PX4 ownership/STOP lineage | `docs/safety/archive/runtime_safety_legacy_full.md:321-349,22114-22142` | `6453c7587a9780ab393febe7f696e4fb9e3e4139` |
| Parameter contract | `docs/architecture/parameter_contract.md:1-80` | `6b171cd17b35d496adce9c16b1fe59a534d1f9ed` |
| Waypoint design | `docs/architecture/continuous_waypoint_trajectory_plan.md:193-225,294-325` | `0e1e45b482f8d2b4518f941d9605382fafa7f299` |
| Mission fields | `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/mission_controller.hpp:110-130` | `775422f25c2ac9288c5ee25625dba1cb16befb59` |
| Mission crossing | `src/px4/px4_navigation_external_mode/src/mission_controller.cpp:340-371,465-590` | `eb576715484edd5db156417b284806afd91df7ac` |
| Mission lifecycle, checkpoint, readiness and temporal gates | `src/px4/px4_navigation_external_mode/src/mission_controller.cpp:28-89,101-338,374-445,520-680` | `eb576715484edd5db156417b284806afd91df7ac` |
| Pre-stop nominal recovery | `src/px4/px4_navigation_external_mode/src/mission_controller.cpp:144-165` | `eb576715484edd5db156417b284806afd91df7ac` |
| Route cursor/tie | `src/contracts/navigation_mission/src/route_progress.cpp:256-345` | `9f3c9f41446270f3beaabeb3e43e886b7d0a5fa1` |
| Episode | `src/runtime/navigation_runtime/include/navigation_runtime/execution_episode.hpp:41-260` | `9e1d62d9fcafef4f9cf56775ee263ef3cef908dc` |
| Store | `src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp:30-35,821-829` | `43047ecc7f7da8027860bcf42c2d807c2e555885` |
| Runtime fields/locks | `src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp:429-560` | `bcd4235d7abe5dcef8730a751760b130f745cac2` |
| World suspension | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:2983-3004,3670-3690,7704-7719` | `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| Command publication | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:7935-7950,8595-8640` | `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| Sampled BACKUP | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:8200-8229` | `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| PX4 fields | `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/navigation_mode.hpp:135-261,281-290` | `0bf4112406a2d859486aded7c8d85ca06adcc526` |
| PX4 command/mission | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:1328-1410` | `00588a8a5eeac921651f5a00153e3388557d5cdc` |
| PX4 Hold | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:2846-2937` | `00588a8a5eeac921651f5a00153e3388557d5cdc` |
| Candidate callbacks | `src/planning/navigation_planning/include/navigation_planning/candidate_bundle.hpp:119-162` | `d25323cde4bf0ac569e8e0f3daa76a690f1dd6ba` |
| Worker | `src/runtime/navigation_runtime/include/navigation_runtime/planning_worker.hpp:268-350` | `a927210c7b63a5b830a930f968b647874e38cc48` |
| World owner | `src/mapping/navigation_mapping/include/navigation_mapping/world_snapshot_store.hpp:131-178` | `59c12f8a71da3c9c1b62fc4763f1efe9ea493eee` |
| Execution state ingress | `src/execution/navigation_execution/include/navigation_execution/execution_state_store.hpp:12-48` | `0ecb073af0b5b99862571d3d50d093eaeeb88e07` |
| Derivative history | `src/runtime/navigation_runtime/include/navigation_runtime/kinematic_derivative_estimator.hpp:20-91` | `810cbbd28da840c5d63502f5b0848dee15271cc5` |
| Mapping worker lifecycle | `src/mapping/navigation_mapping/include/navigation_mapping/mapping_worker.hpp:23-293` | `94f14cc9812bb483f8e81e81818a0ec73fcae9ba` |
| Heading worker lifecycle | `src/runtime/navigation_runtime/include/navigation_runtime/heading_rebind_worker.hpp:26-121` | `d6854a2d4049048698dfa8807fb561ccca2a6223` |
| Quality receipt | `src/runtime/navigation_runtime/include/navigation_runtime/baseline_refinement.hpp:30-103` | `56418c95b38ddaf007145d9737e81c676af6b078` |
| Trace-only store | `src/runtime/navigation_runtime/include/navigation_runtime/execution_trace_snapshot.hpp:135-177` | `a107cdc015948fe4461ddff16199e04ef5d4e835` |
| Command schema | `src/contracts/navigation_contracts/msg/NavigationCommand.msg:1-116` | `e8b2269aad5fab2b527af1720749ef828e826071` |
| Planning key | `src/planning/navigation_planning/include/navigation_planning/planning_request.hpp:1-130` | `fb9dfdc2d5a9e4ddfa5149bb3f976c01d44ca9ff` |
| Speed governor | `src/planning/navigation_planning_backend/include/planner_core/evidence_speed_governor.hpp:1-240` | `4fdb9635da176b4fb9c968628fa2a68219a005fd` |
| Tracking experiment loader | `src/contracts/navigation_contracts/include/navigation_contracts/tracking_experiment.hpp:58-88,147` | `4c267b5ce352c60181a9a75a659c21e4f1c821f1` |
| Runtime rest-recovery timer | `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:5400-5425` | `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| Adapter recovery timer | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:1220-1250,1340-1355` | `00588a8a5eeac921651f5a00153e3388557d5cdc` |
| PX4 library `scheduleMode` and completion callback | `src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/components/mode_executor.cpp:225-260,484-519` at gitlink `4a3370f084ac6f1ef001a4afa2b007845ffd0837` | `d8f23a6ccfb25d6f5943140cc44b1a85be44598d` |

## Prior-artifact provenance and TARGET revalidation

- AS-IS artifact `f2bd3f46f9936d622377ea4761f733f988273b66`: local dirty snapshot at recorded HEAD `9534d8dc`; used as hypothesis only. Its own report was `PARTIAL_AS_IS`.
- H0-H7 artifact `f2ed429bef80b2c7a2964b3b32d3c00e8089e55f`: previous local counterexamples and source scope, not TARGET product-path proof.
- State-contract artifact `6ff508217778aa42e60e208d4dd0a49785a4fae6`: 56-row selected contract inventory and delta; not whole-product coverage. Its A→TARGET delta already identified stale H5 grid and desired/active identity change; rechecked here on TARGET paths.

| Prior claim | TARGET assessment | Evidence limit |
|---|---|---|
| H0 profile | CONFIRMED_WITH_SCOPE | TARGET loader still sets suppression from `use_sim_time` and tracking gate; no live TARGET parameter dump. |
| H1 PASS_THROUGH | CONDITIONAL | TARGET mission code still overwrites previous sample and requires same-update witness; prior local helper counterexample is not full producer-path reachability. |
| H2 world suspension | CONFIRMED_WITH_SCOPE | TARGET source suspends publication and exact-resume path; actual PX4 lease expiry not observed. |
| H3 publish lock | CONFIRMED_WITH_SCOPE | TARGET still nests localization/input/command and Store publication; no workload bottleneck distribution. |
| H4 recovery timers | SPECIFICATION_GAP | TARGET runtime steady failure timer and adapter ROS deadline still differ; no cross-process equivalence trace. |
| H5 fixed 16-speed grid | REFUTED on TARGET | TARGET governor changed; no performance/completeness proof. |
| H6 Hold order | CONDITIONAL | TARGET adapter separates `scheduleMode` completion from VehicleStatus; pinned library shows the callback can be a ModeCompleted result, not merely command ACK. Ordering was not run. |
| H7 bottleneck ranking | UNRESOLVED | No matched target workload trace. |

`FACT_FROM_EXISTING_TEST` means test source or earlier run only. This audit executed only its independent abstract model and structural scripts; no TARGET product binary, ROS pair, SITL or hardware run. Therefore all target runtime behavior and performance claims are `RUNTIME_UNVERIFIED`.

# H0 — profile/config matrix

## Baseline and effective-value limit

Source scope A is the frozen source_snapshot from artifact commit f2bd3f46f9936d622377ea4761f733f988273b66. Checkout B at audit start was HEAD 9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2 on codex/close-proven-findings; all 596 manifest paths matched A byte-for-byte then. B had 34 pre-existing status entries, no staged diff, and 1,778,436 bytes of unstaged diff before this audit output. Checkout C later moved to 8eaab3d33db36e9e636ad4005ed91b8f055f4d68; 15 paths differ from A. This matrix describes A only. See baseline/repo_state.json and validation/final_checkout_drift.json.

There is no runtime parameter dump, active launch command, mission-file identity, active RMW/domain capture, or binary-to-run provenance. Effective live profile is unknown. Checked-in YAML and launch defaults below are source facts, not a claim about a deployed run.

| Value / control | Declared source | Loader / override | Consumer and effective known value | Live status |
|---|---|---|---|---|
| use_sim_time | navigation_runtime.launch.py defaults false; avoidance_mission.launch.py forwards launch substitution to runtime and external-mode processes. | Each ROS node reads its own parameter. | Runtime and PX4 adapter independently derive enabled=simulated; odometry bridge separately enforces clock-domain compatibility. | Unknown per process. |
| Tracking coefficients | config/runtime/mapping.yaml: base_m=0, lateral_alpha_s=0, longitudinal_beta_s=0; loader defaults all zero. | Coefficients finite and nonnegative. trackingGateEnabled is true if any is positive. | sim=false: experiment disabled, suppression false. sim=true plus all-zero coefficients: enabled and both suppress flags true. Any positive valid coefficient enables adaptive tracking and disables both suppress flags. | Source truth table tested with production loader fixture; not an operational profile. |
| Velocity-only tracking | Loader defaults disabled; parameters default zero. | Enabling while sim=false throws. Enabled limits must be finite and positive; transport and PX4-consume bounds may be zero. | Runtime and PX4 adapter instantiate their own policy objects; same values are not guaranteed unless both receive the same YAML/override. | Unknown per process. |
| deployment_profile | mapping.yaml declares sitl; runtime defaults sitl and accepts sitl/hardware, but hardware hard-fails because this baseline lacks the required visibility-certificate verifier. | Launch config can override. | Runtime candidate authorization. PX4 adapter does not inherit this node parameter automatically. | Unknown live. |
| Frames | mapping.yaml declares lio_odom/base_link; sim.yaml declares clock_domain=simulation_time. | Launch can select another config. Bridge validates simulation_time iff sim time true; otherwise ros_time or system_time. | Runtime frame conversion, state epoch/frame validity and PX4 frame alignment. | Unknown live config and stream. |
| Dynamic/unknown policy | src/runtime/navigation_runtime/config/planner.yaml separates nominal MAIN intent from physical BACKUP limits and explicit unknown-space policy. | Planner loader validates values, limits and policy. | Planner builds guide/corridor and stopping evidence; final world validator independently checks candidate under its certificate policy. | No run-effective config/mission identified. |
| Timing | PlanningTimingContract: planner period 0.10 s, solve deadline 0.08 s, command period 0.02 s, minimum MAIN reserve 0.62 s; runtime freshness 0.5 s and command stream timeout 0.10 s. PX4 defaults: command age 0.10 s, state age 0.20 s, recovery wait 5 s. | Runtime timing inputs are positive-validated. PX4 independently validates timing and recovery not greater than initial limit. | Solve deadline uses ROS simulation deadline and steady deadline. PX4 freshness uses ROS header age and steady receive age; handover retry uses steady time. | Source defaults/config known; live values unknown. |
| QoS/executor | Command writer/adapter reader use reliable KeepLast(1); propagated odometry BestEffort KeepLast(1); health BestEffort KeepLast(10). Runtime uses two-thread MultiThreadedExecutor and distinct callback groups. | Build/launch chooses ROS distribution, RMW and process topology. | QoS is transport policy, not PX4 acceptance or callback total order. | Current shell had no sourced ROS overlay or RMW_IMPLEMENTATION; no process inspection. |
| Build/install | Existing build/compile_commands.json has 57 entries, all PX4 interface-library scope, SHA 32e3692ca5c79a3cab76266b502fe9996142e1494b39fdee6506a7e5e40fc2c1; build/CMakeCache.txt absent. Existing install/setup.bash was not sourced for claims. | Audit tests use fresh Release colcon build rooted at A under this output, ROS Jazzy, GCC 13.3. | Audit binaries are not claimed as active installation or target workload binary. | Live binary unknown. |

## Loader truth table

| sim clock | tracking coefficients | velocity-only | Loader result | suppress braking / health response |
|---|---|---|---|---|
| false | all zero | false | valid; experiment disabled | false / false |
| true | all zero | false | valid; experiment enabled | true / true |
| true | any positive valid coefficient | false | valid; adaptive tracking gate enabled | false / false |
| false | any valid coefficient | false | valid; experiment disabled | false / false |
| false | any | true | rejected by loader | no policy returned |
| true | all zero | true, all required velocity parameters valid | valid | true / true; velocity-only does not clear suppression |
| true | invalid/non-finite/negative coefficient or malformed enabled velocity limits | any | rejected by valid() | no policy returned |

Narrowing: the sim=true plus zero-coefficients mapping is a real policy difference in A. It does not disable typed health receipt, health freshness, epoch matching, state age, frame conversion/alignment, command/bundle leases, identity checks, world recertification or physical recovery certification. PX4 suppress_braking only admits its guarded adaptive-assessment path after identity, freshness, frame and geometric-support guards. suppress_estimator_health_response does not make absent/stale typed health or epoch mismatch fresh.

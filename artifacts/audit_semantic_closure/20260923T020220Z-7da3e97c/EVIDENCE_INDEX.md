# Evidence and provenance index

## Pins and drift

| Item | Exact identity / observation |
|---|---|
| product TARGET | `7da3e97cb399c2e39d62cfe60213a45e8a92300e` |
| parent audit | `02ed671a47cb701e02f554a7d2949f9892ec704c`; verdict PARTIAL, not an approved architecture |
| new branch | `codex/audit-semantic-closure-20260923` from parent audit commit, separate worktree |
| original product checkout at inspection | `7da3e97cb399c2e39d62cfe60213a45e8a92300e`; no observed drift |
| pinned dependency | `src/external/px4_ros2_interface_lib` gitlink and original-checkout submodule HEAD `4a3370f084ac6f1ef001a4afa2b007845ffd0837`; `mode_executor.cpp` blob `d8f23a6ccfb25d6f5943140cc44b1a85be44598d` |
| environmental PX4 checkout | `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty elsewhere; **not product-pinned firmware**. Inspected clean-at-HEAD blobs `loiter.cpp` `86efa42ee9ded18b5400f680e3243b2a60d37ec2`, `navigator_main.cpp` `69ca9896d4b2235f55f3c81f461b9c72d6bfac70` |

No product source/config or safety-document set was edited. Product diff from TARGET under `src/` and `docs/safety/` is empty. If the original checkout advances, this artifact remains pinned to TARGET and any new target needs a separate artifact.

## Source ledger

| Label | Claim and exact source | TARGET blob |
|---|---|---|
| FACT_FROM_TARGET_CODE | PASS same-update acceptance and previous callback-time sample: `src/px4/px4_navigation_external_mode/src/mission_controller.cpp:367-613` | `eb576715484edd5db156417b284806afd91df7ac` |
| FACT_FROM_TARGET_CODE | 3-D route projection/order and crossing geometry: `src/contracts/navigation_mission/src/route_progress.cpp:254-345,430-490` | `9f3c9f41446270f3beaabeb3e43e886b7d0a5fa1` |
| FACT_FROM_TARGET_CODE | adapter odometry, continuation, suffix, command arrival: `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:1328-1464,1670-1717` | `00588a8a5eeac921651f5a00153e3388557d5cdc` |
| FACT_FROM_TARGET_CODE | runtime one-way recovery: `src/runtime/navigation_runtime/include/navigation_runtime/execution_recovery_state.hpp:28-88`; Episode phase/policy and commit: `execution_episode.hpp:49-230` | `5a21f1aa9d3096ad380aac09b27187fbaa09eded`, `9e1d62d9fcafef4f9cf56775ee263ef3cef908dc` |
| FACT_FROM_TARGET_CODE | adapter recoverable Braking: `mission_controller.cpp:95-240,374-445`; runtime stop: `navigation_runtime_node.cpp:3792-4050,8180-8235` | `eb576...`, `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| FACT_FROM_TARGET_CODE | repository product call graph: `navigation_mode_node.cpp:1272-1287` invokes native readiness methods; `rg 'onTrajectory\\(' src` finds definitions and test calls but no product invocation. Native methods `mission_controller.cpp:247-285` do not enter `Braking` | `00588...`, `eb576...` |
| FACT_FROM_TARGET_CODE | Hold pending/retry/status/callback: `navigation_mode_node.cpp:2846-2937` | `00588...` |
| FACT_FROM_PINNED_DEPENDENCY | ACK/schedule/completion/deactivation: `px4_ros2_cpp/src/components/mode_executor.cpp:108-117,125-260,361-419,484-519` | pinned blob `d8f23a6ccfb25d6f5943140cc44b1a85be44598d` |
| FACT_FROM_TARGET_CODE | world publication/recertification and suspension: `navigation_runtime_node.cpp:940-1305,7696-7720`; command `valid_until`: `:8497-8505` | `0d3ebe7253b3e3cfb0877ef0265ab734e3af9b9e` |
| FACT_FROM_TARGET_CODE | periods/deadlines: `src/planning/navigation_planning/include/navigation_planning/planning_timing.hpp:9-24`; adapter lease/state age: `navigation_mode_node.cpp:118-155,2639-2659` | `58af62247c0585edadd19b8b440f58e5b08344f4`, `00588...` |
| FACT_FROM_TARGET_CODE | safety contract HG-017/023/031/035 and latest-world/retained command policy: `docs/safety/runtime_safety_current.md:40-61,95-125`; targeted lineage `runtime_safety_index.md:37,43,51,55,77,145,610`, archive `runtime_safety_legacy_full.md:321-339,2497-2514,17414-17432` | TARGET checkout |
| FACT_FROM_EXISTING_TEST | PASS and Braking unit test source: `src/px4/px4_navigation_external_mode/test/test_mission.cpp:290-~800,1301-1547`; planner timing test `src/runtime/navigation_runtime/test/test_planner_fsm.cpp:193-201` | `519e959b489db9de52dced61feb68ad95fc54ca1` for mission test |
| FACT_FROM_RUNTIME_TRACE | none produced or cited as qualification in this audit | n/a |
| INFERENCE | t0/t1 crossing loss, candidate fact ownership, D_f/W_t target expectations and event models | explicitly conditional |
| SPECIFICATION_GAP | A retention/reachability; B policy/cross-layer callback; C firmware/status witness; D timing tails/expiry action | see blocker docs |

## Validation scope

`python3 tools/cluster_inventory.py` (from artifact root) maps the prior 239 rows. `python3 -m unittest discover -s tests -p '*_model.py' -v` runs only audit abstract models. `dot -Tsvg` renders each diagram. These establish file/model consistency, **not** component equivalence, SITL observation or flight qualification. No safety threshold was changed, so the safety-ledger validator is not triggered by this branch; `git diff --check` remains required for artifact hygiene.

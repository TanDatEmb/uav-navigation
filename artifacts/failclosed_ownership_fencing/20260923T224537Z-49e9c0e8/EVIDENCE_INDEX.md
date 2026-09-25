# Evidence index

| Claim | Source/evidence |
|---|---|
| Base/submodules/PX4 provenance | `BASE_PROVENANCE.md`; pinned `git rev-parse` and external binary hash |
| All destructive sites before mutation | `DESTRUCTIVE_TRANSITION_AUDIT.md`, `FAILCLOSED_SITE_CLASSIFICATION.csv` |
| HG-023 incumbent retention policy | `docs/safety/runtime_safety_current.md`; targeted archive `docs/safety/archive/runtime_safety_legacy_full.md:3384` |
| Planner failure classification and exact solve owner | `planner_fsm.hpp::classifyPlannerResult`; `navigation_runtime_node.cpp::runCycle`; `ExecutionAuthority::failClosedIfCurrentSnapshot` |
| Watchdog solve-generation/owner fence | `runtime_boundaries.hpp::PlannerSolveFailureWitness` and `PlannerSolveActivityScope`; `navigation_runtime_node.cpp::publishCommand` |
| State L1→L2 supersession | `ExecutionStateStore::publish/load`; `onPropagatedOdometry()` producer shares `localization_transition_mutex_` with destructive boundary; `runtime_boundaries.hpp::failedExecutionLeaseIsCurrent`; barrier tests in `test_planner_fsm.cpp` and actual `NavigationRuntimeNode::publishCommand()` tests in `test_navigation_runtime_terminal_monitor.cpp` |
| Pending cleanup | `PendingGoalHandoffOwner::clearIfCurrent`; barrier test in `test_planner_fsm.cpp` |
| World conditional suspension/publication | `ExecutionAuthority::suspendIfCurrentSnapshot`; `WorldSnapshotStore::publishAndFinalizeDecision` and runtime world finalizer |
| Stale E1→E2 owner mutation | barrier test in `test_execution_authority.cpp` |
| Static source guard | `tools/check_failclosed_ownership_fencing.py` |
| Clean Release manifest and build/test | `TEST_EVIDENCE.md`; `install/.uav_navigation_build_manifest.json` pins source `0ac3edc4` with `git_dirty=false` |
| Three nominal SITL sessions and exact adapter gaps | `SITL_RESULTS.md`, `HANDOFF_MEASUREMENTS.csv`; session suffixes `233224-587903`, `233507-591132`, `233732-594439`; each retained `report.json`, `metadata.json`, `scenario.jsonl`, bag and logs under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/` |
| Repeated failure/BACKUP, Core pause, non-arming hook attempts | `SITL_RESULTS.md`; suffixes `234222-600949`, `234519-604253`, `234010-597697`, `234724-607506` respectively; expected fault observations are not nominal qualification |
| Performance limits | `PERFORMANCE.md`; bag analyzer `artifacts/execution_authority_unification/20260923T123424Z-8ffa14e5/analyze_sitl.py` |

Evidence classes remain separate: source proof, deterministic component/model test, focused SITL observation and flight qualification. No focused test or SITL run in this branch is flight qualification.

# Evidence index

| Claim | Source/evidence |
|---|---|
| Base/submodules/PX4 provenance | `BASE_PROVENANCE.md`; pinned `git rev-parse` and external binary hash |
| All destructive sites before mutation | `DESTRUCTIVE_TRANSITION_AUDIT.md`, `FAILCLOSED_SITE_CLASSIFICATION.csv` |
| HG-023 incumbent retention policy | `docs/safety/runtime_safety_current.md`; targeted archive `docs/safety/archive/runtime_safety_legacy_full.md:3384` |
| Planner failure classification and exact solve owner | `planner_fsm.hpp::classifyPlannerResult`; `navigation_runtime_node.cpp::runCycle`; `ExecutionAuthority::failClosedIfCurrentSnapshot` |
| Watchdog solve-generation/owner fence | `runtime_boundaries.hpp::PlannerSolveFailureWitness` and `PlannerSolveActivityScope`; `navigation_runtime_node.cpp::publishCommand` |
| State L1→L2 supersession | `ExecutionStateStore::publish/load`; `runtime_boundaries.hpp::failedExecutionLeaseIsCurrent`; barrier test in `test_planner_fsm.cpp` |
| Pending cleanup | `PendingGoalHandoffOwner::clearIfCurrent`; barrier test in `test_planner_fsm.cpp` |
| World conditional suspension/publication | `ExecutionAuthority::suspendIfCurrentSnapshot`; `WorldSnapshotStore::publishAndFinalizeDecision` and runtime world finalizer |
| Stale E1→E2 owner mutation | barrier test in `test_execution_authority.cpp` |
| Static source guard | `tools/check_failclosed_ownership_fencing.py` |
| Build/test/SITL limits | `TEST_EVIDENCE.md`, `SITL_RESULTS.md`, `PERFORMANCE.md` |

Evidence classes remain separate: source proof, deterministic component/model test, focused SITL observation and flight qualification. No focused test or SITL run in this branch is flight qualification.

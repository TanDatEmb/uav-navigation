# Evidence index

| Claim | Evidence | Level |
|---|---|---|
| Exact repaired base, tree, submodules and PX4 binary | `BASE_PROVENANCE.md` | Git and local binary provenance |
| Initial owners and field dispositions | `AS_IS_EXECUTION_OWNERS.md`, `STATE_CONSERVATION.md`, `IDENTITY_PREDICATE_AUDIT.md`, `EXECUTION_STATE_REACHABILITY.md` | Pinned source review |
| One owner, active/staged full payload and typed lifecycle | `src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp`, `TARGET_EXECUTION_OWNER.md`, `OWNER_MIGRATION_CHECKPOINT.md` | Source structure; tests below |
| No RuntimeNode execution identity mirrors | `navigation_runtime_node.hpp/.cpp`, `tools/check_execution_authority_cut.py`, `STATE_DIFF.md` | Source and static guard |
| Mission writer and PX4 boundary remain separate | `tools/check_mission_authority_cut.py`, `SAFETY_INVARIANTS.md` | Source/static, not flight qualification |
| Atomic activation, world recertification, rollback, publication races | `test_committed_bundle_store.cpp`, `test_mission_progress.cpp`, `test_navigation_runtime_terminal_monitor.cpp`, `test_same_identity_renewal_injection.cpp`, `TEST_EVIDENCE.md` | Deterministic component tests |
| Repaired behavioral reference | `artifacts/repair_core_mission_liveness/20260923T102222Z-db5880a0/SITL_RESULTS.md`; exact repair sessions `112040-309344`, `112251-312566`, `112456-315691` | Pinned repair diagnostic SITL, 3/3 matched |
| This cut's nominal mission and handoff parity | `SITL_RESULTS.md`; sessions `134651-412471`, `134910-415618`, `135131-418913`; `HANDOFF_MEASUREMENTS.csv`, `analyze_sitl.py` | Clean-HEAD diagnostic SITL, 3/3; exact adapter admission gap; no flight qualification |
| Heartbeat loss and PX4 Hold boundary | `SITL_RESULTS.md`; session `135502-422343`; `heartbeat_fault_marker.txt`; adapter log stale-PVA/Hold and report | Focused process-pause SITL; mapping cloud loss concurrent, not isolated world freshness |
| BACKUP safety ownership, analytic hold and measured restart | `SITL_RESULTS.md`; session `140534-436416`, 28 structured injected-failure records, command role/recovery and propagated odometry | Focused diagnostic SITL; no emergency-path observation |
| Pillar adversarial control | Sessions `135916-426322`, `140103-429559` (cut) and `140324-433062` (repair); historical repair `111139-302607` | Matched focused SITL, early no-suffix stop reproduced on repair; isolated new-world invalidation not re-observed |
| Publication, owner lock and scheduling timing | `PERFORMANCE.md`, `HANDOFF_MEASUREMENTS.csv`, `analyze_sitl.py` | Matched diagnostic distributions with measurement limits |

The product source is `8ffa14e5` plus this branch's incremental commits. Audit documents are not substituted for source or runtime evidence. No threshold or safety-document set was edited. Nominal runner-level `FAIL` is the versioned evaluation qualification result; mission completion is a separate witnessed event. The isolated world-revision and emergency SITL gaps are identified in `SITL_RESULTS.md` and `REMAINING_DEBT.md`.

# Evidence index

| Claim | Evidence | Level |
|---|---|---|
| Exact repaired base, tree, submodules and PX4 binary | `BASE_PROVENANCE.md` | Git and local binary provenance |
| Initial owners and field dispositions | `AS_IS_EXECUTION_OWNERS.md`, `STATE_CONSERVATION.md`, `IDENTITY_PREDICATE_AUDIT.md`, `EXECUTION_STATE_REACHABILITY.md` | Pinned source review |
| One owner, active/staged full payload and typed lifecycle | `src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp`, `TARGET_EXECUTION_OWNER.md`, `OWNER_MIGRATION_CHECKPOINT.md` | Source structure; tests below |
| No RuntimeNode execution identity mirrors | `navigation_runtime_node.hpp/.cpp`, `tools/check_execution_authority_cut.py`, `STATE_DIFF.md` | Source and static guard |
| Mission writer and PX4 boundary remain separate | `tools/check_mission_authority_cut.py`, `SAFETY_INVARIANTS.md` | Source/static, not flight qualification |
| Atomic activation, world recertification, rollback, publication races | `test_committed_bundle_store.cpp`, `test_mission_progress.cpp`, `test_navigation_runtime_terminal_monitor.cpp`, `test_same_identity_renewal_injection.cpp`, `TEST_EVIDENCE.md` | Deterministic component tests |
| Repaired behavioral reference | `artifacts/repair_core_mission_liveness/20260923T102222Z-db5880a0/SITL_RESULTS.md` | Pinned repair diagnostic SITL, 3/3 matched |
| This cut's runtime behavior and timing | `SITL_RESULTS.md`, `PERFORMANCE.md` | Focused SITL after committed Release build; no flight qualification |

The product source is `8ffa14e5` plus this branch's incremental commits. Audit documents are not substituted for source or runtime evidence. No threshold or safety-document set was edited.

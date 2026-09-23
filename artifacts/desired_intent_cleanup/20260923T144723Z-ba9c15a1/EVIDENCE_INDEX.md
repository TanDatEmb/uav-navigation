# Evidence index

| Artifact | Evidence role |
|---|---|
| `BASE_PROVENANCE.md` | Exact Git/submodule/PX4 build identity before edits. |
| `AS_IS_DESIRED_STATE.md` | Pinned pre-cut desired-state ownership and field inventory. |
| `PLANNING_TRANSITION_REACHABILITY.md` | Source-derived flag-combination proof. |
| `ADMISSION_CONTEXT_SEMANTICS.md` | Desired producer revision versus execution admission fence. |
| `IDENTITY_COMPARISON_AUDIT_V2.md` | Pinned helper and manual-compare inventory, including conditional fail-close paths. |
| `TARGET_DESIRED_INTENT.md` | Target state and lifecycle contract. |
| `STATE_DIFF.md`, `VOCABULARY_DIFF.md` | Final state and lexical before/after metrics. |
| `TEST_EVIDENCE.md` | Build, CTest, Python, static, and safety-ledger evidence. |
| `SITL_RESULTS.md`, `PERFORMANCE.md` | Focused matched runtime and handoff timing evidence; separate from qualification. |
| `REMAINING_DEBT.md` | Independent boundary evidence debts. |
| `VERDICT.md` | Final gate and delivery verdict once runtime evidence is complete. |

Primary product source for this cut is `src/runtime/navigation_runtime/include/navigation_runtime/desired_planning_intent.hpp`, `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`, and `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp`. The pinned reference is `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`; the first source/test checkpoint is `c9006768`. Test code and scripts are listed in `TEST_EVIDENCE.md`.

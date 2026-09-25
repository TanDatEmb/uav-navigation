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
| `SITL_RESULTS.md`, `PERFORMANCE.md`, `HANDOFF_MEASUREMENTS.csv` | Clean-manifest diagnostic SITL: three matched completions, exact 12 adapter admission gaps, Core-pause Hold fence, BACKUP restart, and measured terminal STOP; separate from qualification. |
| `REMAINING_DEBT.md` | Independent boundary evidence debts. |
| `VERDICT.md` | Architecture and focused runtime parity verdict, with explicit qualification limit. |

Primary product source for this cut is `src/runtime/navigation_runtime/include/navigation_runtime/desired_planning_intent.hpp`, `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`, and `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp`. The pinned reference is `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`; the first source/test checkpoint is `c9006768`. Test code and scripts are listed in `TEST_EVIDENCE.md`.

All new-cut SITL sessions pin clean source commit `473b2121817b2ba248c0c6eb40f9cfed89a3c2d3`, fingerprint `8ffce50ad2bd3adefcc0e056e1f990b5589b7369eb23a271e25ce6d0fedfacef`. Session suffixes `151450-509084`, `151832-512307`, `152036-515456`, `152307-518741`, and `152851-525435` are under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260923T`. The excluded no-injection setup check is `152624-522113`. Raw `report.json`, `metadata.json`, `scenario.jsonl`, bags and logs remain in each session. Bag extraction uses the pinned prior `artifacts/execution_authority_unification/20260923T123424Z-8ffa14e5/analyze_sitl.py` method; the CSV records the derived handoff values.

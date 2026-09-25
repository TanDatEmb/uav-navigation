# Evidence index

- `BASE_PROVENANCE.md`: exact candidate base/tree, submodules, PX4 checkout/binary, clean initial repository worktree.
- `QUALIFICATION_SCOPE_OUTPUT_CONTRACT.md`, `QUALIFICATION_SCOPE_RECONCILIATION.md`: report/evaluator scope separation and Q1–Q4.
- `WORLD_SESSION_REEVALUATION.md`: immutable raw session re-evaluation.
- `N1_LIDAR_CLASSIFICATION.md`, `N2_TERMINAL_RECOVERY_CLASSIFICATION.md`: causal classifications and boundaries.
- `CURRENT_SHA_OPTIMIZATION_FAILED.md`: current-source injection witness; raw `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T144137-138251`.
- `CORE_PAUSE_REGRESSION.md`: current Core pause -> adapter lease expiry -> Hold; raw `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T150807-150443`.
- `NOMINAL_5RUN_COHORT.md`, `PERFORMANCE.md`: 5 valid runs and distributions. Raw runs under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/` with IDs in cohort table.
- `BACKUP_EMERGENCY_REGRESSION.md`, `WORLD_REGRESSION.md`: scope and deterministic regression coverage.
- `PRE_MAIN_TEST_MATRIX.md`, `AUTHORITATIVE_TEST_GATE.md`, `TEST_EVIDENCE.md`: test tiers, authoritative command and results.
- `P0_P1_REGISTER.md`, `PRE_BETA_DEBT.md`: known debt and classifications.
- `MAIN_INTEGRATION_DIFF.md`, `MAIN_BEHAVIOR_CHANGE_INVENTORY.md`, `MAIN_KNOWN_LIMITATIONS.md`, `MAIN_INTEGRATION_READINESS.md`: PR review packet.
- `VERDICT.md`: current gate state. `RAW_SESSION_MANIFEST.csv` records absolute paths, byte sizes, and SHA256 for metadata, reports, scenario traces, injection analysis, and rosbag DB3 files of incoming World/N sessions, all five valid cohort runs, invalid pre-launch attempt, exploratory run, exact injection, and Core-pause fault. Raw sessions remain external to Git and were not modified.


Raw manifest format: one row per retained file with absolute path, exact size in bytes, and SHA256. Rosbags are retained outside Git.
# PR #2 final integration closure addendum

- `PR_REVIEW_FIX.md`: P1 description and repair status.
- `SCOPE_GUARD_MIGRATION.md`: explicit historical baseline use and branch-scope guard tests.
- `CI_CONTRACT.md`: hosted static/Python jobs and controlled ROS build limitation.
- `GOVERNANCE.md`: single-maintainer review process and unprotected-main status.
- `LATEST_CODE_REVIEW.md`: current review target and unresolved-thread state.
- `MAIN_MERGE_RECEIPT.md`: merge not performed.
- `FRESH_MAIN_VALIDATION.md`: post-merge validation pending.
- `tools/runtime/tests/test_branch_scope_guards.py`: executable Git-fixture regression tests.
- `.github/workflows/ci.yml`: canonical static-contract and Python checks.

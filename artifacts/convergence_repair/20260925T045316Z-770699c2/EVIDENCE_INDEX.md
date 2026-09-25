# Evidence index

- Source commits: `c1464a9` (C1 physical duration boundary), `c464f67` (C1 support representation edge), `6828953` (C2 request-owned witness).
- C1 implementation: `src/planning/navigation_planning_backend/include/planner_core/backup_braking.hpp`; support check: `.../evidence_speed_governor.hpp`; tests: `src/planning/navigation_planning_backend/test/test_planner_config.cpp`.
- C2 contract: `src/planning/navigation_planning/include/navigation_planning/planning_request.hpp`; producer: `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`; consumer: `src/planning/navigation_planning_backend/src/planner_core/planner.cpp`; tests: `src/planning/navigation_planning/test/test_planning_contracts.cpp`.
- Prior convergence inventory: `artifacts/world_runtime_evidence_closure/20260925T020509Z-bbbf34ab/PARALLEL_BRANCH_CONVERGENCE.csv`.
- Exact-base raw sessions: `.artifacts/convergence-repair-20260925/baseline-runtime/`; file sizes and SHA256s are in `RAW_SHA256_MANIFEST.json`.
- Repair raw sessions: `.artifacts/convergence-repair-20260925/runtime/`; session folders include reports, metadata, logs and available raw data; all 405 files are listed with SHA256/size in `RAW_SHA256_MANIFEST.json`.
- PX4 binary SHA256 and build-manifest SHA are in `BASE_PROVENANCE.md` and `TEST_EVIDENCE.md`.
- CTest/Python/static/ledger results: `FOCUSED_TESTS.md`, `TEST_EVIDENCE.md`.
- This evidence supports a focused canonical repair assessment only; it is not flight qualification.

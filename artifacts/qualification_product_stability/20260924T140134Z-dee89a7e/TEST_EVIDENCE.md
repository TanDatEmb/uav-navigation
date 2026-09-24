# Build and deterministic test evidence

- Product C++ source and configuration have **zero changes** from base `dee89a7e`; `git diff dee89a7e -- src config docs/safety` is empty. Added code is diagnostic-only Python and offline analysis.
- First clean Release build compiled **23 packages** but correctly withheld its authoritative manifest because audit source changed during the build. After the first two commits stabilized the source, a second full Release build completed **23/23 packages** in 27.0 s and wrote `install/.uav_navigation_build_manifest.json` for `aee446c5018675a356defabbdd1551623f8f0efb`.
- Runtime Python suite: **416 passed, 1 skipped** after install was complete. The earlier pre-install run failed only the test requiring installed `navigation_runtime/config/planner.yaml`; it passed after build.
- First CTest invocation overlapped the Python suite. `PlannerFacade.CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary` exceeded its real solve deadline and failed at 82 ms; the exact test passed alone at 67 ms. The full CTest run repeated without competing Python work: **90 test cases, 0 errors, 0 failures, 0 skipped** across 14 tested packages. No deadline/expectation was relaxed.
- `check_mission_authority_cut.py`, `check_execution_authority_cut.py`, `check_desired_intent_cut.py`, `check_failclosed_ownership_fencing.py`, `check_exact_optimization_injection.py`: PASS.
- `validate_runtime_safety_ledger.py`: PASS. No safety-document or threshold change was made. `git diff --check`: PASS at the pre-SITL checkpoint.
- Temporal classifier tests: 9 pass, including native clock/stats slowdown with nonzero source progress and conservative `UNRESOLVED` when the deficit is below the diagnostic trigger. Native observer steady-clock-step test passes.

Final HEAD must be rebuilt and retested after the diagnostic classifier and evidence updates; the earlier manifest cannot be represented as matching final source.

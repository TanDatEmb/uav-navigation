# Test evidence

Environment: Release package builds in the new worktree, using dependency installations from the exact base repair worktree as an underlay. No threshold, planner objective, safety policy, or PX4 behavior was changed.

## Build

- `navigation_planning_backend`: Release package build passed.
- `navigation_runtime`: Release package build passed.
- Both package build trees were rebuilt successfully in full after the source commit, including all package targets and test binaries.
- This was a selected package build, not the canonical full-product Release build/manifest.

## CTest

- Fresh package CTest after the source commit: `navigation_planning_backend` 9/9 and `navigation_runtime` 19/19 passed.
- The focused C2 tests `RuntimeRequestHistoryCapturesExactAdmittedEmergencyPredecessor` and `AdmittedEmergencyWitnessAuthorizesRealStopCorrectionRequest` passed; terminal-monitor CTest passed after the final gtest-property additions.
- Focused runtime tests: `RuntimeRequestHistoryCapturesExactAdmittedEmergencyPredecessor` and `AdmittedEmergencyWitnessAuthorizesRealStopCorrectionRequest` passed.
- Existing emergency transition control `EmergencyExecutionCutoverPrecedesAckAndWorldCopy` passed before the final test-only property addition.

## Python and guards

- `tools/tests`: 7 passed.
- `tools/runtime/tests`: 444 passed, 1 skipped.
- Mission authority, execution authority, desired intent, and fail-closed ownership guards: PASS.
- Runtime safety ledger validator: PASS (500 current-contract lines, 706 decisions, 36 gates / 34 active, 5 bypass records / 1 active).
- `git diff --check`: PASS at the validation checkpoint; rerun after final documentation edits.

## Limits

- No SITL was run for this C2 closure; the direct component test exercises actual runtime emergency ownership through actual planner candidate authorization without Gazebo/PX4.
- No full workspace Release build was produced in this isolated worktree. The source and behavior changes are confined to planning authorization/request provenance and its tests; exact package Release builds and package CTest suites passed.
- This evidence does not qualify PX4 consumption, flight tracking, or C0-IFP.

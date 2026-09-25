# Focused verification

## Build and deterministic tests

- Full Release build via `PARALLEL_WORKERS=2 MAKE_JOBS=4 python3 tools/runtime/build.py --mode release build`: PASS after CMake test dependency correction. Each runnable SITL session recorded a `VALID` authoritative Release manifest in `metadata.json`.
- `python3 tools/runtime/build.py --mode release test --packages navigation_runtime`: 16/16 CTest targets PASS. Includes new `DesiredPassGateAdvanceCannotRevokeInFlightPredecessorSample`, `EndToEndHotHandoffRetainsPredecessorUntilAtomicCutover`, and `EstablishedActivationKeepsMeasuredCrossingWhenModeHeartbeatIsLate` tests.
- `python3 tools/runtime/build.py --mode release test --packages navigation_mission navigation_execution navigation_contracts px4_navigation_external_mode`: 13/13 CTest targets PASS (1+1+2+9).
- `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`: 390 tests, PASS, one skip.
- `python3 tools/check_mission_authority_cut.py`: `MISSION_AUTHORITY_STATIC_CHECK: PASS`.
- `python3 tools/validate_runtime_safety_ledger.py`: PASS, current contract 500 lines/706 decisions/36 gates/5 bypasses.
- `git diff --check`: PASS.

## L1–L10 seam coverage

| Case | Evidence |
|---|---|
| L1 desired PASS accepted with predecessor still active | End-to-end MissionProgress/timeline/Episode model and live H4/H5 repair trace |
| L2 successor delayed 50/100/150 ms | End-to-end model samples retained predecessor at 50/100/150+ ms |
| L3/L4 delayed or lost adapter receipt | Model retains crossing; no acceptance without exact receipt; receipt absence does not revoke existing execution |
| L5 atomic cutover/late old sample | Model commits successor and rejects old request sample after cutover |
| L6 desired gate N+1/execution N | Pure final-publication identity test and live H5 distinct epochs |
| L7 planner successor failure | Model failed world candidate preserves predecessor while its current certificate remains valid; existing runtime fallback remains unchanged |
| L8 candidate continuity failure | End-to-end test reserves the actual predecessor execution anchor; production `candidateMatchesAnchor()` rejects synthetic P/V/A/J mismatches independently, keeps predecessor, and permits only exact matching successor cutover. No tolerance changed. Pillar separately demonstrates fail-closed when latest-world revalidation revokes active and no brake remains. |
| L9 ModeStatus heartbeat >200 ms | Real RuntimeNode component test with established activation, measured crossing and exact fresh adapter admission |
| L10 near-100 ms lease | `commandValidAt` true at exact finite deadline, false 1 ns later; adapter package lease tests remain PASS |

These tests verify deterministic component/model seams. They do not establish PX4 flight qualification or runtime tail distributions. The unmodified pinned PX4 dependency's known full-workspace lint failures are not relabeled PASS.

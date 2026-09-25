# Verdict: ADAPTER_ADMISSION_CONTRACT_REFACTORED

This verdict is for the Core→adapter admission architecture cut at product source `33ec3362`. It is **not** flight qualification. The evaluator-owned overall nominal reports remain `FAIL`/`NOT_EVALUABLE` and the pre-existing tracking-off configuration mismatch is explicit in `SITL_RESULTS.md`.

## Architecture result

- All 74 original `NavigationCommand` fields have an in-repository consumer classification: 27 retained control-wire fields (11 identity, 4 temporal, 6 control, 6 safety), 42 observer diagnostics moved, 5 provenance fields moved, 0 unresolved, 0 deleted. The schema loses 47 fields without losing any adapter decision input found in the pinned source.
- Admission now produces typed stage, reason and disposition. Contract and inclusive temporal lease assessments are pure. Session identity assessment preserves the old health/activation/mission/request/world order. Dynamic odometry, sample ordering and tracking failures retain their distinct previous actions. The old string-only `valid=0` path is gone.
- `NavigationCommandAdmission` remains a success receipt issued after exact adapter commit. `NavigationCommandRejection` is observer-only, and the adapter has no diagnostic-topic subscription. Core publishes observer diagnostics best effort outside execution authorization; mission issuance uses its Core-local witness.
- `onNavigationCommand()` fell from 351 to 211 source lines, with a single typed assessment flowing through the callback. The existing adapter lock, execution/mission/world owners, 100/200/500 ms gates, UNKNOWN policy, planner algorithm and PX4 Hold policy remain unchanged.

## Validation result

- Authoritative Release build: 23 packages. Full CTest: 89/89. Runtime Python: 393 passed, 1 skipped. Static guards, safety ledger validator and `git diff --check`: pass.
- Matched nominal mission observation: 3/3 `long_featured`, seed 0, runner tracking argument `off`, mission `COMPLETE`, accepted `[0,1,2,3,4]`, no unexpected Hold or lease/identity/continuity failure. Twelve adapter handoffs: min/median/max 19.848/19.988/20.266 ms.
- N3's four pre-admission rejects have exact `TEMPORAL_LEASE / NOT_YET_VALID / REJECT_RETAIN_PREVIOUS` evidence. This identifies the new run's cause while retaining the previous command; it does not retroactively identify the two old `valid=0` occurrences.
- A 350.190 ms controlled Core pause produced stale-PVA detection, Hold request and Hold observation. Repeated `kFailed` injection exercised BACKUP→measured low-speed restart and mission completion. One exact `kOptimizationFailed` injection exercised the real classifier/HG-023 retained-command path, nine further predecessor admissions, successor admission and mission completion.
- Live CDR command size fell 680→320 B. The observer stream adds 520 B/sample, so combined serialized payload rises if both topics publish at 50 Hz. Callback p50/p95/p99/max and actual DDS bandwidth are unmeasured; no performance improvement is claimed from the size reduction alone.

## Evidence boundaries

The baseline and new SITL sessions both show `tracking_experiment_mode="off"` in metadata while the pinned adapter logs `TRACKING_EXPERIMENT_BYPASS` from zero coefficient parameters under `use_sim_time=true`. The evaluator sets `config_mismatch=true`, `qualification_eligible=false`. The architecture/refactor parity observation is bounded to the **same existing harness behavior**; unsuppressed tracking-off safety parity is **not established**. Fixing that harness/product policy is separate work and was not smuggled into this cut.

The old 74-field rosbag cannot be deserialized by the new ROSIDL overlay. Frozen old-predicate unit comparisons and new-source SITL were used for state-independent equivalence; an every-sample old-wire replay through the new C++ assessment was not completed. Dynamic callback interleavings are not exhaustively proven. External ROS consumers outside this repository must migrate to the new schema before deployment.

No source or config changes were made to planner optimization, world recertification, tracking thresholds, adapter lease, emergency behavior or PX4 Hold protocol. Focused SITL is not firmware consumption proof or flight acceptance.

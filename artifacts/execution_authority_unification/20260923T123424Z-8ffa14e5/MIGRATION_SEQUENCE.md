# Reviewable migration sequence

1. Pin repaired source/tree, external binary and baseline build/tests. Read the safety contract and targeted history. Inventory all execution fields, desired-vs-active predicates, reachable lifecycle combinations and exact call-site transitions before mutation.
2. Introduce one typed execution state/snapshot and admission context in `navigation_execution`, with a single mutex. Preserve standalone timeline tests and add full-goal and lifecycle tests before RuntimeNode migration.
3. Make immediate commit and pending activation atomically set active record plus lifecycle. Keep planner-history finalizer rollback. Move all Episode mutation operations into the same owner; remove its mutable class and its mutex.
4. Move RuntimeNode `executing_goal_` and `command_goal_epoch_` into the active record. Replace publication, completion, retained recovery, renewal, world recertification and diagnostics reads with exact immutable owner snapshots/tokens. Preserve the repaired predecessor publication seam.
5. Add adversarial handoff, continuity and barrier-based race tests. Add a guard against duplicated execution owner/mirrors and against adapter mission writes. Build/test, run matched SITL and faults, record timing and state/authority metrics, then decide verdict.

Deletion checkpoints: no Episode class deletion until all lifecycle reads/writes, telemetry, recovery and tests map to the new owner; no RuntimeNode mirror deletion until publication, retained validation, completion, terminal handling, recovery, world recertification and planner renewal use the exact owner record. Intermediate commits may carry temporary adapters; final HEAD must not dual-write.

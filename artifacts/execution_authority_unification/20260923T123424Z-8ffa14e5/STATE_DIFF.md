# Execution authority state diff

Counts below name persisted semantic fields, not stack snapshots, telemetry, or the independent command-lease latch. Source: pinned `8ffa14e5` and final `ExecutionAuthority` declaration in `committed_bundle_store.hpp`.

| | Before | After |
|---|---:|---:|
| Authoritative execution persistent fields | 24 | 16 |
| Execution-owner diagnostic fields | 0 | 1 |
| Execution-internal mutexes | 2 | 1 |
| Mutable active-identity representations | 4 | 1 |
| RuntimeNode active-identity mirrors | 2 | 0 |
| RuntimeNode world-suspension mirrors | 2 | 0 |

Before: Episode's 12 fields, TimelineStore's 8 fields, RuntimeNode's `executing_goal_` and `command_goal_epoch_`, plus its two world-suspension witnesses. After: admission localization/goal epochs (2), authority version/lineage/transaction watermark (3), world identity (1), active goal/bundle (2), staged goal/bundle/activation (3), and typed lifecycle phase/recovery/exposure/safety/restart (5). This is a semantic count; records and `shared_ptr` subfields are listed explicitly so the reduction is reviewable.

Persistent state added: no new independent fact. `ActiveRecord.goal` and `StagedRecord.goal` move full immutable payloads into the same owner as their bundles; typed exposure/safety/restart replace boolean combinations. `admission_localization_epoch_` preserves the former Episode localization fence while admission goal epoch is renamed from misleading `active_goal_epoch_`.

Persistent state removed: Episode's independent state/mutex, RuntimeNode active goal/epoch mirrors, and RuntimeNode world-suspension generation/safety mirrors. The Episode's active epoch/request/generation and desired request fields are derived or omitted from the read-only telemetry view. `last_command_sampled_generation_` and similar diagnostic atomics remain observations, not authority.

Persistent state merged: active and staged full goal payloads, bundle pointers, lifecycle, world identity and version fences reside under one owner mutex. WorldModel, MissionProgress, adapter receive lease, and PX4 mode protocol remain separate.

Persistent state derived: active goal epoch/request/generation from the active bundle; suspension generation from `{exposure=Suspended, active bundle}`; safety ownership from typed lifecycle; legacy `execution_episode_*` diagnostics from the owner snapshot.

The one added diagnostic atomic records the most recent `publishIfCurrent` mutex wait in microseconds. Its sole writer is the publication attempt; it survives until the next diagnostic tick, has no authority or safety-policy reader, and deleting it would only lose lock-wait measurement. It is excluded from `D_f` and `W_t` authority counts. No product threshold depends on it.

Semantic Duplication Factor `D_f` counts independently mutable *authority representations*, excluding read-only observations. Active execution identity: `4 -> 1`; active generation: `2 -> 1`; execution exposure: `1 -> 1` (typed in the owner); execution recovery: `1 -> 1` (moved, not deleted). Desired MissionProgress and PX4 adapter each remain independent authorities for their own domains.

Execution-internal transition width `W_t`: immediate candidate commit `3 -> 1` (Timeline + Episode + RuntimeNode mirrors, now one owner); staged successor activation `3 -> 1`; world recertification/revocation `2-3 -> 1`; fail closed `2-3 -> 1`; measured stop lifecycle `2 -> 1`. External WorldModel publication, planner history finalization, mission acceptance, and adapter protocol are distinct transactions and are not counted as duplicated execution ownership.

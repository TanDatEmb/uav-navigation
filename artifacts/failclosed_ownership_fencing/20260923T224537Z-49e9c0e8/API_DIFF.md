# API diff

| API | Purpose |
|---|---|
| `ExecutionAuthority::failClosedIfCurrentSnapshot(expected)` | Atomically reject a stale exact owner snapshot; return applied/stale/already-failed. Includes exact empty snapshot for no-command failure, fenced by version/admission. Policy remains in RuntimeNode. |
| `ExecutionAuthority::suspendIfCurrentSnapshot(expected)` | Suspend only the exact execution whose world freshness failed. |
| `ExecutionAuthority::isCurrentSnapshot(expected)` | Owner comparison for asynchronous result/timeout. |
| `ExecutionAuthority::matchesAuthorityIdentity(expected)` | Compare active/lineage/admission identity when a version change is not the causal boundary. |
| `PendingGoalHandoffOwner::clearIfCurrent(expected)` | Prevent old cleanup from erasing newer pending intent. |
| `failedExecutionLeaseIsCurrent(failed,current)` | Pure exact immutable lease comparison used by product callbacks and barrier test. |

The raw `failClosedLocked()` primitive remains for global/reset/synchronous current-owner paths. Async planner result and watchdog paths use conditional owner operations. No planner status policy moves into `ExecutionAuthority`.

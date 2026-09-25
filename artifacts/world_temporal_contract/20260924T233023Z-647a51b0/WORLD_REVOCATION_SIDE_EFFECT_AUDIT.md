# World revocation side-effect audit

| Side effect | Trigger/currentness | Assessment |
|---|---|---|
| Clear active/staged bundles and goals; increment active lineage/timeline; fail lifecycle | Exact active bundle/pointers and timeline remained current, but preparation/revalidation did not retain active | Core execution authority consequence; prevents command exposure. |
| `consumeHotRetarget()` | Runtime finalizer invoked only for exact current active revocation (`exact_owner`) | Planning transition cleanup; tied to revoking active transition. Needs event-level evidence but ownership is bounded. |
| clear `skip_replan_once_` | Same exact revocation finalizer | Runtime planner scheduling cleanup; does not itself grant authority. |
| retire completion witness; clear reaches-goal marker and terminal generation | Same exact active revocation | Prevents stale completion evidence surviving the revoked bundle. Semantically related to active identity, but spans runtime fields. |
| `pending_goal_owner_.clearIfCurrent(retired_pending_goal)` | Only after exact current active invalidation; pointer captured then checked at clear | Prevents clearing a newer pending goal. |
| command latch `invalidate()` and mapping worker throw | Any non-`kCommitted` except explicit `kSuperseded` at caller | Broad fail-stop fallback. Likely prevents continued updates after a dependent publication transaction could not finalize, but `kNoPublishedWorld`, `kWorldAdvanced`, `kCancelled`, `kCandidateRejected` reachability and side effects are not yet separately proven. Do not weaken or preserve as fully justified without matrix tests. |

Revocation finalizer executes inside the lock/gate transaction and must remain bounded/noexcept. No planner backend call or full validation is inside locks.

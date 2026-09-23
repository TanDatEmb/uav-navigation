# Pending-goal cleanup ownership

`PendingGoalHandoffOwner::clearIfCurrent(expected)` compares the exact immutable pending-goal pointer under its existing mutex. Old planner watchdog, command lease and world-revocation callbacks capture the pending pointer they intend to retire; a stale callback cannot clear a successor enqueued later. The terminal mode-status path likewise captures the matched pending pointer before clearing. Global localization reset, explicit mission cancellation and external-authority transition retain their unconditional session-wide cleanup.

`OldTimeoutCannotClearNewPendingGoal` uses barriers to put a newer request in place before the old timeout attempts cleanup. The old pointer is rejected and the newer request remains.

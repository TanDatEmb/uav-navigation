# Execution-state lease race

`ExecutionStateStore` publishes immutable `shared_ptr<const ExecutionStateLease>` values with monotonic ingress sequence. The exact pointer is the causal witness; no new ID is needed. The command callback's first freshness result is provisional. Under localization/input/command transition locks it reloads the lease. If L1 was replaced by L2 while the callback waited, L1 cannot latch failure or revoke execution; the next callback evaluates L2. If L1 remains current and is still stale at that lock boundary, existing fail-close/latch policy remains.

The final publication veto similarly compares the failed final lease with the current store lease and exact execution identity before latching or fail-closing. The retained-command validator and motion-after-stop branch also reject a prepared observation if its exact state lease was superseded before destructive mutation. Diagnostics use the failed ingress sequence and current freshness result.

The barrier test `SupersededStateLeaseCannotAuthorizeFailure` installs L2 strictly between L1 capture and final comparison. Its reverse case confirms that an unsuperseded lease remains eligible for the existing safety action. The test is a component/model proof; SITL Core pause checks the downstream heartbeat-loss fence separately.

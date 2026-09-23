# Decisions required before implementation

1. **PASS_THROUGH retention:** choose a source-time maximum age, spatial departure/reentry rule, route projection tie/uncertainty policy and exact invalidation for a physical observation awaiting continuation. Verify producer→consumer t0/t1 reachability before classifying runtime severity.
2. **Braking policy:** choose A preserve recoverable behavior, B one-way stopping, C explicit two-stage semantics, or D insufficient evidence. This audit selects **D**: runtime committed safety is one-way; the recoverable `MissionController::onTrajectory` path has no product call site in this repository and is exercised in tests. Confirm whether its public API is intended for external integration and whether any pre-commit rolling recovery is required before designing a new lifecycle. Do not infer a runtime conflict from the two local state machines.
3. **PX4 Hold transfer:** pin actual PX4 firmware/build and specify whether fresh AUTO_LOITER plus executor-charge change is the approved authority witness. Decide retry during status delay, operator takeover, failsafe and shutdown. `ModeCompleted(Success)`/`Deactivated` alone cannot close request.
4. **World/lease authority:** choose an explicit certificate expiry and heartbeat/stop handover policy after representative queue, recertification and transport tails are measured. Current recoverable publication silence is not equivalent to explicit safety command.

No production implementation recommendation is made while `SHADOW_REDUCER_READY = NO`.

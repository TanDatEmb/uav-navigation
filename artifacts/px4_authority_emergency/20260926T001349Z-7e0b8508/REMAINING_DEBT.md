# Remaining debt

1. **Retry exhaustion policy (P1):** user decision pending on a finite cap and escalation versus continuing the 250 ms retry loop until PX4 state confirms `AUTO_LOITER`. Current code preserves the existing indefinitely retrying behavior; it does not claim total retry boundedness.
2. **C0-SW handover reducer (P1):** request, callback, and VehicleStatus evidence need producer-owned correlation by activation generation/attempt, plus 0 unresolved/conflicts/missing references/drops.
3. **Operator/failsafe race evidence (P1):** source/library establish deactivation semantics, but pinned-binary timelines are missing for operator takeover, failsafe takeover, status regression, and reactivation after Hold.
4. **EMERGENCY end-to-end (P1):** no single run yet proves measured emergency state → dynamic+World certificate → ExecutionAuthority → admitted NavigationCommand → PX4 setpoint, or uncertifiable candidate fallback to Hold.
5. **H3 controlled adapter-state stale:** deterministic admission coverage exists; a focused SITL state-stale→Hold timeline is not yet captured.

No safety threshold, planner dynamics, World policy, Mission/Desired/Execution owner, NavigationCommand field, PX4 parameter, or controller gain changed.

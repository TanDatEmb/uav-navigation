# Adapter ingress timing boundary

The independent state-input node receives `/lio/odometry_propagated` using reliable KeepLast(1) (`navigation_mode_node.cpp:322-353`). On each callback, the diagnostic trace records callback entry, lock request/acquire, typed disposition and accepted receive time. Only `ACCEPTED` rows have a nonzero accepted-receive witness. The command boundary uses last **accepted** odometry receive time; callback entry or rejected messages do not renew it. The trace is published after releasing `trajectory_mutex_` and is never consumed by admission or Hold policy.

Producer-to-callback includes DDS and executor. Callback-to-accept includes input checks and lock wait. The accepted-state gap is the direct witness for the 200 ms receive boundary. Per-sequence distributions await instrumented SITL.

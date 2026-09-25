# Remaining boundary debt

1. Adapter pre-admission `valid=0` sample rejection remains separate; no command contract or 100 ms adapter receive lease was changed.
2. Isolated world source-time stale and world/command temporal policy require their own boundary evidence; this repair only fences stale callback mutation and retains current world policy.
3. PX4 Hold callback ordering/emergency runtime evidence remains separate; no Hold protocol or emergency planner algorithm was changed.

4. The two existing one-shot failure hooks did not arm during matched long-featured sessions. The repeated hook emitted `kFailed`, while the exact hot-retarget `OptimizationFailed` path has deterministic component proof only. A separately controlled runtime injection of that exact status would close this evidence gap; no product-only test mode or safety bypass was added here. The branch verdict relies on the full source/owner proof, deterministic race tests, nominal parity and independent safety-fault runs, not on a non-injected hook attempt.

# Remaining boundary debt

1. Adapter pre-admission `valid=0` sample rejection remains separate; no command contract or 100 ms adapter receive lease was changed.
2. Isolated world source-time stale and world/command temporal policy require their own boundary evidence; this repair only fences stale callback mutation and retains current world policy.
3. PX4 Hold callback ordering/emergency runtime evidence remains separate; no Hold protocol or emergency planner algorithm was changed.

If the focused fault harness cannot force the exact `OptimizationFailed` status without a product-only test mode, record component proof and runtime gap separately in `SITL_RESULTS.md`. Do not call the branch merge-ready solely from a model test.

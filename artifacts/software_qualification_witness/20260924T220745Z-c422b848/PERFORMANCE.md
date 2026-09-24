# Descriptive timing and observer overhead

Capture-time report metrics for the five primary runs (each `report.json`, `evaluation.metrics.timing`) show PVA observer interarrival p50 19.80–19.96 ms, p95 48.12–50.92 ms, p99 55.21–60.13 ms, and maximum 67.87–74.98 ms. PX4 setpoint-update duration p50 0.0375–0.0424 ms, p95 0.0598–0.0689 ms, p99 0.0758–0.0867 ms, and maximum 0.118–0.466 ms. These are descriptive observations at the captured source, not worst-case execution bounds or qualification thresholds.

Primary scenario+monitor writers accepted and wrote 420,486 records with zero queue/snapshot/closed drops, serialization errors, write errors or pending records. Verification writers accepted and wrote 342,942, also with zero loss. Exact diagnostic publication overhead and before/after A/B latency at the same scenario are `RUNTIME_UNVERIFIED`. The added diagnostic emission should be profiled if a future tail approaches a safety lease. No timing threshold was changed here.

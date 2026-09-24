# Timing distributions

The pinned A2 capture did not contain per-sequence producer publication or adapter callback/lock witnesses; p50/p95/p99/max for those boundaries cannot be reconstructed. `receive_age_ms=208.583` is a safety boundary observation, not a transport distribution. New instrumented SITL sessions must be analyzed with `python3 tools/runtime/state_transport_analysis.py <session>`; report sample counts, missing trace count, quantiles and maximum, and retain all attempted runs. Do not infer hard WCET from a small cohort.

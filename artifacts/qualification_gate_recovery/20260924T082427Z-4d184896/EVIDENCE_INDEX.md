# Evidence index

| Claim | Evidence and class | Limit |
| --- | --- | --- |
| Baseline/source/PX4 provenance | `BASE_PROVENANCE.md`; exact `metadata.json` and validated authoritative build manifest in each raw session | PX4 checkout has pre-existing dirty files, listed there; pinned binary hash controls runtime identity. |
| Pinned A2 receive lease safety response | Raw `external-mode-check-20260924T073836-106951/{monitor.json,report.json,samples.jsonl}`; SITL | No per-sequence producer or adapter trace in A2. |
| Natural reproduced clock/producer tail | Pilot 3 `external-mode-check-20260924T093208-186236/{samples.jsonl,monitor.json,rosbag/rosbag_0.db3,state_transport_analysis.json}`; SITL diagnostic | Rosbag `/clock` timestamp is recorder arrival, not Gazebo publication. First upstream component unresolved. |
| Exact ingress joins and timing distributions | `tools/runtime/state_transport_analysis.py`, `NOMINAL_TIMING.csv`, ten cohort raw sessions; diagnostic SITL | Per-run quantiles; no hard WCET or flight qualification. |
| Ten-run denominator and outcomes | `.artifacts/runtime/qgr-nominal-cohort.json`, `NOMINAL_COHORT.md`, ten raw `report.json` and `metadata.json` files; SITL | 8 COMPLETE, 2 terminal safety stops; all 10 evaluator NOT_EVALUABLE. |
| Clock blind-spot repair | `tools/runtime/monitor.py`, focused Python clock test, offline rosbag fallback test; UNIT + source | Separate diagnostic threshold does not alter 500 ms stale assessment. |
| Exact heartbeat/reducer repair | `tools/runtime/evaluation.py`, `tools/runtime/external_mode_scenario.py`, focused Python tests, `LIFECYCLE_ATTRIBUTION.md`; UNIT + source | 140 unresolved lifecycle transactions remain in ten-run cohort. |
| Exact OptimizationFailed regression | `external-mode-check-20260924T101731-232477/{scenario.jsonl,report.json,logs/mapping.log}`; injected SITL | Experiment-specific; excluded from nominal cohort. |
| Earlier Core pause/BACKUP/STOP | `artifacts/failclosed_ownership_fencing/20260923T224537Z-49e9c0e8/SITL_RESULTS.md`; prior pinned SITL | Not rerun on this source. |
| Safety threshold and bypass preservation | `docs/safety/runtime_safety_current.md`, static trace-scope guard, `TEST_EVIDENCE.md`, source diff; SOURCE + UNIT | No policy tuning or qualification inference. |

`RAW_EVIDENCE_MANIFEST.csv` lists each retained raw path, byte count and SHA256. Large raw sessions remain in the shared `.artifacts/runtime/` root and are not committed; each session's own metadata carries source and build provenance. The pilot harness failures remain visible in `SITL_RESULTS.md`; no failed attempt was relabeled successful.

# Natural nominal cohort

Requested profile: `long_featured`, seed 0, tracking experiment `off`, dynamics experiment `off`, external mode, with default-off diagnostic state-transport trace explicitly enabled. All ten attempts used source `96ed8d089e576c505abdc15500746e9b5be10e50`, the same authoritative Release build and the pinned PX4 binary. The runner retained every attempt in `.artifacts/runtime/qgr-nominal-cohort.json`; exact raw paths and SHA256 values are in `RAW_EVIDENCE_MANIFEST.csv`.

| Run | Mission outcome | Accepted waypoints | Max accepted-state gap (ms) | Max observed `/clock` gap (ms) | Trace matched | Unexpected Hold |
| ---: | --- | --- | ---: | ---: | --- | --- |
| 1 | PAUSED_SAFETY_STOP | 0–3 | 27.742 | 11.548 | yes | yes |
| 2 | PAUSED_SAFETY_STOP | 0–3 | 35.592 | 15.181 | yes | yes |
| 3 | COMPLETE | 0–4 | 27.856 | 11.195 | yes | no |
| 4 | COMPLETE | 0–4 | 35.582 | 14.283 | yes | no |
| 5 | COMPLETE | 0–4 | 34.872 | 12.875 | yes | no |
| 6 | COMPLETE | 0–4 | 35.263 | 13.464 | yes | no |
| 7 | COMPLETE | 0–4 | 28.060 | 13.942 | yes | no |
| 8 | COMPLETE | 0–4 | 30.349 | 12.645 | yes | no |
| 9 | COMPLETE | 0–4 | 27.274 | 11.429 | yes | no |
| 10 | COMPLETE | 0–4 | 26.687 | 11.426 | yes | no |

Eight COMPLETE outcomes, including eight consecutive completions (runs 3–10), satisfy the focused mission liveness streak. The two failures remain in the denominator. Both entered bounded terminal endpoint recovery at waypoint 4, then safety-stopped: execution boundary rejections recorded anchor errors 0.762 m and 0.759 m against the unchanged 0.750 m limit in runs 1 and 2. This is a distinct terminal planning/execution finding, not a state-transport tail. Both requested PX4 Hold after the stop. No cohort run had a >100 ms accepted-state gap. The 200 ms margin at the largest gap was 164.408 ms.

Requested and effective tracking mode were `off` in all ten metadata records, with braking and estimator-health suppression false. Summed external-mode active intervals from exact `external_mode_entered`/`external_mode_exit_observed` scenario events: 542.272 simulated seconds. This is characterization, not a reliability estimate. All ten evaluator results remain `NOT_EVALUABLE` for lifecycle attribution, reference lineage and missing approved tracking/motion policies; mission COMPLETE is never relabeled evaluator PASS.

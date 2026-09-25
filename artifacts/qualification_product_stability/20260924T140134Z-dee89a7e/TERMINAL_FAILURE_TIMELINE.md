# Reconstructed terminal failure timeline

Raw inputs: prior true tracking-off sessions `external-mode-check-20260924T094122-193534` (run 1) and `external-mode-check-20260924T094341-193534` (run 2) under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`. `tools/runtime/analyze_terminal_recovery.py` joins exact command generation, first analytic completion, propagated state source time, PX4 input trace, and the logged Core gate rejection. `TERMINAL_RESULTS.csv` covers all ten predecessor runs. Full per-offset numeric records are in `terminal_run_1.json` and `terminal_run_2.json`.

| Witness | Run 1 | Run 2 |
|---|---:|---:|
| Request / emergency generation | 5 / 54 | 5 / 40 |
| Emergency takeover command position step from prior sample | 0.611 m / 4 ms | 0.450 m / 16 ms |
| First `STATUS_COMPLETED` source time | 70.120 s | 65.208 s |
| Measured endpoint error / speed at first completion | 0.406 m / 0.128 m/s | 0.283 m / 0.069 m/s |
| PX4 input after completion | repeated exact endpoint `position_hold` | repeated exact endpoint `position_hold` |
| Measured error at Core STOPPED_HOLD rejection | 0.7616 m (log 0.762) | 0.7586 m (log 0.759) |
| Measured speed at rejection | 0.580 m/s | 0.863 m/s |
| Core gate | 0.750 m | 0.750 m |
| PX4 input diagnostic drops for generation | 0 | 0 |
| Maximum command observer steady interval after first completion before rejection | 72.707 ms | 56.682 ms |
| Maximum PX4 input observer steady interval in same window | 71.062 ms | 57.800 ms |
| Command endpoint variation after first completion | 0 m over 60 samples | 0 m over 61 samples |
| Evaluation-only truth displacement, aligned locally at first completion: endpoint error at rejection | 0.747 m | 0.732 m |
| LIO-vs-truth displacement difference over hold | 0.058 m | 0.044 m |
| Evaluation-only truth speed, first completion → rejection | 0.161 → 0.600 m/s | 0.127 → 0.879 m/s |

The measured vehicle first approaches the commanded endpoint closely, then moves away while the exact endpoint hold remains published. Run 1: error ~0.028 m at 70.512 s, then ~0.705 m at 71.012 s. Run 2: error ~0.076 m at 65.580 s, then ~0.578 m at 66.000 s. The stop/hold is not physically settled, even though the analytic polynomial has ended. `STATUS_COMPLETED` is therefore not early relative to its analytic meaning; it is earlier than physical acceptance by design.

The recovery attempts diverge: run 1 has no logged PlanFromRest attempt for generation 54 before the proximity rejection; its short near-zero-speed interval falls between planner cycles. Run 2 enters measured-stop recovery at 65.220 s, but that PlanFromRest solve returns result 4 (`kFailed`, `replan_code=-12`) and retains the endpoint while retrying. Its speed then rises above the stationary gate, so the bounded retry cannot produce a replacement before the anchor rejection. The planner result is a contributing event in run 2, not a license to discard the valid endpoint.

The new emergency command starts from the measured state after prior command-relative tracking exceeds its budget; the step is not evidence of a failed candidate-continuity validator. Run 1 log reports old-command anchor error 0.635 m against 0.250 m tracking limit, and measured-state emergency brake; run 2 follows the same recovery family. The command step and the subsequent overshoot are recorded, but a controller gain/dynamics defect has **not** been isolated from these two runs.

The independent truth cross-check uses only **displacement** between first analytic completion and rejection, rotates it by the run's recorded `T_L_G`, and aligns at first completion. It avoids treating the initial LIO/Gazebo translation as a persistent zero-drift pose registration. Truth samples at rejection are 8 ms later in run 1 and coincident in run 2. The physical-motion estimate is also near the 0.750 m boundary; it does **not** prove that the physical body exceeded 0.750 m. The control boundary acts on LIO odometry, whose measured error did exceed it. No evaluation-only truth is fed into the flight gate.

The command/PX4 cadence figures use the independent scenario observer steady clock over the terminal source-time window. Source-time intervals alone were 44/44 ms maximum and would understate the wall-clock tail. Neither observed steady gap reached the current 100 ms adapter receive lease, but run 1's ~71 ms PX4 input tail has limited margin and is retained as timing evidence.

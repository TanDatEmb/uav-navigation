# Nominal regression cohort

Three fresh sessions used long_featured, seed 0, C0_SW_V1, tracking experiment explicitly requested off, and the same PX4 binary. No World fault gate was enabled.

| Run | Session | Scenario | Runner report | Relevant finding |
|---|---|---|---|---|
| N1 | `external-mode-check-20260925T092305-366082` | COMPLETE | FAIL | report flagged `lidar timestamp/freshness/validity violation`; no World suspension; C0-SW remains NOT_EVALUABLE for pre-existing motion/reference/tracking policy gaps |
| N2 | `external-mode-check-20260925T092630-369436` | PAUSED_SAFETY_STOP | BLOCKED | accepted waypoints 0–3; four accepted waypoint events total. Exact `retained_decision` records final bridge unusable for current request/generation and fail-closes; no World suspension/staleness event occurred. This is an existing terminal/recovery safety path outside World closure, not evidence that World caused the stop. |
| N3 | `external-mode-check-20260925T092909-373055` | COMPLETE | FAIL | C0-SW NOT_EVALUABLE for motion policy, reference lineage, tracking coverage; no World fault |

All three tracked-off requests were effective as off in the runtime configuration; braking and health suppression were false. Two of three missions completed. The third took an observed fail-closed safety stop after `final_bridge_usable=false`. No result is represented as C0-SW qualified. This is a behavioral regression cohort, not a qualification pass. The N2 terminal recovery condition remains separately tracked and was not changed in this World-only branch.

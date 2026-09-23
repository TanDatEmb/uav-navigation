# Differential replay results

Command: `python3 -m tools.shadow_reducer.cli --normalized-root <instrumentation-worktree>/.artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c --output REPLAY_RESULTS.json`. Machine-readable per-run counts and bounded issue list are in `REPLAY_RESULTS.json`. The input is ignored raw normalized evidence from the clean instrumentation worktree, not new flight data.

| Semantic comparison | Result | Classification and limit |
|---|---:|---|
| O1 PASS accepted events | 39 MATCH in 10 completed episodes | Same-callback measured ball/crossing plus continuation; PT2 crossing-first only model tested, not observed. |
| O1 STOP accepted events | 10 MATCH | STOP outcome is source-classified measured confirmation; raw velocity time series is not reconstructed by shadow. |
| O2 command exact pairs | 6,398 exact, 214 ambiguous/unpaired | Ambiguous keys are `INSUFFICIENT_TRACE`; these are cumulative across all 14 runs, not the C-only figure (C: 2,680 exact / 21 ambiguous). Clock proof is shared simulated time for C, not transport latency. |
| O4 fault runs W2–W5 | 4 lease expiries, 4 Hold requests, zero observed old-session resume or downstream admitted command after fence | W1 is a fresh-world control with terminal Hold; 4 fault runs are observations, not proof of every scheduling interleaving. |
| Old upstream publications after Hold fence | 3,550 across all runs | This is **not** a command-authority divergence: 3,338 downstream receives after fence were rejected. Published upstream samples can continue while adapter owns the fence. |
| Startup rearm O1-R09 | 1 inferred new-session boundary after downstream valid receive; explicit rearm event absent | `INSUFFICIENT_TRACE` for cross-process ordering; initial Hold preceded any observed active execution. Do not misclassify as resurrection of old generation. |
| PX4 Hold | 15 audit requests across all runs, 14 final Hold-observed states | 1 additional O1-R09 phase-4 callback is a source-identified LOITER `Rejected` result, outside the six O3 episodes; it does not confirm Hold. ACK/callback never substitutes for status. |
| Trace integrity | 26 startup gap ranges, 12 internal gap ranges, 13 reported audit drops, 275 bag-observer order inversions | These categories are independent. Pair ambiguity is propagated to `INSUFFICIENT_TRACE`; bag order is not producer order. Direct same-callback acceptance has complete local witness. |

The replay deliberately compares semantic outcomes, not equality of `safety_suffix_active`, `ExecutionEpisode.phase` or other legacy internals. No `TARGET_MODEL_BUG` or proven `LEGACY_BEHAVIOR_HAZARD` arose from the observed causal windows. PT2 differs from callback-local legacy by target policy (`POLICY_DIFFERENCE` with a source-derived loss mechanism), not a reproduced runtime divergence. PX4 final transfer remains `EXTERNAL_CONTRACT_UNKNOWN`; latency tails remain `TIMING_UNRESOLVED`.

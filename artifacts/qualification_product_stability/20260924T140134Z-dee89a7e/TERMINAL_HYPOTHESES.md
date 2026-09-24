# Terminal hypotheses

| Hypothesis | Assessment | Evidence |
|---|---|---|
| H1 measured LIO tracking residual | Supported at control boundary; independent physical threshold crossing unresolved | Propagated position reconstructs both logged gate errors within 0.001 m. Locally aligned evaluation-only truth displacement yields 0.747/0.732 m, below but near the 0.750 m threshold. |
| H2 `STATUS_COMPLETED` emitted early | Rejected for analytic semantics | Candidate reached declared end; measured completion is separately gated. |
| H3 mismatched command/state times | Not causal for static endpoint | Command endpoint stays fixed through repeated hold; state age/skew is millisecond scale. |
| H4 stale execution state | Not supported; estimator displacement difference remains open | Core log says execution support valid; nearest source samples closely precede gate. LIO-vs-truth displacement differs 0.058/0.044 m over the terminal hold. |
| H5 endpoint identity switch | Not supported after completion | Same request/generation/endpoint persists to rejection; no new active generation in the hold. |
| H6 replan/handoff discontinuity | Observed, causal contribution unresolved | Emergency measured-state takeover steps command 0.450–0.611 m after prior tracking breach. This is deliberate recovery, not automatically a defect. |
| H7 stale terminal state | Not supported | Source stamp and monitor cadence remain fresh. |
| H8 controller/stopping behavior | Supported as immediate mechanism; deeper tuning cause unresolved | Evaluation-only truth speed rises after near-stop and position hold, and LIO endpoint error grows beyond the gate. PX4 input recorder has a 71.062/57.800 ms steady-clock tail; its causal contribution remains unisolated. |

No threshold, candidate certificate, planner algorithm, or PX4 controller parameter was changed to convert the two failures to COMPLETE.

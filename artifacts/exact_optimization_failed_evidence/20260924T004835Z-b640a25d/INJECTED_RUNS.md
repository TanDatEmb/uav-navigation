# Injected runs — exact denominator

All four final attempts used source `47a5c05e9b36feb558e5e2ecd3646cc36157613d`, `long_featured`, seed 0, tracking off, external-mode SITL and the same PX4 binary recorded in `BASE_PROVENANCE.md`. The hook was explicitly enabled; normal controls below disabled it. Here “arm” means the semantic trigger produced `FAULT_INJECTION_ARMED`, not merely that the parameter was enabled.

| Run | Raw session | Armed | Applied | Rejected exact pairs | Active→desired | Predecessor admissions after fault | Fault→successor admission ms | Maximum adapter gap ms | Focused result |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| I1 | `external-mode-check-20260924T013310-690969` | 1 | 1 | 0 | 2→3 | 29 | 585.045 | 20.174 | FOCUSED_PASS |
| I2 | `external-mode-check-20260924T013528-694397` | 0 | 0 | 2 | —→— | 0 | — | — | FOCUSED_FAIL |
| I3 | `external-mode-check-20260924T013858-697883` | 1 | 1 | 1 | 3→4 | 28 | 575.780 | 20.414 | FOCUSED_PASS |
| I4 | `external-mode-check-20260924T014135-701439` | 1 | 1 | 0 | 2→3 | 28 | 576.866 | 20.726 | FOCUSED_PASS |

**Denominator:** attempted 4; armed 3; injected 3; exact code-6 OptimizationFailed events 3; HG-023 certified predecessor retained 3; mission COMPLETE 4/4; 3/3 injected positives met every focused assertion. I2 is a visible `INJECTION_REJECTED`/`FOCUSED_FAIL`, never counted as injected evidence. All four reports accept `[0,1,2,3,4]` and record no unexpected Hold request. The versioned evaluator's overall verdict is `FAIL`, `qualification_eligible=false`; this focused observation is not flight qualification.

I2 first pair 2→3 was rejected because owner version changed 306→307 and a staged successor appeared (`pending_at_solve=0`, `pending_current=1`). The alternate 3→4 was rejected on owner version 411→412/currentness false; the trace does not establish which internal event advanced the version. I3 rejected 2→3, then applied once at current 3→4. These rejections demonstrate that the hook refuses stale snapshots rather than forcing the fault.

Each positive had backend original `kSuccess` (0), injected exact `kOptimizationFailed` (6), actual classifier `RetainCommittedCommand` (4), HG-023 final `CertifiedCommandPreserved` (8), and no post-decision loss of command availability. All positive runs had successor adapter admission and no late predecessor admission after cutover.

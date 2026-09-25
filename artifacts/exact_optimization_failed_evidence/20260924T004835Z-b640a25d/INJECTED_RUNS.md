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

## Earlier harness-development attempts — separate source baselines

These four attempts occurred before the behavior-bearing source was pinned at `47a5c05e...`. They are **included in the all-attempt denominator** but not in the final same-source 3/3 evidence gate.

| Session suffix | Source SHA prefix | Armed | Applied exact status | Focused result | Reason excluded from final gate |
| --- | --- | ---: | ---: | --- | --- |
| `011329-666867` | `3dee6476` | 1 | 1 | PASS | Earlier source; HG-023 disposition 8, mission COMPLETE |
| `011551-670384` | `3dee6476` | 0 | 0 | FAIL | Hook never armed; mission COMPLETE |
| `012156-676836` | `42a9086e` | 1 | 1 | FAIL | HG-023 returned `Superseded` (disposition 3), so no certified retention claim; mission COMPLETE |
| `012545-680439` | `42a9086e` | 0 | 0 | FAIL | Owner snapshot changed and trigger rejected; mission COMPLETE |

**All branch attempts:** 8 injected-profile sessions attempted; 5 armed/applied exact code 6; 4 positive retained-command chains across different source commits; 8/8 mission COMPLETE. The verdict uses only the **4 final pinned-source attempts**, with **3 exact positive runs**. `012156` demonstrates that injecting code 6 does not bypass HG-023: the superseded owner was not certified for retention.

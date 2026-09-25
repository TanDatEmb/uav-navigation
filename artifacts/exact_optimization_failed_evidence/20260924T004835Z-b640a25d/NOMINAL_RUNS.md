# Injection-disabled nominal controls

Source `47a5c05e9b36feb558e5e2ecd3646cc36157613d`; same `long_featured`, seed 0, tracking off and PX4 binary as final injected attempts. The runner did not write an enable parameter. No fault event was applied.

| Run | Raw session | Focused result | Armed | Applied | Accepted | COMPLETE | Hold request |
| --- | --- | --- | ---: | ---: | --- | --- | --- |
| N1 | `external-mode-check-20260924T014351-704845` | FOCUSED_PASS | 0 | 0 | [0, 1, 2, 3, 4] | yes | none |
| N2 | `external-mode-check-20260924T014646-708399` | FOCUSED_PASS | 0 | 0 | [0, 1, 2, 3, 4] | yes | none |
| N3 | `external-mode-check-20260924T014922-711772` | FOCUSED_PASS | 0 | 0 | [0, 1, 2, 3, 4] | yes | none |

Fresh controls: **3/3 mission COMPLETE**. Their versioned evaluator overall result is still `FAIL`/qualification ineligible, and is retained as such. The focused criterion is mission/command seam continuity only.

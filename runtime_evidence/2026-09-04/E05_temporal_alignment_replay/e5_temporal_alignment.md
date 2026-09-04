# E5 temporal alignment analysis

This report uses exact immutable-bundle samples when direct telemetry is present. It does not estimate command position from speed, age, or neighboring command samples.

- Direct retained-validation samples: `2`
- Usable alignment samples: `2`
- Legacy temporal fields missing: `False`
- Figures: `runtime_evidence/2026-09-04/E05_temporal_alignment_replay/figures/plot_E5_temporal_alignment.png, runtime_evidence/2026-09-04/E05_temporal_alignment_replay/figures/plot_E5_temporal_alignment_scatter.png`

## Measured decomposition

- Boundary sample timestamp: `28256000000`
- raw anchor error: `0.31348915573903663` m
- time-aligned anchor error: `0.26808731736339164` m
- command motion during state age: `0.046697754461953206` m
- raw minus aligned: `0.04540183837564499` m
- source age: `16.0` ms; receive age: `21.382264` ms

## Offline predicate replay

The runtime predicate remains unchanged. The aligned branch below is conditional only where legacy artifacts omit freshness/lease/role inputs; it is not presented as an exact runtime authorization decision unless all inputs are recorded.

```json
{
  "aligned_emergency_authorized_actual_anchor_branch": true,
  "aligned_retained_suffix_usable_conditional": false,
  "aligned_tracking_certificate_exceeded": true,
  "predicate_replay_exact": false,
  "raw_emergency_authorized": true,
  "raw_retained_suffix_usable": false,
  "raw_tracking_certificate_exceeded": true
}
```

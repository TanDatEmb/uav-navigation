# E5 temporal alignment analysis

This report uses exact immutable-bundle samples when direct telemetry is present. It does not estimate command position from speed, age, or neighboring command samples.

- Direct retained-validation samples: `1`
- Usable alignment samples: `1`
- Legacy temporal fields missing: `False`
- Figures: `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/figures/plot_E5_temporal_alignment.png, runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact/figures/plot_E5_temporal_alignment_scatter.png`

## Measured decomposition

- Boundary sample timestamp: `30268000000`
- raw anchor error: `0.46515379425721387` m
- time-aligned anchor error: `0.42242751718868116` m
- command motion during state age: `0.04350460700352208` m
- raw minus aligned: `0.042726277068532714` m
- source age: `16.0` ms; receive age: `20.893902` ms

## Offline predicate replay

The runtime predicate remains unchanged. The aligned branch below is conditional only where legacy artifacts omit freshness/lease/role inputs; it is not presented as an exact runtime authorization decision unless all inputs are recorded.

```json
{
  "aligned_actual_anchor_branch": false,
  "aligned_emergency_authorized": false,
  "aligned_projected_anchor_error_m": NaN,
  "aligned_projected_main_only_branch": false,
  "aligned_retained_suffix_usable_conditional": false,
  "aligned_tracking_certificate_exceeded": true,
  "predicate_replay_exact": true,
  "raw_actual_anchor_branch": false,
  "raw_emergency_authorized": false,
  "raw_projected_main_only_branch": false,
  "raw_retained_suffix_usable": false,
  "raw_tracking_certificate_exceeded": true
}
```

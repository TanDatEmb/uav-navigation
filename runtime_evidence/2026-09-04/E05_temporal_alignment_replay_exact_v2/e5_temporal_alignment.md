# E5 temporal alignment analysis

This report uses exact immutable-bundle samples when direct telemetry is present. It does not estimate command position from speed, age, or neighboring command samples.

- Direct retained-validation samples: `1`
- Usable alignment samples: `1`
- Legacy temporal fields missing: `False`
- Figures: `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_temporal_alignment.png, runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_temporal_alignment_scatter.png`

## Measured decomposition

- Boundary sample timestamp: `29924000000`
- raw anchor error: `0.4532910267671672` m
- time-aligned anchor error: `0.4428736168588373` m
- command motion during state age: `0.010839246322547656` m
- raw minus aligned: `0.0104174099083299` m
- source age: `4.0` ms; receive age: `6.128108` ms

## Offline predicate replay

The runtime predicate remains unchanged. Where all retained predicate inputs are serialized, this is an exact offline replay with only the anchor-error input substituted; legacy artifacts remain explicitly non-replayable.

```json
{
  "aligned_actual_anchor_branch": true,
  "aligned_emergency_authorized": true,
  "aligned_projected_anchor_error_m": 0.46679093524109544,
  "aligned_projected_main_only_branch": false,
  "aligned_retained_suffix_usable_conditional": false,
  "aligned_tracking_certificate_exceeded": true,
  "predicate_replay_exact": true,
  "raw_actual_anchor_branch": true,
  "raw_emergency_authorized": true,
  "raw_projected_main_only_branch": false,
  "raw_retained_suffix_usable": false,
  "raw_tracking_certificate_exceeded": true
}
```

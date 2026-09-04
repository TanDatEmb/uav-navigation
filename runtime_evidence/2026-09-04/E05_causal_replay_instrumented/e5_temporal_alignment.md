# E5 temporal alignment analysis

This report uses exact immutable-bundle samples when direct telemetry is present. It does not estimate command position from speed, age, or neighboring command samples.

- Direct retained-validation samples: `0`
- Usable alignment samples: `0`
- Legacy temporal fields missing: `True`
- Figures: `NOT_GENERATED`

## Result

`NOT_TESTED`: the input artifact has no direct evaluation/state timestamp pair and no exact immutable-polynomial samples at both timestamps. Required fields are emitted as `NOT_RECORDED`; no H7 conclusion is claimed.

The legacy `anchor_error_m` field, when present, is not substituted for `raw_anchor_error_m` because it does not prove the command sample and measured state share a timestamp.

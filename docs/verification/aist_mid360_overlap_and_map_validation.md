# AIST Mid-360 overlap and local-map validation

The prepared `aist-mid360-drive` PointCloud2 stream uses absolute per-point
timestamps. Direct inspection of all 2,772 messages found approximately
100 ms scan windows with small boundary overlap: p50 311,808 ns, p95
403,712 ns, p99 412,416 ns, and maximum 438,016 ns. Although 42.42% of
messages overlapped the previous accepted interval, only 80,383 of
55,440,864 points (0.145%) were in the overlap. This is bounded boundary
duplication, not a large rolling cloud or a timestamp unit/reference error.

The AIST configuration therefore enables deterministic adapter-boundary
normalization with a 1 ms maximum overlap and a 1,000-point minimum emitted
scan. Points at or before the previous emitted end are removed while every
remaining absolute timestamp is preserved. Larger overlaps remain visible
to the core fail-closed overlap gate. Offline run provenance records input,
emitted, and dropped-overlap point counts and the dropped ratio.

The `diagnostics.csv` stage flags are authoritative. Rejected results do not
inherit per-scan deskew, registration, map-operation, or timing fields from a
previous result. Report distributions select rows using the explicit stage
flags and include their sample counts.

`residual_rms` measures internal scan-to-map registration consistency. It is
not geometric map accuracy. AIST provides no ground truth, so final map
quality still requires a same-view visual review or a separate reference
dataset.

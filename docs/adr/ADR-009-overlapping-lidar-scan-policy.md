# ADR-009: Overlapping LiDAR scan policy

## Context

The Mid-360 dataset contains LiDAR messages whose per-point time intervals can
overlap the interval of the preceding synchronized scan. The M1 estimator owns
one forward-only state epoch and deskews each accepted scan with the IMU
trajectory from the preceding estimator epoch through the current scan end.

Accepting an overlapping interval without historical poses would either
propagate the filter backward or integrate part of the same IMU interval twice.
Both violate the estimator time contract.

## Decision

M1 rejects overlapping LiDAR scan intervals fail-closed with
`StatusCode::kOverlappingLidarInterval`. The rejection does not advance the
last synchronized end time and does not prune or consume an IMU interval. The
next non-overlapping scan continues from the last successful synchronized end.

Each rejection reports:

- `previous_synchronized_end_ns`
- `current_scan_start_ns`
- `current_scan_end_ns`
- `overlap_duration_ns`
- `scan_index`

## Rationale

The current estimator has no pose history covering the overlapping interval.
Removing the overlap gate would make deskew and prediction mathematically
incorrect. A dedicated rejection is preferable to classifying this as a
timestamp regression: each individual scan remains internally valid and
monotonic.

## Consequences

Overlap messages can be accepted by the input buffer but do not become
synchronized groups, correction attempts, odometry outputs, or map insertions.
This reduces effective corrected output rate while preserving forward-only
filter propagation. The policy does not change registration, residual, IMU-gap,
noise, or convergence thresholds.

## Current dataset statistics

For `swarm_lio2_mutual_avoidance_uav1`, the validated baseline has 1,384 raw
LiDAR messages, 1,384 buffer-accepted messages, 380 overlap rejections, 1,002
synchronized groups, 974 correction attempts, 973 successful corrections, and
one failed correction. The exact values are re-measured for every acceptance
run rather than encoded as runtime policy.

## Effective output rate

Reports keep these quantities distinct:

```text
buffer acceptance ratio = buffer accepted LiDAR / raw LiDAR
synchronization ratio = synchronized groups / buffer accepted LiDAR
correction success ratio = successful corrections / correction attempts
effective corrected output rate = successful corrections / dataset duration
```

Therefore `973 / 974 = 99.8973%` is the correction success ratio among
attempted updates. It is not a success percentage over all LiDAR input.

## Risks

Sequences with extensive overlap produce fewer corrected outputs and map
insertions. A long run of overlapping intervals can reduce observability even
though the following valid scan remains processable. Diagnostics and the
effective rate make this visible.

## Future overlap-aware design

This is backlog only and has no placeholder implementation in M1:

- pose history covering the overlap;
- deskew from the historical trajectory;
- no backward filter propagation;
- no double IMU integration;
- bounded history retention.

## Acceptance criteria

- overlap uses its dedicated status and complete timestamp context;
- the last synchronized epoch is unchanged by rejection;
- no artificial IMU gap is created;
- no IMU propagation interval is consumed twice;
- the first following non-overlapping scan synchronizes normally;
- reports expose raw, buffer, synchronization, attempt, success, and failure
  counters plus all three ratios.

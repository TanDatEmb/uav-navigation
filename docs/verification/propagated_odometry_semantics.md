# Propagated odometry semantics

`/lio/odometry_corrected` remains the authoritative FAST-LIO output. It is
published only after a successful LiDAR correction and its timestamp remains
the scan end time. `/lio/odometry_propagated` is an optional, higher-rate view
of an independent IKFoM filter propagated with canonical IMU samples.

Both topics express the P0.3 base-link contract: `header.frame_id` is the
configured odom frame, `child_frame_id` is the configured `base_link`, and
linear/angular velocity use the base child-frame convention. Dynamic TF
ownership remains exclusive between corrected and propagated output according
to the P0.3 owner policy.

## Sensor-time timeline

The propagator tracks correction anchor time, propagated time, latest accepted
IMU time, last correction time, last published time, and the next publication
deadline separately. Every propagated message uses the actual timestamp of the
last IMU sample integrated by the filter. Wall time and ROS node time are never
used to timestamp a propagated state.

Prediction runs for every valid IMU event. Publication is rate-limited by a
sensor-time deadline (50 Hz by default), is triggered only while processing an
IMU event, and requires a timestamp strictly greater than the prior output. If
processing misses deadlines, only the newest state is published; the publisher
does not emit a catch-up burst or synthesize timestamps.

The propagated worker uses split channels: IMU ingress is drained as a batch,
while estimator control and correction state use a coalescing mailbox. Taking a
correction moves its value and clears the mailbox in the same critical section.
Corrections carry the current control generation, so invalidation and load
shedding cannot be reversed by an older replay. A correction without a retained
history end bracket remains pending and is retried after later IMU history
arrives. A correction whose start bracket has already been pruned is terminal:
it is dropped and counted rather than retried indefinitely. The worker takes a
correction under its control lock, performs replay without that lock, and
revalidates generation and main-estimator state before committing the result.
The `replay_in_progress` diagnostic exposes this interval for runtime
observation; it must not prevent IMU ingress or diagnostics snapshots.
Worker instances are single-use after `stop()`; runtime history and
diagnostics are not reset for a second start.

An accepted IMU sample is first offered to the main estimator. The propagated
fanout then always attempts the same sample when the main path accepted it;
load shedding is requested independently when the overload threshold is
reached or auxiliary ingress rejects the sample. A main-path rejection requests
shedding and skips fanout. Load-shedding requests are idempotent and do not
silently discard an IMU sample that the main path accepted.

An accepted forward interval larger than
`ikfom.maximum_integration_step_ns` starts a new continuity epoch. The gap
sample is retained as the first sample of the new epoch, old history and
pending prediction state are cleared, and the worker increments its control
generation and suspends publication. Remaining samples in the same drained
batch are still recorded into the new epoch; they are never predicted across
the gap. `requires_reanchor=true` remains fail-closed until a current-generation
correction successfully replays the new history. The epoch, reset count, and
reset timestamp are exported in diagnostics.

When suspended, IMU samples continue through validation into bounded replay
history without invoking prediction or appending active prediction samples.
Recovery requires a current-generation valid main state and a successful
correction replay. A drained non-empty IMU batch produces at most one propagated
publication, and a correction alone never publishes.

## Correction, replay, and validity

A successful corrected state reanchors the independent propagation filter.
Bounded IMU history retains a sample on each side of the correction boundary,
including interpolation when the boundary lies between samples. The filter
then replays history to its latest accepted IMU time. Reanchor and replay are
transactional: failure restores the prior state and covariance, invalidates
propagated output, and never affects the main FAST-LIO pipeline.

A correction event never publishes and never changes the last published time.
After replay, output can resume only on a later IMU event. Publication also
requires main estimator status `Tracking`, `navigation_valid=true`, a valid
propagated state/covariance, continuous IMU timestamps, and correction age no
greater than `maximum_correction_age_ns`. Stale correction, main degradation or
loss, timestamp regression, an excessive IMU integration interval, missing
history bracket, numerical failure, or propagated IMU-ingress overflow stops
publication fail-closed. A new confirmed tracking correction can recover the
path, again only on the following IMU event.

Corrections are monotonic against the last applied correction. Older sequence
numbers, older timestamps, and duplicates are dropped and counted. If a newer
same-generation correction arrives while replay is outside the worker mutex,
the in-flight result is invalidated instead of being committed; the newer
correction remains pending and is the only state eligible for recovery. No
propagated output is published while a correction is pending, replay is in
progress, a generation transition is observed, or re-anchor is required.

## Covariance and deferred integration

The IKFoM covariance is a 23-DoF manifold error-state covariance and cannot be
copied directly into `nav_msgs/msg/Odometry`. P0.5R must project it into pose
error `[delta p_odom_base, delta theta_odom_base]` in `odom` and base linear
velocity error `delta v_base` in `base_link`, including full cross terms. The
velocity result is conditional on the resolved bias-corrected gyro sample at
the output epoch. AIST all-zero gyro covariance and missing REAL per-sample
covariance do not block this projection; angular-rate covariance remains
unavailable unless a later hardware profile supplies it.

PX4 bridging, VehicleOdometry, time synchronization, ENU/FLU to NED/FRD
conversion, and correction smoothing are intentionally deferred. P0.5R
covariance projection is implemented at the shared corrected/propagated ROS
boundary; propagated runtime acceptance remains dependent on the enabled
propagated-output profile.

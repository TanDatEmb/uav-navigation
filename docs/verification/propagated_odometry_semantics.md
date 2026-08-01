# Propagated odometry semantics

`/lio/odometry_corrected` remains the authoritative FAST-LIO output. It is
published only after a successful LiDAR correction and its timestamp remains
the scan end time. `/lio/odometry_propagated` is an optional, higher-rate view
of an independent IKFoM filter propagated with canonical IMU samples.

Both topics express `T_odom_imu`: `header.frame_id` is the configured odom
frame, `child_frame_id` is the configured IMU frame, and linear velocity uses
the same child-frame convention as corrected odometry. Propagated odometry
does not publish TF.

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
history bracket, numerical failure, or propagated event-queue overflow stops
publication fail-closed. A new confirmed tracking correction can recover the
path, again only on the following IMU event.

## Covariance and deferred integration

The IKFoM covariance is a 23-DoF manifold error-state covariance and cannot be
copied directly into `nav_msgs/msg/Odometry`. The propagated message therefore
uses the ROS unknown-covariance sentinel (`covariance[0] = -1`) for pose and
twist. A verified tangent-state projection, including orientation-velocity
cross-covariance and an angular-velocity model, is required before this topic
is suitable as a PX4 external-vision input.

PX4 bridging, VehicleOdometry, time synchronization, ENU/FLU to NED/FRD
conversion, propagated TF, covariance projection, and correction smoothing are
intentionally deferred.

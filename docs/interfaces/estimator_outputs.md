# Estimator outputs

`/lio/odometry` communicates only a corrected pose after a successful LiDAR
update in `Tracking`. It never carries a speculative propagated state, a zero
placeholder, or an initialization state. The header frame is `odom` and child
frame is `base_link`.

`/lio/registered_points` is the current scan after the configured deskew policy
and the corrected state have placed it in `odom`. `/lio/local_map` is a
visualization/debug snapshot of the registration map in the same frame; it has
no free/occupied/unknown semantics.

Diagnostics use `DiagnosticArray` and include the counters and values listed in
the M1 milestone: rates/drops, timing brackets, IMU initialization quality,
deskew mode and failures, residual/convergence data, map insertion/removal, and
estimator lifecycle/correction state. Consumers must use diagnostic status rather
than infer estimator validity from message arrival alone.

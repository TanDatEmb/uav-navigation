# navigation_interfaces

This is a contract-only package. It contains generic navigation services and
validity/covariance masks shared across package boundaries.

It deliberately does not contain sensor drivers or estimator implementation.
The current `SampleOdometryAtTime` service is implemented by
`px4_odometry_bridge` for sampling buffered PX4 odometry across a valid
generation boundary.

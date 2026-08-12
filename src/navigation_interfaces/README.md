# navigation_interfaces

This is a contract-only package. It contains generic navigation services and
validity/covariance masks shared across package boundaries.

It deliberately does not contain sensor drivers or estimator implementation.
The current `SampleOdometryAtTime` service is implemented by
`px4_odometry_bridge` for sampling buffered PX4 odometry across a valid
generation boundary.

`LidarMappingObservation` is the atomic geometric LiDAR observation contract
between `fast_lio_ros` (producer) and `navigation_mapping` (consumer). It
carries a deskewed, common-filtered point cloud and the corrected sensor pose
for the same estimator epoch, plus the active `LioPublicFrameGeneration`
value. It contains no ROG-Map, occupancy, or planner concept; see
`docs/architecture/navigation_layers.md`.

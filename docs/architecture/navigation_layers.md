# Navigation layers

The current product boundary is the estimator and its PX4 integration:

```text
LiDAR + IMU -> FastLioPipeline -> corrected/propagated odometry
                              -> registration map (internal)
                              -> PX4 health-gated external odometry (simulation)
```

The registration map supports scan matching only. It is not an occupancy map,
world model, planning input, obstacle representation, or safety authority.
Planning, world modelling, and flight safety are outside this repository.

The package dependency direction is
`ikfom_vendor/ikd_tree_vendor -> fast_lio_core -> fast_lio_ros -> navigation_bringup`,
with `livox_ros_driver2` supplying the sensor package and custom message,
`fast_lio_tools` consuming the core, and
`navigation_interfaces -> px4_odometry_bridge` defining the PX4 bridge
contract. ROS, Gazebo, bags, PX4 types, and vendor drivers must not enter
`fast_lio_core`.

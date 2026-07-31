# Navigation estimator boundary

The long-term navigation system separates estimator, world model, planner,
safety, and flight-controller integration. This repository currently implements
the estimator layer and its replay/simulation validation tools:

```text
LiDAR + IMU -> FastLioPipeline -> corrected odometry + registered points
                                      -> registration map (odom)
```

The registration map supports scan matching only. It is not an occupancy map,
world model, planning input, obstacle representation, or safety authority.
No NED/FRD or flight-controller conversion is performed in the estimator.

The package dependency direction is `ikfom_vendor -> fast_lio_core ->
fast_lio_ros -> navigation_bringup`, with `fast_lio_tools` also consuming the
core. ROS, Gazebo, bags, and PX4 types must not enter `fast_lio_core`.

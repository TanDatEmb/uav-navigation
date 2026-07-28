# Navigation layers and M1 boundary

The long-term navigation system separates estimator, world model, planner,
safety, and PX4 integration. M1 implements only the estimator layer:

```text
LiDAR + IMU -> FastLioPipeline -> corrected odometry + registered points
                                      -> registration map (odom)
```

The registration map supports scan matching only. It is not an occupancy map,
world model, planning input, obstacle representation, or safety authority.
No NED/FRD or PX4 conversion is performed in M1.

The package dependency direction is `ikfom_vendor -> fast_lio_core ->
fast_lio_ros -> navigation_bringup`, with `fast_lio_tools` also consuming the
core. ROS, Gazebo, bags, and PX4 types must not enter `fast_lio_core`.

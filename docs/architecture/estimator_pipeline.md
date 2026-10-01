# Estimator pipeline

`RosLidarAdapter` and `RosImuAdapter` validate message layouts, units, frames,
and timestamp semantics at the ROS boundary. They convert once to core data
types. `MeasurementSynchronizer` rejects timestamp regressions and creates a
scan interval with IMU brackets. The pipeline initializes IMU gravity/biases,
propagates an IMU trajectory, deskews or explicitly bypasses deskew, preprocesses
points, constructs scan-to-map residuals, performs an iterated correction, and
only then transforms/inserts accepted points in the `odom` registration map.

Public odometry is only emitted during `Tracking` after a successful LiDAR
correction. Initialization, lost, and rejected measurements are diagnostics,
not zero/default odometry.

```text
adapters -> synchronization -> IMU init/propagation -> deskew
         -> preprocessing -> correspondence/residuals -> correction
         -> corrected odometry + odom points -> registration-map insertion
```

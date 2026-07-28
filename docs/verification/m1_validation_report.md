# M1 validation report

**Status:** implementation baseline complete; M1 is not accepted without
sensor/calibration and trajectory evidence.

This report is deliberately not evidence of sensor or flight performance. Complete
each row with command output, dataset/bag identity, configuration revision,
calibration provenance, date, operator, and diagnostic artifacts before declaring
M1 accepted.

| Gate | Evidence required | Current result |
| --- | --- | --- |
| Jazzy workspace build and unit tests | clean `colcon build --symlink-install` and test report | pass on 2026-07-28: seven packages, 80 tests, zero failures |
| Sensor truth | topic/type/rate/unit/frame/timestamp record | pending hardware or validated bridge |
| Frame/extrinsic truth | generated URDF, measured transform and basis checks | blocked: package values are placeholders |
| SIM deskew | simultaneous-scan diagnostics and replay | synthetic bypass test passes; Gazebo replay pending |
| REAL deskew | per-point timing, IMU brackets, synthetic test | translation/yaw synthetic tests pass; validated real input pending |
| Corrected odometry | no initialization/default publications | lifecycle/pipeline tests pass; bag or flight evidence pending |
| Registration map | static/yaw/translation/vertical/square evidence in odom | plane/map/pipeline tests pass; acceptance trajectories pending |
| Offline evaluator | repeatable report generated from same core | executable builds and uses `FastLioPipeline`; representative dataset report pending |

Implementation limitations recorded during this run:

- `IkdTreeRegistrationMap` is currently a deterministic voxel-centroid map
  with exhaustive nearest-neighbor queries, not the upstream ikd-tree backend.
- The manifold state and correction follow the documented
  FAST-LIO2/IKFoM-compatible convention, but upstream IKFoM source is not
  compiled into this repository.
- The default PointCloud2 adapter is built and tested. Selecting
  `livox_custom` is rejected until the optional `livox_ros_driver2` adapter is
  compiled explicitly.
- Public `base_link` odometry currently requires the placeholder
  `base_link`-to-`imu_link` transform to be identity; measured non-identity
  calibration must be applied before real operation.

Known environment limitation: this workspace image does not provide the `xacro`
executable, so runtime URDF expansion has not been run here. XML and launch-file
syntax have been validated. Do not replace this limitation with manual frame
assumptions.

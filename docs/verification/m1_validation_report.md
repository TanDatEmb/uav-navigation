# M1 validation report

**Status:** M1 is not accepted. Simulation-interface and benchmark tooling exist,
but no real Mid-360 capture, calibration, or trajectory evidence has been run.

This report is deliberately not evidence of sensor or flight performance. Complete
each row with command output, dataset/bag identity, configuration revision,
calibration provenance, date, operator, and diagnostic artifacts before declaring
M1 accepted.

| Gate | Evidence required | Current result |
| --- | --- | --- |
| Jazzy workspace build and unit tests | clean `colcon build --symlink-install` and test report | pass on 2026-07-28: 10 packages, 105 tests, zero failures |
| Sensor truth | topic/type/rate/unit/frame/timestamp record | real hardware not run; live simulator bridge unavailable in this environment |
| Frame/extrinsic truth | generated URDF, measured transform and basis checks | blocked: package values are placeholders |
| SIM deskew | simultaneous-scan diagnostics and replay | harness/checker exists; no live Gazebo result recorded |
| REAL deskew | per-point timing, IMU brackets, synthetic test | typed CustomMsg adapter and timing tests pass; blocked on real bag replay |
| Corrected odometry | no initialization/default publications | lifecycle/pipeline tests pass; bag or flight evidence pending |
| Registration map | static/yaw/translation/vertical/square evidence in odom | no live scenario/trajectory evidence recorded |
| Offline evaluator | repeatable report generated from same core | executable and typed runtime adapter build; no raw bag replay report exists |
| ikd-Tree performance | raw benchmark plus host metadata | upstream-only smoke harness passed; not target-hardware evidence |

Implementation limitations recorded during this run:

- Upstream IKFoM and ikd-Tree are compiled and used by production. This is
  source-level and unit-test evidence, not real-sensor acceptance evidence.
- The typed Livox CustomMsg adapter is built and tested from pinned upstream
  message definitions; the hardware SDK/driver executable is not bundled.
- Public `base_link` odometry currently requires the placeholder
  `base_link`-to-`imu_link` transform to be identity; measured non-identity
  calibration must be applied before real operation.

Known environment limitations: this workspace image provides `gz`, but not
`ros_gz_bridge`, and does not provide the `xacro` executable. Runtime Gazebo
launch, ROS bridge and URDF expansion have therefore not been run here. XML,
launch-file syntax and package asset tests have been validated. Do not replace
these limitations with manual frame assumptions.

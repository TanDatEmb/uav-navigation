# M1 Mid-360 dataset validation report

## Direct answers

1. Downloaded dataset: Swarm-LIO2 `mutual_avoidance_uav1.bag` (real Livox
   Mid-360 UAV sequence).
2. SHA-256:
   `43f25ac10deb11f8eed4febe33574b7c7bbc67171b5a93268931e827ad40cf24`.
3. Original LiDAR type: `livox_ros_driver/CustomMsg`.
4. Original IMU type: `sensor_msgs/Imu`.
5. `timebase` and every `uint32 offset_time` plus x/y/z/reflectivity/tag/line
   were copied field-for-field to `livox_ros_driver2/msg/CustomMsg`; no
   PointCloud2 conversion was used. Counts before/after match.
6. Extrinsic: official Swarm-LIO2 `swarm_lio/config/mid360.yaml`, commit
   `a5f751a797bb92baa3104cdd384a312d3c8e7744`;
   `T_imu_lidar.t=[-0.019391,-0.000278,0.080926] m`, identity rotation.
7. Output graph: `odom -> imu_link` dynamic, calibrated
   `imu_link -> lidar_link` static contract. Raw input aliases are `base_link`
   and `livox_frame`; no `base_link` output is fabricated.
8. Initialization completed after the 200-sample stationary collection window,
   before the first map-reference and correction events.
9. Corrections: 1 successful, 1 non-converged, followed by 1,343 timing
   rejections; nine overlapping raw scan intervals and two leading bracket
   failures were also recorded.
10. The single published pose is finite. Full-sequence covariance/runtime
    stability cannot be certified because sustained correction failed.
11. Exported partial map: 3,841 points from exactly one successful corrected
    scan. It is evidence of the failure, not a valid registration map.
12. Map:
    `reports/m1_dataset/swarm_lio2_mutual_avoidance_uav1_release_run_20260728/map_full.pcd`.
13. View: `make dataset-view RUN=swarm_lio2_mutual_avoidance_uav1_release_run_20260728`.
14. Upstream comparison is not computed: the host has no ROS1 Noetic or
    Docker/Podman, and the project trajectory contains only one corrected pose.
15. Playback took 46.61 s for 46.13 s of data (1.0x). Stage p95/p99 is not
    available because the estimator currently lacks per-stage production
    instrumentation and tracking failed.
16. **M1 DATASET VALIDATION: FAIL**.

## Dataset and timestamp evidence

Inspection covers 1,384 LiDAR messages at 30.022 Hz and 8,000 IMU messages at
173.562 Hz. Maximum raw IMU gap is 16,989,504 ns. Header/timebase difference is
within -239..239 ns, so the declared policy is `timebase_authoritative`.

The damaged final bag tail leaves the last scan truncated; that raw scan has one
per-point offset regression. It is preserved in conversion and rejected
fail-closed by production validation. The first two scans lack complete IMU
brackets at the recording boundary. Neither defect is silently repaired.

## Production replay result and root cause

The full converted dataset was replayed at 1.0x through `fast_lio_node`, with
production outputs recorded for all 46.10 seconds. The output bag contains
1,384 diagnostics, one corrected odometry, one registered scan and one local
map. IKFoM was therefore genuinely called and one correction converged; map
insertion occurred only for that successful transaction.

The blocking discrepancy is that the synchronizer reports `IMU gap exceeds
configured synchronization limit` for 1,343 later scans even though the
independent raw inspection maximum is below the configured 20 ms. The exact
observed gap is now included in the synchronizer error for the next diagnostic
run. Raising the threshold was deliberately rejected because it would hide the
defect. This requires a focused synchronization-state regression test and fix
before another acceptance replay.

## Runtime host

- CPU: Intel Core i7-12700H, 20 logical CPUs.
- RAM: 15 GiB.
- OS: Ubuntu 24.04 kernel 7.0.0-28-generic.
- ROS: ROS 2 Jazzy; build: RelWithDebInfo, C++20.
- ROS bag player peak RSS: 97,684 KiB; elapsed: 46.61 s.

These are player measurements, not estimator stage benchmarks, and are not
presented as such.

## Evidence and viewing

- Inspection:
  `reports/m1_dataset/swarm_lio2_mutual_avoidance_uav1_inspection_20260728/`
- Failed production run:
  `reports/m1_dataset/swarm_lio2_mutual_avoidance_uav1_release_run_20260728/`
- Map projections: `map_xy.png`, `map_xz.png`, `map_yz.png`, `map_3d.png`.
- Raw output: `production_outputs_valid/`.
- Direct PCL command:
  `pcl_viewer reports/m1_dataset/swarm_lio2_mutual_avoidance_uav1_release_run_20260728/map_full.pcd`.

The partial map must not be interpreted as evidence that M1 passes. Sanitizer
replay, determinism, full benchmark, and upstream comparison remain failed or
blocked gates because production tracking is not sustained.

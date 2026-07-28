# M1-D2 merge-hardening report

## Verdict

```text
M1-D2 MERGE HARDENING: PASS
```

Dataset SHA-256 remains
`43f25ac10deb11f8eed4febe33574b7c7bbc67171b5a93268931e827ad40cf24`.
No residual, IMU-gap, convergence or noise threshold changed.

## Direct answers

1. **Dense versus compact comparison:** test-only helpers use identical `P`,
   `H`, positive diagonal `R`, innovation, `dx_new`, manifold state and
   convergence limit. They run M=5/22/23/24/50/200, 20 fixed seeds, and four
   conditioning groups. Gain action, `K H`, increment, manifold state,
   covariance, convergence and increment norm are compared.
2. **Maximum error:** gain action 4.91992e-10; `K H` 4.00149e-10; increment
   4.91748e-10; manifold tangent 4.71523e-10; covariance 6.64101e-8 in
   deliberately ill-conditioned cases; norm 3.15416e-10.
3. **Solver:** Eigen `LLT` for SPD covariance and `LDLT` for the information
   system. Failure/non-finite output rejects and rolls back the update.
4. **Canonical direct/ROS config:**
   `src/navigation_estimator/fast_lio_ros/config/mid360_mutual_avoidance_uav1.yaml`.
5. **Config SHA-256:**
   `ee37646f3b4668f13cad1febce5b79dcaa5f896c91709a9ca80740c72dcb0a3e`.
6. **Direct hard-code:** no Mid-360 estimator parameter remains hard-coded in
   `mid360_dataset_runner`; it requires the YAML path on its CLI.
7. **Why 380 overlaps are rejected:** the state is forward-only and M1 has no
   historical pose trajectory covering overlap. Accepting them would require
   backward propagation or duplicate IMU integration.
8. **Buffer acceptance:** 1,384/1,384 = 100.0000%.
9. **Synchronization:** 1,002/1,384 = 72.3988%.
10. **Correction success among attempts:** 973/974 = 99.8973%.
11. **Effective corrected output:** 973/46.1256 s = 21.0946 Hz.
12. **Direct determinism:** yes. Runs C/D have identical trajectory and map
    SHA-256 and both contain 94,970 map points.
13. **ROS 1.0x input:** yes. 8,000/8,000 IMU and 1,384/1,384 LiDAR received and
    core-accepted. Queue HWM 12; no transport loss.
14. **Runtime regression:** best direct hardening p95 is 37.650 ms versus
    34.012 ms (+10.7%); ROS is 41.236 ms versus 32.331 ms. The rerun host was
    under `powersave` governor and load average about 4.3. The difference is
    disclosed; queues remained bounded and all input was processed. No
    threshold or algorithm policy was changed.
15. **Merge readiness:** yes. The three locked deficiencies are closed, the
    full workspace has 128 passing tests, deterministic direct replay passes,
    and final ROS production replay passes.

## Evidence

| Evidence | Location |
|---|---|
| direct deterministic run C | `reports/m1_dataset/m1_d2_hardening_direct_c/` |
| direct deterministic run D | `reports/m1_dataset/m1_d2_hardening_direct_d/` |
| ROS 1.0x acceptance | `reports/m1_dataset/m1_d2_hardening_ros_1x_final_pass/` |
| overlap decision | `docs/adr/ADR-009-overlapping-lidar-scan-policy.md` |
| vendor patch provenance | `src/navigation_estimator/ikfom_vendor/PATCHES.md` |

Direct C/D hashes:

```text
trajectory e30312d128861417549cf9ce2c61fecc78303bfc2c5cf468ba154272a5bdbc83
map        2d5bf6bf5009d755e6016ab44c2682377582b39f4d8f06a5a5bb671035a3579e
```

Final ROS map:

```text
points 94901
frame odom
file reports/m1_dataset/m1_d2_hardening_ros_1x_final_pass/map_full.pcd
```

View:

```bash
make dataset-view RUN=m1_d2_hardening_ros_1x_final_pass
```

## Remaining limitations

M1 intentionally does not implement overlap-aware pose history. ROS1 upstream
reference remains blocked on this host because ROS1 Noetic and a container
runtime are unavailable; the previously documented reference limitation is
unchanged. Pi 5, simulation, PX4, loop closure, online extrinsics and temporal
calibration remain outside this task.

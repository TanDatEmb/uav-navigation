# Remediation Plan — UAV Navigation Migration

This plan aggregates findings from:

- `reviews/review_feature_parity.md`
- `reviews/review_transform_conventions.md`
- `reviews/review_timestamp_sync.md`
- `reviews/review_logging_monitoring.md`
- `reviews/review_critical_assessment.md`

and converts them into executable phases. Each phase has a concrete
output, verification gate, and dependency list.

**Adopted strategic decision**: timestamp synchronization is delegated to
the MicroXRCE-DDS agent (PX4 Scenario A). ROS 2 nodes use the ROS 2 clock
only. Manual PX4 wall-clock offset capture is intentionally **not**
carried forward unless agent sync proves unreliable in SITL.

---

## Phase 0 — Foundation Fixes (no executable nodes yet)

Goal: close the convention and silent-risk gaps before any node
migration starts.

| # | Task | Output | Owner package | Verification |
|---|------|--------|---------------|--------------|
| 0.1 | Implement `QuaternionAircraftToBaselink` / `QuaternionBaselinkToAircraft` and point/vector variants | `px4_ros_com::frame_transforms` helpers + tests | `px4_ros_com` | New unit tests pass |
| 0.2 | Add ENU↔ECEF helpers with map-origin parameter (placeholders OK if not immediately used) | `px4_ros_com::frame_transforms` declarations / stubs | `px4_ros_com` | Compiles; stubs documented |
| 0.3 | Implement covariance transform helpers (`CovarianceToArray`, `CovarianceUrtToArray`, `ArrayUrtToCovarianceMatrix`) | `px4_ros_com::frame_transforms` definitions + tests | `px4_ros_com` | Tests pass |
| 0.4 | Add two-step helpers `Px4ToRosOrientation` and `RosToPx4Orientation` | `px4_ros_com::frame_transforms` helpers + tests | `px4_ros_com` | Tests pass |
| 0.5 | Define frame-ID string constants (`map_ned`, `camera_init`, `base_link`, `aircraft`) | `px4_common::frame::kMapNed`, etc. | `px4_common` | Used in all future nodes; grep enforces |
| 0.6 | Add quaternion-order guard tests that fail if convention changes | `px4_ros_com/test/test_frame_transforms.cpp` additions | `px4_ros_com` | Tests pass |
| 0.7 | Add pose-buffer / timestamped-pose utilities with monotonic invariant + diagnostic counters | `px4_common::time::PoseSample`, `px4_common::time::PoseBuffer` | `px4_common` | Unit tests for monotonic drop + SLERP |
| 0.8 | Add message-version topic helper (optional: replicate legacy `getMessageNameVersion<T>`) | `px4_ros_com::topic::MessageVersionSuffix<T>()` | `px4_ros_com` | Test with a few `px4_msgs` types |
| 0.9 | Create executable-node logging policy document | Update `docs/conventions.md` logging section | `docs` | Reviewed |

**Phase 0 exit gate**: `colcon test` across `px4_common`, `px4_mapping`,
`px4_navigation`, `px4_ros_com` is all green and new helpers are
unit-tested.

---

## Phase 1 — Mapping Layer

Goal: restore the FAST-LIO2 → NED bridge and voxel-map manager from the
legacy repo.

| # | Task | Output | Owner package | Dependencies | Verification |
|---|------|--------|---------------|--------------|------------|
| 1.1 | Migrate `ned_transform.cpp` to new package | `src/px4_mapping/src/ned_transform_node.cpp` | `px4_mapping` | 0.1, 0.5, 0.7 | Builds; runs in SITL |
| 1.2 | Port `LioBuffer` / `PoseSample` to `px4_common::time::PoseBuffer` | Use shared buffer in `ned_transform_node.cpp` | `px4_mapping` + `px4_common` | 0.7 | Tests pass |
| 1.3 | Implement FAST-LIO2 EV publisher with optional initial alignment | `ned_transform_node` publishes `/fmu/in/vehicle_visual_odometry` | `px4_mapping` | 0.1, 0.4 | SITL EKF2 accepts EV aiding |
| 1.4 | Migrate `voxmap_node.cpp` to new package | `src/px4_mapping/src/voxmap_manager_node.cpp` | `px4_mapping` | 0.5, 0.7, 1.1 | Builds; subscribes `/livox_processed_ned` |
| 1.5 | Implement distance-based eviction (15 m radius) and diagnostic counters in `VoxelHashMap` manager | Update `IVoxMapManager` + node | `px4_mapping` | 1.4 | Memory bounded in long SITL runs |
| 1.6 | Publish `/livox_map` pointcloud for downstream nodes and RViz | `sensor_msgs::msg::PointCloud2` publisher | `px4_mapping` | 1.4 | RViz shows map |
| 1.7 | Port sensor configs (`avia`, `mid360`, `horizon`, etc.) | `src/px4_mapping/config/*.yaml` | `px4_mapping` | 1.1 | Launch loads config |
| 1.8 | Add launch file for mapping stack | `src/px4_mapping/launch/mapping.launch.py` | `px4_mapping` | 1.1–1.7 | Launch starts all nodes |

**Phase 1 exit gate**: Mapping stack launches in SITL, publishes
`/livox_map`, and does not drift or crash over a 5-minute flight.

---

## Phase 2 — Navigation Layer

Goal: restore 3D planning + control loop.

| # | Task | Output | Owner package | Dependencies | Verification |
|---|------|--------|---------------|--------------|------------|
| 2.1 | Migrate `uniform_bspline.cpp` and `bspline_optimizer.cpp` | `src/px4_navigation/src/uniform_bspline.cpp`, `bspline_optimizer.cpp` | `px4_navigation` | Phase 0 | Unit tests for B-spline eval |
| 2.2 | Integrate B-spline optimizer with `AStarPlanner` | `Plan()` returns raw A* path, separate `Optimize()` returns B-spline | `px4_navigation` | 2.1 | Fallback to A* if optimizer fails |
| 2.3 | Migrate `navigation3d_controller.cpp` | `src/px4_navigation/src/navigation3d_controller_node.cpp` | `px4_navigation` | 0.5, 0.8, 1.6, 2.2 | Builds; mode handlers compile |
| 2.4 | Port mode state machine (IDLE → MISSION → ANTICIPATION → AVOIDANCE → RETURNING) | Node state machine + safety guards | `px4_navigation` | 2.3 | State transitions logged |
| 2.5 | Port pure-pursuit path follower + repulsion + velocity smoothing | Control loop publishes `trajectory_setpoint` | `px4_navigation` | 2.3 | UAV follows planned path in SITL |
| 2.6 | Port `trajectory_logger.cpp` | CSV logger with same schema | `px4_navigation` | 2.3 | CSV files readable by legacy `visualize.py` |
| 2.7 | Publish `/nav_path` and `/plan_grid_inflated` for RViz | Visualization topics | `px4_navigation` | 2.3 | RViz shows path and grid |
| 2.8 | Add navigation launch file | `src/px4_navigation/launch/navigation3d.launch.py` | `px4_navigation` | 2.1–2.7 | Launch starts controller |

**Phase 2 exit gate**: UAV follows a simple mission in SITL with
obstacle avoidance and B-spline-smoothed paths; `/nav_path` and
`/plan_grid_inflated` are visible in RViz.

---

## Phase 3 — PX4 ROS Bridge and Integration

Goal: robust offboard/health integration.

| # | Task | Output | Owner package | Dependencies | Verification |
|---|------|--------|---------------|--------------|------------|
| 3.1 | Implement offboard mode manager node | `src/px4_ros_com/src/offboard_mode_manager.cpp` | `px4_ros_com` | 0.1, 0.4 | Node arms, switches to offboard, handles failsafe |
| 3.2 | Implement health watcher / heartbeat publisher | Diagnostic topic + liveness checks | `px4_ros_com` | 0.5 | Heartbeat visible; watchdog triggers on missing nodes |
| 3.3 | Add TF publishers (`map_ned` → `base_link`, etc.) | `tf2_ros::TransformBroadcaster` | `px4_ros_com` | 0.1, 0.5 | `tf2` tree valid in RViz |
| 3.4 | Add top-level orchestration launch file | `launch/navigation_stack.launch.py` | workspace root | 1.8, 2.8, 3.1, 3.3 | Single launch starts full stack |
| 3.5 | Migrate offboard control examples | `src/px4_ros_com/src/examples/` | `px4_ros_com` | 3.1 | Examples compile and run |

**Phase 3 exit gate**: Full stack launches with one command, offboard
mode engages, health topic is published, and failsafe behavior is
tested in SITL.

---

## Phase 4 — Visualization, Recording, and Configs

Goal: restore observability and deployment artifacts.

| # | Task | Output | Owner package | Dependencies | Verification |
|---|------|--------|---------------|--------------|------------|
| 4.1 | Migrate `recorder.py` | `src/px4_visualization/px4_visualization/recorder.py` | `px4_visualization` | Phase 3 | Records flight data bag/CSV |
| 4.2 | Migrate `visualize.py` | `src/px4_visualization/px4_visualization/visualize.py` | `px4_visualization` | 4.1 | HTML replay renders |
| 4.3 | Create RViz config | `src/px4_visualization/rviz/navigation.rviz` | `px4_visualization` | 1.6, 2.7 | RViz layout loads |
| 4.4 | Create global config YAML | `config/navigation_stack.yaml` | workspace root | 3.4 | Parameters loaded by launch |
| 4.5 | Add bag-recording helper script | `tools/record.sh` | `tools` | 4.1 | Records essential topics |

**Phase 4 exit gate**: Full flight can be recorded and replayed; RViz
config is committed; global YAML loads without warnings.

---

## Cross-Cutting Requirements

Apply to every phase:

1. **No silent operation**: every executable node must emit a startup
   log and publish a periodic status/heartbeat topic.
2. **Tests**: every new pure function gets a unit test before the phase
   closes. Nodes get SITL smoke tests where feasible.
3. **Code comments in English**; no magic numbers; every parameter
   declared + loaded in the same commit.
4. **Commit only after approval**; push only when requested.
5. **Use parallel subagents for large phases** with independent review
   agents, per project convention.

---

## Suggested Immediate Next Action

Begin **Phase 0.1–0.4** (transform helpers in `px4_ros_com`) first.
They are small, self-contained, unblock every downstream node, and are
the highest-leverage fixes identified in the critical assessment.

After Phase 0 is complete, the dependency graph frees Phase 1 (mapping)
and Phase 2 (navigation) to run mostly in parallel once the shared
buffer and transform helpers are merged.

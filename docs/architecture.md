# Architecture — uav-navigation

## Design Goals

1. **Modular**: each package has one responsibility; dependencies are explicit.
2. **Testable**: pure functions live in `px4_common`; nodes are thin wrappers.
3. **PX4-native**: conventions align with PX4 frames and uORB message semantics.
4. **Simulation-first, reality-ready**: SITL is the default test surface; real-flight parameters are YAML-gated.
5. **Observable**: health, timing, and planning metrics are exposed as ROS 2 topics.

## Package Responsibilities

### `px4_msgs` (submodule)

- ROS 2 equivalents of PX4 uORB messages.
- Tracked from upstream `PX4/px4_msgs`.

### `px4_common`

- geometry, quaternion, NED↔ENU transforms
- interpolation, clamping, grid↔world helpers
- parameter loading helpers
- unit tests for all pure functions

### `px4_mapping`

- LiDAR/IMU odometry (IESKF / FAST-LIO2 successor)
- local occupancy / voxel map
- sensor simulation handlers
- NED bridge to PX4 EKF2 if required

### `px4_navigation`

- `livox_mid360_processor_node`: 2D distance bin (PX4 Collision Prevention) + 1D virtual scan.
- Thư viện: `LocalPlanGrid` (dense 3D grid cho A*), `AStarPlanner` (3D A* trên LocalPlanGrid), `VirtualScan` (1D perception 360 bin, khác với livox_mid360_virtual_scan).
- Ngoài scope lock hiện tại: B-spline optimizer, navigation controller đầy đủ, state machine, map local 3D executable.

### `px4_ros_com`

- **Self-developed** PX4 ↔ ROS 2 bridge and transform helpers.
- Do NOT submodule upstream `PX4/px4_ros_com`; it is ROS 1 legacy and poorly maintained for ROS 2 Jazzy.
- Frame transform publishing (NED↔ENU, body↔baselink).
- Offboard mode manager / health watcher.
- ROS 2 message conversion utilities on top of `px4_common` math.

> Note: `px4_visualization` was removed on 2026-07-08. Visualization helpers
> (RViz configs, plotting, bag workflows) will be added to `px4_ros_com` or
> maintained as external Foxglove/RViz configs when needed.

## Time Synchronization

- **Adopted strategy**: PX4 Scenario A — ROS 2 nodes use the ROS 2 clock
  (`rclcpp::Clock`, `this->now()`, `header.stamp`). The MicroXRCE-DDS agent
  is responsible for translating between PX4 wall-clock (`timestamp_sample`)
  and ROS 2 time when crossing the PX4↔ROS 2 boundary.
- **Consequence**: We do **not** maintain a manual `px4_to_ros_offset`
  inside this workspace. Any node that publishes to PX4 (`vehicle_visual_odometry`,
  `trajectory_setpoint`, etc.) must set `timestamp` and `timestamp_sample`
  from the same ROS 2 clock source as the rest of the pipeline.
- **Exception**: If SITL real-time factor (RTF) deviates significantly from
  1.0 or if agent sync is proven unreliable, revisit the legacy manual-offset
  approach captured in `reviews/review_timestamp_sync.md`.

## Frame Tree

- `map_ned` — local North-East-Down world frame (PX4 default).
- `camera_init` — FAST-LIO2 initialization frame (legacy).
- `odom` / `base_link` — ROS REP-103 body frame, Forward-Left-Up.
- `aircraft` — PX4 body frame, Forward-Right-Down.
- Transforms are implemented in `px4_common::math` and exposed in
  `px4_ros_com::frame_transforms`.

## Output Taxonomy (effective 2026-07-09, chốt để tránh nhầm)

Hệ thống có 5 đầu ra chính. Mỗi đầu ra có tên, frame, producer, mục đích rõ ràng. **Không gộp chúng thành “map local” chung chung.**

| Tên chính thức | Tên cũ (legacy) | Topic | Producer | Loại | Mục đích |
| --- | --- | --- | --- | --- | --- |
| Map global 3D | VoxelHashMap, IVoxMapManager | `/livox_map` | `voxmap_manager_node` | PointCloud2 3D, frame `map_ned` (production) | Toàn bộ voxel map quanh UAV, dùng cho visualize, recorder, và làm đầu vào cho map local 3D. |
| Map local 3D | LocalGridMap (ring buffer) | chưa có topic executable | **chưa có executable node** | 3D occupancy ring buffer (capacity ~50K points) | Bản rút gọn của map global quanh UAV, có eviction theo khoảng cách, cung cấp dữ liệu cho planner 3D. **Hiện chỉ có class ở mức thư viện, chưa có node executable.** |
| Distance bin 2D (PX4 CP) | ObstacleDistance | `/fmu/in/obstacle_distance` | `livox_mid360_processor_node` | 72 bin, body FRD | Dữ liệu cho PX4 Collision Prevention, 2D. |
| Virtual scan 1D | VirtualScan (perception) | `/local_virtual_scan` | `livox_mid360_processor_node` | LaserScan 72 beam, frame `aircraft` | Perception 1D phục vụ debug/RViz, **không phải map**. |
| Visual Odometry (EV) | vehicle_visual_odometry | `/fmu/in/vehicle_visual_odometry` | `ned_transform_node` | PX4 VehicleOdometry | Odometry chính xác từ FAST-LIO2 (Lio + IMU + IEKF) gửi cho PX4 EKF2 để correct drift của PX4 odom nội bộ. |

**Quy tắc đặt tên (cố định):**

1. **Map global 3D** = `/livox_map` toàn scene.
2. **Map local 3D** = bản rút gọn theo khoảng cách quanh UAV, dùng cho planner 3D, **chưa tồn tại executable** trong repo hiện tại.
3. **Distance bin 2D** = `/fmu/in/obstacle_distance` cho PX4 CP, **không phải map**.
4. **Virtual scan 1D** = `/local_virtual_scan`, perception 1D, **không phải map**.
5. **EV** = `/fmu/in/vehicle_visual_odometry`, **là odometry, không phải map**.

## Topic Contract (effective 2026-07-09)

Mỗi topic cốt lõi dưới đây phải giữ đúng frame_id và clock domain theo mode. Thay đổi phải qua decision gate.

| Topic | Producer | Frame (production) | Frame (debug) | Clock domain | QoS | Ghi chú |
| --- | --- | --- | --- | --- | --- | --- |
| `/livox_processed` | `fast_lio2_node` | `camera_init` | `camera_init` | ROS 2 time (sim_time) | reliable | Cloud sau preprocess L1. |
| `/odometry` | `fast_lio2_node` | `camera_init` (parent), `base_link` (child) | giống production | ROS 2 time | reliable | Odom trong world của L1. |
| `/livox_processed_ned` | `ned_transform_node` | `map_ned` | `map_ned` | ROS 2 time | reliable | Hardcode `map_ned`, không phụ thuộc mode. |
| `/livox_map` | `voxmap_manager_node` | `map_ned` (production) | `camera_init` (debug) | ROS 2 time | reliable | Production dùng `input_source=px4_full`; debug có thể dùng `lio_world` nhưng không dùng để đánh giá cuối. |
| `/fmu/in/obstacle_distance` | `livox_mid360_processor_node` | n/a (message frame) | n/a | ROS 2 time | reliable | 72 bin cho PX4 CP. |
| `/local_virtual_scan` | `livox_mid360_processor_node` | `aircraft` | `aircraft` | ROS 2 time | reliable | 72 beam LaserScan, frame FRD. |
| `/fmu/in/vehicle_visual_odometry` | `ned_transform_node` | n/a (PX4 EV message) | n/a | ROS 2 time | best_effort | EV cho PX4 EKF2. |

### Chính sách chung

1. Production dùng `map_ned` cho mọi map/cloud world-frame. Mode debug chỉ dùng `camera_init` khi chạy với `input_source=lio_world` và phải ghi rõ trong report.
2. Clock duy nhất là ROS 2 clock (`this->now()`, `header.stamp`); không giữ manual offset trong node.
3. Mọi topic trong bảng publish bằng `rclcpp::QoS(20).reliable()` trừ khi giao tiếp với PX4 best_effort.
4. Mọi thay đổi contract phải cập nhật đồng thời docs này, YAML config, và SITL analyzer trong cùng commit.

## Data Flow (mục tiêu cuối, sau khi map local 3D có executable)

```text
sensors (LiDAR, IMU, camera, GPS if available)
        ↓
px4_mapping (L1 fast_lio2 + L2 voxmap_manager)
        ↓
Map global 3D (/livox_map) + odometry (/odometry) + EV (/fmu/in/vehicle_visual_odometry)
        ↓
px4_navigation
  ├── livox_mid360_processor_node → Distance bin 2D (/fmu/in/obstacle_distance) + Virtual scan 1D (/local_virtual_scan)
  └── (tương lai) local_map_node → Map local 3D từ /livox_map + /odometry
        ↓
Planner 3D (A* + B-spline) + State machine
        ↓
trajectory_setpoint / offboard_control_mode
        ↓
px4_ros_com (helper transforms, executable bridge/offboard/health watcher khi có)
        ↓
PX4 uORB
```

**Trạng thái hiện tại (2026-07-09):**

1. `px4_mapping`: L1 + L2 + EV đang chạy, đã verify trong SITL.
2. `px4_navigation` hiện có: livox_mid360_processor_node (2D CP + 1D virtual scan), thư viện `LocalPlanGrid` / `VirtualScan` / `AStarPlanner` (chưa có executable).
3. **Map local 3D executable: chưa có**, sẽ thuộc phạm vi Phase sau, không nằm trong scope lock hiện tại.
4. `px4_ros_com` executable bridge/offboard/health watcher: chưa có, chỉ có header-only helpers.

## Configuration Hierarchy

1. **Compile-time** constants in headers (only for truly fixed values).
2. **Package-level YAML** in `src/<pkg>/config/`.
3. **Global YAML** in `config/` for cross-package orchestration.
4. **Runtime overrides** via launch arguments or `ros2 param set`.

## Repository Layout

Standard ROS 2 workspace:

```text
uav-navigation/
├── src/
│   ├── px4_msgs/            # upstream PX4 uORB message definitions (submodule)
│   ├── px4_common/          # shared math, geometry, transforms
│   ├── px4_mapping/         # odometry, local voxel map
│   ├── px4_navigation/      # planner, controller, state machine
│   └── px4_ros_com/         # ROS 2 ↔ PX4 bridge, TF publishers
├── config/                  # global runtime parameters
├── launch/                  # top-level orchestration
├── docs/                    # conventions and architecture
├── tests/                   # integration tests
└── tools/                   # build, format, simulation scripts
```

## Status (cập nhật 2026-07-09)

**Đã chạy được trong SITL (verified bằng analyzer):**

- [x] `px4_common`: math, transforms, parameter helpers, tests.
- [x] `px4_mapping::fast_lio2_node`: cloud preprocess + odom output.
- [x] `px4_mapping::ned_transform_node`: NED bridge, EV publisher.
- [x] `px4_mapping::voxmap_manager_node`: **Map global 3D** ở production mode `px4_full`.
- [x] `px4_navigation::livox_mid360_processor_node`: **Distance bin 2D** (PX4 CP) + **Virtual scan 1D** (debug).
- [x] `px4_ros_com` (header-only helpers): frame_transforms, topic_helpers, tests.

**Có ở mức thư viện, CHƯA có executable node:**

- [ ] `px4_navigation::LocalPlanGrid` / `AStarPlanner` / `VirtualScan` (perception, khác với livox_mid360_virtual_scan).
- [ ] `px4_navigation::LocalGridMap` ring buffer (legacy) — **Map local 3D executable chưa có**.

**Chưa làm, ngoài scope lock hiện tại:**

- [ ] B-spline optimizer.
- [ ] Navigation controller đầy đủ + state machine IDLE/MISSION/ANTICIPATION/AVOIDANCE/RETURNING.
- [ ] `px4_ros_com` executable bridge/offboard/health watcher.

This document is a living draft. Output taxonomy chốt ngày 2026-07-09 để tránh nhầm lẫn giữa map local 3D (chưa có executable) và virtual scan 1D (đã có).

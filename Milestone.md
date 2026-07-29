# HƯỚNG DẪN TRIỂN KHAI MILESTONE M1

## LiDAR–IMU Estimator và Registration Map từ thư mục trắng

---

## 0. Mục đích của tài liệu

Tài liệu này quy định kiến trúc, phạm vi, cấu trúc mã nguồn, quy ước hệ tọa độ, thời gian, trạng thái estimator, topic ROS 2, class, kiểm thử và tiêu chí nghiệm thu cho giai đoạn đầu của dự án UAV Navigation.

Coding agent phải tuân thủ tài liệu này trong quá trình triển khai. Không được tự mở rộng sang obstacle avoidance, occupancy mapping, planning, safety control hoặc PX4 integration khi Milestone M1 chưa hoàn thành và chưa được nghiệm thu.

Mục tiêu cuối của toàn dự án là kiến trúc:

```text
Navigation Core
├── Estimator
├── World Model
├── Planner
├── Safety
└── PX4 Interface
```

Tuy nhiên, phạm vi triển khai hiện tại chỉ gồm:

```text
Stage 0–4
└── Estimator
    ├── đọc LiDAR và IMU;
    ├── kiểm tra timestamp;
    ├── đồng bộ measurement;
    ├── khởi tạo IMU;
    ├── propagation;
    ├── deskew;
    ├── scan-to-map registration;
    ├── iterated Kalman update;
    ├── corrected odometry;
    └── registration map.
```

Không được tạo các package planner, safety, world model hoặc PX4 bridge rỗng để “chuẩn bị trước”.

---

# 1. Mục tiêu Milestone M1

Milestone chính thức:

> **M1 — Reliable LiDAR–IMU Odometry and Registration Mapping**

Pipeline cần đạt được:

```text
LiDAR + IMU
    ↓
timestamp đúng
    ↓
frame và extrinsic đúng
    ↓
đồng bộ measurement đúng
    ↓
IMU initialization đúng
    ↓
IMU propagation đúng
    ↓
SIM bypass deskew hoặc REAL deskew đúng
    ↓
scan-to-map association đúng
    ↓
IKFoM-compatible iterative correction
    ↓
corrected odometry đúng
    ↓
registered points đúng trong odom
    ↓
registration map chồng khớp ổn định
```

Ở cuối M1, hệ thống phải chứng minh được:

* LiDAR và IMU được đọc đúng;
* timestamp không bị sai đơn vị hoặc mất thứ tự;
* extrinsic được áp dụng đúng chiều;
* gravity và bias được khởi tạo hợp lý;
* deskew đúng với dữ liệu thật;
* simulator có đường bypass deskew rõ ràng;
* odometry có chiều chuyển động đúng;
* map không xoay theo UAV;
* map không bị kéo theo yaw;
* point cloud từ nhiều scan chồng khớp trong `odom`;
* có thể replay offline bằng cùng estimator core;
* có diagnostics đủ để xác định lỗi thuộc sensor, timing, frame, deskew, estimator hay map.

M1 không có trách nhiệm tạo bản đồ đẹp để trình diễn hoặc sử dụng cho planning.

---

# 2. Phạm vi bị loại trừ

Agent không được triển khai trong milestone này:

```text
- occupancy voxel map;
- free / occupied / unknown representation;
- ray casting cho planning;
- ESDF hoặc TSDF;
- obstacle inflation;
- collision checking;
- obstacle-distance bin cho PX4;
- dynamic-object tracking;
- local planner;
- global planner;
- trajectory optimizer;
- safety brake authority;
- PX4 mission modification;
- high-rate control output;
- loop closure;
- global localization;
- GNSS map alignment;
- map → odom transform;
- behavior tree;
- plugin manager cho planner;
- generic multi-LiDAR framework.
```

Không tạo abstraction tương lai nếu chưa có implementation và consumer thực tế.

---

# 3. Nền tảng kỹ thuật

Target chính:

```text
OS: Ubuntu 24.04
Middleware: ROS 2 Jazzy
Compute target: Raspberry Pi 5
LiDAR: Livox Mid-360/Mid-360S class
IMU: IMU gắn với hệ thống LiDAR hoặc nguồn IMU đã được xác định
Simulation: Gazebo Harmonic
Language chính: C++20
Build system: colcon + ament_cmake
```

FAST-LIO sử dụng tightly coupled iterated Kalman filtering để kết hợp LiDAR và IMU; FAST-LIO2 đăng ký trực tiếp raw points với incremental map. IKFoM cung cấp nền tảng iterated Kalman filtering trên manifold, tách cấu trúc manifold khỏi system model. Không được tự viết lại các phép toán manifold hoặc tạo một “custom 15-state” mới.

---

# 4. Kiến trúc repository dài hạn

Đây là bản đồ vị trí dài hạn, không phải yêu cầu tạo tất cả ngay:

```text
uav-navigation/
├── src/
│   ├── navigation_estimator/          # Stage 0–4
│   │   ├── ikfom_vendor/
│   │   ├── fast_lio_core/
│   │   ├── fast_lio_ros/
│   │   └── fast_lio_tools/
│   │
│   ├── navigation_world_model/        # Stage 5, chưa tạo
│   │   ├── local_mapping_core/
│   │   └── local_mapping_ros/
│   │
│   ├── navigation_safety/             # Stage 6, chưa tạo
│   │   ├── safety_core/
│   │   └── safety_ros/
│   │
│   ├── navigation_planner/            # Stage 7, chưa tạo
│   │   ├── local_planner_core/
│   │   └── local_planner_ros/
│   │
│   ├── navigation_px4/                # Stage 8, chưa tạo
│   │   └── px4_navigation_bridge/
│   │
│   ├── navigation_interfaces/         # Chỉ tạo khi cần custom message
│   ├── navigation_bringup/
│   ├── uav_description/
│   └── uav_simulation/
│
├── docs/
│   ├── architecture/
│   ├── interfaces/
│   ├── verification/
│   └── adr/
│
└── tools/
```

Trong M1 chỉ được tạo:

```text
src/
├── navigation_estimator/
│   ├── ikfom_vendor/
│   ├── fast_lio_core/
│   ├── fast_lio_ros/
│   └── fast_lio_tools/
├── navigation_bringup/
├── uav_description/
└── uav_simulation/
```

---

# 5. Tạo repository từ thư mục trắng

Cây ban đầu:

```text
uav-navigation/
├── .clang-format
├── .clang-tidy
├── .gitignore
├── README.md
├── LICENSE
│
├── src/
│   ├── navigation_estimator/
│   ├── navigation_bringup/
│   ├── uav_description/
│   └── uav_simulation/
│
├── docs/
│   ├── architecture/
│   ├── interfaces/
│   ├── verification/
│   └── adr/
│
└── tools/
```

Workspace phải build được bằng:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Không đặt toàn bộ project vào một package tên `fast_lio`.

Không đặt Gazebo world, model hoặc scenario trong estimator package.

---

# 6. Package `ikfom_vendor`

Mục đích:

* quản lý dependency IKFoM;
* pin một commit cụ thể;
* lưu license và provenance;
* tránh copy mã IKFoM rải rác;
* cho phép `fast_lio_core` phụ thuộc bằng CMake target rõ ràng.

Cấu trúc:

```text
navigation_estimator/ikfom_vendor/
├── CMakeLists.txt
├── package.xml
├── cmake/
├── vendor/
├── LICENSES/
├── UPSTREAM.md
└── PATCHES.md
```

`UPSTREAM.md` phải ghi:

```text
- repository upstream;
- commit được pin;
- ngày tích hợp;
- license;
- các file được sử dụng;
- khác biệt so với upstream.
```

Không sửa trực tiếp công thức filter mà không ghi trong `PATCHES.md`.

Nếu IKFoM không có release package thích hợp, có thể dùng vendor package hoặc pinned source. Không dùng branch floating.

---

# 7. Package `fast_lio_core`

## 7.1 Trách nhiệm

`fast_lio_core` chứa thuật toán thuần C++:

```text
- core time types;
- geometry và SE(3);
- sensor-domain data structures;
- measurement buffering;
- synchronization;
- IMU initialization;
- IMU propagation;
- pose history;
- LiDAR deskew;
- point preprocessing;
- correspondence search;
- plane estimation;
- residual construction;
- iterated Kalman update;
- registration map;
- local registration-map management;
- diagnostics;
- pipeline orchestration.
```

Không phụ thuộc:

```text
- rclcpp;
- sensor_msgs;
- nav_msgs;
- tf2_ros;
- rosbag2;
- px4_msgs;
- Gazebo;
- launch files.
```

## 7.2 Cấu trúc

```text
fast_lio_core/
├── CMakeLists.txt
├── package.xml
│
├── include/fast_lio_core/
│   ├── common/
│   │   ├── constants.hpp
│   │   ├── result.hpp
│   │   ├── status.hpp
│   │   └── types.hpp
│   │
│   ├── time/
│   │   ├── timestamp.hpp
│   │   ├── duration.hpp
│   │   ├── clock_domain.hpp
│   │   └── timestamp_validator.hpp
│   │
│   ├── geometry/
│   │   ├── frame.hpp
│   │   ├── frame_ids.hpp
│   │   ├── rigid_transform.hpp
│   │   ├── transform_utils.hpp
│   │   └── pose_interpolator.hpp
│   │
│   ├── sensor/
│   │   ├── lidar_point.hpp
│   │   ├── lidar_scan.hpp
│   │   ├── imu_sample.hpp
│   │   └── measurement_group.hpp
│   │
│   ├── synchronization/
│   │   ├── measurement_buffer.hpp
│   │   └── measurement_synchronizer.hpp
│   │
│   ├── initialization/
│   │   ├── imu_initializer.hpp
│   │   └── initialization_result.hpp
│   │
│   ├── estimation/
│   │   ├── manifold_state.hpp
│   │   ├── process_model.hpp
│   │   ├── imu_propagator.hpp
│   │   ├── imu_trajectory.hpp
│   │   ├── iterated_kalman_filter.hpp
│   │   └── convergence_monitor.hpp
│   │
│   ├── deskew/
│   │   ├── deskew_mode.hpp
│   │   ├── scan_deskewer.hpp
│   │   └── deskew_result.hpp
│   │
│   ├── preprocessing/
│   │   ├── point_filter.hpp
│   │   ├── voxel_filter.hpp
│   │   └── point_cloud_preprocessor.hpp
│   │
│   ├── registration/
│   │   ├── correspondence.hpp
│   │   ├── correspondence_search.hpp
│   │   ├── plane_estimator.hpp
│   │   ├── residual_builder.hpp
│   │   ├── residual_gate.hpp
│   │   └── registration_result.hpp
│   │
│   ├── mapping/
│   │   ├── registration_map.hpp
│   │   ├── ikd_tree_registration_map.hpp
│   │   ├── local_map_manager.hpp
│   │   └── map_insertion_policy.hpp
│   │
│   ├── configuration/
│   │   └── estimator_config.hpp
│   │
│   └── pipeline/
│       ├── fast_lio_pipeline.hpp
│       ├── process_result.hpp
│       └── estimator_diagnostics.hpp
│
├── src/
│   └── cấu trúc tương ứng với include/
│
└── test/
    ├── test_timestamp.cpp
    ├── test_duration.cpp
    ├── test_rigid_transform.cpp
    ├── test_frame_conventions.cpp
    ├── test_measurement_buffer.cpp
    ├── test_measurement_synchronizer.cpp
    ├── test_imu_initializer.cpp
    ├── test_imu_propagator.cpp
    ├── test_pose_interpolator.cpp
    ├── test_scan_deskewer.cpp
    ├── test_plane_estimator.cpp
    ├── test_residual_builder.cpp
    ├── test_registration_map.cpp
    └── test_fast_lio_pipeline.cpp
```

---

# 8. Package `fast_lio_ros`

## 8.1 Trách nhiệm

```text
- ROS 2 subscriptions;
- ROS message conversion;
- parameter loading;
- QoS;
- lifecycle và runtime status;
- gọi FastLioPipeline;
- publish corrected odometry;
- publish registered scan;
- publish registration-map snapshot;
- publish diagnostics;
- publish dynamic TF odom → base_link.
```

Node ROS phải mỏng. Không đặt công thức filter hoặc map algorithm trong node.

## 8.2 Cấu trúc

```text
fast_lio_ros/
├── CMakeLists.txt
├── package.xml
│
├── include/fast_lio_ros/
│   ├── fast_lio_node.hpp
│   ├── ros_lidar_adapter.hpp
│   ├── ros_imu_adapter.hpp
│   ├── ros_time_converter.hpp
│   ├── ros_output_publisher.hpp
│   ├── ros_transform_publisher.hpp
│   ├── parameter_loader.hpp
│   └── qos_profiles.hpp
│
├── src/
│   ├── fast_lio_node.cpp
│   ├── ros_lidar_adapter.cpp
│   ├── ros_imu_adapter.cpp
│   ├── ros_time_converter.cpp
│   ├── ros_output_publisher.cpp
│   ├── ros_transform_publisher.cpp
│   ├── parameter_loader.cpp
│   ├── qos_profiles.cpp
│   └── main.cpp
│
├── config/
│   ├── mid360-sim.yaml
│   └── mid360-real.yaml
│
├── launch/
│   ├── fast_lio_sim.launch.py
│   └── fast_lio_real.launch.py
│
└── test/
    ├── test_ros_lidar_adapter.cpp
    ├── test_ros_imu_adapter.cpp
    ├── test_ros_time_converter.cpp
    └── test_parameter_loader.cpp
```

Không tạo các node riêng:

```text
decoder_node
synchronizer_node
deskew_node
filter_node
registration_node
map_node
```

Các thành phần trên phải là class nội bộ vì chúng chia sẻ state và phải chạy theo thứ tự xác định.

---

# 9. Package `fast_lio_tools`

## 9.1 Trách nhiệm

Cung cấp công cụ offline sử dụng cùng `FastLioPipeline`.

```text
fast_lio_tools/
├── CMakeLists.txt
├── package.xml
│
├── include/fast_lio_tools/
│   ├── offline_evaluator.hpp
│   ├── dataset_reader.hpp
│   ├── evaluation_metrics.hpp
│   ├── report_writer.hpp
│   └── evaluation_config.hpp
│
├── src/
│   ├── fast_lio_offline_evaluator.cpp
│   ├── fast_lio_dataset_player.cpp
│   ├── fast_lio_frame_inspector.cpp
│   ├── fast_lio_timing_report.cpp
│   └── fast_lio_map_exporter.cpp
│
└── config/
    └── evaluation.yaml
```

Bắt buộc có executable:

```text
fast_lio_offline_evaluator
```

Input:

```text
- rosbag2 hoặc dataset adapter;
- estimator config;
- optional ground truth;
- optional expected frame/timing metadata.
```

Output:

```text
evaluation/
├── summary.json
├── timing.csv
├── synchronization.csv
├── state.csv
├── trajectory.csv
├── residuals.csv
├── deskew.csv
├── map.pcd
└── report.md
```

Realtime và offline phải dùng chung core:

```text
FastLioNode ─────────────┐
                         ├── FastLioPipeline
OfflineEvaluator ────────┘
```

Không tạo một bản estimator riêng cho offline.

---

# 10. Namespace C++

Dùng thống nhất:

```cpp
namespace uav::nav::lio
```

Các package tương lai:

```cpp
namespace uav::nav::world
namespace uav::nav::safety
namespace uav::nav::planning
namespace uav::nav::px4
```

Không trộn các namespace:

```text
fast_lio
fastlio
lio2
navigation
uav_navigation
```

---

# 11. Hệ tọa độ chuẩn

ROS REP-103 quy định hệ tay phải, đơn vị SI và convention thân robot `x forward`, `y left`, `z up`. REP-105 phân biệt `odom` là frame local liên tục nhưng có thể drift, còn `map` là frame global có thể thay đổi do localization hoặc loop closure.

## 11.1 Cây TF trong M1

```text
odom
└── base_link
    ├── imu_link
    └── lidar_link
```

Không publish:

```text
map → odom
```

trong M1.

## 11.2 `odom`

`odom` là world frame của LIO:

```text
- hệ tay phải;
- z hướng lên;
- liên tục;
- có thể drift;
- không có loop-closure jump;
- không được gọi là global map;
- không được gọi là ENU khi chưa align East/North;
- registration map nằm hoàn toàn trong odom.
```

## 11.3 `base_link`

```text
+x: forward
+y: left
+z: up
```

Origin:

* đặt tại tâm quy ước của UAV;
* ưu tiên gần tâm khối lượng hoặc tâm điều khiển;
* phải được ghi rõ trong URDF;
* không thay đổi tùy package.

## 11.4 `imu_link`

* origin tại IMU sensing origin;
* dữ liệu gyro và acceleration sau adapter được biểu diễn trong frame này;
* orientation phải được chuẩn hóa về FLU;
* estimator state dùng IMU frame làm body state.

## 11.5 `lidar_link`

* origin tại origin point-cloud của LiDAR;
* orientation sau adapter theo FLU;
* raw point của estimator được hiểu là tọa độ trong `lidar_link`.

LiDAR được gắn phía trên, gần tâm UAV:

```text
base_link → lidar_link
x ≈ 0
y ≈ 0
z > 0
```

Tuy nhiên không mặc định translation bằng zero. Giá trị phải được đo và lưu trong URDF/config calibration.

## 11.6 State pose

Estimator core ước lượng:

[
{}^{O}T_I
]

Trong đó:

```text
O = odom
I = imu_link
```

Public ROS odometry có thể xuất `base_link`:

[
{}^{O}T_B
=========

{}^{O}T_I {}^{I}T_B
]

## 11.7 Extrinsic LiDAR–IMU

Dùng:

[
{}^{I}T_L
]

nghĩa là biến đổi point từ LiDAR sang IMU:

[
{}^{I}p
=======

{}^{I}T_L {}^{L}p
]

Point trong world:

[
{}^{O}p
=======

{}^{O}T_I(t),
{}^{I}T_L,
{}^{L}p
]

Không được đảo direction mà vẫn dùng cùng tên.

---

# 12. Quy tắc SE(3)

Ký hiệu duy nhất:

[
{}^{A}T_B
]

Ý nghĩa:

> Biến đổi tọa độ của một point từ frame B sang frame A.

Composition:

[
{}^{A}T_C
=========

{}^{A}T_B {}^{B}T_C
]

Inverse:

[
{}^{B}T_A
=========

\left({}^{A}T_B\right)^{-1}
]

Tên biến:

```cpp
T_odom_imu
T_imu_lidar
T_odom_base
```

Không dùng tên mơ hồ:

```cpp
transform
extrinsic
lidar_pose
imu_to_map
rotation_matrix
```

mà không thể hiện rõ nguồn và đích.

Không truyền `Eigen::Matrix4d` vô danh giữa các module.

Dùng abstraction:

```cpp
RigidTransform
```

hoặc Sophus/IKFoM-compatible type có direction được tài liệu hóa.

Không tự viết lại:

```text
- quaternion multiplication;
- SO(3) exp/log;
- boxplus/boxminus;
- manifold Jacobian;
- quaternion normalization policy.
```

---

# 13. Nơi lưu source of truth cho frame

Ba tầng bắt buộc:

## 13.1 Tài liệu

```text
docs/architecture/frame_conventions.md
```

Phải ghi:

```text
- frame name;
- parent;
- origin;
- axis convention;
- static/dynamic;
- source of transform;
- notation;
- transform direction;
- quaternion ordering;
- map semantics;
- PX4 conversion boundary tương lai.
```

## 13.2 URDF/Xacro

```text
uav_description/urdf/uav_sensor_frames.urdf.xacro
```

Source of truth cho:

```text
base_link → imu_link
base_link → lidar_link
```

## 13.3 Configuration/runtime validation

Frame name được load từ YAML và validate.

Không hard-code topic/frame name rải rác trong code.

---

# 14. NED và FRD

PX4 sử dụng:

```text
NED:
+x north
+y east
+z down

FRD:
+x forward
+y right
+z down
```

M1 không được xử lý NED/FRD.

Tương lai, toàn bộ conversion chỉ nằm tại:

```text
navigation_px4/px4_navigation_bridge/
```

Cấm copy hàm đổi ENU/NED hoặc FLU/FRD vào nhiều package.

Cấm đổi trục bằng các đoạn ad hoc như:

```cpp
out.x = in.y;
out.y = in.x;
out.z = -in.z;
```

Nếu sau này cần conversion, phải có một implementation duy nhất và unit test basis vector, rotation và quaternion.

---

# 15. Timestamp contract

## 15.1 Tuyệt đối không dùng floating point cho timestamp tuyệt đối

Cấm:

```cpp
float stamp;
double lidar_time;
double imu_time;
double scan_start;
```

nếu giá trị là timestamp tuyệt đối.

Dùng:

```cpp
class Timestamp {
public:
    explicit Timestamp(std::int64_t nanoseconds);

    [[nodiscard]]
    std::int64_t nanoseconds() const noexcept;

private:
    std::int64_t nanoseconds_;
};
```

Duration:

```cpp
class Duration {
public:
    explicit Duration(std::int64_t nanoseconds);

    [[nodiscard]]
    std::int64_t nanoseconds() const noexcept;

private:
    std::int64_t nanoseconds_;
};
```

Cho phép dùng `double` cho `dt` cục bộ:

```cpp
const double dt =
    static_cast<double>(duration.nanoseconds()) * 1e-9;
```

## 15.2 Clock domain

```cpp
enum class ClockDomain {
    kRosTime,
    kSimulationTime,
    kSensorTime,
    kSystemTime,
    kSteadyTime
};
```

Không trừ hai timestamp khác clock domain nếu chưa có offset/synchronization model.

## 15.3 Nơi duy nhất đổi ROS timestamp

```text
fast_lio_ros/ros_time_converter.*
```

Cấm lặp lại:

```cpp
stamp.sec + stamp.nanosec * 1e-9
```

trong nhiều file.

## 15.4 LiDAR timing

Core representation:

```cpp
struct LidarPoint {
    Eigen::Vector3f position_lidar_m;
    std::uint32_t relative_time_ns;
    std::uint8_t reflectivity;
    std::uint8_t tag;
    std::uint8_t line;
};

struct LidarScan {
    Timestamp start_time;
    Timestamp end_time;
    std::vector<LidarPoint> points;
    bool has_per_point_time;
};
```

`header.stamp` không được tự động giả định là scan start. Adapter phải xác định rõ semantics theo message type.

## 15.5 IMU timing

```cpp
struct ImuSample {
    Timestamp time;
    Eigen::Vector3d angular_velocity_imu_rad_s;
    Eigen::Vector3d linear_acceleration_imu_m_s2;
};
```

Timestamp là thời điểm measurement được lấy, không phải thời điểm callback được chạy.

---

# 16. SIM và REAL

Không tạo hai estimator khác nhau.

Cùng dùng:

```text
FastLioPipeline
```

Khác biệt chỉ tại sensor adapter và deskew mode.

## 16.1 Simulation

Gazebo hiện tại mô phỏng các tia trong một scan có cùng timestamp. Đây là giới hạn được chấp nhận.

Config:

```yaml
lidar:
  timing_mode: simultaneous_scan
```

Hành vi:

```text
relative_time = 0 cho mọi point
deskew = bypass
scan được xử lý như measurement tức thời
```

Diagnostics:

```text
deskew_status = BYPASSED_SIMULTANEOUS_SCAN
```

Không được:

* tự tạo thời gian giả từ point index;
* dùng curvature làm thời gian nếu dữ liệu không định nghĩa như vậy;
* giả vờ deskew đã được áp dụng;
* thay đổi estimator riêng cho SIM.

## 16.2 Real

Config:

```yaml
lidar:
  timing_mode: per_point
```

Hành vi:

```text
scan start + point relative time
    ↓
IMU bracketing
    ↓
pose interpolation
    ↓
transform point về common scan time
```

Diagnostics:

```text
deskew_status = APPLIED
```

## 16.3 Offline dataset

Cho phép:

```yaml
lidar:
  timing_mode: auto
```

`auto` chỉ dành cho evaluator và development. Production config phải explicit.

---

# 17. State estimator

Không sử dụng hoặc nhắc lại thiết kế “custom 15-state”.

Dùng state tương thích FAST-LIO2/IKFoM:

[
x =
\left(
{}^{O}p_I,
{}^{O}R_I,
{}^{I}R_L,
{}^{I}p_L,
{}^{O}v_I,
b_g,
b_a,
{}^{O}g
\right)
]

| Thành phần    | Ý nghĩa                       |
| ------------- | ----------------------------- |
| ( {}^{O}p_I ) | vị trí IMU trong `odom`       |
| ( {}^{O}R_I ) | orientation IMU trong `odom`  |
| ( {}^{I}R_L ) | rotation từ LiDAR sang IMU    |
| ( {}^{I}p_L ) | vị trí LiDAR origin trong IMU |
| ( {}^{O}v_I ) | velocity IMU trong `odom`     |
| (b_g)         | gyro bias                     |
| (b_a)         | accelerometer bias            |
| ( {}^{O}g )   | gravity trong `odom`          |

File định nghĩa duy nhất:

```text
fast_lio_core/estimation/manifold_state.hpp
```

Tài liệu:

```text
docs/architecture/estimator_state.md
```

Phải ghi rõ:

```text
- nominal state;
- error state;
- covariance dimension;
- manifold của từng thành phần;
- boxplus/boxminus convention;
- thành phần được estimate;
- thành phần cố định;
- initialization policy.
```

## Extrinsic

Extrinsic nằm trong state-compatible model nhưng mặc định:

```yaml
extrinsic:
  estimate_online: false
```

Lý do:

* LiDAR gắn cứng;
* dễ debug frame;
* tránh extrinsic tự trôi;
* tránh che giấu mounting/config sai;
* phù hợp nghiệm thu M1.

Chỉ thử online extrinsic estimation sau khi baseline đúng và có dataset đủ excitation.

---

# 18. Class names bắt buộc

## Core

```text
Timestamp
Duration
LidarPoint
LidarScan
ImuSample
MeasurementGroup

TimestampValidator
MeasurementBuffer
MeasurementSynchronizer

ImuInitializer
ImuPropagator
ImuTrajectory
PoseInterpolator

ScanDeskewer
DeskewResult

PointCloudPreprocessor
PointFilter
VoxelFilter

ManifoldState
IteratedKalmanFilter
ConvergenceMonitor

Correspondence
CorrespondenceSearch
PlaneEstimator
ResidualBuilder
ResidualGate
RegistrationResult

RegistrationMap
IkdTreeRegistrationMap
LocalMapManager
MapInsertionPolicy

FastLioPipeline
ProcessResult
EstimatorDiagnostics
```

## ROS

```text
FastLioNode
RosLidarAdapter
RosImuAdapter
RosTimeConverter
RosOutputPublisher
RosTransformPublisher
ParameterLoader
```

## Tools

```text
OfflineEvaluator
DatasetReader
EvaluationMetrics
ReportWriter
```

Không dùng nhiều tên cho cùng trách nhiệm.

Đặc biệt:

```text
RegistrationMap
```

chỉ là map dùng cho scan registration.

Không gọi nó là:

```text
WorldModel
OccupancyMap
PlanningMap
ObstacleMap
GlobalMap
```

---

# 19. Pipeline runtime

```text
Livox point message
frame = lidar_link
scan ≈ 10 Hz
        │
        ▼
RosLidarAdapter
- validate fields
- convert units
- preserve point time
- validate tag/line
        │
        ▼
LidarScan
        │
        ├─────────────────────────────────┐
        │                                 │
        │                       sensor_msgs/Imu
        │                       frame = imu_link
        │                       target ≈ 200 Hz
        │                                 │
        │                                 ▼
        │                       RosImuAdapter
        │                       - units
        │                       - signs
        │                       - timestamp
        │                                 │
        └──────────────┬──────────────────┘
                       ▼
            MeasurementSynchronizer
            - ordered queues
            - regression detection
            - scan interval
            - IMU bracket
                       │
                       ▼
              MeasurementGroup
                       │
                       ▼
               ImuInitializer
          gravity + biases + quality gate
                       │
                       ▼
                ImuPropagator
       state/covariance + trajectory in scan
                       │
                       ▼
                 ScanDeskewer
       SIM: bypass simultaneous timestamps
       REAL: point-by-point compensation
                       │
                       ▼
           PointCloudPreprocessor
                       │
                       ▼
          Scan-to-map correspondence
              query RegistrationMap
                       │
                       ▼
      IKFoM-compatible iterative correction
                       │
                       ▼
             Corrected manifold state
                  ^odom T_imu
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
convert to base_link       transform points to odom
          │                         │
          ▼                         ▼
 /lio/odometry_corrected       /lio/registered_points
                                    │
                                    ▼
                       RegistrationMap insertion
                                    │
                                    ▼
                           /lio/local_map
```

---

# 20. Topic contract

Topic phải có thể remap từ launch. Không hard-code sensor prefix trong core.

## Input mặc định

```text
/lidar/points
/lidar/imu
```

Nếu dùng IMU ngoài:

```text
/imu/data
```

Input LiDAR hỗ trợ:

```text
livox_ros_driver2/msg/CustomMsg
sensor_msgs/msg/PointCloud2
```

Chọn bằng config:

```yaml
input:
  lidar_topic: /lidar/points
  imu_topic: /lidar/imu
  lidar_message_type: livox_custom
```

Không dùng runtime generic type magic nếu hai adapter rõ ràng dễ kiểm thử hơn.

## Output

```text
/lio/odometry_corrected
/lio/registered_points
/lio/local_map
/lio/diagnostics
/tf
```

### `/lio/odometry_corrected`

```text
type: nav_msgs/msg/Odometry
header.frame_id: odom
child_frame_id: base_link
```

Chỉ publish corrected odometry sau LiDAR correction thành công.

### `/lio/registered_points`

```text
type: sensor_msgs/msg/PointCloud2
frame_id: odom
```

Là scan hiện tại đã deskew và transform vào `odom`.

### `/lio/local_map`

```text
type: sensor_msgs/msg/PointCloud2
frame_id: odom
```

Chỉ phục vụ debug/visualization. Publish ở tần số thấp, ví dụ 1–2 Hz hoặc on-demand.

### `/lio/diagnostics`

```text
type: diagnostic_msgs/msg/DiagnosticArray
```

Chưa tạo custom message nếu DiagnosticArray đủ dùng.

### `/tf`

Dynamic:

```text
odom → base_link
```

Static:

```text
base_link → imu_link
base_link → lidar_link
```

Static TF đến từ URDF/robot_state_publisher hoặc static publisher. FAST-LIO node không publish lặp lại static transform.

---

# 21. Corrected odometry policy

Trong M1 chỉ xác minh corrected odometry.

```text
WaitingForSensors:
    không publish odometry

CollectingImu:
    không publish odometry

InitializingImu:
    không publish odometry

InitializingMap:
    không publish public odometry;
    có thể xuất debug diagnostics

Tracking + successful LiDAR update:
    publish corrected odometry

Degraded:
    chỉ publish nếu status được ghi rõ và policy cho phép

Lost:
    không tiếp tục publish state như hợp lệ
```

Không publish zero odometry trước tracking.

Không publish initial/default state lặp lại.

Không publish high-rate propagated state trong M1.

High-rate propagation chỉ được thêm ở milestone sau khi corrected state đã được kiểm chứng.

---

# 22. Estimator lifecycle

Dùng enum:

```cpp
enum class EstimatorStatus {
    kWaitingForSensors,
    kCollectingImu,
    kInitializingImu,
    kInitializingMap,
    kTracking,
    kDegraded,
    kLost,
    kResetting
};
```

Mỗi transition phải có:

```text
- điều kiện vào;
- điều kiện thoát;
- timeout;
- diagnostic reason;
- counter;
- hành vi output.
```

Không dùng một boolean `initialized`.

---

# 23. Registration map

## 23.1 Semantics

```text
Class: RegistrationMap
Frame: odom
```

Trách nhiệm:

```text
- nearest-neighbor query;
- plane fitting support;
- incremental insertion;
- local map cropping;
- map-point downsampling;
- visualization snapshot;
- scan registration.
```

Không có trách nhiệm:

```text
- biểu diễn free space;
- biểu diễn unknown;
- ray casting;
- collision checking;
- obstacle inflation;
- planning;
- dynamic obstacle removal;
- map beautification;
- loop closure.
```

## 23.2 Insertion

Chỉ insert point khi:

```text
- estimator đang Tracking;
- LiDAR update thành công;
- result hội tụ;
- transform hợp lệ;
- point pass filter;
- map policy cho phép.
```

Không insert scan vào map trước correction.

Không insert khi initialization hoặc update thất bại.

---

# 24. Configuration

Tách SIM và REAL config nhưng giữ cùng schema.

Ví dụ:

```yaml
fast_lio:
  ros__parameters:
    frames:
      odom: odom
      base: base_link
      imu: imu_link
      lidar: lidar_link

    input:
      lidar_topic: /lidar/points
      imu_topic: /lidar/imu
      lidar_message_type: livox_custom

    timing:
      lidar_mode: simultaneous_scan
      max_imu_gap_ns: 20000000
      reject_timestamp_regression: true

    extrinsic:
      estimate_online: false
      translation_imu_lidar: [0.0, 0.0, 0.0]
      rotation_imu_lidar_xyzw: [0.0, 0.0, 0.0, 1.0]

    initialization:
      minimum_imu_samples: 200
      require_stationary: true

    preprocessing:
      minimum_range_m: 0.1
      maximum_range_m: 40.0
      voxel_size_m: 0.2

    registration:
      maximum_iterations: 4

    output:
      publish_registered_points: true
      publish_local_map: true
      local_map_rate_hz: 1.0
```

Các giá trị extrinsic ở trên chỉ là placeholder, không phải calibration chính thức.

Agent phải đánh dấu rõ placeholder và không tuyên bố calibration hoàn tất.

---

# 25. Quy tắc dependency

```text
ikfom_vendor
    ↓
fast_lio_core
    ↓
fast_lio_ros
    ↓
navigation_bringup
```

```text
fast_lio_core
    ↓
fast_lio_tools
```

Cấm:

```text
fast_lio_core → fast_lio_ros
fast_lio_core → px4_msgs
fast_lio_core → Gazebo
fast_lio_core → rosbag2
fast_lio_core → launch
```

---

# 26. Quy tắc clean code

## 26.1 Single source of truth

| Khái niệm                 | Nơi định nghĩa          |
| ------------------------- | ----------------------- |
| Frame semantics           | `frame_conventions.md`  |
| Static transforms         | URDF/calibration config |
| Runtime frame names       | YAML + parameter loader |
| Timestamp type            | `fast_lio_core/time`    |
| ROS time conversion       | `RosTimeConverter`      |
| SE(3) operation           | geometry abstraction    |
| State definition          | `manifold_state.hpp`    |
| Topic names               | config/launch           |
| Message conversion        | ROS adapters            |
| Registration map API      | `registration_map.hpp`  |
| Upstream algorithm source | `UPSTREAM.md`           |

## 26.2 Convert once at boundaries

```text
ROS message
    ↓
ROS adapter
    ↓
core representation theo SI, frame và time chuẩn
    ↓
toàn bộ estimator pipeline
```

Không đổi đơn vị hoặc đổi trục giữa các module core.

## 26.3 Không anonymous transform

Cấm truyền matrix không rõ direction.

## 26.4 Không absolute floating timestamp

Đã quy định bắt buộc.

## 26.5 Không duplicated coordinate conversion

Đã quy định bắt buộc.

## 26.6 Không làm mất traceability upstream

Phần kế thừa FAST-LIO2/IKFoM phải ghi rõ nguồn, commit và thay đổi.

## 26.7 Diagnostics trước optimization

Không tối ưu sớm bằng cách xóa log/metric cần cho debug.

## 26.8 Không catch lỗi rồi tiếp tục với state giả

Lỗi timestamp, transform hoặc synchronization nghiêm trọng phải làm measurement bị reject và diagnostics ghi lý do.

---

# 27. Diagnostics bắt buộc

## Sensor

```text
lidar_message_rate_hz
imu_message_rate_hz
lidar_drop_count
imu_drop_count
timestamp_regression_count
invalid_point_count
```

## Synchronization

```text
scan_start_ns
scan_end_ns
scan_duration_ns
imu_samples_per_scan
imu_gap_max_ns
has_start_bracket
has_end_bracket
sync_rejection_reason
```

## Initialization

```text
samples_collected
gyro_mean
gyro_variance
accel_mean
accel_variance
gravity_norm
initialization_status
```

## Deskew

```text
deskew_mode
deskew_applied
point_time_min_ns
point_time_max_ns
interpolation_failure_count
deskew_runtime_us
```

## Registration

```text
input_point_count
filtered_point_count
query_count
valid_plane_count
accepted_residual_count
rejected_residual_count
residual_rms
iteration_count
converged
```

## Map

```text
map_point_count
inserted_point_count
removed_point_count
local_map_center
map_update_runtime_us
```

## Estimator

```text
status
position
velocity
gyro_bias
accel_bias
gravity
covariance_trace
last_lidar_correction_age
correction_translation_norm
correction_rotation_norm
```

---

# 28. Kế hoạch triển khai theo stage

## Stage 0 — Workspace và sensor truth

### Công việc

```text
- tạo repository và package skeleton;
- build tất cả package;
- tạo ROS adapters;
- đọc LiDAR;
- đọc IMU;
- validate field layout;
- validate rate;
- validate timestamps;
- validate units;
- xuất timing report.
```

### Chưa làm

```text
- filter;
- deskew;
- map;
- odometry.
```

### Gate

Chỉ pass khi:

```text
- topic đúng;
- message type đúng;
- rate hợp lý;
- timestamp đơn điệu;
- point field đúng;
- IMU unit đúng;
- không có conversion float timestamp.
```

---

## Stage 1 — Frame và extrinsic truth

### Công việc

```text
- tạo frame_conventions.md;
- tạo URDF/Xacro;
- xác định base_link;
- xác định imu_link;
- xác định lidar_link;
- load ^I T_L;
- runtime dump transform;
- unit test transform direction.
```

### Test vật lý/simulation

```text
- tiến trước;
- dịch trái;
- nâng lên;
- yaw dương;
- kiểm tra dấu từng trục.
```

### Gate

Không tiếp tục nếu:

```text
- chưa xác định được LiDAR axis;
- chưa xác định được IMU axis;
- extrinsic chỉ là phỏng đoán;
- tồn tại axis swap ẩn trong decoder;
- map/frame name còn dùng lẫn lộn.
```

---

## Stage 2 — Synchronization, initialization và deskew

### Công việc

```text
- measurement queues;
- timestamp regression handling;
- scan interval;
- IMU bracketing;
- IMU initializer;
- propagation;
- pose trajectory;
- SIM bypass;
- REAL per-point deskew;
- offline deskew reports.
```

### Gate

```text
- SIM bypass được ghi rõ;
- REAL path hỗ trợ point time;
- không tạo fake point time;
- IMU bracket đúng;
- propagation không NaN;
- gravity/bias hợp lý;
- deskew có test synthetic.
```

---

## Stage 3 — Iterated estimator

### Công việc

```text
- integrate IKFoM-compatible state;
- process model;
- measurement model;
- correspondence search;
- plane fitting;
- residual gating;
- iterative update;
- convergence diagnostics;
- corrected state.
```

### Gate

```text
- đứng yên không drift bất thường;
- orientation đúng dấu;
- residual có ý nghĩa;
- covariance hữu hạn;
- correction hợp lý;
- không publish odometry giả;
- cùng dữ liệu offline cho kết quả lặp lại.
```

---

## Stage 4 — Registration map

### Công việc

```text
- ikd-tree registration map;
- local-map management;
- transform corrected points vào odom;
- insert sau correction;
- publish registered scan;
- publish local-map snapshot;
- export PCD;
- nghiệm thu alignment.
```

### Gate cuối M1

```text
- static test;
- pure-yaw test;
- translation test;
- vertical-motion test;
- square trajectory test;
- indoor-to-semi-open test;
- offline replay;
- map không xoay/kéo theo yaw;
- point cloud chồng khớp;
- corrected odometry đúng hướng.
```

---

# 29. Bộ test nghiệm thu

## Test A — Đứng yên

Thời lượng:

```text
1–2 phút
```

Kiểm tra:

```text
- yaw không tự quay;
- Z không rơi liên tục;
- velocity gần zero;
- map không tạo nhiều lớp;
- bias ổn định;
- residual không tăng dần.
```

## Test B — Pure yaw

```text
- quay tại chỗ;
- quay về hướng ban đầu.
```

Kiểm tra:

```text
- tường chồng lại;
- cloud không kéo theo yaw;
- tâm quay không dịch lớn;
- yaw đúng chiều.
```

## Test C — Tịnh tiến thẳng

```text
- tiến theo +x base_link;
- quay lại vị trí ban đầu.
```

Kiểm tra:

```text
- odom x đúng dấu;
- y/z không đổi bất thường;
- map không nghiêng.
```

## Test D — Vertical

```text
- nâng UAV;
- hạ UAV.
```

Kiểm tra:

```text
- z odom tăng khi nâng;
- x/y không dịch đáng kể;
- gravity đúng.
```

## Test E — Hình vuông

Kiểm tra:

```text
- relative closure;
- axis swap;
- sign;
- yaw-position coupling;
- drift.
```

## Test F — Bán mở

Di chuyển từ trong nhà ra vùng bán mở.

Kiểm tra:

```text
- estimator không đổi frame;
- geometry degradation được báo;
- map không sụp đột ngột;
- status chuyển đúng nếu chất lượng giảm.
```

---

# 30. Tiêu chí dừng công việc

Agent phải dừng mở rộng thuật toán và báo rõ nếu gặp một trong các vấn đề:

```text
- chưa xác định được timestamp semantics;
- không có per-point time cho REAL nhưng config yêu cầu deskew;
- extrinsic chưa biết;
- LiDAR/IMU frame không xác định;
- state convention không khớp upstream;
- map point bị transform hai lần;
- timestamp regression;
- IMU không bracket scan;
- covariance hoặc state NaN;
- map xoay theo UAV;
- corrected odometry có dấu trục sai.
```

Không được che lỗi bằng:

```text
- axis swap tạm;
- yaw offset thủ công;
- reset map liên tục;
- tắt deskew không báo;
- đổi frame_id chỉ để RViz nhìn đúng;
- publish zero/default odometry;
- tăng filter threshold tùy ý;
- thêm planner/safety để “thử bay”.
```

---

# 31. Tài liệu bắt buộc phải tạo

```text
docs/
├── architecture/
│   ├── navigation_layers.md
│   ├── repository_layout.md
│   ├── estimator_pipeline.md
│   ├── frame_conventions.md
│   ├── timing_contract.md
│   ├── estimator_state.md
│   └── topic_contract.md
│
├── interfaces/
│   └── estimator_outputs.md
│
├── verification/
│   ├── m1_acceptance_criteria.md
│   ├── sensor_validation.md
│   ├── frame_validation.md
│   ├── deskew_validation.md
│   ├── registration_map_validation.md
│   └── offline_evaluation.md
│
└── adr/
    ├── ADR-001-ros-flu-internal-frames.md
    ├── ADR-002-odom-as-lio-world.md
    ├── ADR-003-ikfom-compatible-state.md
    ├── ADR-004-integer-nanosecond-time.md
    ├── ADR-005-sim-simultaneous-scan.md
    ├── ADR-006-corrected-odometry-first.md
    ├── ADR-007-fixed-extrinsic-by-default.md
    └── ADR-008-registration-map-is-not-world-model.md
```

---

# 32. Quy trình commit đề xuất

Không triển khai tất cả trong một commit.

```text
1. chore: initialize ROS 2 workspace and quality tooling
2. docs: define repository, frame and timing conventions
3. feat: add core time and geometry types
4. feat: add ROS LiDAR and IMU adapters
5. feat: add measurement buffering and synchronization
6. feat: add IMU initialization and propagation
7. feat: add SIM and REAL deskew policies
8. feat: integrate IKFoM-compatible estimator state
9. feat: add scan-to-map residual pipeline
10. feat: add registration map backend
11. feat: publish corrected odometry and registered cloud
12. feat: add offline evaluator
13. test: add M1 integration and regression tests
14. docs: publish M1 validation report
```

Mỗi commit phải build và test được.

---

# 33. Definition of Done

M1 chỉ được coi là hoàn thành khi:

```text
[ ] Workspace build sạch trên Ubuntu 24.04 + ROS 2 Jazzy.
[ ] Unit tests pass.
[ ] Không có absolute timestamp bằng float/double.
[ ] Frame convention được tài liệu hóa.
[ ] Static transforms nằm ở một source of truth.
[ ] Không có duplicated ENU/NED conversion.
[ ] Không có custom 15-state.
[ ] IKFoM/FAST-LIO source được pin và traceable.
[ ] SIM simultaneous-scan path rõ ràng.
[ ] REAL per-point deskew path tồn tại và có test.
[ ] Corrected odometry chỉ publish sau correction hợp lệ.
[ ] Không publish zero odometry trước tracking.
[ ] Offline evaluator dùng cùng FastLioPipeline.
[ ] Registered points có frame_id = odom.
[ ] Registration map có frame = odom.
[ ] Pure-yaw test không làm map quay theo UAV.
[ ] Vertical motion đúng dấu.
[ ] Static test không tạo map nhiều lớp bất thường.
[ ] Diagnostics đủ truy nguyên lỗi.
[ ] Không có code planner, occupancy map, safety hoặc PX4 integration.
[ ] Có báo cáo nghiệm thu M1.
```

---

# 34. Chỉ thị cuối cho coding agent

Ưu tiên theo thứ tự:

```text
Correctness
    ↓
Observability
    ↓
Traceability
    ↓
Testability
    ↓
Performance
    ↓
Future extensibility
```

Không ưu tiên “kiến trúc đẹp” hơn khả năng chứng minh dữ liệu đúng.

Không phát triển lớp sau khi lớp trước chưa có bằng chứng nghiệm thu.

Mọi điểm LiDAR cuối cùng được insert vào registration map phải có thể truy vết qua chuỗi:

```text
raw message
→ decoded point
→ timestamp
→ deskew pose
→ LiDAR–IMU extrinsic
→ corrected estimator pose
→ odom point
→ registration-map insertion
```

Nếu không truy vết được chuỗi này, implementation chưa đạt yêu cầu.

Kết thúc công việc tại:

```text
LiDAR + IMU
    ↓
corrected odometry
    ↓
registered point cloud
    ↓
stable registration map in odom
```

Không tiếp tục sang planning cho tới khi có quyết định mới sau nghiệm thu M1.

# uav-navigation — Agent Rules & Standards

**Version:** 2026-07-08  
**Scope:** Toàn bộ workspace `/home/letandat/Dev/uav-navigation`  
**Purpose:** Tài liệu duy nhất agent cần đọc để hiểu quy tắc, tiêu chuẩn và roadmap của dự án. Mọi hành động trên repo phải tuân thủ file này.

---

## 1. Triết lý dự án

- **Modular**: mỗi package một trách nhiệm, dependency explicit.
- **Testable**: pure functions ở `px4_common`; node là thin wrapper.
- **PX4-native**: frame, message semantics, naming theo PX4.
- **Simulation-first, reality-ready**: SITL là mặc định; real-flight parameters được YAML-gate.
- **Observable**: health, timing, planning metrics exposed qua ROS 2 topics.
- **Không tự commit/push**: chỉ commit sau khi Đạt duyệt rõ ràng. Không tự chạy sim/robot nếu Đạt chưa đồng ý.

### 1.1 Scope lock hiện tại (effective 2026-07-09)

- Mục tiêu triển khai hiện tại chỉ gồm:
  - SLAM chính xác cho Livox Mid-360.
  - Xây dựng map global + map local ổn định.
  - Cung cấp odometry/external vision chính xác cho PX4.
  - Tối ưu hiệu năng và độ ổn định pipeline mapping.
- Tạm thời KHÔNG triển khai:
  - B-spline optimizer.
  - Navigation controller đầy đủ.
  - State machine navigation nâng cao.
- Lý do: giảm rủi ro trôi scope, ưu tiên đóng baseline chất lượng cho mapping + odometry trước.

### 1.2 Output taxonomy (khóa ngày 2026-07-09)

Hệ thống có 5 đầu ra chính. Tên gọi cố định, không dùng “map local” chung chung.

| Tên chính thức       | Topic (nếu có)                    | Loại                | Mục đích                                           |
| -------------------- | --------------------------------- | ------------------- | -------------------------------------------------- |
| Map global 3D        | `/mapping/global`                      | PointCloud2 3D      | Toàn bộ voxel map, visualize, recorder.            |
| Map local 3D         | (chưa có executable)              | 3D ring buffer      | Bản rút gọn theo khoảng cách, dùng cho planner 3D. |
| Distance bin 2D      | `/fmu/in/obstacle_distance`       | 72 bin, body FRD    | PX4 Collision Prevention.                          |
| Virtual scan 1D      | `/local_virtual_scan`             | LaserScan 72 beam   | Perception 1D debug, không phải map.               |
| Visual Odometry (EV) | `/fmu/in/vehicle_visual_odometry` | PX4 VehicleOdometry | Odometry cho PX4 EKF2.                             |

**Quy tắc dùng tên:**

1. **Map local 3D** chỉ dùng khi nói về bản rút gọn quanh UAV cho planner. Hiện chỉ có class thư viện, chưa có executable node.
2. **Distance bin 2D** và **Virtual scan 1D** là perception, không phải map. Không gọi chúng là “map local”.
3. **EV** là odometry, không phải map.

---

## 2. Tổ chức repository

```text
uav-navigation/
├── src/
│   ├── px4_msgs/            # submodule upstream PX4/px4_msgs, phải match PX4 firmware
│   ├── px4_common/          # shared math, geometry, transforms, parameter helpers, tests
│   ├── px4_mapping/         # LiDAR/IMU odometry, local voxel map, NED bridge
│   ├── px4_navigation/      # virtual scan, planner, state machine, trajectory follower
│   └── px4_ros_com/         # self-developed PX4↔ROS 2 bridge, transforms, offboard helpers
├── config/                  # global runtime parameters
├── launch/                  # top-level orchestration
├── docs/                    # conventions, architecture, agent rules
├── reviews/                 # review reports và status
├── tests/                   # integration tests
├── tools/                   # build, format, simulation scripts
└── assets/                  # RViz, Gazebo models, worlds
```

> Note (2026-07-08): `px4_visualization` đã bị xóa. Các helper visualization sẽ
> được bổ sung vào `px4_ros_com` hoặc duy trì dưới dạng config RViz/Foxgolve bên
> ngoài khi cần.

### Quy tắc package

| Loại package                              | CMake target                   | Cài đặt bắt buộc                                                                                               |
| ----------------------------------------- | ------------------------------ | -------------------------------------------------------------------------------------------------------------- |
| Header-only (`px4_common`, `px4_mapping`) | `INTERFACE`                    | `install(TARGETS ...)`, `install(DIRECTORY include/ ...)`, `ament_export_targets`, `ament_export_dependencies` |
| Compiled library (`px4_navigation`)       | `SHARED`                       | `install(TARGETS ... ARCHIVE/LIBRARY/RUNTIME)`, `install(DIRECTORY include/ ...)`, exports                     |
| Executable node                           | `add_executable`               | `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`                                                         |
| Python package                            | `ament_python_install_package` | Không commit `__pycache__/` hoặc `*.pyc`                                                                       |

- `buildtool_depend` phải là `ament_cmake` (không `ament_cmake_ros`) trừ khi thực sự cần.
- `px4_msgs` phải là Git submodule, không dùng installed package lung tung.
- **KHÔNG** submodule upstream `PX4/px4_ros_com`. Tự implement bridge trên `px4_common` + `px4_msgs`.

### 2.1 Nguyên tắc "modular nhưng tinh gọn"

- Không mở package/module mới nếu có thể mở rộng an toàn trong module hiện có.
- Mọi thay đổi lớn phải qua decision gate:
  1. Có phục vụ trực tiếp mục tiêu SLAM/map/odom hiện tại không?
  2. Có metric kiểm chứng cải thiện trước/sau không?
  3. Có giảm độ phức tạp vận hành hoặc tăng độ tin cậy thực tế không?
  4. Có tránh được duplicate responsibility giữa các package không?
- Nếu bất kỳ câu trả lời nào là "không", thay đổi phải được hoãn hoặc thu nhỏ phạm vi.

### 2.2 Chính sách tạo file

- Không sinh nhiều file review/notes rời rạc khi có thể cập nhật file hiện hữu.
- File mới chỉ được tạo khi có "ý đồ dài hạn" và owner rõ ràng.
- Với file tạm (debug/experiment), phải có kế hoạch dọn dẹp hoặc hợp nhất.

---

## 3. Coding style

| Thành phần                 | Quy tắc                                                  | Ví dụ                                               |
| -------------------------- | -------------------------------------------------------- | --------------------------------------------------- |
| Class / struct             | `PascalCase`                                             | `LocalPlanGrid`, `ObstaclePerception`             |
| Method / function          | `PascalCase` (đã resolve conflict, theo ROS 2/PX4 style) | `Reset()`, `MarkOccupied()`, `BuildSphericalGrid()` |
| Member variable            | `snake_case_`                                            | `grid_`, `cloud_points_`                            |
| Local variable / parameter | `snake_case`                                             | `yaw_bin`, `dist_horiz_m`                           |
| Namespace                  | `snake_case`                                             | `px4_common::math`                                  |
| File name                  | `snake_case.cpp/.hpp`                                    | `local_plan_grid.cpp`                               |
| `constexpr`                | `kPascalCase`                                            | `kDefaultYawBins`, `kNoObstacle`                    |
| `#define`                  | `SCREAMING_SNAKE_CASE` — hạn chế dùng                    | —                                                   |
| Indent                     | 4 spaces                                                 | —                                                   |
| Braces                     | attach (K&R)                                             | —                                                   |
| Line length                | ≤ 100 ký tự                                              | —                                                   |

### Include style

- Dùng **angle brackets** cho mọi project header: `#include <px4_common/math/transforms.hpp>`.
- Không dùng quotes `#include "px4_mapping/voxel.hpp"` nữa.
- Thứ tự include:
  1. Own header
  2. Standard library
  3. Third-party (Eigen)
  4. ROS 2 / project headers

### CMake

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

- Header-only package dùng `INTERFACE` library.
- Compiled package dùng `SHARED` library.
- Không mix `target_link_libraries` và `ament_target_dependencies` cho cùng target; ưu tiên `target_link_libraries` với `::` syntax.
- Test files **KHÔNG** có `int main()` — `ament_add_gtest` cung cấp.

---

## 4. Comment & documentation

- **Tất cả comment code phải là English.** Không Vietnamese trong code comments hoặc commit messages.
- Public API phải có Doxygen `/** ... */` với `@brief`.
- Không để commented-out debug code trong commit final.
- Giải thích **why**, không chỉ **what**, khi intent không hiển nhiên.
- `TODO`/`FIXME` phải có issue reference hoặc owner.

---

## 5. Parameters & constants

- **Declare → Load → Use**: mọi member mới phải có `declare_parameter` + `get_parameter` trong cùng commit.
- Không magic numbers trong logic; dùng named constants hoặc YAML parameters.
- Ưu tiên typed ROS 2 parameters với descriptors và bounds.
- Mọi parameter phải được validate trong `LoadParameters()` và log warning khi fallback về default.

---

## 6. Frame conventions (KHÔNG được vi phạm)

| Frame             | Định nghĩa                 | Sử dụng                 |
| ----------------- | -------------------------- | ----------------------- |
| `map_ned` (NED)   | X=North, Y=East, Z=Down    | PX4 local world frame   |
| ENU               | X=East, Y=North, Z=Up      | ROS default world frame |
| `aircraft` (FRD)  | X=Forward, Y=Right, Z=Down | PX4 body frame          |
| `base_link` (FLU) | X=Forward, Y=Left, Z=Up    | ROS REP-103 body frame  |

### Transform cơ bản

- **ENU → NED point**: `(x, y, z) → (y, x, -z)` (involution).
- **FRD ↔ FLU rotation**: `diag(1, -1, -1)` passive frame rotation.
- Tên hàm transform phải ghi rõ source→destination: `EnuToNed`, `QuaternionAircraftToBaselink`.

### Quaternion

- **Eigen constructor**: `Eigen::Quaterniond(w, x, y, z)`.
- **Eigen `coeffs()`**: `[x, y, z, w]`.
- **PX4 array**: `[w, x, y, z]`.
- **Helpers**: `EigenQuatToArray()` → `[w,x,y,z]`; `ArrayToEigenQuat()` expects `[w,x,y,z]`.
- **Euler**: ZYX intrinsic (yaw, pitch, roll).
- **Transform direction**: passive — `q_rot * q * q_rot.conjugate()`.

### Frame ID constants

Phải định nghĩa trong `px4_common::frame` và dùng ở mọi node:

```cpp
inline constexpr char kMapNed[] = "map_ned";
inline constexpr char kCameraInit[] = "camera_init";
inline constexpr char kBaseLink[] = "base_link";
inline constexpr char kAircraft[] = "aircraft";
```

---

## 7. Time conventions

- **Single clock source**: ROS 2 clock (`rclcpp::Clock`, `this->now()`, `header.stamp`).
- MicroXRCE-DDS agent đảm nhận dịch `timestamp_sample` (PX4 wall-clock μs) ↔ ROS 2 time. Không maintain manual offset trong node.
- Internal messages dùng `std::chrono::nanoseconds` hoặc `rclcpp::Time`. Tránh mix `double seconds` / `int64_t nanoseconds` trong cùng buffer.
- Pose buffer: enforce strict monotonic timestamps, track non-monotonic / overflow / miss counters.
- Nếu publish lên PX4 (`vehicle_visual_odometry`, `trajectory_setpoint`), `timestamp` và `timestamp_sample` phải từ cùng ROS 2 clock source.

---

## 8. Logging & monitoring

- Node C++ dùng `RCLCPP_INFO/WARN/ERROR`; high-frequency dùng `RCLCPP_*_THROTTLE`.
- Log lifecycle events (start, parameter load, mode transitions) ở `INFO`.
- Log recoverable faults ở `WARN` với counter nếu lặp.
- Log safety-critical / unrecoverable ở `ERROR`.
- **Không để node chạy trong im lặng**: mỗi executable phải có ít nhất một log startup và publish periodic health/status topic.
- Python tooling (`recorder.py`, `visualize.py`) giữ CSV schemas ổn định hoặc document breaking changes.

---

## 9. Control flow & safety

- **Single setpoint / command publish per control cycle.**
- Handle invalid `dt`, missing data, stale input sớm với explicit return/error.
- Failsafe paths phải rõ ràng, documented.
- Mọi node executable phải publish heartbeat/status topic.

---

## 10. Pre-commit checklist

- [ ] New members have declare + load parameter.
- [ ] `compute_*` functions receive correct input type and unit.
- [ ] No Vietnamese comments.
- [ ] No leftover debug logs.
- [ ] Single publish path per cycle.
- [ ] `clang-format` passes (`./tools/format.sh`).
- [ ] `colcon test` passes cho package liên quan.
- [ ] Commit message English, theo Conventional Commits.

---

## 11. Roadmap & phân pha

| Phase | Mục tiêu                                                                                           | Trạng thái tóm tắt                                                    |
| ----- | -------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| **0** | Foundation: transforms, pose buffer, frame constants, logging policy                               | Đang tiến hành — phải đóng trước khi mở rộng sang navigation nâng cao |
| **1** | Mapping: `lidar_odometry`, `localization_bridge`, `global_mapper`, Livox Mid-360 optimization | Trọng tâm chính hiện tại                                              |
| **2** | Navigation nâng cao: B-spline, `navigation3d_controller_node`, state machine                       | Tạm hoãn theo scope lock                                              |
| **3** | PX4 bridge mở rộng: offboard mode manager, health watcher, TF publishers, top-level launch         | Tạm hoãn, chỉ giữ mức cần thiết cho mapping + odom                    |
| **4** | Visualization & recording mở rộng                                                                  | Tạm hoãn, chỉ giữ tooling tối thiểu phục vụ đánh giá SLAM/map/odom    |

### Nguyên tắc thực hiện

1. **Phase 0 trước**: không migrate node mới khi chưa xong foundation.
2. Mỗi phase kết thúc bằng `colcon test` green + SITL smoke test.
3. Dùng subagent song song cho các phần độc lập.
4. Chỉ commit sau khi Đạt duyệt.
5. Scope lock có ưu tiên cao: không thêm feature không phục vụ trực tiếp mục tiêu hiện tại.

---

## 12. Nguồn tham chiếu

- `docs/conventions.md`
- `docs/architecture.md`
- `reviews/project_status_review.md`

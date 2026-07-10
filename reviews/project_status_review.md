# Project Status Review — Scope-Locked SLAM Plan

**Date:** 2026-07-09  
**Reviewer:** AI assistant  
**Workspace:** `/home/letandat/Dev/uav-navigation`  
**Purpose:** Chốt hiện trạng thật, khóa mục tiêu thực tế, và đưa kế hoạch triển khai cụ thể theo hướng tinh gọn.

---

## 1. Scope Lock (áp dụng ngay)

### 1.1 Mục tiêu làm

1. SLAM ổn định và chính xác cho Livox Mid-360.
2. Xây dựng được map global và map local có ý nghĩa vận hành.
3. Cung cấp odometry/EV cho PX4 ổn định, nhất quán frame/time.
4. Tối ưu đường xử lý để chạy được liên tục trong SITL mà không phá kiến trúc.

### 1.2 Mục tiêu chưa làm (intentionally out-of-scope)

1. Không làm B-spline optimizer.
2. Không làm navigation controller đầy đủ.
3. Không làm state machine navigation nâng cao.
4. Không mở rộng tính năng chỉ để "đủ danh sách" khi chưa cần cho mục tiêu SLAM/map/odom.

---

## 2. Hiện trạng kỹ thuật

### 2.1 Đã đạt

1. Pipeline mapping chạy được trong SITL với các mốc chính:
   - M2 NED transform: OK.
   - M3 global map topic có dữ liệu.
   - M4 local virtual scan: đã từ NO_DATA sang OK.
   - M5 visual odometry: có dữ liệu liên tục.
2. Đã có các node vận hành chính cho hướng hiện tại:
   - `lidar_odometry` (adapter/tiền xử lý + odom output)
   - `localization_bridge`
   - `global_mapper`
   - `obstacle_perception`
3. Test package cốt lõi đang xanh ở các lần chạy gần đây.

### 2.2 Chưa đạt / còn lệch mục tiêu

1. Top-level launch stack vẫn ở dạng placeholder, chưa phải orchestrator hoàn chỉnh.
2. `px4_ros_com` còn ở mức helper/interface, chưa có executable bridge/offboard/health watcher.
3. Có độ lệch giữa policy "single-clock/no manual offset" và một số xử lý runtime offset hiện tại trong mapping nodes.
4. M3 semantic frame cần chuẩn hóa rõ giữa `map_ned` và `camera_init` theo mode chạy, tránh báo cáo "OK" nhưng nghĩa đầu ra không đồng nhất.
5. Scope đang có nguy cơ trôi sang bài toán navigation nâng cao, trong khi mục tiêu hiện tại là SLAM/map/odom.

---

## 3. Đánh giá chất lượng theo mục tiêu hiện tại

### 3.1 Mức đạt hiện tại

1. **Functional quality:** đạt mức chạy được và đo được (SITL milestones có kết quả).
2. **Architecture quality:** đạt một phần, còn việc cần chốt để không vi phạm thiết kế.
3. **Product quality theo mục tiêu mới:** tạm đạt mức "usable baseline", chưa đạt mức "long-run robust baseline".

### 3.2 Điểm nghẽn quan trọng

1. Time-domain consistency cần khóa một cách nhất quán trong toàn pipeline.
2. Frame semantics cho map output cần quy ước rõ theo từng mode và phản ánh đúng trong analyzer/report.
3. Chưa có acceptance criteria định lượng ổn định dài hơi (ví dụ 10-15 phút SITL liên tục) cho map/odom.

### 3.3 Acceptance Criteria cho mục tiêu cuối (khóa ngày 2026-07-09)

Mọi lần đánh giá sau này dùng cùng bộ tiêu chí này, không đổi giữa chừng.

| Mốc                     | Mục tiêu đo                                        | Production mode            | Ngưỡng pass (khóa)                                                    | Ghi chú                                                                     |
| ----------------------- | -------------------------------------------------- | -------------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- |
| M1 Collision Prevention | `/fmu/in/obstacle_distance` rate + valid bin ratio | SITL chuẩn                 | rate >= 18 Hz, valid_bins_ratio >= 0.7                                | Trong 10 phút SITL liên tục                                                 |
| M2 NED transform        | `/world/cloud` rate + frame                | SITL chuẩn                 | rate >= 8 Hz, frame = `map_ned`                                       | Frame luôn đúng, không dao động                                             |
| M3 Global map           | `/mapping/global` rate + frame + density                | `px4_full` (chốt ở Step 4) | rate >= 8 Hz, frame = `map_ned`, occupied voxels ổn định theo clutter | Không dùng `lio_world` cho đánh giá cuối vì node tự cảnh báo dev/debug only |
| M4 Local virtual scan   | `/local_virtual_scan` rate + frame                 | SITL chuẩn                 | rate >= 18 Hz, frame = `aircraft`                                     | 72 bins liên tục                                                            |
| M5 Visual odometry      | RMSE EV vs PX4 position                            | SITL chuẩn 10 phút         | RMSE < 2.0 m, mean_error < 1.0 m                                      | Nhiều phiên phải ổn định, không chỉ một phiên tốt                           |

**Chế độ đánh giá chất lượng cuối (production):** SITL chuẩn, 10 phút liên tục, chạy với cấu hình đã chốt, không bật mode dev/debug.

**Chế độ debug (không dùng cho pass/fail):** profile riêng dùng để tìm nguyên nhân khi đánh giá fail.

### 3.4 Số liệu so sánh profile map (Step 4)

Số liệu thu từ 2 cặp phiên SITL, cùng môi trường `obstacle_course`, cùng PX4 SITL. Cặp mới nhất (213740 vs 213918) chạy liên tiếp với cùng điều kiện để so sánh trực tiếp; cặp cũ giữ để có thêm dữ liệu.

| Metric                   | `px4_full` (213740) | `lio_world` (213918) | `px4_full` (212457) | `lio_world` (205108) | Ghi chú                                    |
| ------------------------ | ------------------- | -------------------- | ------------------- | -------------------- | ------------------------------------------ |
| Thời lượng               | ~120 s              | ~120 s               | ~90 s               | ~120 s               | Cặp 213xxx cùng thời điểm, cặp cũ rời rạc  |
| Map global 3D frame_id   | `map_ned`           | `camera_init`        | `map_ned`           | `camera_init`        | `lio_world` không khớp contract production |
| Map global 3D status     | OK                  | FAIL_DEBUG_MODE      | OK                  | FAIL_DEBUG_MODE      | Guard mới ngăn OK giả                      |
| Map global 3D rate       | 8.03 Hz             | 8.32 Hz              | 8.30 Hz             | 9.15 Hz              | đều đạt ngưỡng 8 Hz                        |
| Map global 3D max_voxels | 2166                | 1896                 | 1713                | 1967                 | dao động theo clutter                      |
| Map global 3D avg_voxels | 1231                | 1123                 | 963                 | 1263                 | dao động theo clutter                      |
| Map local 3D executable  | chưa có             | chưa có              | chưa có             | chưa có              | Chỉ có class ở thư viện, ngoài scope lock  |
| Distance bin 2D rate     | 20.06 Hz            | 20.06 Hz             | 19.95 Hz            | 20.04 Hz             | OK cả 4                                    |
| Virtual scan 1D rate     | 20.00 Hz            | 20.00 Hz             | 19.95 Hz            | 20.00 Hz             | OK cả 4                                    |
| M2 frame_id              | `map_ned`           | `map_ned`            | `map_ned`           | `map_ned`            | OK cả 4                                    |
| M2 rate                  | 8.03 Hz             | 8.39 Hz              | 8.35 Hz             | 9.11 Hz              | OK cả 4                                    |
| M5 RMSE                  | 5.93 m              | 6.28 m               | 7.04 m              | -                    | ngoài ngưỡng 2 m, Step 6 sẽ xử lý          |

**Kết luận Step 4:**

1. `px4_full` được chốt là profile production duy nhất. Cả 2 phiên `px4_full` đều OK cho map global 3D, không có regression so với Step 3.
2. `lio_world` giữ lại cho debug, analyzer sẽ fail M3 (map global) với lý do rõ ràng khi chạy nhầm mode.
3. **Map local 3D executable chưa tồn tại** trong repo hiện tại; chỉ có class `LocalPlanGrid` (dense grid cho A\*) và `VirtualScan` (perception 1D, không phải map). Đây là tính năng của scope sau, không nằm trong scope lock hiện tại.
4. `avg_voxels` dao động lớn giữa các phiên (818 / 963 / 1231 cho production), dù cùng scene, nên không dùng làm tiêu chí đánh giá đơn lẻ; cần chuẩn hóa scene hoặc thêm ground-truth trước khi đánh giá chi tiết.
5. M5 RMSE còn cao (~6 m trong 90-120s), nằm ngoài ngưỡng 2 m, Step 6 sẽ xử lý riêng. Việc map global OK với cùng scene cho thấy map output ổn, vấn đề nằm ở chất lượng EV.
6. Không thay đổi tham số runtime ở Step 4, mọi điều chỉnh để map ổn hơn sẽ nằm ở Step 5 nếu cần.

### 3.5 Sai lầm và khắc phục trong quá trình review

Khi trình bày kết quả ở các lượt trước, mình đã dùng “map local” theo nghĩa chung chung, dễ gây hiểu nhầm với **map local 3D** (bản rút gọn quanh UAV cho planner). Sai lầm cụ thể:

1. Đã gọi `/local_virtual_scan` là “map local” — thực ra đây là perception 1D, không phải map.
2. Đã khẳng định “map local có ý nghĩa vận hành” — thực tế chưa có executable node map local 3D.
3. Chưa phân biệt 2D distance bin (PX4 CP) với 1D virtual scan, gộp chung thành “obstacle representation”.

**Khắc phục:**

1. Đã thêm Output Taxonomy trong [docs/architecture.md](docs/architecture.md) và [docs/uav_navigation_rules.md](docs/uav_navigation_rules.md) với 5 đầu ra chính, tên cố định, không dùng “map local” chung chung.
2. Bảng Step 4 ở trên đã chuyển sang taxonomy rõ: map global 3D (đã có), map local 3D (chưa có executable), distance bin 2D, virtual scan 1D, EV.
3. Mọi báo cáo tiếp theo sẽ dùng tên trong taxonomy này.

---

## 4. Kế hoạch công việc cụ thể (không mở rộng sang navigation nâng cao)

## 4.1 Sprint A — Chuẩn hóa semantics (2-3 ngày)

1. Chốt contract frame/time cho từng topic cốt lõi: `/odometry`, `/world/cloud`, `/mapping/global`, `/fmu/in/vehicle_visual_odometry`.
2. Đồng bộ analyzer để phản ánh đúng nghĩa theo mode vận hành (không "OK giả").
3. Đưa tất cả cảnh báo/exception runtime quan trọng về dạng fail-fast có log rõ nguyên nhân.

### Definition of Done A

1. Không còn mâu thuẫn policy-time/frame giữa docs và hành vi runtime trong mode mục tiêu.
2. Báo cáo SITL đọc vào là hiểu ngay output có đúng nghĩa thiết kế hay không.

## 4.2 Sprint B — Tối ưu Livox Mid-360 và ổn định map (3-5 ngày)

1. Tối ưu đường cloud -> grid -> obstacle/map với profiling tối thiểu.
2. Giảm điểm nghẽn lock contention/copy không cần thiết ở callback path nóng.
3. Rà soát thông số mặc định để map ổn định hơn trong môi trường clutter vừa.

### Definition of Done B

1. Giữ được tần số xuất bản mục tiêu ổn định trong SITL chuẩn.
2. Không có cảnh báo stale/timeout lặp bất thường khi tải mô phỏng bình thường.

## 4.3 Sprint C — Hardening cho baseline bàn giao (2-3 ngày)

1. Viết checklist verify ngắn cho mỗi phiên SITL (build, test, smoke run, analyzer).
2. Chốt "known limits" rõ ràng: mode dev/debug nào không dùng cho đánh giá chất lượng cuối.
3. Chỉ giữ lại artifacts cần thiết cho vận hành và review.

### Definition of Done C

1. Có baseline runbook rõ để lặp lại kết quả.
2. Không phát sinh thêm module lớn ngoài scope lock.

---

## 5. Quy tắc tinh gọn codebase (phản biện trước khi mở rộng)

### 5.1 Decision gate trước mọi thay đổi "to"

Một thay đổi chỉ được làm khi trả lời "yes" cho tất cả câu hỏi sau:

1. Có phục vụ trực tiếp mục tiêu SLAM/map/odom hiện tại không?
2. Có thể giải bằng cách mở rộng module hiện có thay vì tạo module/file mới không?
3. Có tiêu chí đo được để chứng minh thay đổi này làm tốt hơn không?
4. Có làm giảm độ phức tạp vận hành hoặc tăng độ tin cậy thật sự không?

Nếu bất kỳ câu nào là "no": hoãn.

### 5.2 Policy tạo file mới

1. Không tạo file review mới nếu có thể cập nhật file review hiện hữu.
2. Không tạo package/module mới cho nhu cầu tạm thời hoặc debug một lần.
3. Với mỗi file mới bắt buộc phải nêu rõ:
   - lý do tồn tại,
   - owner,
   - thời điểm dự kiến gỡ hoặc hợp nhất nếu chỉ là tạm.

---

## 6. Kết luận hành động

1. Dự án đang ở trạng thái có nền tảng tốt để tập trung vào baseline SLAM/map/odom.
2. Hướng tối ưu hiện tại là **đóng chất lượng và nhất quán kiến trúc**, không mở rộng tính năng navigation nâng cao.
3. Mọi quyết định mở rộng phải qua decision gate để tránh phình codebase nhưng tăng giá trị thấp.

---

## 7. Refactor Plan (Khắc phục toàn bộ vấn đề review)

Mục tiêu phần này: xử lý triệt để các vấn đề đã review gồm naming mơ hồ, ranh giới node chưa rõ, topic path thiếu nhất quán, mâu thuẫn clock policy, và chi phí pipeline do truyền dữ liệu qua nhiều stage.

### 7.1 Tiêu chuẩn bắt buộc dùng để thiết kế lại

1. REP-103: thống nhất frame semantics (ENU/FLU trong ROS, NED/FRD khi giao tiếp PX4).
2. REP-2000 và khuyến nghị ROS 2 node design: mỗi node một trách nhiệm chính, tham số có validation.
3. PX4 uORB/ROS bridge convention: `/fmu/in/*` best-effort, timestamp/timestamp_sample đúng domain.
4. Internal policy của repo: scope lock chỉ SLAM/map/EV; không mở rộng navigation nâng cao ở refactor này.

### 7.2 Đích kiến trúc sau refactor (target architecture)

Giữ 4 runtime node cốt lõi, nhưng chuẩn hóa boundary và naming để minh bạch hơn:

1. `lidar_odometry` (đổi tên từ `lidar_odometry`)
   - Trách nhiệm: ingest cloud + px4 odom/imu, tạo cloud L1 + odom L1.
2. `localization_bridge` (đổi tên từ `localization_bridge`)
   - Trách nhiệm: camera_init -> map_ned cho cloud và EV publish cho PX4.
3. `global_mapper` (đổi tên từ `global_mapper`)
   - Trách nhiệm: build/publish map global, eviction, readiness gate.
4. `obstacle_perception_node` (đổi tên từ `obstacle_perception`)
   - Trách nhiệm: ObstacleDistance + virtual scan.

Node `obstacle_distance_visualizer_node` giữ dạng debug-only, off-by-default.

### 7.3 Chuẩn hóa topic path (rõ ràng, có phân tầng)

Không đổi topic PX4 (`/fmu/in/*`, `/fmu/out/*`). Chuẩn hóa internal topic như sau:

| Lớp           | Topic hiện tại               | Topic mục tiêu                | Ghi chú                          |
| ------------- | ---------------------------- | ----------------------------- | -------------------------------- |
| L1 cloud      | `/localization/cloud`           | `/localization/cloud`             | Camera-init cloud sau preprocess |
| L1 odom       | `/odometry`                  | `/localization/odometry`          | Odom camera_init                 |
| World cloud   | `/world/cloud`       | `/world/cloud`          | map_ned cloud                    |
| Global map    | `/mapping/global`                 | `/mapping/global`           | map_ned sparse map               |
| Virtual scan  | `/local_virtual_scan`        | `/perception/scan_1d`   | 72-beam debug/perception         |
| Debug markers | `/visualization/obstacle_markers` | `/visualization/obstacle_markers` | Debug only                       |

Trong 1 sprint chuyển đổi, giữ remap compatibility để không phá launch cũ/analyzer cũ.

### 7.4 Khắc phục mâu thuẫn clock policy

Issue: docs ghi “không giữ manual offset trong node”, nhưng runtime hiện có offset nội bộ.

Quyết định refactor:

1. Chọn một policy duy nhất và chốt bằng code + docs trong cùng commit.
2. Khuyến nghị production:
   - Dùng duy nhất policy agent-synced timestamp tại PX4 boundary.
   - Không giữ manual offset trong node; chuyển đổi chỉ còn `us <-> ns` bằng helper chung.
3. Thay cho offset fallback, thêm runtime guard phát hiện lệch domain timestamp và fail-fast với log rõ nguyên nhân.

### 7.5 Kế hoạch thực thi theo sprint

#### Sprint R0 (0.5 ngày) — Baseline và guardrail

1. Freeze baseline metrics M1-M5 (2 phiên liên tiếp).
2. Snapshot CPU/RAM của 4 node cốt lõi.
3. Add CI check để fail khi frame contract sai ở production mode.

Done khi:

1. Có file baseline metrics trong `reviews/`.
2. Có lệnh chạy benchmark lặp lại được.

#### Sprint R1 (1 ngày) — Naming và API clarity (không đổi hành vi)

1. Rename class/node executable theo taxonomy mục tiêu.
2. Giữ topic cũ qua remap alias để không phá runtime.
3. Cập nhật launch/config/docs/analyzer mapping.

Done khi:

1. Build/test xanh.
2. Runtime output không đổi so với baseline (M1-M5 không regression).

#### Sprint R2 (1-2 ngày) — Topic path migration có tương thích ngược

1. Publish cả topic mới và cũ trong giai đoạn chuyển tiếp.
2. Chuyển analyzer đọc topic mới, fallback topic cũ.
3. Đổi RViz profile sang topic path mới.

Done khi:

1. Session mới chỉ cần topic mới vẫn pass M1-M5.
2. Session cũ vẫn phân tích được bằng fallback.

#### Sprint R3 (1-2 ngày) — Tối ưu latency và giảm copy

1. Bật intra-process communication cho chain L1->world->map.
2. Giảm copy cloud ở callback nóng (chuyển sang move/unique_ptr khi phù hợp).
3. Tắt debug publishers mặc định (`viz markers`, `min_distance_cloud`).

Done khi:

1. CPU path cloud giảm tối thiểu 15% so baseline.
2. Không tăng miss/drop counters ở runtime.

#### Sprint R4 (1 ngày) — Đồng bộ clock policy và offset semantics

1. Chốt policy cuối cho timestamp conversion.
2. Xóa code path trái policy.
3. Cập nhật docs/rules/analyzer cùng commit.

Done khi:

1. Không còn mâu thuẫn giữa docs và runtime.
2. M5 vẫn đạt ngưỡng đã khóa.

#### Sprint R5 (1 ngày) — Dọn kỹ thuật và anti-bloat gate

1. Xóa mode debug không cần thiết khỏi production launch.
2. Gắn decision gate checklist vào PR template.
3. Dọn alias topic cũ sau 1 vòng release nội bộ.

Done khi:

1. Codebase gọn hơn (giảm ít nhất 10% lines ở launch/config glue).
2. Không còn dependency vòng giữa package mapping/navigation.

### 7.6 Ma trận rủi ro và rollback

| Rủi ro                                | Mức        | Cách giảm rủi ro                        | Rollback                                     |
| ------------------------------------- | ---------- | --------------------------------------- | -------------------------------------------- |
| Rename làm vỡ launch/analyzer         | Trung bình | Dùng alias/remap 1 sprint               | Re-enable topic cũ ngay trong launch         |
| Tối ưu copy làm sai frame             | Cao        | Thêm frame assertion ở callback publish | Revert riêng commit tối ưu, giữ naming/topic |
| Thay đổi clock policy làm M5 xấu đi   | Cao        | Chạy A/B 10 phút trước merge            | Giữ policy cũ và cập nhật docs theo thực tế  |
| Tắt debug topic làm khó debug lỗi mới | Thấp       | Off-by-default nhưng có flag bật nhanh  | Bật lại bằng runtime param                   |

### 7.7 Tiêu chí nghiệm thu cuối refactor

1. M1-M5 pass với production mode trong 10 phút liên tục.
2. Topic path thống nhất theo taxonomy mới, không còn tên mơ hồ.
3. Không còn mâu thuẫn docs-runtime về timestamp policy.
4. CPU hoặc độ trễ pipeline cải thiện đo được so baseline.
5. Không mở rộng ngoài scope lock (không thêm controller/state machine/B-spline).

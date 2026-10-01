# KB-02: Các lớp: chức năng, nhiệm vụ, nhu cầu thông tin

Chi tiết từng class nằm ở `areas/R*_layer.md`, là báo cáo của reviewer theo từng vùng. Tài liệu này là bản hợp nhất để dùng khi thiết kế.

---
## L0: Common / Contracts
**Chức năng:** định nghĩa kiểu dữ liệu và bất biến dùng chung: msg ROS, identity, thời gian, frame, mission schema, world-model query contract, planning request/outcome, CandidateBundle.

**Nhiệm vụ:**
- `navigation_common`: ma trận ENU↔NED, FLU↔FRD; chuyển đổi thời gian có kiểm tra; SPSC queue.
- `navigation_contracts`: 9 msg control (`NavigationCommand`, `NavigationGoal`, `ModeStatus`, `CommandAdmission`, `MissionProgress`, `PropagatedOdometry`, `RegisteredScan`, `EstimatorHealth`, …), cùng các header predicate: `assessCommandContract`, `evaluateExecutionStateFreshness`, `kCommandAnchorErrorLimitM = 0.75`.
- `navigation_planning`: `PlanningRequest/Key/Budget/TimingContract` (10 Hz / 50 Hz / 0.10 s / 5 s), `CandidateBundle`.
- `navigation_world_model`: `CellState`, `UnknownPolicy`, `WorldSnapshotIdentity`, continuous clearance, `WorldCommitAuthorizer`.
- `navigation_mission`: mission YAML là owner duy nhất của schema và của UNKNOWN policy.

**Nhu cầu thông tin:** không có. Lớp này chỉ là định nghĩa.

**Lỗi lớp:**
- Header chứa logic nặng: `CandidateBundle::valid`, role schedule, và loader tracking-experiment có phụ thuộc ROS (R5-36).
- Bundle có hai biểu diễn thời gian, double s và int ns (N3, R5-33).
- `NavigationCommand` không ghi rõ frame và đơn vị (R5-39).
- Tolerance bị lặp (N4, R5-34).
- Hai hàm đổi giây sang ns làm tròn khác nhau: truncation và rounding (R5-01, time.hpp).

**Giữ:** identity typed, checked time, ma trận frame tập trung một chỗ.
**Bỏ hoặc chuyển:** logic trong header chuyển sang unit đã biên dịch có test; header chỉ còn data và bất biến nhỏ.

---
## L1: Estimation (FAST-LIO)
**Chức năng:** nguồn state điều hướng **duy nhất**, là IKFoM iterated ESEKF 23-DoF (LiDAR + IMU).

**Nhiệm vụ:**
1. Đồng bộ IMU/LiDAR: bracket IMU theo scan, gate khoảng hở IMU.
2. Khởi tạo: gravity, bias, prior.
3. Predict, deskew, lọc điểm, correct point-to-plane (tối đa 4 vòng), cập nhật ikd-tree.
4. Máy trạng thái lifecycle: WaitingForSensors → … → Tracking / Degraded / Lost.
5. Propagator IMU tần số cao (50 Hz): re-anchor và replay mỗi lần có correction.
6. Publish:
   - `/lio/odometry_propagated` (kèm epoch, sequence);
   - `/lio/odometry_corrected`;
   - `/lio/mapping_observation` (RegisteredScan kèm sensor origin và các endpoint visibility);
   - `/lio/health` (typed);
   - `/lio/diagnostics` (chuỗi);
   - TF.

**Nhu cầu thông tin:**

| Input | Nguồn | Frame | Clock | Bound | Authority |
|---|---|---|---|---|---|
| IMU | `/lidar/imu` | livox_imu_frame FLU | `timing.clock_domain` (sim) | gap ≤ 0.02 s. Có **3 ngưỡng độc lập** (R4-13) | duy nhất |
| LiDAR | `/lidar/points` | livox_frame | cùng domain với IMU | degraded 0.2 s / lost 1.0 s **theo wall clock** (R4-22) | duy nhất |
| Extrinsic IMU→LiDAR | YAML | ^imu T_lidar | — | tĩnh, bị đóng băng | có **2 bản** (trong filter, và YAML ở output publisher) |
| base→IMU | `/tf_static` | ^base T_imu | — | đọc một lần | TF |
| Visibility cloud | topic | livox_frame | stamp phải bằng stamp scan | chờ 10 ms (R4-07) | tuỳ chọn |
| Prior | zero (sim) / topic | lio_odom ← base_link | — | age ≤ 0.5 s | YAML policy |

**Output và quyền:** là authority của localization epoch. **Nhưng** epoch không bao giờ tăng trong suốt vòng đời process, vì node không gọi `reset()` (R4 layer, R4-01/12/16).

**Giữ:**
- `Timestamp` + `ClockDomain`;
- predict/correct có rollback;
- Jacobian đã được xác minh;
- bàn giao correction có guard generation;
- từ chối parameter không biết.

**Sửa về cấu trúc:**
- `/lio/health` chỉ một owner và một clock (R4-06);
- có đường reset/relocalize kèm epoch bump (R4-01, R4-12, R4-16);
- một object duy nhất cho chính sách continuity IMU (R4-13);
- supervision chạy theo sensor clock (R4-22);
- đưa publish và map maintenance ra khỏi đường correction (R4-07, R4-18);
- publish acceleration đã hiệu chỉnh bias, để runtime khỏi phải sai phân hữu hạn lại.

**Thừa:**
- hai producer health;
- merge history IMU cho simultaneous scan (R4-14);
- ba ngưỡng gap;
- `DeskewMode::kAuto` (node từ chối nó);
- `normalizeConfig()` no-op.

---
## L2: Mapping / World model
**Chức năng:** owner **duy nhất** của occupancy map mutable, đồng thời là nơi sinh ra snapshot world bất biến mà planner pin vào.

**Nhiệm vụ:**
- Kiểm tra observation.
- Log-odds hit/miss có kẹp giá trị.
- Raycast DDA cho miss, kể cả endpoint không có return.
- Sliding window 1.5 m.
- Inflation.
- Export snapshot (full hoặc patch).
- Identity: epoch / generation / revision / stamp.
- Change history (256 bản ghi).
- `WorldSnapshotStore` với gate publication.
- Query cho planner và execution: classify, `isSegmentTraversable` (supercover + tube clearance), `nearestNotOccupied`, `observedOccupiedPoints`.

**Nhu cầu thông tin:**

| Input | Nguồn | Frame | Clock | Bound | Authority |
|---|---|---|---|---|---|
| Registered cloud + pose + sensor origin + endpoints no-return | RegisteredScan qua runtime | lio_odom | ROS ns; stamp pose = stamp cloud | 0.5 s, kiểm lúc submit và lúc dequeue | estimator |
| Localization epoch | runtime | — | đơn điệu | epoch mới ⇒ rebuild map | runtime |
| Config | `planner.yaml` mục `rog_map` | — | startup | **thiếu key thì default âm thầm** (R6-20) | config |

**Giữ:** actor single-owner, snapshot bất biến, change-history certificate, worker latest-only, các bản sửa của dự án trên vendor (index checked, guard DDA, sensor origin tách riêng).

**Sửa:**
- Unknown chỉ một seed (R6-02).
- `GridType` ↔ `CellState` qua bảng map tường minh (R6-11).
- Outcome sạch không được poison actor (R6-04).
- Tube clearance phải định nghĩa rõ quantization của voxel (R6-13).

**Thừa:**
- Cơ chế patch snapshot (chuỗi depth 8, 7 lý do full export) gần như không bao giờ thắng (R6-09).
- `MappingWorldModelView` chỉ tests dùng.
- Virtual plane: tắt trong product nhưng mang 4 predicate lệch nhau (R6-01/03/07).
- ESDF / frontier.

---
## L3: Planning (planner_core + traj_opt)
**Chức năng:** nhận một `PlanningRequest` bất biến, trả về tối đa một `CandidateBundle` MAIN(+BACKUP) hoàn chỉnh, hoặc một failure typed.

**Nhiệm vụ:**
- Resolve goal: chiếu điểm STOP vào quả cầu acceptance.
- A* guide 26-connected.
- Corridor CIRI.
- MINCO nominal (L-BFGS kèm retry ladder).
- Yaw quintic.
- BACKUP braking (seed tất định, backward switch search).
- Certificate phía producer: yaw, flatness, route regression, swept world.
- Stage / export candidate.
- Heading rebind.
- Emergency brake.

**Nhu cầu thông tin:**

| Input | Nguồn | Frame | Clock | Bound | Authority |
|---|---|---|---|---|---|
| Start state (P,V,A,J,q,yaw) | runtime request | ENU | source ns → double s | planner không kiểm | measured. A/J là ước lượng |
| Execution anchor | ExecutionAuthority | ENU | int ns, so với `getSimTime()` double | phải nằm ở tương lai | execution |
| World view pin | request; bản mới nhất khi authorize | ENU | observation ns | pin theo từng request | world |
| Route snapshot | mission | ENU | revision | bất biến | mission |
| Dynamics | request so với `cfg_` | — | — | phải bằng `back_traj_cfg` | `cfg_`; request chỉ được hạ cruise (**R1-13 lệch**) |
| Deadline / cancel | runtime | — | steady ns | cứng | runtime |
| Giới hạn tilt PX4 | **không có** | — | — | — | **không ai** (R2-13) |
| Tham số flatness | YAML | — | — | default âm thầm g = 1, m = 1 (R2-16) | config |

**Giữ:** một request cho một outcome; activation do execution sở hữu; toán polynomial / MINCO / Sturm; seed BACKUP tất định; deadline kép có steady làm authority; closure capture theo value.

**Sửa:**
- Tách `Planner` god object thành các stage thuần.
- Một nguồn cho nominal speed (R1-13).
- Lý do reject typed (R1-40).
- SimplifySFC phải có bound (R2-24).
- Thêm tilt certificate và margin giữa các mẫu flatness (R2-11/13).
- Sửa PM allocator (R2-17).

**Thừa:**
- Retry ladder 8 bậc.
- `BackupTrajOpt` (933 LOC, đang tắt).
- MINCO S2/S3 chết.
- Hai bản builder heading rebind (R1-08).
- JSON snapshot writer nằm trong TU của optimizer.
- API facade legacy (R-06).

---
## L4: Execution / Runtime
**Chức năng:** điều phối vòng autonomy trên máy bay, và là writer **duy nhất** của `NavigationCommand`.

**Nhiệm vụ:**
- Nhận scan: MappingWorker, sau đó revalidate bundle và publishAndFinalize.
- Nhận state: lease trong `ExecutionStateStore`.
- Goal / mission: `DesiredPlanningIntent`, `MissionProgress`.
- Lập lịch planner 10 Hz: `currentPlanningKey` → `PlanningWorker` → `runCycle`.
- Commit candidate (16 kiểu reject).
- Retained validation: tracking certificate, emergency, BACKUP, fail-closed.
- Command 50 Hz: sampler, lease, stopped-hold, publish.
- Watchdog.
- Diagnostics.

**Nhu cầu thông tin:** xem R3 §Information needs. Các input chính:
- RegisteredScan (0.5 s);
- PropagatedOdometry (0.5 s; ROS-only ở chỗ commit, ROS + steady ở chỗ command: **R3-03 lệch**);
- EstimatorHealth (chỉ để lấy epoch);
- ModeStatus (literal 200 ms);
- CommandAdmission;
- World snapshot (0.5 s source age);
- Bundle.

**Giữ:**
- ExecutionAuthority là authority duy nhất;
- validation nặng chạy ngoài lock, lock chỉ để re-check identity;
- lease hai clock ở chỗ lộ command;
- id đơn điệu;
- giao thức drain khi reset epoch;
- predicate thuần có bộ test lớn.

**Sửa:**
- Một `TrackingAssessment` duy nhất (R3-09/10/11).
- Planner chỉ truy cập qua một cửa (R3-17, R3-01).
- Publish ngoài lock (R3-14).
- Không bỏ tick command khi lease mới đến (R3-15).
- Decode route một lần (R3-13).
- Fault injection và decision trace (khoảng 400 field mỗi solve) chuyển sang observer.

**Thừa:**
- 4 mô hình tracking acceptance;
- khoảng 20 bản inline của cùng một check "identity còn hiện hành";
- 7 tham số injection nằm trong product.

---
## L5: PX4 boundary
**Chức năng:** biên cuối trước PX4:
- NavigationCommand ENU → `TrajectorySetpoint` NED, cùng PX4 Hold handover (adapter);
- LIO → `vehicle_visual_odometry` (EV bridge);
- PX4 → ROS evidence (ingress bridge).

**Nhiệm vụ:**
- Admission command: contract, lease, session identity, sample order, tracking envelope.
- `updateSetpoint` 50 Hz: các gate airborne / odometry / health / stale, rồi PVA, velocity-only hoặc hold.
- Căn frame LIO → PX4: **chỉ tịnh tiến** (R5-17).
- EV bridge: identity, conversion, variance, timestamp µs, jump continuity + latch, gate.
- Ingress: timestamp validator, reset compensator, ring buffer.

**Nhu cầu thông tin:**

| Input | Nguồn | Frame | Clock | Bound | Authority |
|---|---|---|---|---|---|
| Odometry LIO (adapter) | `/lio/odometry_propagated` | ENU / FLU twist | ROS source + steady receive | 0.2 s | epoch từ health |
| Health | `/lio/health` | — | 2 cơ sở stamp (R4-06) | 0.2 s | epoch |
| PX4 local position | `/fmu/out/vehicle_local_position` | NED | µs PX4 | 0.2 s steady | reset counter |
| Command | runtime | ENU (không ghi trong msg) | header + `valid_until` | 0.1 s | mode_activation_id, goal, world |
| Odometry cho EV | `/lio/odometry_propagated` | ENU / FLU | ROS → µs | 0.5 s | (epoch, sequence) |
| Generation cho reset_counter | `/lio/diagnostics` **chuỗi** | — | ROS | 2 s | **nguồn thứ hai** (R5-09) |

**Giữ:**
- chuyển đổi odometry sang PX4 (đúng toán);
- bridge không tự áp offset timesync;
- latch chỉ được xoá bởi generation mới hơn hẳn;
- thread state-input riêng, join trước khi giải phóng.

**Sửa:**
- Một mô hình căn frame SE(2) được latch và kiểm heading (R5-17/18).
- Small-dt không được reseed ở trạng thái trusted (R5-08).
- Generation lấy từ msg typed (R5-09).
- Envelope dạng norm (R5-16).
- Bỏ `mutable` / `const` giả (R5-26).

**Thừa:**
- witness velocity-only tự chứng nhận (R5-20) và branch emergency đã chết (N6);
- `MissionController` deprecated nhưng vẫn build (R5-24);
- `ReferencePointConverter` không dùng.

---
## L6: Qualification (runner / judge / config)
**Chức năng:** điều phối SITL và replay, thu evidence, chấm PASS / FAIL / NOT_EVALUABLE. **Không** phải flight authority, nhưng là authority **duy nhất** cho một kết luận PASS.

**Nhiệm vụ:**
- runner: resolve scene / profile / world / mission, sinh param, khởi động 14 role, chờ readiness.
- scenario: FSM arm / takeoff / activate, sự thật va chạm từ ground truth, watchdog localization.
- monitor: tần số, khoảng hở, stale.
- report / evaluation: verdict và các trục C0.
- static guards.

**Nhu cầu thông tin:**
- ground truth: pose ENU; twist ở **body frame** (R7-25);
- sản phẩm: `/lio/*`, `/navigation/*`, `/fmu/*`;
- registry `map_profiles.yaml`;
- `resolved_mission.yaml`;
- log witness của config hiệu lực.

**Giữ:** param theo session với witness "requested vs effective"; một nguồn mission; session process-group; cô lập DDS / GZ.

**Sửa:**
- Velocity body-frame (R7-25).
- Một hàm percentile duy nhất (R7-23).
- Cross-track chỉ một owner (R7-04/41).
- Freshness theo sim time (R7-39).
- Mode tracking mặc định của runner (R7-16).
- Guard dạng AST hoặc unit test thay cho regex/`assert` (R7-10/11/12).
- Collision check swept (R7-32).
- Bỏ mọi chỗ dẫn xuất lại ngữ nghĩa C++ (V7, R7-31) bằng cách dùng evidence typed.

**Thừa:** 3 map alias profile → world, các bảng per-profile trong runner, REPORT.html tự tính verdict riêng (R7-41).

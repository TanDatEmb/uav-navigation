# WP-A2 — ICD hiện tại: topic, msg, field, param, TF

**Loại:** phân tích read-only. **Môi trường:** clone, Python 3.12, PyYAML. **Phụ thuộc:** WP-D0.

## Mục tiêu
Viết Interface Control Document **của hiện trạng**, chi tiết tới từng field: ai publish, ai subscribe, QoS thực, rate, điều kiện tạo interface, field nào thực sự được đọc. Tổng công trình sư sẽ dựa vào đây để thiết kế ICD đích (`nav_evidence_msgs`, SafetyProfile witness, discontinuity).

## Phạm vi
- Mọi node product: `fast_lio_node`, `navigation_runtime_node`, `px4_navigation_external_mode_node`, `px4_odometry_bridge_external_node`.
- Node chỉ có trong SITL: `px4_odometry_bridge_node`, `gz_visibility_bridge`, các bridge `ros_gz` do runner khởi động, `world_observation_gate.py`, observer Python trong `tools/runtime/external_mode_scenario.py`.
- Topic `/fmu/in/*` và `/fmu/out/*` kèm version suffix của `px4_msgs`.
- Các interface của `px4_ros2_interface_lib` (mode registration, service, action nếu có).
- TF: frame, ai broadcast (static/dynamic), ai tra cứu.

## Deliverable
### 1. `docs/refactor/WP-A2/icd_topics.yaml`
```yaml
- topic: /navigation/navigation_command
  type: navigation_contracts/msg/NavigationCommand
  scope: product | sitl_only | evidence_only
  publishers:
    - node: navigation_runtime_node
      at: src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:1747
      qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 1}
      trigger: "command_timer_ (create_wall_timer, 50 Hz)"   # gì làm nó publish
      created_when: always | "<điều kiện>"                  # ví dụ: chỉ khi có mission_file
  subscribers:
    - node: px4_navigation_external_mode
      at: <file:line>
      qos: {...}
      callback: NavigationMode::onNavigationCommand
      callback_group: <tên hoặc default>
      created_when: always
  qos_compatible: true            # tính theo luật ROS 2 (reliability/durability)
  recorded_in_evidence_bag: true  # có trong RUNTIME_EVIDENCE_TOPICS (runner.py:132) hay không
  notes: ""
```

### 2. `docs/refactor/WP-A2/icd_msgs.yaml`
Cho từng msg trong `src/contracts/navigation_contracts/msg/*.msg` (12 file), và cho mỗi msg `px4_msgs` mà product dùng:
```yaml
- msg: NavigationCommand
  fields:
    - name: localization_epoch
      type: uint64
      unit: "-"
      frame: "-"
      semantics: "<1 câu>"
      set_by: [<file:line>, ...]
      read_by: [{node: ..., at: <file:line>, purpose: gate|compute|log|evidence}]
      unused: false   # true nếu không subscriber nào đọc field này
```
Đặc biệt chú ý `NavigationExecutionDiagnostics` (65 field), `NavigationCommand` (41), `NavigationCommandRejection` (32), `NavigationModeStatus` (33).

### 3. `docs/refactor/WP-A2/diagnostic_channels.yaml`
Mọi `DiagnosticStatus.name` được publish (ví dụ `navigation_runtime/retained_command_decision`): publisher `file:line`, danh sách key, kiểu giá trị ngầm định, ai parse nó (C++ hoặc Python) kèm `file:line`, và dùng để làm gì: `safety_gate` | `judge` | `observability`. Mọi chỗ **product dùng diagnostics chuỗi để gate** phải đánh dấu `safety_gate` (đã biết: `px4_external_odometry_bridge_node.cpp:156-205`).

### 4. `docs/refactor/WP-A2/icd_params.yaml`
Cho mỗi node: `param, type, default (file:line), validation (range/pin, file:line), values_in_yaml: {config/runtime/<f>.yaml: value, ...}, set_by_runner (runner.py:line, nếu runner override), read_at (file:line)`. Đánh dấu:
- `pinned`: param khai báo được nhưng code bắt buộc phải bằng một literal, ví dụ `navigation_mode_node.cpp:255-258`.
- `fault_injection`: 12 param `navigation_runtime.inject_*`.
- `yaml_only_orphan`: key có trong YAML nhưng không có `declare_parameter` tương ứng. Chú ý: các param declare trong initializer list, và fast_lio declare qua `parameter_loader.cpp`.

### 5. `docs/refactor/WP-A2/tf_frames.md`
Cây frame; nơi hard-code chuỗi frame (`"lio_odom"`, `"base_link"`, `"livox_frame"`...) so với nguồn chuẩn (`fast_lio_core/.../frame_ids.hpp`).

### 6. `docs/refactor/WP-A2/findings.md`
Chỉ ghi các bất thường, mỗi mục có `file:line` và verdict. Tối thiểu kiểm các loại:
- Cặp pub/sub QoS không tương thích.
- Publisher không ai subscribe, hoặc subscriber không ai publish, trong product.
- Interface chỉ được tạo khi thoả điều kiện, dẫn tới một phía có còn phía kia thì không (đã biết: mission_progress / command_admission).
- Field được set mà không ai đọc.
- Doc nói một QoS nhưng code dùng QoS khác (đã biết: `/lio/mapping_observation`).
- Topic product phụ thuộc vào node chỉ có trong SITL (đã biết: `/lidar/free_space_endpoints`).

## Nghiệm thu
- Script `docs/refactor/WP-A2/extract_check.py` quét lại mọi `create_publisher|create_subscription|create_service|create_client` trong `src/` (trừ test/external/vendor) và in ra những cái thiếu trong `icd_topics.yaml`; kết quả phải rỗng.
- Cả 12 msg đều có đủ field trong `icd_msgs.yaml`.
- Không sửa `src/`.

---

## HỢP ĐỒNG CHUNG (bắt buộc, áp dụng cho mọi work package)

**Repo:** `github.com/TanDatEmb/uav-navigation`. ROS 2 Jazzy, FAST-LIO, PX4 External Mode, beta chỉ SITL.
**Baseline:** `main @ 7e0b850`. Tạo branch `refactor/<WP-ID>` từ đúng commit này. Các nhánh khác (`codex/*`, `feat/*`) chỉ để đọc tham khảo: KHÔNG merge, KHÔNG cherry-pick.

**Đọc trước khi làm:**
1. `AGENTS.md`.
2. `docs/refactor/ARCHITECTURE_REVIEW.md`: kiến trúc hiện tại, V1–V7, RC1–RC6, tên module đích, 8 quy tắc cứng, các phase.
3. `docs/refactor/adr/ADR-013..016`.
4. `docs/refactor/risk_register_20260928.md` (R-01…R-11).
5. Chỉ khi WP đụng estimation/mapping/planning/control/PX4/threshold: đọc thêm `docs/safety/runtime_safety_current.md`.

**Ràng buộc không thương lượng:**
- KHÔNG đổi giá trị ngưỡng, deadline, lease, budget, UNKNOWN policy, tolerance, trừ khi WP ghi rõ là được phép.
- Refactor và thay đổi hành vi không bao giờ nằm chung một commit.
- Mọi khẳng định trong deliverable phải có `file:line` trên commit baseline. Khi phân loại thì dùng CONFIRMED (đã tái hiện bằng test/trace), CONDITIONAL (đường code có thật nhưng chưa chứng minh được là tới được), SPECULATIVE.
- Không suy đoán hành vi runtime khi chưa đo. Số đo nào chưa có thì ghi `NOT_MEASURED`, tuyệt đối không bịa.
- Không sửa code product nếu WP là loại phân tích (read-only).
- Khi prompt mâu thuẫn với code hoặc với tài liệu an toàn: DỪNG phần đó, ghi vào `docs/refactor/<WP-ID>/OPEN_QUESTIONS.md` (câu hỏi, bằng chứng, các phương án), làm tiếp phần còn lại, không tự quyết.

**Nơi đặt kết quả:** `docs/refactor/<WP-ID>/`. Bắt buộc có `REPORT.md` gồm:
1. Tóm tắt 5–10 dòng.
2. Danh sách deliverable kèm đường dẫn.
3. Lệnh verify đã chạy, kèm output thật (exit code, số test pass/fail).
4. Những chỗ lệch khỏi prompt và lý do.
5. Open questions.
6. Commit SHA của từng commit.

Báo cáo viết tiếng Việt; identifier, tên file và tên cột giữ tiếng Anh.

**Hoàn tất:** push branch `refactor/<WP-ID>`, mở PR vào `main` ở trạng thái **draft**, không tự merge. Tổng công trình sư sẽ review.

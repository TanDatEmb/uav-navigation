# A2 — ICD: hiện tại và đích

Baseline `main @ 7e0b850`. Nguồn hiện trạng chi tiết (producer/consumer `file:line` của từng field, QoS của từng endpoint) nằm ở `docs/refactor/WP-A2/icd_topics.yaml`, `icd_msgs.yaml`, `diagnostic_channels.yaml`. Kiến trúc sư đã kiểm lại: `validate_icd.py` PASS (phủ create-call, đủ 12 msg, 17 diagnostic channel). Tài liệu này là **quyết định đích** cho từng topic và từng field.

Ký hiệu disposition: **K** giữ nguyên · **C** đổi kiểu hoặc ngữ nghĩa · **A** thêm · **R** xoá · **E** chuyển sang `nav_evidence_msgs` · **P** phase thực hiện.

## 1. Nguyên tắc ICD đích
1. **Tách control với evidence.** Msg control chỉ chứa những gì consumer dùng để quyết định. Mọi thứ phục vụ judge hoặc chẩn đoán đi vào `nav_evidence_msgs` (ADR-015).
2. **Thời gian:** trong một field mới, dùng `int64 *_ns` kèm clock domain đã khai báo trong contract. `std_msgs/Header.stamp` giữ theo quy ước ROS. Field kiểu `builtin_interfaces/Time` hiện có thì giữ (**K**), không churn.
3. **Authority phải được đặt tên.** Field quyết định quyền, ví dụ quyền emergency, phải là enum hoặc struct tường minh, không phải magic value.
4. **QoS lấy từ một catalog duy nhất** (`navigation_contracts/qos_catalog.hpp`, P1). P1 không đổi giá trị QoS nào; mọi thay đổi QoS là behavior change, cần đo trước.
5. Mọi enum trong msg đều có giá trị tường minh; không tái sử dụng giá trị cũ.

## 2. Bản đồ topic

### 2.1 Product data path
| Topic | Type | Pub (QoS) | Sub (QoS) | Đích |
|---|---|---|---|---|
| `/lio/odometry_propagated` | `PropagatedOdometry` | fast_lio (rel, d10) | runtime (BE, d1) · adapter (rel, d1) · bridge (rel, d20) | **C**: thêm field generation (§3.2). QoS **K** |
| `/lio/mapping_observation` | `RegisteredScan` | fast_lio (rel, d10) | runtime (BE, d1) | **K**. Sửa doc cho đúng QoS thực (WP-P0.3 I3) |
| `/lio/health` | `EstimatorHealth` | fast_lio (rel, d10) | runtime (BE, d10) · adapter (BE, d10) · **bridge (mới)** | **C** (§3.1). Bridge chuyển sang dùng topic này (P6) |
| `/lio/diagnostics` | `DiagnosticArray` | fast_lio | bridge (**safety gate**, V5) · judge | Bridge **R** subscription (P6). Topic giữ lại, chỉ cho observability |
| `/lidar/free_space_endpoints` | `PointCloud2` | `gz_visibility_bridge` (chỉ SITL) | fast_lio | **K** cho beta (ADR-016). Consumer phải đi qua `VisibilitySource` |
| `/navigation/navigation_command` | `NavigationCommand` | runtime (rel, d1) | adapter (rel, d1) | **C** (§3.3) |
| `/navigation/mode_status` | `NavigationModeStatus` | adapter (rel, TL, d1) | runtime (rel, TL, d1) | **K**; các field echo mission xem §3.5 |
| `/navigation/command_admission` | `NavigationCommandAdmission` | adapter (rel, d10) | runtime (rel, d10, **chỉ khi có mission_file**) | **C**: runtime luôn tạo subscription (§2.3) |
| `/navigation/mission_progress` | `NavigationMissionProgress` | runtime (rel, d10, **chỉ khi có mission_file**) | adapter (rel, d10, luôn tạo) | **C**: xem §2.3 |
| `/navigation/mission_complete` | `std_msgs/Bool` | runtime (chỉ khi có mission_file) | — | **R** ở P2. Trùng thông tin với `mission_progress.event=COMPLETE` |
| `/navigation/goal` | `NavigationGoal` | harness | runtime (rel, d1) | **C**: chỉ còn là đường SITL/test (§2.3) |
| `/fmu/in/trajectory_setpoint` | px4 | adapter | PX4 | **K** |
| `/fmu/in/vehicle_visual_odometry` | px4 `VehicleOdometry` | bridge (BE, d10) | PX4 EKF2 | **K**; field `quality` hiện không có consumer (WP-A2), vẫn giữ theo contract PX4 |
| `/fmu/out/vehicle_local_position_v1`, `vehicle_status`, … | px4 | PX4 | adapter | **K** |

### 2.2 Evidence và observability
| Topic | Hiện tại | Đích |
|---|---|---|
| `/navigation/diagnostics` (runtime d5 + adapter d50 cùng ghi) | 17 `DiagnosticStatus.name` channel, lên tới 118 key/channel | Tách thành `/evidence/*` (§4). Topic cũ chỉ còn observability; judge không đọc nó nữa (P2) |
| `/navigation/execution_diagnostics` | `NavigationExecutionDiagnostics`, 61 field, BE d1 | **E** → `/evidence/execution` (reliable, d≥1000) |
| `/navigation/command_rejection` | `NavigationCommandRejection`, BE d20 | **E** → `/evidence/command_rejection` (reliable) |
| `/lio/odometry_transport_trace`, `/navigation/odometry_ingress_trace` | `OdometryTransportTrace` | **E** → `/evidence/state_transport` (chỉ bật trong SITL build) |
| `/px4/estimator_odometry`, `/px4/diagnostics` | reverse bridge | chuyển sang `sitl_harness` |
| — | — | **A** `/nav/config_witness` (`ConfigWitness`, reliable, transient_local, d1) cho từng process (A5 §3.4) |

### 2.3 Interface được tạo có điều kiện: quyết định
Hiện tại runtime chỉ tạo admission sub, progress pub và complete pub khi có `mission_file` (`navigation_runtime_node.cpp:1753-1763`), trong khi adapter luôn tạo các endpoint phía mình (`navigation_mode_node.cpp:285-296`).

**Quyết định (chờ duyệt, ADR-017 D6):** product chỉ có **một đường là mission**, và mission một waypoint thay cho single-goal. `/navigation/goal` trở thành interface của `sitl_harness` và test. Runtime **luôn** tạo toàn bộ interface mission.

Hệ quả hành vi: ở chế độ goal hiện tại, adapter không bao giờ nhận completion receipt. Sau thay đổi, nó sẽ nhận receipt, nên đây là **behavior change** (P2, commit riêng, kèm ledger entry).

## 3. Msg control: quyết định theo field

### 3.1 `EstimatorHealth` (`/lio/health`)
| Field | Hiện tại | Đích |
|---|---|---|
| `header`, `localization_epoch`, `state` (+ 6 hằng), `navigation_valid`, `covariance_valid`, `observability_valid`, `correction_fresh`, `propagation_valid`, `last_correction_stamp`, `last_propagated_state_stamp`, `reason_code` | runtime / adapter gate | **K** |
| — | generation chỉ có trong diagnostics chuỗi (`ros_output_publisher.cpp:781`), bridge parse nó (`px4_external_odometry_bridge_node.cpp:171`) | **A** `uint64 public_frame_generation`, `bool public_frame_generation_valid`, `uint64 discontinuity_count`, `uint8 last_frame_event` (enum khớp `PublicFrameEvent`: `CONTINUOUS=0, INTERNAL_GENERATION_CHANGE=1, CORRECTED_PROPAGATED_HANDOFF=2, PX4_RESET=3, PUBLIC_FRAME_DISCONTINUITY=4`). P6 |
| — | bridge đọc `corrected_estimate_valid`, `pose/twist_covariance_available` từ chuỗi | Map vào `navigation_valid` / `covariance_valid`. Nếu ngữ nghĩa không trùng khớp hoàn toàn thì **A** field riêng; quyết định cụ thể ở P6 sau khi đối chiếu `ros_output_publisher.cpp` |

### 3.2 `PropagatedOdometry`
| Field | Đích |
|---|---|
| `odometry` (nav_msgs/Odometry), `localization_epoch`, `sequence` | **K** |
| — | **A** `uint64 public_frame_generation`. Generation gắn theo **từng sample** để bridge và runtime khỏi phải ghép với heartbeat health có tuổi tới 2.0 s. Đây là điều kiện cần để sửa R-02 (P6) |

### 3.3 `NavigationCommand` (41 dòng, 27 field)
| Nhóm field | Đích |
|---|---|
| Identity: `header`, `localization_epoch`, `goal_epoch`, `mode_activation_id`, `mission_id`, `waypoint_index`, `request_id`, `bundle_generation`, `sample_id` | **K** |
| World: `world_generation`, `world_revision`, `world_observation_stamp` | **K** |
| Lease/time: `state_source_stamp`, `valid_until`, `trajectory_time_s` | **K** |
| Continuation: `certified_main_continuation`, `continuation_boundary_stamp_ns` | **K** |
| Role/status: `role` (MAIN/BACKUP/EMERGENCY), `status` (EMPTY…HANDOVER) | **K** |
| Setpoint: `position`, `velocity`, `acceleration`, `jerk`, `yaw`, `yaw_rate` | **K**; jerk không đi sang PX4 (invariant 6) |
| `emergency_authorization_reason` (5 hằng), `emergency_candidate_commit_result` | **C**. Hiện tại được gán từ causal *trace* (`navigation_runtime_node.cpp:9416, 9420`). Adapter dùng chúng để **cấp quyền** velocity-only emergency bằng magic value `commit_result == 1U` (`navigation_mode_node.cpp:1702-1707`), tức authority đang được mã hoá như một trường diagnostic. Kiến trúc sư đã kiểm tiếp:
- Đây là **nơi duy nhất** adapter dùng các field này. Nó nằm trong đường `tracking_experiment.velocity_only` (`:2513-2523`).
- Branch này **chết**. `certified_emergency` đòi `STATUS_BRAKING`, nhưng check ngay sau đó từ chối mọi `status != STATUS_READY` (`:1714-1717`).
- Branch chỉ nhận lý do `ACTUAL_ANCHOR…`, bỏ qua `PROJECTED_MAIN_ONLY` và `INDETERMINATE_PRE_START` (`NavigationCommand.msg:53-58`).
- Hệ quả: trong experiment velocity-only, mọi emergency đều rơi về `requestVelocityOnlyHold`. Hướng lỗi là an toàn, nhưng emergency brake không bao giờ được thực thi ở mode này.
- Dòng chính (PVA `TrajectorySetpoint`) không đọc các field này. Đích: thay bằng `uint8 emergency_authorization` với enum tường minh `NONE=0, CERTIFIED_MEASURED_STATE_BRAKE=1` và `uint64 emergency_certificate_id` trỏ tới `CertificateRecord`. Lý do nằm trong enum cũ chuyển sang evidence. P3 |

### 3.4 `NavigationGoal`, `RouteSnapshot`
**K** toàn bộ, gồm `target`, `acceptance_radius_m`, `behavior`, `next_target`, `route`, `waypoint_*[]` và `measured_*`. Hai điểm cần lưu ý:
- `RouteSnapshot.measured_*` là tiến độ đo được do Core ghi. Nó không được đi ngược vào planner làm input nếu không nằm trong `PlanningRequest`.
- `NavigationGoal.has_next_target`/`next_target` bị trùng với `route`. Đánh dấu **R** ở P5, sau khi planner chỉ còn đọc route.

### 3.5 `NavigationModeStatus`
| Field | Đích |
|---|---|
| `header`, `mission_id`, `waypoint_index`, `request_id`, `activation_id`, `airborne`, `state` (+5 hằng), `reason` (+5), `external_mode_state` (+9), `external_mode_reason` | **K** |
| `waypoint_accepted`, `accepted_waypoint_index`, `acceptance_position_error_m`, `acceptance_speed_mps` | Adapter chỉ **echo** receipt mission mà Core đã gửi (`navigation_mode_node.cpp:377-380`), rồi runtime đọc lại (WP-A2: purpose `gate`). Đây là một vòng lặp authority. Đích: **R** ở P2. Runtime dùng chính quyết định `MissionProgress` của nó; judge đọc `/navigation/mission_progress` |

### 3.6 `NavigationCommandAdmission`, `NavigationMissionProgress`
- `NavigationCommandAdmission`: **K** toàn bộ 9 field. Đây là receipt factual của adapter, là đầu vào cần thiết cho `nav_mission`.
- `NavigationMissionProgress`: **K**. Thêm **A** `uint64 safety_profile_hash` ở P1, để adapter từ chối receipt từ một Core chạy profile khác.

### 3.7 `RegisteredScan`
**K** 18 field. Các field `visibility_*` là provenance của SITL visibility (HG-011). Nhóm này tiếp tục gắn với `VisibilitySource`.

## 4. Msg evidence đích (`nav_evidence_msgs`, P2)
Mỗi msg evidence đều bắt đầu bằng khối chung `EvidenceHeader`:
```
# EvidenceHeader.msg
std_msgs/Header header            # stamp = source time of the fact
string producer_id                # e.g. "nav_core_node", "px4_adapter_node"
uint64 producer_instance_id       # random per process start
uint64 sequence                   # strictly +1 per producer
uint8  CLOCK_ROS=0
uint8  CLOCK_STEADY=1
uint8  clock_domain
uint16 schema_version
uint64 safety_profile_hash
```

| Msg đích | Thay cho channel/msg hiện tại | Nội dung (từ các key judge đang đọc; WP-A2 `evidence_candidates.md`) |
|---|---|---|
| `LifecycleEvent` | `navigation_runtime/execution_activation_witness`, `execution_pending_superseded_witness`, `heading_rebind_admission_witness`, `recovery_retry_witness`; và các phase request/export/authorize/publish mà judge đang tự dựng | `phase` (enum: REQUEST, EXPORT, CERTIFY, AUTHORIZE, STAGE, ACTIVATE, PUBLISH, SUPERSEDE, RETRY, DISCARD), identity (`localization_epoch, goal_epoch, request_id, bundle_generation, bundle_source, bundle_owner_cycle_id, parent_bundle_generation, sample_id`), world identity, `disposition`, `reason` |
| `RetainedDecisionEvidence` | `navigation_runtime/retained_command_decision` (118 key) | `purpose`, `disposition` với enum giá trị tường minh khớp `RetainedDecisionDisposition` (chấm dứt R-05), identity desired/executing, world expected/captured, freshness witnesses, flag emergency preparation/admission. Accounting `published_before`/`suppressed`/`failed` thay bằng `EvidenceHeader.sequence` + `DropCounter` |
| `WorldTransactionEvidence` | `navigation_runtime/world_transaction_witness`, `navigation_mapping/world_model` (các key `world_transaction_*`) | `transaction_kind` (SUSPENDED, PUBLICATION_COMMITTED, RECERTIFIED, RESUMED), world identity trước/sau, danh sách bundle bị ảnh hưởng |
| `CertificateEvidence` | (mới) | Toàn bộ `CertificateRecord` (A5 §1.4) cho mọi lần certify, kể cả khi fail |
| `ExecutionEvidence` | `NavigationExecutionDiagnostics` (61 field) | Giữ nguyên nội dung, đổi tên package; các enum dùng giá trị tường minh |
| `CommandRejectionEvidence` | `NavigationCommandRejection` | Giữ nguyên 20 field, thêm `EvidenceHeader` |
| `SetpointInputEvidence` | `navigation_external_mode/PX4_INPUT_SETPOINT` (25 key) | `trace_sequence` được thay bằng `EvidenceHeader.sequence`; `setpoint_boundary`, `setpoint_update_duration_ns`, identity command/world, PVA gửi sang PX4 |
| `PlannerCycleEvidence` | `navigation_runtime/planner` (101 key), `planner_path` (JSON) | Thời gian từng stage, outcome, failure stage/reason, candidate id. Path snapshot tách thành msg `PlannerPathSnapshot` dạng typed, chỉ bật trong SITL build |
| `EstimatorEvidence` | `fast_lio/estimator` (45 key), `fast_lio/propagated_odometry`, `fast_lio/transport` | Các key judge đang đọc (`map_point_count`, `absolute_guard_*`, `map_maintenance_us`, observability) |
| `AlignmentLatchEvidence` | `navigation_external_mode/ALIGNMENT_LATCH_WITNESS` | 15 key; hiện judge giữ nguyên dạng opaque. Đích là typed |
| `FaultInjectionEvidence` | `navigation_runtime/exact_optimization_failure_injection` (29 key) | Chỉ tồn tại trong SITL build |
| `ConfigWitness` | log `RUNTIME_CONFIG_EFFECTIVE` cộng việc runner đi so khớp | `profile_name`, `safety_profile_hash`, `tracking_experiment_mode`, `effective` (danh sách key/value đã resolve) |

Luật chung cho evidence:
- QoS reliable, keep_last ≥ 1000, volatile. Record bằng MCAP (P0.2).
- Có khoảng hở trong `sequence` ⇒ evidence loss ⇒ `NOT_EVALUABLE`.
- Judge chỉ đọc decoder được sinh ra từ các file `.msg` này (tool `rosbags`); không được parse chuỗi.

## 5. Lịch chuyển đổi
| Phase | Thay đổi ICD | Loại |
|---|---|---|
| P1 | `qos_catalog.hpp`; `ConfigWitness`; `safety_profile_hash` trong `MissionProgress` | refactor + additive |
| P2 | `nav_evidence_msgs` (dual emit song song với DiagnosticArray); R `/navigation/mission_complete`; R echo acceptance trong `ModeStatus`; interface mission không điều kiện (D6) | additive → behavior (D6) |
| P3 | `NavigationCommand.emergency_authorization` + `emergency_certificate_id` | behavior tại biên, qua shadow |
| P6 | Thêm generation vào `EstimatorHealth` và `PropagatedOdometry`; bridge bỏ subscription `/lio/diagnostics` | behavior (R-02) |

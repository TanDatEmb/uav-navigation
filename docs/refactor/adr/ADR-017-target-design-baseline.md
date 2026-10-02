# ADR-017: Baseline thiết kế đích (A1–A5)

**Trạng thái:** ACCEPTED (rev 1, 2026-09-29). Chủ dự án đã cho triển khai theo kiến trúc này. Các mục trong §6.3 vẫn chờ chủ dự án quyết định riêng.
**Ngày:** 2026-09-28. **Baseline:** `main @ 7e0b850`.
**Phụ thuộc:** ADR-013 (topology), ADR-014 (certifier), ADR-015 (evidence + MCAP), ADR-016 (beta SITL).
**Tài liệu chi tiết:** `docs/refactor/design/A1_module_ownership_map.md`, `A2_icd_current_and_target.md`, `A3_pipeline_timing_target.md`, `A4_reducer_design.md`, `A5_contract_specs.md`.

## Bối cảnh
- Wave 1 (các WP A1–A6 R1) đã cho inventory.
- Tài liệu design do kiến trúc sư tự viết bằng phân tích tĩnh trên baseline. Chỗ nào mâu thuẫn với output của agent thì **tài liệu design là authority**.
- ADR này chốt những quyết định mà các spec và prompt đợt 2 (P1–P6) sẽ dựa vào. Những gì **chưa đo được** nằm ở §4. Không quyết định nào dưới đây tune ngưỡng.

## 1. Quyết định cần duyệt

| ID | Quyết định | Loại thay đổi | Chi tiết |
|---|---|---|---|
| **D1** | **Bản đồ module.**<br>• `navigation_runtime` SPLIT thành `nav_execution` (reducer), `nav_planning_policy`, `nav_mission`, `nav_core_node` (shell).<br>• Các validator chuyển từ backend sang `nav_certifier`.<br>• `Piece`/`Trajectory` chuyển sang `nav_plan_contract`, để gỡ cycle planner↔certifier.<br>• `MissionController` DELETE.<br>• `tracking_experiment` chuyển sang `sitl_harness`.<br>CMake gate: certifier không link planner; execution không link planner hay world. | refactor | A1 §3–4 |
| **D2** | **ICD.**<br>• (a) `EstimatorHealth`, `PropagatedOdometry` thêm `public_frame_generation` (P6).<br>• (b) `NavigationCommand`: magic value `emergency_authorization_reason` / `commit_result` đổi thành enum `emergency_authorization` + `emergency_certificate_id` (P3).<br>• (c) Xoá echo acceptance trong `ModeStatus` và topic `/navigation/mission_complete` (P2).<br>• (d) `safety_profile_hash` có trong `MissionProgress` và `EvidenceHeader`. | behavior tại biên; mỗi mục một commit | A2 §3, §5 |
| **D3** | **Lane.**<br>• `nav_core_node` gồm 1 decision thread và 3 lane (mapping / planning / fast); mọi timer theo ROS clock.<br>• **P4a:** recert và emergency **vẫn chạy trên planning lane**, đúng thứ tự như hiện tại.<br>• **P4b:** chỉ chuyển sang fast lane khi đủ số đo M1–M2 và là commit behavior riêng.<br>• Quyết định này **sửa trình tự triển khai của ADR-014 §3**; kiến trúc đích giữ nguyên. | refactor (P4a), behavior (P4b) | A3 §2, §4 |
| **D4** | **Reducer.**<br>• `ExecutionState` là sum type (Idle / TrackingMain / SafetySuffix{Backup, Emergency} / StoppedHold / Px4Hold) thay cho 5 enum độc lập.<br>• Effect là data và được thực thi sau reducer.<br>• 45 hàm `planner_fsm` được **di chuyển nguyên văn**.<br>• **Oracle P4** = exit-site index A4-R1 + predicate thuần + **characterization trace (WP mới P4-0)**. Không mở A4-R2. | refactor | A4 §0–§7 |
| **D5** | **Certifier và bundle.**<br>• `CandidateBundle` thành pure data: bỏ 2 `std::function`, chỉ còn int ns.<br>• `CertificateRecord` thay cờ bool; commit check O(1).<br>• P3a move → P3b shadow → P3c switch. Switch chỉ khi shadow cho **0 bất đồng** (planner=true / certifier=false) trên bag P0.2. | refactor (a, b), behavior (c) | A5 §1–2 |
| **D6** | **Một đường mission.**<br>• Interface mission được tạo không điều kiện.<br>• `/navigation/goal` chỉ còn trong SITL build.<br>• Hiện tại: `navigation_runtime_node.cpp:1753-1763` tạo interface mission có điều kiện, nên đang có 2 đường goal. | behavior (P2) | A2 §2.3 |
| **D7** | **SafetyProfile.**<br>• (a) **HG-001:** profile lấy giá trị **đang chạy** A* 30/60 ms và solve 80 ms. Ledger sửa bằng một commit docs riêng có lineage.<br>• (b) `0.15 m/s`: giữ 5 key riêng cùng giá trị; chỉ gộp khi chủ dự án xác nhận cùng ngữ nghĩa.<br>• (c) Không default C++ cho key safety: thiếu key ⇒ process không arm. Đây là **behavior lúc khởi động**.<br>• (d) Mission YAML không chứa key safety.<br>• (e) Adapter bỏ 4 literal pin (`navigation_mode_node.cpp:255-258`). | (a), (b): docs / refactor; (c), (d), (e): behavior | A5 §3 |
| **D8** | **Evidence.**<br>• `nav_evidence_msgs` (rev 1: package `navigation_evidence`, xem §6.2) (EvidenceHeader, 12 họ msg, CertificateEvidence, ConfigWitness, DropCounter).<br>• Enum tường minh có `static_assert`.<br>• Decoder sinh từ `.msg`.<br>• Dual emit suốt P2; judge v1 và v2 phải cho verdict bằng nhau trên mọi bag P0.2. | additive → behavior (tắt DiagnosticArray sau P2) | A2 §4, A5 §4 |
| **D9** | **Merge khi CI bị khoá billing:** `make ci-local` trên đúng head SHA của PR, log dán vào PR, kiến trúc sư approve. Khi billing mở lại, chạy lại hosted CI cho `main`. | quy trình | REVIEW wave 1 |

## 2. Finding mới từ phân tích design (chưa có trong risk register 2026-09-28)

| ID | Finding | Bằng chứng | Xử lý |
|---|---|---|---|
| N1 | Hai đường goal: mission service/action có điều kiện, cộng `/navigation/goal` trực tiếp | `navigation_runtime_node.cpp:1753-1763` | D6 |
| N2 | Adapter echo mission acceptance trong `ModeStatus`, tạo authority thứ hai cho acceptance | `navigation_mode_node.cpp:377-380` | D2(c) |
| N3 | Bundle mang hai biểu diễn thời gian (double s và int ns) nên phải kiểm chéo. Emergency commit truyền `now().seconds()` dạng double | `candidate_bundle.hpp:141-148,256-262`; `navigation_runtime_node.cpp:8143` | D5 |
| N4 | `execution_anchor.hpp` có hai bản (execution và planning); roundoff tolerance bị lặp | `candidate_bundle.hpp:398-403`; `execution_anchor.hpp:47-52` | D1, D7 |
| N5 | Planner tự đặt cờ certificate; `world_validator` là closure capture state của planner | `planner.cpp:1057-1059,1137` | D5 (đã biết là RC2; ghi lại vị trí chính xác) |
| N6 | `NavigationCommand.emergency_authorization_reason` / `commit_result` là authority được mã hoá bằng magic value. Chỉ đường velocity-only (experiment) dùng chúng, và ở đó branch emergency **chết**: đòi `STATUS_BRAKING` nhưng check kế tiếp đòi `STATUS_READY`. Branch cũng chỉ nhận lý do ACTUAL, bỏ qua PROJECTED và INDETERMINATE. Kết quả: mọi emergency ở mode này đều thành Hold, nên kết quả experiment velocity-only về emergency không có giá trị | `navigation_runtime_node.cpp:9416,9420`; `navigation_mode_node.cpp:1702-1717,2513-2523`; `NavigationCommand.msg:53-58` | D2(b). Experiment chuyển sang `sitl_harness` (D1) |
| N7 | Emergency brake và recert chạy trên thread PlanningWorker. Không có deadline riêng và không có số đo trigger→publish. Chu kỳ planner đo được 148 ms so với period 100 ms (E10) | A3 T3–T5; WP-A3 S15 | D3; M1, M4 |
| N8 | 5 enum lifecycle độc lập (480 tổ hợp); ít nhất 2 tổ hợp dựng được nhưng chưa rõ reachability | `execution_lifecycle.hpp:52-60`; WP-A4 `lifecycle_enums.md` | D4 |
| N9 | Bảng A4-R1 là index exit site: 320/327 guard là số dòng, 239 rule `NO_OP`. `RT-300` có trong `coverage_gaps.md` nhưng thiếu trong bảng | A4 §0 | D4 (P4-0 / P4-1) |
| N10 | HG-031 legacy ghi "MAIN/BACKUP", còn code chỉ cho emergency từ `kTrackMain` | `runtime_safety_legacy_full.md:3392` (đã gỡ ở baseline 2026-10-01; HG-031 còn trong `runtime_safety_current.md`); `planner_fsm.hpp:501-531` | Commit docs làm rõ ledger; reducer theo code |
| N11 | Mapping update không có budget (p50/p95/max = 20/40/57 ms) | WP-A3 `timing_budget.csv` | M3 → key `PROVISIONAL` |

## 3. WP mới do ADR này sinh ra
- **P4-0: characterization trace.**
  - Instrument baseline (chỉ trong SITL build) để ghi `(event, state digest, effects)` tại từng `RT-*`.
  - Thu trace từ toàn bộ test hiện có và các run P0.2.
  - Làm sau P0.2; phải xong trước mọi commit P4.
- **P4-1: checker map `RT-*` → transition.**
  - Kiểm 1-1, không có transition mồ côi, tập effect khớp trace.

## 4. Mục chưa quyết được: cần đo (P0.2 hoặc shadow)
| ID | Cần đo | Dùng cho |
|---|---|---|
| M1 | Phân phối trigger→publish của emergency | D3 P4b, key `emergency_prepare_deadline` |
| M2 | Phân phối thời gian recert active và staged theo world revision | D3 P4b |
| M3 | Phân phối mapping update | key `mapping_update_budget` (`PROVISIONAL`) |
| M4 | Phân rã chu kỳ planner: solve, recert, diagnostic, chờ lock | xác định overrun 148 ms nằm ở đâu |
| M5 | Command lease, state age, freshness thực tế (hiện `NOT_MEASURED`) | xác nhận profile AS-IS |
| M6 | Chi phí `certify` độc lập (aggregate đã thấy tới 19.9 ms) | ngân sách của planning lane ở P3b |
| M7 | Sai khác giữa `sample()` và `evaluator` trên lưới 1 ms | D5 |

## 5. Hệ quả
- Spec và prompt đợt 2 (P1 SafetyProfile, P2 evidence, P3 certifier, P4-0) được viết **sau** khi ADR này được duyệt.
- Trạng thái A4-R1: **ACCEPT như exit-site index**, không đòi R2. A6-R1 là input cho P1. B6 predicate coverage là oracle cho P6.
- Mọi quyết định có nhãn "behavior" đi thành commit riêng, có ledger entry và evidence SITL theo AGENTS.md. Không commit nào vừa refactor vừa đổi behavior.

## 6. Amendment 1 (2026-09-29): bổ sung từ knowledge base (`docs/refactor/kb/`)

### 6.1 Quyết định bổ sung
| ID | Quyết định | Phase |
|---|---|---|
| D10 | Estimator có đường reset/relocalize kèm epoch bump. Mọi latch hoặc trạng thái terminal phải khai báo điều kiện thoát (KB-08 quy tắc S3) | P6 |
| D11 | Một `TimedState` và một `TrackingAssessment` dùng chung cho runtime, emergency và adapter envelope | P4 (runtime), P6 (adapter) |
| D12 | Căn frame LIO↔PX4 bằng SE(2), latch lúc đứng yên, kiểm heading liên tục, fail-closed khi lệch | P6 |
| D13 | `/lio/health` có một owner và một clock; msg mang generation và sequence | P6 |
| D14 | Mọi vòng lặp hình học (SimplifySFC, corridor, root finder) có bound và kiểm deadline | P3 |
| H2 | Sửa sớm R5-08 trên code hiện tại. Mẫu có dt < min hoặc dt ≤ 0 bị **từ chối, giữ nguyên baseline** (không reseed); mẫu đó không được publish | wave 2 |
| J1 | Sửa tính đúng đắn của judge (frame vận tốc, percentile, verdict HTML, clock stale, guard). Chỉ sửa Python, mỗi sửa một commit | wave 2 |
| P7 | Mapping storage: chunked COW, một seed unknown (behavior), bỏ poison sai | sau P4 |

### 6.2 Thu gọn thiết kế (chống over-engineering, KB-09 §3)
- **Package contract:** 3 package thay vì 7. `nav_contracts` là package C++ với các interface target con `types` / `world` / `plan` / `mission`, để CMake vẫn enforce hướng phụ thuộc. Hai package còn lại là `nav_safety_profile` và `navigation_evidence` (chỉ chứa msg). Msg control giữ nguyên package `navigation_contracts` (O4: không đổi tên chỉ để đẹp).
- **D3:** mặc định **2 lane** (mapping, planning). Fast lane chỉ lập nếu M1/M2 chứng minh là cần.
- **D8:** chỉ tạo msg evidence cho các key mà judge dùng để ra verdict. Tên package: `navigation_evidence`.
- **Codegen SafetyProfile:** 2 output (header C++ + loader; module Python). Schema CI sinh từ module Python.
- **P4-0:** chỉ trace tại các call site có effect, khoảng 60 site.
- **Process `px4_ingress`:** chuyển sang qualification trong beta SITL.

### 6.3 Chờ chủ dự án quyết định (agent không được tự chọn)
| ID | Câu hỏi | Đề xuất của kiến trúc sư |
|---|---|---|
| Q-HG001 | Profile lấy A* 30/60 ms và solve 80 ms (đang chạy), rồi sửa ledger | Đồng ý. P1 dùng giá trị đang chạy |
| Q-ENV | Tracking envelope của adapter chuyển từ box (√2·L) sang norm L (R5-16) | Chuyển sang norm. Đây là **siết** gate, cần evidence SITL |
| Q-VOX | Dung sai lượng tử hoá voxel có nằm trong mapping budget 0.10 m không (R6-13) | Certificate đo khoảng cách tới hộp voxel thay vì tới tâm voxel |
| Q-UNK | Một seed unknown duy nhất (R6-02); chọn midpoint hay 0 | Chọn midpoint (bảo thủ). Làm ở P7, có evidence |
| Q-TRK | Mode tracking mặc định của runner: `relaxed` (đang dùng) hay `off` (theo YAML) (R7-16) | Chọn `off`, vì đó là sự thật của product |
| Q-XTRK | Có gate cross-track từ registry hay không (R7-04) | Chưa gate. J1 chỉ làm HTML khớp `report.json` |

### 6.4 Sửa D9 (merge khi CI bị khoá)
Môi trường của agent và của kiến trúc sư đều **không pull được image** (Docker Hub bị chặn). Vì vậy `make ci-local` chạy bằng container là bất khả thi. Gate thay thế cho một PR là: chạy **native** trên đúng head SHA của PR đủ ba job (static-contract, python, và ros-jazzy-build nếu PR chạm `src/` hoặc CMake), rồi dán log vào PR. PR chỉ chạm docs hoặc Python thì không cần job ros-jazzy-build.

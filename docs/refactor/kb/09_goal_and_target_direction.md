# KB-09: Mục tiêu cuối cùng, tiêu chí đạt, và định hướng kiến trúc đích (đã rà lại chống over-engineering)

## 1. Mục tiêu cuối cùng (định nghĩa kiểm được)
"Một dự án ổn định, đáp ứng tiêu chuẩn, tài liệu rõ ràng, đồng bộ từ đầu đến cuối, chạy ổn định, an toàn, hành vi rõ ràng, không over-engineering". Tách ra thành các tiêu chí đo được:

| # | Tiêu chí | Đo bằng | Trạng thái hiện tại |
|---|---|---|---|
| G1 An toàn | 0 finding S1 còn mở. Mọi S2 đã đóng, hoặc được chấp nhận bằng ADR có lý do. Không PASS giả ở judge | `data/findings.csv` + ledger | **S1 = 2, S2 = 19** |
| G2 Hành vi rõ ràng | Mọi máy trạng thái là reducer có bảng transition và test. Mọi dòng trong ma trận sự cố (KB-06) có hành vi xác định và **độ trễ hữu hạn đã đo** | reducer test; P0.2 | 5 dòng "vô hạn" (B3, B4, B12, B19, B23) |
| G3 Ổn định | Chạy lặp N lần trên ma trận scenario, có phân phối (không kết luận từ một run). Không crash hay kẹt không giải thích được. Mọi budget có phân phối | P0.2 / M1–M7 | budget mapping và emergency chưa đo |
| G4 Tiêu chuẩn | Theo KB-08 cộng các chuẩn ngoài phù hợp phạm vi beta: ROS REP-103 (đơn vị, frame) và REP-105 (tên frame); ROS 2 QoS best practice; clang-tidy (`bugprone-*`, `cppcoreguidelines-*` chọn lọc); lizard CCN; px4_ros2 External Mode contract | CI | chưa có gate |
| G5 Đồng bộ | Một SafetyProfile có hash cho mọi process và judge. Không còn giá trị nào lệch giữa các lớp | ConfigWitness | 51 CONFLICT + 15 CONFIG |
| G6 Tài liệu | Bộ kiến trúc hiện hành duy nhất (KB + design + ADR) được lint đối chiếu với code. Lịch sử tách riêng | doc-lint | D-01..D-05 |
| G7 Không over-engineering | Code chết và experiment không còn trong product. Mỗi abstraction mới xoá được ít nhất một finding (O1). LOC product giảm | LOC, lizard | xem §3 |

## 2. Việc cần bổ sung vào ADR-017 (từ review này)
| ID | Bổ sung | Lý do |
|---|---|---|
| D10 | **Estimator có đường reset/relocalize** kèm epoch bump; mỗi latch khai báo điều kiện thoát (quy tắc S3) | R4-01, R4-12, R4-16; epoch hiện không bao giờ đổi, nên toàn bộ luồng F11 chưa từng chạy thật |
| D11 | **Một `TimedState` và một `TrackingAssessment`** dùng chung cho runtime, emergency và adapter envelope (norm-based) | R3-09/10/11, R5-16. Việc chọn ngưỡng cho envelope norm là quyết định của chủ dự án |
| D12 | **Căn frame LIO ↔ PX4:** latch SE(2) lúc đứng yên, kiểm heading liên tục, fail-closed khi lệch. Ghi giả định vào `frame_conventions.md` | R5-17, R5-18, D-03 |
| D13 | **`/lio/health`: một owner, một clock**, có generation và sequence | R4-06, R5-09 |
| D14 | **Bound cho mọi vòng lặp hình học** (SimplifySFC, corridor, root finder) cùng deadline check | R2-24 |
| H2 | **Sửa sớm R5-08 (S1) trên code hiện tại**, giống tiền lệ H1: small-dt / non-increasing reseed ở trạng thái untrusted, kèm test repro. Đây là thay đổi behavior nhỏ, cô lập, và P6 sẽ viết lại toàn bộ bridge sau | S1 có thể xảy ra ngay trong SITL. Chờ tới P6 thì mọi evidence SITL trước đó đều thiếu bảo vệ |

## 3. Rà lại ADR-017 theo quy tắc chống over-engineering (O1–O5)
| Đề xuất cũ (ADR-017 / A1–A5) | Rủi ro over-design | Điều chỉnh |
|---|---|---|
| 7 package contract (`nav_core_types`, `nav_safety_profile`, `nav_world_contract`, `nav_plan_contract`, `nav_mission_contract`, `navigation_contracts`, `nav_evidence_msgs`) | Quá nhiều package cho khoảng 3k LOC contract | **3 package:** `nav_contracts` (C++; các interface target con `types` / `world` / `plan` / `mission` để CMake vẫn enforce hướng phụ thuộc), `nav_safety_profile` (codegen), `navigation_msgs` (gộp msg control và evidence, hoặc tách evidence nếu cần tách QoS) |
| 3 lane + decision thread | Fast lane có thể không cần | Mặc định **chỉ 2 lane** (mapping, planning), vốn đã có sẵn. Fast lane chỉ lập nếu M1/M2 chứng minh cần (đã là điều kiện của D3; nay ghi rõ mặc định là không có) |
| 12 họ msg evidence + dual emit | Có thể thừa so với nhu cầu verdict | Chỉ tạo msg cho **các key mà judge thực sự đọc để ra verdict**, khởi đầu với Lifecycle, Certificate, Execution, CommandRejection, SetpointInput, Estimator, ConfigWitness. `PlannerCycleEvidence` chỉ giữ key judge dùng. Dual emit kết thúc ngay khi parity đạt |
| Codegen SafetyProfile ra C++, Python và ROS param schema | 3 output | **2 output:** header C++ + loader, module Python. Schema dùng để kiểm trong CI được sinh từ module Python |
| P4-0 trace tại cả 327 exit site | Instrument quá dày | Chỉ trace tại **các call site có effect**: publish, fail-closed, commit, cancel, submit, emergency, khoảng 60 site. Exit site `NO_OP` (239) chỉ cần map trong checker |
| Process `px4_ingress` là product | Nó chỉ phục vụ evidence và prior | Chuyển sang qualification cho beta SITL; chỉ đưa lại product khi hardware cần prior |

**Xoá hẳn** (không bọc lại, theo O5). Ước tính vài nghìn LOC; số chính xác đo khi thực hiện.
- Planning: `BackupTrajOpt` (933 LOC, tắt), MINCO S2/S3 và header trùng (khoảng 480 LOC), bản sao builder heading rebind, JSON snapshot writer trong TU optimizer (chuyển sang tool SITL).
- Mapping: patch snapshot chain (thay bằng chunked COW, R6-09), ESDF/frontier trong đường build product, virtual plane (tắt trong product, có 4 predicate lệch nhau).
- PX4 boundary: experiment velocity-only và witness tự chứng nhận (R5-20, N6), `MissionController` cùng test 1.6k LOC, `ReferencePointConverter`.
- Runtime: 3 trong 4 mô hình tracking acceptance, 7 tham số fault injection (chuyển sang `sitl_harness`).
- Judge: 3 map alias profile, bảng per-profile trong runner, verdict tự tính trong HTML.

## 4. Phân bổ finding theo phase (để giao việc bước tiếp)
| Phase | Nội dung | Finding được đóng |
|---|---|---|
| P0 (đang làm) | CI, baseline, H1, **H2** | R-01, R-04, **R5-08** |
| P1 SafetyProfile | nguồn duy nhất, không default, hash | A6, R1-13, R1-36, R2-16, R3-24, R4-11, R4-13, R6-07, R6-20, R7-02, R7-08, R7-16, R-08, V6 |
| P2 Evidence + judge | msg typed, decoder, bỏ V7 | R-05, R-07, R7-17, R7-23, R7-25, R7-26, R7-27, R7-31, R7-39, R7-41, R7-04, R1-40 (lý do typed) |
| P3 Certifier | shadow → switch | N5, R1-18 (anchor continuity), R1-22, R2-11, R2-13, R6-13 (định nghĩa khoảng cách), R2-24 (bound) |
| P4 Runtime reducer + lane | sum type, TrackingAssessment, publish ngoài lock | N7, N8, R3-01/09/10/11/13/14/15/16/17, R5-29, R5-16 (envelope dùng chung) |
| P5 Planner SolveContext | tách god object | R-06, R1-08/10/14/17/19/23, R2-05/21, R1-21/31 |
| P6 Adapter / bridge / estimator discontinuity | frame SE(2), generation typed, health một owner, reset estimator | R-02, R5-09, R5-17, R5-18, R4-01, R4-06, R4-12, R4-16, R4-22 |
| P7 (mới) Mapping storage | chunked COW, một seed unknown, bỏ poison sai | R6-02 (behavior), R6-04, R6-09, R6-11, N11 |

Mỗi phase giữ nguyên nguyên tắc cũ: refactor trước, behavior sau, đo trước khi đặt ngưỡng.

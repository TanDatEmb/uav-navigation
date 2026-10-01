# KB-08: Quy tắc chung toàn dự án (bản đề xuất để đồng bộ)

Cách áp dụng:
- Mỗi quy tắc có **ID**, **nội dung**, **cách kiểm** (tự động, nếu được) và **finding mà nó ngăn**.
- Quy tắc áp dụng cho code mới và cho code được chạm vào trong refactor. Code cũ chưa chạm thì liệt kê như nợ.
- Không có quy tắc nào cho phép tune ngưỡng; mọi thay đổi ngưỡng đi theo AGENTS.md.

## A. Kiến trúc và phụ thuộc
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| A1 | Phụ thuộc chỉ đi xuống: contract ← core ← shell ← qualification. Core không include ROS | CMake test đọc `LINK_LIBRARIES` + grep `rclcpp` trong core | V1, V2, RC1 |
| A2 | Contract chỉ chứa data và bất biến nhỏ; không `std::function`, không ROS loader, không logic trên 20 dòng | clang-tidy/grep trong `*_contract` | V4, R5-36 |
| A3 | Planner là bên sinh không được tin; certifier không link planner; mọi authority cho command nằm ở execution | CMake test | RC2, N5 |
| A4 | Một khái niệm chỉ có một owner, một reducer và một nguồn sự thật. Bản sao phải được **sinh ra** từ nguồn, không chép tay | review + codegen check | RC3, R1-13, R4-13, R6-07 |
| A5 | Code experiment, fault injection và visualization chỉ được build trong SITL (`sitl_harness`); product build không chứa chúng | CMake option + test symbol | N6, R5-20, R5-36 |
| A6 | Không giữ code chết. API không có call site thì xoá trong cùng PR refactor | `-Wunused` + script tìm symbol không có caller | R-06, R2-04, R5-15, R5-24 |

## B. Thời gian
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| T1 | Mọi thời điểm tuyệt đối là `TimestampNs{int64, ClockDomain}`; so sánh khác domain là lỗi compile. Đổi sang giây chỉ ở phép tính cục bộ | Kiểu dữ liệu; grep `double .*_time_s`, `now().seconds()` | N3, D-01, R3-09 |
| T2 | Chỉ có **một** hàm giây → ns, với chính sách làm tròn tường minh | grep | R5-01 |
| T3 | Timer quyết định trong product dùng ROS clock; cấm `create_wall_timer` trong product | grep gate | T2 (A3), R4-22, R7-39 |
| T4 | Mọi quyết định so sánh state với command phải cùng thời điểm (time-aligned); cấm so "command tại now" với "state tại source" | `TrackingAssessment` duy nhất | R3-09/10/11, R3-02 |
| T5 | Freshness và lease là đại lượng của SafetyProfile, định nghĩa riêng cho từng stream và dùng chung giữa product và judge | codegen | R7-08, R3-03 |

## C. Frame và đơn vị
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| F1 | Tên biến chứa frame: `p_world_enu`, `v_body_flu`, `^target T_source`. Msg ghi frame và đơn vị cho **từng field** | review + lint msg comment | R5-39, R7-25 |
| F2 | Chuyển frame chỉ qua `navigation_common::frames`; cấm tự viết ma trận tại chỗ | grep | R7 (NED ↔ ENU ở html_report) |
| F3 | Mọi giả định căn frame giữa hai hệ (LIO ↔ PX4) phải được **đo và kiểm liên tục**; lệch thì fail-closed | test | R5-17, R5-18 |
| F4 | Đơn vị ghi trong tên key config (`_s`, `_m`, `_mps`, `_rad`); ns/µs chỉ xuất hiện ở biên | loader check | parameter_contract |

## D. Config (SafetyProfile)
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| C1 | Tham số safety chỉ lấy từ SafetyProfile; **cấm default trong C++ và Python**; thiếu key thì process không arm | loader `required=true` + test | R2-16, R6-20, A6 |
| C2 | Overlay chỉ được đổi key có đánh dấu `overlay: allowed`; mission YAML không chứa key safety | schema.json | A6 |
| C3 | Mọi process publish `ConfigWitness` (kèm hash); hash lệch thì không arm | runtime check | RC3 |
| C4 | Giá trị dẫn xuất (`robot_r`, reserve, budget sum) được tính trong codegen, không ghi tay | codegen | R3-24, A6 I-02 |
| C5 | Không thêm tham số nếu hành vi đó dẫn xuất được, hoặc nếu tham số chỉ để che một lỗi ownership | review (đã có trong parameter_contract) | — |

## E. Trạng thái và quyết định
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| S1 | Mỗi khái niệm có một reducer `State × Event → (State, Effects)`. State là sum type, tổ hợp mâu thuẫn không biểu diễn được | reducer test + `static_assert` | N8, R5-29 |
| S2 | Effect là data và được thực thi sau reducer; không gọi ROS, không cancel, không publish bên trong reducer hay trong lock | review + test | R3-14, R3-16 |
| S3 | **Mọi latch hoặc trạng thái terminal phải khai báo điều kiện thoát** (reset, epoch mới, hoặc restart process có chủ đích) và có test cho điều kiện đó | bảng latch + test | R4-01, R4-12, R4-16, R6-04 |
| S4 | Mọi vòng lặp trên dữ liệu ngoài phải có bound hoặc deadline | review + test fuzz | R2-24 |
| S5 | Lý do reject và fail là enum typed, đi xuyên suốt tới evidence; cấm gộp nguyên nhân | test | R1-40 |
| S6 | Enum qua biên (msg, byte, judge) có giá trị tường minh và `static_assert`; cấm reinterpret theo ordinal | `static_assert` | R-05, R6-11, R7-31 |

## F. Concurrency
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| P1 | Mỗi process có một decision thread; lane chỉ nhận input bất biến; mutable object có đúng một owner thread | review + TSan | R3-17, R3-01 |
| P2 | Không giữ lock khi: publish DDS, copy message lớn, cấp phát lớn, log, cancel | review + lock-scope lint | R3-13/14/16, R5-19, R4-19 |
| P3 | Đường nóng (command 50 Hz, IMU, setpoint) không cấp phát, không dựng chuỗi, không log đồng bộ | benchmark + review | R4-04, R5-10, R3 bottleneck |
| P4 | Mỗi latency budget mới phải có phân phối đo được trước khi chốt giá trị (`PROVISIONAL` cho tới khi có) | SafetyProfile `qualification_status` | N7, N11 |

## G. Lỗi và toán số
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| M1 | Kiểm miền trước `asin`/`acos`/`sqrt`/phép chia; NaN phải bị chặn ở biên stage và không lan | unit test + clamp tường minh | R2-14 |
| M2 | Certificate liên tục hoặc có margin giữa các mẫu; check dạng sampled phải ghi rõ bound cho khoảng giữa các mẫu | test | R2-11, R7-32 |
| M3 | Gradient phải có test finite-difference; allocator và công thức vật lý phải có test kết quả số (không chỉ test finite) | CI | R2-26, R2-17 |
| M4 | Không swallow lỗi: mọi `catch(...)` phải phân loại, ghi evidence và quyết định rõ ràng | clang-tidy `bugprone-empty-catch` | R7-03, R6-04 |
| M5 | Percentile, thống kê và geometry của judge nằm trong **một** thư viện, có test | grep | R7-23 |

## H. Code
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| K1 | Hàm dài tối đa khoảng 150 dòng, CCN ≤ 25 với code mới; mega-function được tách theo phase | lizard trong CI | R-09, R1-17 |
| K2 | Cấm `const` giả: method `const` không được đổi state quan sát được; `mutable` chỉ dùng cho cache hoặc mutex | clang-tidy + review | R5-26 |
| K3 | Không dùng magic number cho ngưỡng; ngưỡng đến từ SafetyProfile hoặc `constexpr` có tên và nguồn | grep số trong điều kiện | V6, N6 |
| K4 | Static guard viết bằng AST hoặc unit test; cấm regex định dạng và `assert` trong script guard | CI | R7-10/11/12 |

## I. Evidence, test, tài liệu, quy trình
| ID | Quy tắc | Kiểm | Ngăn |
|---|---|---|---|
| E1 | Evidence là msg typed có `EvidenceHeader`; judge chỉ đọc qua decoder được sinh | CI grep `DiagnosticArray` trong judge | RC4, V7 |
| E2 | Judge không tự dẫn xuất lại ngữ nghĩa C++; nếu cần một đại lượng thì product phải publish nó | review | V7, R7-17 |
| E3 | HTML và report chỉ hiển thị verdict của `report.json`; không tự tính verdict | test | R7-41 |
| D1 | Một tài liệu kiến trúc hiện hành duy nhất (KB này và `design/`). Báo cáo lịch sử chuyển sang `docs/history/`. Tài liệu contract phải có test đối chiếu với code (tên symbol, topic) | doc-lint script | D-02, D-04 |
| D2 | Mỗi thay đổi kiến trúc có ADR; mỗi thay đổi behavior có ledger entry và evidence | PR template | AGENTS.md |
| Q1 | Refactor và behavior change nằm ở commit riêng; không tune từ một run SITL | PR review | AGENTS.md |
| Q2 | Merge khi `tools/gate.sh all` PASS trên đúng head SHA (ADR-018 E3) | PR checklist | D9 |

## J. Chống over-engineering
| ID | Quy tắc |
|---|---|
| O1 | Chỉ thêm một abstraction mới khi nó **xoá được** ít nhất một finding hoặc một bản sao. Nêu tên finding đó trong PR |
| O2 | Không xây framework chung (plugin, registry, DI) khi hiện chỉ có một implementation |
| O3 | Mỗi đường fallback hoặc retry phải có evidence cho thấy nó từng cứu một run. Không có evidence thì xoá (ví dụ retry ladder 8 bậc, patch snapshot) |
| O4 | Một package mới chỉ được lập khi ranh giới phụ thuộc cần nó; không tách chỉ để "đẹp" |
| O5 | Mặc định là **xoá**, không phải bọc lại: code thử nghiệm, code tắt trong product, API legacy |

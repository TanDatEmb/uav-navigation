# Bảng truy vết (rebuild v2)

File này trả lời câu hỏi "repo đã đi tới đâu". Mỗi dòng nối một yêu cầu với:
- mục thiết kế hiện thực nó, trong [SYSTEM_DESIGN.md](architecture/SYSTEM_DESIGN.md);
- work package (WP) thực hiện nó;
- trạng thái hiện tại;
- bằng chứng.

**Trạng thái:** `todo`, `doing`, `done`, `deferred`.

**Bằng chứng:** tên test, commit hoặc file event log. Thiếu bằng chứng thì chưa được ghi `done`.

Cột WP sẽ được điền khi có implementation plan.

## Vấn đề (P\*) → lời giải

| Yêu cầu | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|
| P1 Hai profile bay (UNKNOWN theo profile) | §0, §5.3 | S3 | | todo | |
| P2 Hai profile định vị | §0, §4.1 | S1 | | todo | |
| P3 Lệch LIO–PX4 | §4.1 (FRD, `T`), §4.2 | S1, S2 | | todo | |
| P4 Kiểu setpoint (A′: P/V/A qua `T`) | §4.2 | S2 | | todo | |
| P5 LIO mất rồi khởi động lại | §3.1, §3.4, §2.2 | S4 | | todo | |
| P6 Không GPS mà LIO fail → bàn giao có Reason | §2.3, §4.2 | S4 | | todo | |
| P7 Trạng thái 50–100 Hz với LiDAR 10 Hz | §3.3 | S1 | | todo | |
| P8 Phạm vi SITL, PX4 1.17 | §0 | S0 | | todo | |
| P9 Chất lượng cấu trúc (M1–M9) | §2, §6, AGENTS.md §2 | mọi lát | | todo | |
| P10 Chống chuyển nhánh liên tục | §2.1, §2.4, §7.3 | S3, S5 | | todo | |

## Lỗi đã kiểm chứng trên `main` (F\*) → không được tái diễn

| Lỗi | Mục spec | Lát | WP | Trạng thái | Bằng chứng (test chống tái diễn) |
|---|---|---|---|---|---|
| F13 Reset counter PX4 là tổng | §4.1 | S1 | | todo | |
| F14 Reset counter EV theo epoch của mẫu | §4.1, §3.4 | S1 | | todo | |
| F15 "Đang bay" theo z LIO, deactivate sai nhãn | §4.2 | S2 | | todo | |
| F18 Quy tắc chết về logic | §2.3 (không còn quy tắc này) | S3 | | todo | |
| F20 Mapping bị poison vĩnh viễn | §5.2 | S3 | | todo | |
| F21 Hai WorldView / hai DDA | §5.2 | S3 | | todo | |
| F22 LIO không tự chuyển LOST khi chỉ LiDAR mất | §3.1 | S1 | | todo | |
| F23 Output 50 Hz nhảy bậc | §3.3 | S1 | | todo | |
| F24, F26 Param bị ghi đè hoặc default lệch | §6.2 | S0, S3 | | todo | |
| F25 Route gate fail-open | §5.3 | S3 | | todo | |
| F27 LIO nhảy Lost→Tracking không xác nhận | §3.1 | S1 | | todo | |
| F28 Hai nguồn trạng thái cho một quyết định | §5.3 | S3 | | todo | |
| F29 Heading rebind bỏ qua certificate | §5.3 (không có đường này) | S3 | | todo | |
| F30, F31 Cancel/deadline bị coi là thành công | §5.3 | S3 | | todo | |
| F34 Nhãn NED cho EV | §4.1 | S1 | | todo | |

## Hạ tầng

| Hạng mục | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|
| Thay `tools/gate.sh`, ledger validator và test của `main` bằng gate tối giản | §7.2 | S0 | | todo | |
| Event log + script KPI | §6.1 | S0, S5 | | todo | |
| Config ba tầng | §6.2 | S0 | | todo | |
| Kiểu thời gian | §6.3 | S0 | | todo | |
| Message v2 | §6.5 | S0 | | todo | |
| Gate beta | §7.3 | S5 | | todo | |

## Lệch thiết kế đang mở

Không có. Khi phát hiện lệch: thêm mục O\* trong [DECISIONS.md](architecture/DECISIONS.md) và một dòng ở đây. Có ≥ 3 dòng thì dừng lại sửa thiết kế (§7.4).

# Wave 2: review R1 và điều phối merge (tổng công trình sư, 2026-09-30)

Baseline: `main` @ `2543b0b4`. Mọi PR dưới đây đều rẽ nhánh từ SHA này.

## 1. Verdict từng PR

| PR | WP | Head | Verdict | Điều kiện |
|---|---|---|---|---|
| #14 | H2 | `84722433` | **APPROVE, merge thứ 1** | Diff đúng spec: reject sample, giữ baseline, gate fail-closed. RED→GREEN, có ledger. |
| #16 | P0.3 | `eb1b6343` | **APPROVE (I1, I3, I4, I5), merge thứ 2** | I2 tách khỏi P0.3 (xem §3.2). Lỗi CTest `px4_ros2_cpp` là lỗi môi trường (§2). |
| #15 | J1 | `b6ff61e2` | **APPROVE, merge thứ 3** | Rebase để giải conflict ledger `docs/safety/runtime_safety_current.md` (chỉ do thứ tự append), rồi chạy lại gate trên head mới. Rejudge 16 session: 0 verdict đổi. |
| #17 | P1 PR-A | `cd7a7d8a` | **APPROVE có điều kiện, merge thứ 4** | (a) Rebase sau #14 và #16. Có conflict `src/px4/px4_navigation_external_mode/CMakeLists.txt` với #16; giữ cả hai thay đổi. (b) Kiểm A/B chống regression (§3.4). (c) `check_against_a6.py` PASS sau khi chủ dự án chốt §3.3. |
| #18 | P0.2 | `f99f56b5` | **Giữ DRAFT, chưa merge** | Session lịch sử không replay được vì thiếu PX4 binary. Cần chạy ma trận mới trên `2543b0b4` (§3.5). |

Kiểm conflict bằng `git merge-tree` trên các head:
- 14+15, 15+16: conflict ở ledger.
- 16+17: conflict ở CMakeLists.
- Mọi cặp còn lại merge sạch.

## 2. Quyết định gate (kiến trúc sư, bổ sung cho COMMON_CONTRACT_v2)

**G1.** `px4_ros2_cpp` là submodule pinned bên ngoài. `integration_tests` của nó cần FMU đang chạy (`waitForFMU` timeout). Vì vậy gate `ros` sẽ chạy CTest với `--packages-skip px4_ros2_cpp`, hoặc `--ctest-args -E integration_tests`. Log của nó chỉ ghi lại, không chặn merge.

**G2.** GitHub Actions đang bị khóa billing (job có `steps=[]`), nên check này không bao giờ xanh. Bằng chứng merge là **gate native trên đúng head SHA** (ADR-017 §6.4), dán trong PR. Sau mỗi lần rebase phải chạy lại gate.

## 3. Việc cần chủ dự án quyết

Mỗi mục có đề xuất của kiến trúc sư. Chủ dự án chỉ cần ghi "đồng ý" hoặc chọn phương án khác.

### 3.1 ADR-017 §6.3

| ID | Đề xuất |
|---|---|
| Q-HG001 | Chấp nhận giá trị đang chạy: A* 30/60 ms, solve 80 ms. Cập nhật ledger. |
| Q-ENV | Tạm giữ envelope dạng box (`√2·L`). Chỉ chuyển sang norm (`L`) khi có SITL evidence ở P2 PR-B. |
| Q-VOX | Certify theo khoảng cách tới voxel **box**, không theo tâm voxel. Tolerance lượng tử hoá nằm trong budget 0.10 m. |
| Q-UNK | Dùng một seed duy nhất, giá trị midpoint (bảo thủ). Thực hiện ở P7. |
| Q-TRK | Default `off`, khớp YAML/product. Tracking chỉ để chẩn đoán (P0.2 Addendum 1). |
| Q-XTRK | Không làm gate verdict; cross-track chỉ để chẩn đoán. |

### 3.2 P0.3 I2 (`PlanningRequest` không biểu diễn được đích và event volume độc lập)

**Đề xuất:** chọn phương án 3 ngay bây giờ: giữ API ambient, và đóng P0.3 mà không có I2.

Sau đó giải bằng phương án 1 ở WP wave 3 (P3, contract planning):
- thêm `planning_goal_world` và `goal_acceptance_radius_m` tường minh vào `PlanningRequest`;
- tách commit refactor contract khỏi commit behavior.

Không lấy code từ `codex/close-findings-implementation`.

### 3.3 A6: ba hàng owner-blocked (P1)

**Thrust.** Profile giữ scalar cấu hình `min_acc_thr` và `max_acc_thr` (đơn vị m/s², đúng với loader). Giá trị N được tính bằng `× mass` và chỉ xuất hiện trong witness.

Sửa hai hàng A6 như sau:
- `maximum_thrust_n` → `max_thrust_acceleration_m_s2`
- `minimum_thrust_n` → `min_thrust_acceleration_m_s2`
- ghi rõ dạng suy ra: `derived_n = value × mass`.

**Yaw acceleration.** Giá trị hiệu lực là 2.0 rad/s², lấy từ `planner.yaml:73`. Giá trị 0.3 ở `planning_limits.hpp:21` là default C++ chết, và sẽ bị xoá ở P1 PR-B (quy tắc: không có default C++). Sửa hàng A6 để trỏ về YAML và giá trị 2.0.

### 3.4 P1: runner FAIL / NOT_EVALUABLE

Run witness `external-mode-check-20260930T064849-78862` kết thúc bằng `MISSION_TIMEOUT`, waypoint accepted là 0..3. Trong P0.2, baseline lịch sử ra PASS 6 / FAIL 6 / BLOCKED 21. Vì vậy một run FAIL đơn lẻ không quy được cho P1.

**Yêu cầu A/B:**
- cùng scene, `tracking=off`;
- 3 run trên `2543b0b4` và 3 run trên head P1 sau rebase.

P1 đạt nếu phân phối outcome không kém baseline. Đây là kiểm tra regression, không phải tune; không đổi ngưỡng nào.

### 3.5 P0.2

- **OQ-P02-01:** chọn phương án 2. Phân loại SAFE/FAST theo policy BACKUP **hiệu lực** ghi trong session, không theo tốc độ yêu cầu.
- **OQ-P02-02:** chọn phương án 2. Bỏ `structured_corner` và ghi M2 là partial. Muốn thêm scene thì mở WP riêng; không bịa geometry.
- **Việc còn lại:** chạy ma trận mới trên SHA `2543b0b4` với PX4 binary được capture. Session lịch sử thiếu binary chỉ dùng tham khảo.
  - Merge H2 không chặn việc này: Addendum 2 §1 cho phép tạo branch từ SHA baseline.

### 3.6 J1

- **R7-27:** chọn phương án 1. `evaluation_window` áp cho cả error statistics lẫn coverage. Chạy rejudge; mọi verdict lịch sử đổi phải ghi vào debt trong ledger kèm finding ID.
- **R7-25, R7-26, R7-17:** giữ OPEN. Đường giải là **typed evidence ở P2**:
  - product phát vận tốc world-frame trong `EstimatorEvidence`, và mission đã resolve (behavior từng waypoint) trong `ExecutionEvidence`/`LifecycleEvent`;
  - judge chỉ đọc lại các giá trị này, không tự tính lại.
  - Cách này thoả F2 và E2 mà không cần binding Python. Bổ sung vào phạm vi P2 PR-A/PR-B.

## 4. Sau khi merge xong #14 → #16 → #15 → #17

- Giao **P2 PR-A**, kèm bổ sung §3.6.
- Giao **P1 PR-B**, kèm §3.3.
- P0.2 chạy ma trận mới trên máy SITL.
- Kiến trúc sư viết spec wave 3 (P3, P4-0, H3) khi P2 PR-A mở.

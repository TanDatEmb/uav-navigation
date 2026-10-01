# WP-H2: Chặn đường jump đi vào PX4 EKF2 qua mẫu có dt quá nhỏ hoặc không tăng (R5-08, S1)

**Loại:** behavior change nhỏ, cô lập. Tiền lệ là H1.
**Phụ thuộc:** baseline v2 (COMMON_CONTRACT_v2). **Môi trường:** ROS 2 Jazzy để build và test `px4_odometry_bridge`.
**Finding:** R5-08 (`docs/refactor/kb/data/findings.csv`). Ngữ cảnh: KB-06 B7, ADR-017 §6.1 H2.

## Hiện trạng (kiểm lại trên baseline trước khi sửa)
File `src/px4/px4_odometry_bridge/src/geometric_jump_continuity.cpp`:
- `:115-120`: `dt ≤ 0` gọi `reseed(..., kTimestampNotIncreasing, true, state)`.
- `:123-130`: `dt < minimum_continuity_dt_s` gọi `reseed(..., kDtTooSmall, true, state)`.

Hàm `reseed` (`:12-25`) ghi `state.last_received = current` và `continuity_trusted = true` mà **không so sánh hình học**.

Hệ quả, cả hai đều CONFIRMED qua `docs/refactor/kb/data/repro/R5/repro_jump_dt_small.cpp`:
- Một mẫu nhảy 5 m tới cách mẫu trước 50 µs sẽ trở thành baseline mới ở trạng thái trusted, và được publish nếu `TimestampConverter` cho qua. Converter chỉ chặn µs trùng hoặc lùi (`timestamp_conversion.cpp:76-92`).
- Mọi mẫu sau đó được so với baseline đã nhảy, nên latch không bao giờ bật.

Node (`px4_external_odometry_bridge_node.cpp:300-333`) cập nhật continuity **trước** khi đánh giá gate. Vì vậy ngay cả khi mẫu bị gate chặn vì timestamp, baseline vẫn đã bị thay.

## Hành vi đích (một commit behavior)
1. Với `dt ≤ 0` và `0 < dt < minimum_continuity_dt_s`: **không đổi `state`** (giữ `last_received`, `last_generation*`, `continuity_trusted`). Trả observation với:
   - `reason` giữ như cũ (`kTimestampNotIncreasing` / `kDtTooSmall`);
   - `baseline_reseeded=false`;
   - `evaluated=false`;
   - field mới `sample_admissible=false`, mặc định là `true` cho mọi nhánh khác.
2. `ExternalOdometryGateInput` thêm `geometric_sample_admissible`, mặc định `false` để giữ tính fail-closed với code chưa set. `evaluate_external_odometry_gate` thêm reason `GEOMETRIC_SAMPLE_NOT_ADMISSIBLE`, xếp ngay sau `GEOMETRIC_JUMP_LATCHED` (`external_odometry_gate.cpp:24-45`). `publication_ready` phải là `false` khi field này là `false`.
3. Node set `gate_input.geometric_sample_admissible = last_jump_observation_.sample_admissible`. Mẫu bị từ chối: không publish, `++gated_count_`, reason đi vào diagnostics hiện có. Không thêm topic mới.
4. **Không đổi:** giá trị `minimum_continuity_dt_s` và `maximum_continuity_dt_s`; mọi nhánh reseed khác, kể cả `kDtTooLarge`, `kSourceInvalid` và `kNoBaseline`. Các nhánh đó thuộc R-02 và để lại P6. Không đổi thứ tự latch.

## Test (viết trước, chạy RED trên baseline, dán output)
Viết trong `test/test_geometric_jump_continuity.cpp` và `test/test_external_odometry_gate.cpp`:
1. `SmallDtJumpDoesNotReplaceBaseline`: baseline tại p0, mẫu tại p0 + 5 m với dt = 50 µs.
   - Kết quả mong đợi: `kDtTooSmall`, `sample_admissible=false`, `state.last_received` bằng p0, `continuity_trusted` không đổi.
   - Sau đó mẫu tại p0 + 5 m với dt = 10 ms so với **baseline p0** → `kJumpDetected`.
2. `NonIncreasingTimestampJumpDoesNotReplaceBaseline`: tương tự test 1 với dt = 0 và dt < 0.
3. `SmallDtWithoutJumpKeepsBaselineAndContinues`: mẫu dt nhỏ không nhảy bị từ chối; mẫu kế tiếp hợp lệ → `kWithinThreshold`, so với baseline cũ.
4. `GateRejectsNonAdmissibleGeometricSample`: mọi input của gate đều true trừ `geometric_sample_admissible` → `publication_ready=false`, reason `GEOMETRIC_SAMPLE_NOT_ADMISSIBLE`.
5. Nếu hạ tầng test node hiện có cho phép (`test_px4_odometry_bridge.cpp`), thêm một test mức node: mẫu dt nhỏ nhảy xa không được publish; mẫu kế tiếp tại vị trí đã nhảy làm latch bật.
6. Chạy lại toàn bộ test hiện có của package và dán kết quả. Test cũ nào phải đổi assertion: DỪNG, ghi OPEN_QUESTIONS, không tự sửa assertion.

## Ledger
Ghi vào `docs/safety/runtime_safety_current.md` mục "Recent effective changes":
- Lifecycle ACTIVE, Implementation IMPLEMENTED, Evidence UNIT_VERIFIED, Authority PRODUCT.
- Mô tả: "geometric continuity rejects dt<=0 and dt<min samples without replacing the trusted baseline; such samples are not published (R5-08)".
- Ghi thêm: SITL evidence `NOT_EVALUABLE` cho tới P0.2.

Sau đó chạy `validate_runtime_safety_ledger.py`.

## Nghiệm thu
- Diff chỉ gồm 4 file source của `px4_odometry_bridge` (`geometric_jump_continuity.{hpp,cpp}`, `external_odometry_gate.{hpp,cpp}`), node cpp, các test, ledger và `docs/refactor/WP-H2/`.
- Gate verify v2 ở cả ba mức: static, python, ros (`px4_odometry_bridge` cùng reverse dependency).
- REPORT có output RED trước khi sửa và GREEN sau khi sửa.

(Áp dụng `COMMON_CONTRACT_v2.md`.)

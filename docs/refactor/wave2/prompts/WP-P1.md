# WP-P1: SafetyProfile, một nguồn cấu hình safety cho mọi process (RC3)

**Loại:** gồm 4 giai đoạn theo thứ tự:
- P1.1 (additive, không đổi behavior)
- P1.2 (witness, không đổi behavior)
- P1.3 (docs)
- P1.4 (behavior, mỗi process group một commit)

Mở **2 PR**: PR-A gồm P1.1, P1.2, P1.3; PR-B gồm P1.4, và **chỉ mở sau khi PR-A đã merge**.

**Phụ thuộc:** baseline v2; A6-R1 đã merge.
**Spec:** `docs/refactor/design/A5_contract_specs.md` §3; ADR-017 D7 và §6.2 (codegen 2 output); KB-08 nhóm D (C1–C5).
**Oracle:** `docs/refactor/WP-A6/constants.csv` (119 semantic row, 1.229 literal đã phân loại) và `safety_profile_draft.yaml`.
**Finding cần đóng:** A6 I-01..I-17, R1-13, R1-36, R2-16, R3-24, R4-11, R4-13, R6-07, R6-20, R-08, V6, R7-02, R7-08.

## Quyết định đã chốt (không tự đổi)
- **Q-HG001:** dùng giá trị đang chạy: `astar_attempt=0.03`, `astar_total=0.06`, `solve_deadline=0.08`.
- **0.15 m/s:** giữ 5 key riêng cùng giá trị (D7(b)).
- **Không có key nào đổi giá trị trong WP này.** Profile `sitl_current_as_is` chứa đúng giá trị **hiệu lực** đang chạy trong SITL.

## Quy tắc resolve cho 13 dòng `CONFLICT` trong draft
1. Giá trị profile = giá trị **hiệu lực** trong SITL: YAML đã nạp đè default C++ nếu YAML có key đó. Phải chứng minh bằng đường code nạp (`file:line`). Mọi nguồn khác ghi vào `superseded_sources`.
2. Estimator (`estimator_min_range_m`, `scan_voxel_m`, `registration_voxel_m`, `local_map_half_extent_*`, `lidar_queue_capacity`): đây là **overlay hợp lệ** giữa `sim` và `dataset`. Đánh dấu `overlay: allowed`, giá trị gốc lấy theo sim, overlay dataset lấy theo `dataset.yaml`.
3. `minimum_thrust_n` / `maximum_thrust_n` (planner.yaml so với `planning_limits.hpp:22-23`, hệ số ×1.64): xác định mỗi giá trị được dùng ở đâu.
   - Nếu cả hai đều được dùng, với hai ngữ nghĩa khác nhau: tạo 2 key với tên nói rõ ngữ nghĩa.
   - Nếu không xác định được: DỪNG dòng này, ghi OPEN_QUESTIONS.
4. `numerical_roundoff_epsilons`: **không đưa vào profile** (quy tắc K3: `constexpr` có tên là đủ). Riêng `anchor_pvaj_roundoff_tolerances` vẫn đưa vào (HG-027).
5. `world_observation_fault_duration_ms`: thuộc fault injection, **không** đưa vào product profile (A5 quy tắc 8).

## P1.1: package và codegen (additive)
1. Tạo package `src/contracts/nav_safety_profile/` (ament_cmake, không phụ thuộc ROS ngoài yaml-cpp).
2. Nguồn dữ liệu: `config/safety_profile/sitl_current_as_is.yaml`. Mỗi key gồm các trường:
   ```yaml
   value:
   unit:
   hg:              # nếu có
   qualification_status:  # PROVISIONAL | ACTIVE | NOT_EVALUABLE
   owners:          # danh sách process
   overlay:         # allowed | forbidden
   derived_from:    # nếu là DERIVED
   sources:         # file:line
   superseded_sources:
   ```
   Nhóm key theo `docs/refactor/design/A5_contract_specs.md` §3.2. Nhóm `judge` và `evidence` đặt trong section `qualification:`.
3. Codegen `tools/safety_profile/gen.py`:
   - (a) sinh `nav_safety_profile/generated/profile.hpp`: struct typed, dùng `TimestampNs`/`DurationNs` cho thời gian nếu `nav_core_types` đã tồn tại, nếu chưa thì dùng `int64_t` ns kèm hậu tố `_ns`. Loader `load(path) → Result<Profile>` với **mọi key bắt buộc**, `validate()` kiểm miền giá trị;
   - (b) sinh `tools/safety_profile/profile.py`: dataclass + loader, cùng quy tắc;
   - (c) hash: SHA-256 của bản canonical (key sort, số in bằng `repr` / `%.17g`). Hàm `hash64` lấy 64 bit đầu.
4. Key `DERIVED` (`planning_radius_sum_m`, `minimum_main_reserve_s`, …) được **tính** trong codegen theo công thức trong `planning_timing.hpp:14-15` và HG-005. YAML chỉ khai báo công thức, không ghi số.
5. Test:
   - gtest: thiếu một key thì load fail; sai miền thì fail; hash ổn định;
   - Python và C++ cho cùng hash trên cùng file (test chéo qua một fixture);
   - thêm vào `check_ci_contract.py` hoặc tạo guard mới `check_safety_profile_generated.py`: chạy lại gen rồi `git diff --exit-code`.
6. **Oracle test:** script `tools/safety_profile/check_against_a6.py` đối chiếu từng row `INCLUDED` của `constants.csv` với profile, gồm tên, giá trị và đơn vị. Mọi row không đưa vào profile phải có lý do lấy từ bảng quy tắc ở trên. Kết quả mong đợi: 0 row thiếu, 0 row lệch.

## P1.2: witness (không đổi behavior, không enforce)
1. Mỗi process product (`fast_lio`, `navigation_runtime`, `px4_navigation_external_mode`, `px4_external_odometry_bridge`, `px4_odometry_bridge`) nhận parameter mới `safety_profile.path`, mặc định là file đã install trong share của `nav_safety_profile`.
2. Lúc khởi động, mỗi process load profile, rồi so **giá trị hiệu lực của chính nó** với profile cho các key có `owners` chứa process đó.
3. Log đúng **một** dòng: `SAFETY_PROFILE_WITNESS hash=<hex16> keys=<n> mismatches=[k=effective/profile,...]`.
4. **Không** fail, không đổi giá trị. Vẫn dùng giá trị hiện có.
5. Runner (`tools/runtime/runner.py`, theo pattern witness ở `runner.py:2319-2458`) đọc dòng witness của từng role và ghi vào `metadata.runtime_configuration.safety_profile`. Mismatch **chỉ ghi lại**, chưa làm FAIL.
6. Test: unit test cho hàm so sánh; Python test cho parser witness của runner.

## P1.3: docs
1. Sửa HG-001 trong `docs/safety/runtime_safety_current.md:91` thành giá trị đang chạy, kèm lineage:
   - trích nguồn 180 ms và 40/80 ms từ archive;
   - lý do: Q-HG001 trong ADR-017 §6.3.
2. Không đổi code.
3. Chạy `validate_runtime_safety_ledger.py`.

## P1.4: consumer lấy giá trị từ profile (behavior; PR-B, **mỗi dòng dưới đây là một commit**)
Mỗi commit phải có đủ ba thứ: test RED, ledger entry, và witness sau commit cho `mismatches=[]`. Giá trị không đổi. Behavior mới chỉ là "thiếu key thì fail lúc start".

| Commit | Nội dung |
|---|---|
| P1.4a | Runtime + planner: `planning_timing.hpp` constexpr → giá trị từ profile. Bỏ default C++ của `planner_core/config.hpp` cho các key safety (R2-16 flatness g/m, `config.hpp:202` yaw rate). Nguồn nominal speed thống nhất cho request (R1-13) **chỉ ở mức lấy từ một chỗ**; logic stage thuộc P5. Chú ý: default của `DynamicLimits` trong `planning_limits.hpp:15-25` (yaw rate 5.0, thrust 6.0×1.64…) là **nguồn thứ ba** cho yaw rate, bên cạnh YAML 2.0 và `config.hpp` 1.5. Truy xem default nào thật sự được dùng trong SITL; không truy được thì ghi OPEN_QUESTIONS |
| P1.4b | Adapter: bỏ 4 literal pin `navigation_mode_node.cpp:255-258` (V6). Tham số lấy từ profile |
| P1.4c | EV bridge và ingress bridge |
| P1.4d | LIO: `parameter_loader.cpp` bỏ các default lệch (R4-11 frame name, R4-13 gap: ba ngưỡng trỏ về **một** key; không đổi giá trị) |
| P1.4e | Mapping: `rog_map` config dùng `required=true` cho các key safety (R6-20); gộp hai bản plane height (R6-07) nếu giá trị hiệu lực bằng nhau, nếu khác thì DỪNG và ghi OPEN_QUESTIONS |
| P1.4f | Judge: `vehicle_radius` (R-08) và freshness theo stream (R7-08) đọc từ section `qualification`. Mỗi judge dùng key riêng của mình; không gộp giá trị với product |
| P1.4g | Mission YAML: bỏ `acceptance_speed` và các key safety khác (D7(d)). Mission loader báo lỗi nếu mission YAML còn chứa key safety |

## Ngoài phạm vi
- Không tạo msg `ConfigWitness` (thuộc P2).
- Không đổi bất kỳ giá trị nào.
- Không refactor logic planner hay runtime ngoài việc thay nguồn đọc giá trị.

## Nghiệm thu
- **PR-A:**
  - `check_against_a6.py` cho 0 thiếu, 0 lệch;
  - gen-check PASS;
  - gate v2 PASS ở cả static, python và ros (cho `nav_safety_profile` và 5 package có witness);
  - REPORT dán dòng witness của từng process, chạy bằng launch hoặc bằng test node có param thật.
- **PR-B:** mỗi commit có RED → GREEN; `mismatches=[]`. Có một run SITL smoke nếu máy có PX4 SITL; nếu không có thì ghi `NOT_EVALUABLE` và PR giữ draft.

(Áp dụng `COMMON_CONTRACT_v2.md`.)

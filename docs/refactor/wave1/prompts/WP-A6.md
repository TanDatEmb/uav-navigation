# WP-A6 — Kiểm kê ngưỡng và hằng số, làm đầu vào cho SafetyProfile

**Loại:** phân tích read-only. **Phụ thuộc:** WP-D0. Có WP-A2 (`icd_params.yaml`) thì tốt nhưng không bắt buộc.

## Mục tiêu
Tìm mọi giá trị có ý nghĩa vật lý hoặc an toàn (thời gian, khoảng cách, tốc độ, gia tốc, jerk, góc, tỉ lệ, số lần retry, kích thước queue), cùng mọi chỗ nó xuất hiện: C++, YAML, Python judge và tài liệu HG. Mục đích là để P1 gom chúng vào một `nav_safety_profile` duy nhất **mà không đổi giá trị nào**.

## Phạm vi
- `src/**` (trừ external, vendor, test): `constexpr`, `static constexpr`, literal số trong biểu thức so sánh, default của `declare_parameter`, `LoadParam(..., default)`.
- `config/runtime/*.yaml`, `src/*/*/config/*.yaml`, `config/runtime/missions/*.yaml` (chỉ các key số).
- `tools/runtime/*.py`: ngưỡng judge, default, ví dụ `vehicle_collision_radius_m`, `stale_after_s`, `0.35` residual, `{420, 430, 520}`.
- `docs/safety/runtime_safety_current.md`: bảng HG-001…HG-036.

## Deliverable
### 1. `docs/refactor/WP-A6/constants.csv`
`group (timing|freshness|lease|envelope|geometry|tolerance_numeric|retry|queue|evidence|judge), proposed_name (snake_case), value, unit, semantics (1 câu), hg_id (nếu có), occurrences (danh sách: file:line|kind[constexpr|literal|param_default|yaml|python|doc]|value_at_site), all_equal (bool), yaml_loadable (bool), pinned (bool), notes`.
- Mỗi *ý nghĩa* là một dòng; không lập mỗi occurrence thành một dòng riêng. Đã biết tối thiểu: 0.15 m/s stationary ×6, 0.5 s freshness ×5, 0.20 s state age, 0.10 s lease, 5.0 s timeout, 0.35 m vehicle radius ×4, 10 vs 12 m/s, tolerance anchor PVAJ ×2 (`candidate_bundle.hpp:398`, `execution_anchor.hpp:47`), 0.20 m goal tolerance.
- `tolerance_numeric`: các epsilon dưới 1e-6 thuần túy để chống sai số làm tròn thì gom chung một nhóm, không cần đặt tên riêng từng cái.

### 2. `docs/refactor/WP-A6/inconsistencies.md`
Chỉ ghi các dòng có `all_equal=false`, hoặc cùng ý nghĩa mà một chỗ YAML-loadable còn chỗ khác là literal, hoặc là param bị pin. Mỗi mục kèm kịch bản cụ thể: đổi YAML X thì chỗ Y vẫn giữ giá trị cũ, dẫn tới hệ quả Z.

### 3. `docs/refactor/WP-A6/safety_profile_draft.yaml`
Bản nháp cấu trúc profile, gom theo group. Giá trị lấy **đúng bằng giá trị hiện đang chạy trong SITL** (resolve YAML override → C++ default). Mỗi key có comment chỉ nguồn. Khi các nguồn khác giá trị nhau thì ghi `CONFLICT: [...]` thay vì chọn một giá trị.
```yaml
freshness:
  observation_max_age_s: 0.5      # HG-006; mapping.yaml:22 data_freshness_window_s; sim.yaml:83 ...
lease:
  adapter_command_lease_s: 0.10   # planning_timing.hpp:19; external_mode.yaml ...; pinned navigation_mode_node.cpp:255
```

## Nghiệm thu
- Script `docs/refactor/WP-A6/scan.py` tái tạo được danh sách occurrence. REPORT ghi rõ regex đã dùng và cách lọc false positive.
- Mọi HG có giá trị số trong `runtime_safety_current.md` đều map được sang ít nhất một dòng.
- Không sửa `src/` và `config/`.

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

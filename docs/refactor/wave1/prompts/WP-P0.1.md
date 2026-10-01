# WP-P0.1 — CI build C++ thật (ROS 2 Jazzy) + sửa test không hermetic (R-04, R-10)

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** tooling/CI. KHÔNG đổi code product. **Phụ thuộc:** WP-D0. **Môi trường:** Docker trên máy dev, GitHub Actions.

## Bối cảnh (đã xác nhận trên main)
- `.github/workflows/ci.yml` hiện chỉ chạy whitespace, 9 static guard và Python unittest. Không build C++. Hosted runner đang bị khoá do billing (`artifacts/pre_main_consolidation/20260925T124709Z-0b477638/CI_CONTRACT.md`).
- R-04: trên clone sạch chạy `python3.12 -m unittest discover -s tools/runtime/tests -p 'test_*.py'` cho 460 test, 1 FAIL: `test_runtime_contract.py:1291-1292` đòi phải có `install/navigation_runtime/share/navigation_runtime/config/planner.yaml`.
- R-10: `tools/runtime/flight_review_report.py:278` dùng `sum()` float, cho 0.87 trên Python 3.12 nhưng 0.8700000000000001 trên 3.11; `test_html_report.py:365` so bằng `assertEqual`.
- Build/test chuẩn: `tools/runtime/build.py --mode release build|test` (danh sách package: `PRODUCT_BUILD_PACKAGES`, `PRODUCT_TEST_PACKAGES`), gate đầy đủ: `tools/runtime/pre_main_gate.py`. `uav_simulation` cần `gz-msgs10`, `gz-transport13`, `ros_gz_bridge`. Submodule: `src/external/px4_msgs`, `src/external/px4_ros2_interface_lib`.

## Việc cần làm
1. **R-04.** Tách assertion phụ thuộc install-tree ra một test riêng, chỉ chạy khi install tồn tại: `@unittest.skipUnless(installed_planner.is_file(), "requires colcon install tree")`. Phần kiểm CMake/YAML còn lại giữ nguyên trong test cũ. Job ROS (bước 3) phải chạy test này **sau khi build**, và ở đó nó KHÔNG được skip: dùng env `UAV_NAV_REQUIRE_INSTALL_TESTS=1`, và khi biến này bật mà test bị skip thì coi là fail.
2. **R-10.** `sum(...)` → `math.fsum(...)` tại `flight_review_report.py:278`; test dùng `assertAlmostEqual(..., places=9)`. Grep toàn bộ `tools/runtime/*.py` tìm những chỗ `sum(` trên float mà kết quả được so bằng `==` hoặc `assertEqual`, rồi liệt kê trong REPORT. Chỉ sửa những chỗ có test đỏ hoặc có so sánh `==` với ngưỡng.
3. **Job CI mới** `uav-nav / ros-jazzy-build` trong `.github/workflows/ci.yml`:
   - `runs-on: ubuntu-24.04`, `container: ros:jazzy-ros-base` (hoặc `osrf/ros:jazzy-desktop` nếu thiếu dependency; ghi lý do chọn).
   - checkout với `submodules: recursive`; `rosdep install --from-paths src --ignore-src -y`; cài `ros-jazzy-ros-gz` cho `uav_simulation`.
   - `python3 tools/runtime/build.py --mode release build`, rồi `python3 tools/runtime/build.py --mode release test`, rồi chạy Python unittest với `UAV_NAV_REQUIRE_INSTALL_TESTS=1`.
   - Cache ccache theo hash lockfile/commit của submodule.
   - Upload artifact: log colcon, `build/*/Testing`, kết quả junit.
   - KHÔNG chạy PX4 SITL/Gazebo trong CI.
4. **Chạy local khi hosted CI bị khoá:** `tools/ci/local_ci.sh` dựng đúng container và chạy đúng các bước của job trên (cả 3 job: static, python, ros-jazzy-build). Thêm target `make ci-local`. Script trả exit code khác 0 nếu bất kỳ bước nào fail.
5. **Guard cho CI:** thêm `tools/check_ci_contract.py`. Nó fail khi workflow không còn job build C++, hoặc khi danh sách package test trong workflow lệch với `PRODUCT_TEST_PACKAGES`. Thêm guard này vào `static_guard_names()` của `pre_main_gate.py` và vào job static.
6. Thêm `docs/refactor/WP-P0.1/CI.md` mô tả các job, cách chạy local, và thời gian chạy đo được.

## Ngoài phạm vi
Sửa test C++ đang fail (nếu có): chỉ báo cáo, không sửa. Sửa cảnh báo compiler. Bật `-Werror`. TSan (để dành cho P4).

## Nghiệm thu
- Trên clone sạch không có `install/`: `python3.12 -m unittest discover -s tools/runtime/tests -p 'test_*.py'` → 0 failure. Skip được phép và phải liệt kê tên.
- `make ci-local` trên máy dev exit 0. REPORT dán tóm tắt: số package build, số CTest pass/total, số Python test, thời gian chạy.
- `actionlint` (hoặc `python -c "import yaml"` parse) workflow không lỗi.
- Diff không đụng `src/**` ngoài test Python dưới `tools/runtime/tests`.
- Nếu CTest có test C++ fail: ghi tên test và log vào REPORT, đánh dấu `PRE_EXISTING_FAILURE`, KHÔNG sửa.

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

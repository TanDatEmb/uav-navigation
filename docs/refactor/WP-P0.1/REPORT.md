# WP-P0.1 — Báo cáo nghiệm thu CI/build contract

## 1. Tóm tắt

WP-P0.1 đã thêm job `uav-nav / ros-jazzy-build` chạy build/test C++ thật trên
`ros:jazzy-ros-base` với runner Ubuntu 24.04.

Job ROS checkout submodule đệ quy, cài dependency qua apt/rosdep, dùng ccache,
chạy Release build và product CTest, rồi chạy Python suite sau build.

Install-dependent test được tách khỏi structural test và có guard fail-closed
khi `UAV_NAV_REQUIRE_INSTALL_TESTS=1`.

R-10 đã đổi phép cộng float của robot radius sang `math.fsum` và đổi assertion
tương ứng sang `assertAlmostEqual(..., places=9)`.

Static contract guard kiểm tra sự tồn tại của job C++ và đối chiếu danh sách
package test với `PRODUCT_TEST_PACKAGES`.

`make ci-local` đã được thêm nhưng chưa thể chạy end-to-end vì host không có
Docker hoặc Podman; script dừng với lỗi rõ ràng thay vì fallback sang host.

Không có thay đổi dưới `src/**`, không khởi động PX4/Gazebo/SITL, và không có
thay đổi ngưỡng an toàn runtime.

## 2. Deliverables

- `.github/workflows/ci.yml`: thêm `ros-jazzy-build`, ccache, artifact upload,
  install-required Python test và static contract guard.
- `tools/check_ci_contract.py`: kiểm tra workflow YAML, môi trường ROS, cấm
  runtime command, và exact package-list drift.
- `tools/ci/local_ci.sh`, `Makefile`: chạy ba job tương ứng trong container;
  fail-closed nếu không có Docker/Podman.
- `tools/runtime/tests/test_runtime_contract.py`: tách install-tree assertion
  và thêm `UAV_NAV_REQUIRE_INSTALL_TESTS=1` guard.
- `tools/runtime/flight_review_report.py:278` và
  `tools/runtime/tests/test_html_report.py`: sửa R-10 (`math.fsum` và
  `assertAlmostEqual(..., places=9)`).
- `tools/runtime/pre_main_gate.py`: đưa `check_ci_contract.py` vào static guard.
- `docs/refactor/WP-P0.1/CI.md`: mô tả job, local runner, cache, artifact và
  thời gian đo được.
- `docs/refactor/WP-P0.1/REPORT.md`: báo cáo này.

## 3. Verification evidence

Các lệnh dưới đây chạy trên baseline `7e0b8508781f68ecbd18d15d40129e019108f3e7`
và worktree `refactor/WP-P0.1`, sau khi khởi tạo submodule tại đúng SHA:

| Kiểm tra | Kết quả |
|---|---|
| `python3 tools/check_ci_contract.py` | exit 0, `CI CONTRACT: PASS` |
| Parse `.github/workflows/ci.yml` bằng PyYAML | exit 0; có `python`, `ros-jazzy-build`, `static-contract` |
| `bash -n tools/ci/local_ci.sh`; `py_compile` các Python file đổi | exit 0 |
| Static guards + ledger validator + `git diff --check` | exit 0; 0.45 s cho static guard run |
| Full Python pre-build | exit 0; 462 tests, 3 skips, `Ran 462 tests in 5.201s`; wall 5.42 s |
| Pre-build `UAV_NAV_REQUIRE_INSTALL_TESTS=1` install checks | exit 1 có chủ đích; guard báo thiếu colcon install tree |
| Release C++ build | exit 0; 23 packages finished, 33m25s; chỉ warnings stderr, không có build failure |
| Release product CTest | exit 0; `93 tests, 0 errors, 0 failures, 0 skipped`; 20.11 s |
| Post-build Python với `UAV_NAV_REQUIRE_INSTALL_TESTS=1` | exit 0; 462 tests, 1 skip, `Ran 462 tests in 5.029s`; wall 5.16 s |
| `git diff --check` sau toàn bộ thay đổi | exit 0 |
| `make ci-local` | exit 2 từ Make, script exit 127: host không có Docker/Podman; chưa phải CI-container PASS |

Skip còn lại sau build là test artifact GUI đã có từ trước:
`test_tracking_experiment_report_preserves_velocity_only_from_real_artifact_shape`.
Hai install-dependent test không bị skip trong post-build run.

R-10 audit các `sum(...)` float còn lại trong `tools/runtime/*.py` cho thấy:

- `external_mode_scenario.py:2695`, `monitor.py:828` và `report.py:2245` là
  tính norm dùng trong bất đẳng thức ngưỡng; không phải equality red case.
- `report.py:3189` cộng số lượng integer trong outcome map.
- Vì vậy chỉ sửa phép cộng robot-radius và assertion liên quan, không thay đổi
  gate hay semantics của các kiểm tra khác.

## 4. Phạm vi và kết quả

- Product test package list gồm 23 package; workflow và guard lấy cùng nguồn
  `PRODUCT_TEST_PACKAGES` từ `tools/runtime/build.py`.
- Submodule đã được init ở các SHA hiện hành: `px4_msgs`
  `86d8239e962f6939e05c3737784f60c02fa884db` và
  `px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- Build đã ghi authoritative manifest vào install tree.
- Các số đo host trực tiếp: static 0.45 s; Python pre-build 5.42 s wall;
  ROS build 33m25s; CTest 20.11 s; Python post-build 5.16 s wall. Exact
  container/GitHub Actions timing vẫn là `NOT_MEASURED`.
- Không có file `src/**` thay đổi; không có C++ source/test thay đổi; không có
  SITL/Gazebo runtime invocation.

## 5. Deviations và open questions

- Workflow/local ROS job cài trực tiếp `libfmt-dev` và `libyaml-cpp-dev`, rồi
  dùng `rosdep ... --skip-keys` cho đúng hai tên này. Đây là workaround CI cho
  package.xml hiện tại: chúng là apt package name nhưng không phải rosdep key
  Jazzy hợp lệ. Các dependency khác vẫn fail-closed qua rosdep. Host không root
  không hoàn tất được bước rosdep vì cần cài `ros-jazzy-ros-gz-sim`; container
  chạy root có apt package explicit tương ứng. Không đổi package.xml trong WP này.
- `make ci-local` chưa nghiệm thu được do thiếu container runtime. Cần chạy lại
  trên máy có Docker/Podman hoặc trên GitHub Actions để xác nhận exact container
  path và thời gian.
- Không có `actionlint` trong host; workflow syntax đã được parse bằng PyYAML
  và contract guard, nhưng hosted Actions run vẫn là kiểm tra cuối cho runner.
- Skip GUI artifact là evidence gap có sẵn, không thuộc scope WP-P0.1.

## 6. Commit và provenance

- Baseline: `main` tại `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Implementation commit: `49b86e99589db4c632abc388f559967e27c9596c`.
- Report commit: commit chứa file này; SHA cuối cùng được xác nhận bằng
  `git rev-parse HEAD` trong handoff cùng draft PR.
- D0 architecture/ADR/risk documents chỉ được đọc từ
  `origin/refactor/WP-D0`; không được copy vào diff WP-P0.1.

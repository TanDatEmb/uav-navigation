# WP-P0.1 — CI build ROS 2 Jazzy

## Mục tiêu

CI có một job build/test C++ thật trên clone sạch, đồng thời giữ hai job
nhanh cho guard tĩnh và Python. CI không khởi động PX4 SITL hoặc Gazebo.

## Các job

| Job | Môi trường | Phạm vi |
|---|---|---|
| `uav-nav / static-contract` | `ubuntu-24.04`, Python 3.12 | `git diff --check`, các guard kiến trúc, `check_ci_contract.py`, safety-ledger validator |
| `uav-nav / python` | `ubuntu-24.04`, Python 3.12 | `tools/runtime/tests` trên clone chưa có install tree; test install-dependent được skip có chủ đích |
| `uav-nav / ros-jazzy-build` | `ubuntu-24.04` runner, `ros:jazzy-ros-base` container | checkout submodule đệ quy, `rosdep`, `ros-jazzy-ros-gz`, build Release, CTest product suite, Python suite với `UAV_NAV_REQUIRE_INSTALL_TESTS=1` |

Danh sách test package của job ROS được khai báo tại
`UAV_NAV_PRODUCT_TEST_PACKAGES` trong workflow và được
`tools/check_ci_contract.py` đối chiếu theo thứ tự với
`PRODUCT_TEST_PACKAGES` trong `tools/runtime/build.py`.

Cache dùng `ccache`, key gồm các manifest `package.xml`, `.gitmodules` và SHA
của hai submodule `px4_msgs`/`px4_ros2_interface_lib`. Artifact ROS luôn upload
`log/**`, `build/**/Testing/**` và `test-results/**`, kể cả khi bước trước fail.

## Chạy local

```bash
make ci-local
```

`tools/ci/local_ci.sh` chạy lần lượt ba job trong các container tương ứng:
`ubuntu:24.04`, `ubuntu:24.04`, rồi `ros:jazzy-ros-base`. Script dừng ở lỗi đầu
tiên và trả exit code khác 0 nếu bất kỳ job nào fail. Script chỉ cài
`ros-jazzy-ros-gz` và `ros-jazzy-ros-gz-sim` để biên dịch/test package `uav_simulation`; `libfmt-dev` và
`libyaml-cpp-dev` được cài trực tiếp vì đây là tên apt package hiện có nhưng
không phải rosdep key Jazzy. `rosdep` vẫn fail-closed cho mọi dependency khác.
Script không gọi runner PX4, Gazebo hay SITL. Có thể chọn runtime bằng
`UAV_NAV_CONTAINER_RUNTIME` (Docker hoặc Podman).

## Thời gian chạy đo được

| Đo trên | Static | Python pre-build | ROS build | CTest | Python post-build | Ghi chú |
|---|---:|---:|---:|---:|---:|---|
| WP-P0.1 worktree, direct host, 2026-09-28 | 0.45 s | 5.42 s wall / 5.201 s test | 33m25s | 20.11 s; 93/93 pass | 5.16 s wall / 5.029 s test | Đây là số đo host trực tiếp, không phải số đo container/GitHub Actions |
| Exact container (`make ci-local`) | `NOT_MEASURED` | `NOT_MEASURED` | `NOT_MEASURED` | `NOT_MEASURED` | `NOT_MEASURED` | Môi trường không có Docker/Podman; script fail-closed |

Các số đo host ở trên chỉ chứng minh các lệnh đã chạy được trên checkout hiện
tại; chúng không thay thế output của `make ci-local` hoặc run GitHub Actions.
Sau khi container runtime khả dụng, cần ghi bổ sung thời gian exact-container
bằng output thật, không dùng số ước lượng làm bằng chứng nghiệm thu.

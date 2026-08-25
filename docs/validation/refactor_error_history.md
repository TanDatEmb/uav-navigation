# Lịch sử lỗi trong quá trình tái thiết kế

Tài liệu này ghi nhận lỗi phát hiện trong từng phase refactor và kết quả
validation song song. Đây là nhật ký kỹ thuật, không phải danh sách phiên bản
code. Mỗi mục phải giữ nguyên lệnh kiểm chứng và artifact để có thể truy vết.

## Quy ước xử lý

- **Fix ngay**: lỗi phá vỡ build, contract, ownership, timestamp/frame, hoặc
  safety gate hiện tại; phải sửa trước khi giao build cho validation tiếp theo.
- **Ghi nhận sau refactor**: lỗi không chặn phase hiện tại nhưng không được
  tuyên bố đã hoàn tất; phải đóng trước acceptance cuối.
- **Môi trường**: thiếu dependency/hardware/dataset hoặc hạ tầng runner; không
  được biến thành bypass trong product code.

## Nhật ký

| Thời điểm/phase | Phát hiện và bằng chứng | Phân loại | Xử lý / điều kiện đóng |
|---|---|---|---|
| 2026-08-25, contract và tree | Runtime còn phụ thuộc tên/package cũ, source tree trộn interface, runtime, mapping và planner backend; generated artifact cũ cũng làm nhiễu review. | Fix ngay | Đã đổi tên các package sản phẩm chính, chuyển generated artifact cũ sang `.artifacts/retired-generated-20260825/`, và thêm layout/ADR. Cần tiếp tục quét tên ngoại lai còn lại sau khi native planner boundary hoàn tất. |
| 2026-08-25, planning/execution boundary | Planner backend vẫn giữ trajectory committed và runtime còn gọi trực tiếp các hàm commit/sample; facade planning/execution mới chưa thay thế toàn bộ đường chạy. | Ghi nhận sau refactor | Đã thêm `navigation_planning` contracts và `navigation_execution` committed store/sampler; chưa đóng cho tới khi runtime chỉ nhận candidate immutable và một execution authority duy nhất. |
| 2026-08-25, typed state | State propagated trước đây đi qua raw runtime struct, chưa có epoch, source/receive timestamp và frame contract dùng chung. | Fix ngay | Đã thêm `KinematicState`, `ExecutionStateStore`, kiểm tra body/world frame và reset theo localization epoch; đã kiểm chứng runtime 107/107. Cần giữ coverage khi cắt hẳn raw path. |
| 2026-08-25, PX4 command gate | Command chỉ kiểm tra hình thức; `valid_until` chưa bị từ chối khi hết hạn và provenance mission/request chưa được đối chiếu đầy đủ. | Fix ngay | Đã thêm kiểm tra thời gian hiệu lực, mission/waypoint/request identity và reset odometry theo epoch; contracts 5/5, external mode 78/78. SITL vẫn mở. |
| 2026-08-25, planner state boundary | Runtime còn dựng `PlannerExecutionState` rồi đổi sang `RobotState` vendor, làm trùng contract state và che khuất ownership timestamp/frame/yaw. | Fix ngay | Đã chuyển planner sang nhận `navigation_planning::KinematicState`, tính yaw tại odometry boundary và loại bỏ adapter trung gian; build xanh, CTest planning 1/1 executable (4 cases), backend 2/2 (40 cases), runtime 8/8. |
| 2026-08-25, mapping ownership | Lần build đầu của `MappingActor` dùng kiểu quaternion không tồn tại trong adapter (`rog_map::Quatf`). | Fix ngay | Đổi biên giới chuyển pose sang Eigen quaternion/vector, build `navigation_runtime` xanh và test mapping 57/57, runtime 107/107. |
| 2026-08-25, mapping API | Runtime vẫn lộ kiểu map/vendor trong header và tự decode/giữ cloud; mutable map đã chuyển vào `MappingActor` nhưng input adapter và telemetry chưa tách hết. | Ghi nhận sau refactor | Tiếp tục đưa cloud, outcome và telemetry về `navigation_mapping` product types; không để runtime giữ mutable map handle. |
| 2026-08-25, validation song song | Agent validation đang chạy bounded sim/dataset/replay trên build đã xanh; kết quả chưa có tại thời điểm ghi mục này. | Chờ kết quả | Chỉ ghi PASS sau khi có scenario, command, artifact path, timestamp và verdict; failure safety phải quay lại fix ngay. |
| 2026-08-25, test artifact provenance | `colcon test-result` cộng một XML 85 test cũ trong build cache với XML 4 case hiện tại, tạo tổng số 89 gây hiểu nhầm. | Fix ngay | Chuyển XML stale sang `.artifacts/test-results-cleanup-20260825/`; CTest hiện tại xác nhận đúng 4 case trong 1 executable của `navigation_planning`. |
| 2026-08-25, test environment | Gọi CTest không source ROS/workspace làm fail `ament_cmake_test`, sau đó source ROS nhưng chưa source workspace làm thiếu `libnavigation_contracts__rosidl_generator_c.so`. | Môi trường | Chuẩn hóa lệnh verification thành `source /opt/ros/jazzy/setup.bash && source install/setup.bash && ctest ...`; với đầy đủ overlay, runtime 8/8 executable passed. Không sửa product code để che lỗi môi trường. |
| 2026-08-25, parallel validation checkpoint | Agent Erdos xác nhận trên snapshot `d3fd0e5`: release build 21 packages, focused 480/480, Python 139/139; dataset shadow planning PASS 100 READY/0 EMERGENCY, mapping 1692/1692. | Ghi nhận sau refactor / môi trường | Dataset replay bị giới hạn 180 giây (`IMU 34173/55435`, `LiDAR 1709/2772`) nên report FAIL vì thiếu source count; SITL `sanity_open/positive` không launch do build manifest stale sau source transition. Cần rebuild/manifest mới rồi chạy lại; chưa có E2E/SITL acceptance claim. Artifact: `.artifacts/runtime/dataset-20260825T154858-128557/report.json`, `.artifacts/runtime/external-mode-check-20260825T155240-132169/report.json`. |
| 2026-08-25, adversarial advisor | Lượt cố vấn Copernicus không trả report sau timeout và được đóng để giải phóng tài nguyên; không có kết luận được dùng làm evidence. | Môi trường agent | Khởi tạo lượt cố vấn Poincare mới, bounded/read-only; chỉ tích hợp findings khi có report và exact anchors. |
| 2026-08-25, adversarial architecture review | Advisor Archimedes chỉ ra runtime vẫn gọi trực tiếp backend commit/sample (`navigation_runtime_node.cpp`), mapping header còn lộ map/vendor types, và planner backend còn giữ execution state; đây là ownership risk dù focused tests xanh. | Ghi nhận sau refactor | Đã đóng state input bằng `KinematicState`; tiếp theo phải chuyển hẳn candidate commit/sample sang `navigation_execution`, che vendor types khỏi runtime API, rồi chạy sanitizer/replay/SITL lặp lại. Không coi unit-test green là acceptance. |
| 2026-08-25, release provenance checkpoint | Sau checkpoint docs `8c5e9b0`, build Release authoritative đã cập nhật manifest với `git_head=8c5e9b0`, `git_dirty=false`, source fingerprint khớp và 318 artifact; validation manifest hợp lệ. CTest chính xác: planning 1/1 executable, backend 2/2, runtime 8/8; Python runtime contract 139/139. | Đã xác minh | Giữ checkpoint này làm rollback point. Đây mới là build/unit-contract evidence; chưa thay thế sanitizer, dataset đầy đủ nguồn, SITL lặp lại hoặc hardware acceptance. |
| 2026-08-25, bounded parallel validation | Hai agent mới được giao một lượt dataset/SITL và một lượt phản biện kiến trúc trên `8c5e9b0`, nhưng không trả báo cáo trong thời gian bounded; đã gửi lệnh chốt, đóng cả hai và xác nhận không còn detached worktree. | Môi trường agent | Không sử dụng lượt này làm PASS/FAIL sản phẩm và không thay đổi gate. Dùng báo cáo Erdos trước đó cùng review Archimedes làm evidence đã có; các lượt validation tiếp theo phải giới hạn thời gian và ghi artifact ngay khi có kết quả. |

## Điều kiện đóng nhật ký

Nhật ký chỉ được đóng khi toàn bộ mục “ghi nhận sau refactor” đã có bằng chứng
đóng, các bypass tạm thời đã được gỡ hoặc có quyết định an toàn rõ ràng trong
decision ledger, và validation cuối đã truy vết ngược được mọi lỗi về commit,
test, scenario và artifact. Không dùng một lần chạy đơn lẻ để nâng ngưỡng hoặc
tuyên bố acceptance SITL/hardware.

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

## Điều kiện đóng nhật ký

Nhật ký chỉ được đóng khi toàn bộ mục “ghi nhận sau refactor” đã có bằng chứng
đóng, các bypass tạm thời đã được gỡ hoặc có quyết định an toàn rõ ràng trong
decision ledger, và validation cuối đã truy vết ngược được mọi lỗi về commit,
test, scenario và artifact. Không dùng một lần chạy đơn lẻ để nâng ngưỡng hoặc
tuyên bố acceptance SITL/hardware.

# Báo cáo P9: chất lượng cấu trúc code (2026-10-06)

Báo cáo này trả lời câu hỏi "code hiện tại tệ ở đâu, vì sao, và bằng chứng là gì"
cho việc thiết kế lại (O9). Dữ liệu thô (manifest, bản ghi đọc JSONL, kết quả clang-tidy/cppcheck) nằm local ở
`docs/superpowers/analysis/2026-10-05/` và tái lập được bằng `docs/superpowers/analysis/tools/`. Các phát hiện quan trọng đã được kiểm tay và ghi vào
[decision log](../architecture/DECISIONS.md) (F11–F32).

## 1. Phương pháp và độ phủ

| Tầng | Công cụ | Kết quả |
|---|---|---|
| Đo tự động | `lizard` | 2202 hàm first-party; 111 hàm CCN > 25; 55 hàm > 150 dòng |
| Đo tự động | `clang-tidy` (bugprone, performance, readability, concurrency) | 801 cảnh báo, trong đó 86 truy cập `optional` chưa kiểm |
| Đo tự động | `cppcheck` | 312 mục; các out-of-bounds đã kiểm tay là false positive |
| Đo tự động | `inventory.py` | 130 cờ bool thành viên, 215 điểm log (65 không kèm giá trị), 118 `declare_parameter` |
| Đọc có kiểm soát độ phủ | 25 chunk, agent đọc từng dòng, `coverage.py verify` | **2202/2202 hàm**, 1887 issue, mọi trích dẫn khớp source, 0 lỗi kiểm chứng |

`coverage.py` chỉ bảo đảm **không bỏ sót** (mọi hàm có bản ghi, hàm > 300 dòng
được chia đoạn ≤ 150 dòng phủ kín, trích dẫn có thật). Nó không bảo đảm agent
**phát hiện hết** lỗi. Mức tin cậy:

- **Đã kiểm tay:** F13–F18, F20, F22, F24, F25, F28, F30, F31 (ghi rõ trong log).
- **Khả nghi, cần test:** F29.
- **Còn lại:** phát hiện của agent, có trích dẫn nhưng chưa kiểm tay từng cái.

## 2. Phân bố issue

| Package | Tổng | logging | param | naming | state_flag | correctness | state_conflict | timing |
|---|---|---|---|---|---|---|---|---|
| navigation_planning_backend | 621 | 110 | 92 | 70 | 52 | 66 | 44 | 32 |
| navigation_runtime | 337 | 76 | 24 | 32 | 39 | 14 | 20 | 34 |
| px4_navigation_external_mode | 173 | 39 | 11 | 9 | 28 | 7 | 7 | 14 |
| fast_lio_core | 158 | 31 | 25 | 16 | 9 | 7 | 24 | 10 |
| fast_lio_ros | 156 | 29 | 22 | 10 | 9 | 8 | 8 | 16 |
| navigation_execution | 109 | 12 | 5 | 17 | 13 | 2 | 17 | 13 |
| px4_odometry_bridge | 93 | 12 | 8 | 9 | 12 | 18 | 8 | 5 |
| Các package khác | 240 | | | | | | | |

Theo loại: logging 346, param 213, naming 202, state_flag 176, correctness 146,
state_conflict 144, timing 139, complexity 113, dead_code 92, cross_layer 74,
concurrency 71, budget 58.

Một hàm, `NavigationRuntimeNode::runCycle` (3564 dòng, CCN 778), chứa 112 issue.

## 3. Chín mẫu gốc rễ, đối chiếu với vấn đề owner nêu

Mỗi mẫu dưới đây lặp lại ở nhiều package. Vì vậy đây là lỗi thiết kế, không phải
lỗi cục bộ.

### M1. State machine ngầm bằng cờ
*Owner nêu: "kiểm soát bằng nhiều cờ, chuyển trạng thái phải bật cờ đúng chuỗi", "trạng thái không rõ, xung đột".*

- **External mode:** quyền sở hữu được mã hoá bằng 4 cờ kiểm theo thứ tự cố định; `onActivate`/`onDeactivate` reset tay khoảng 30 trường (F15).
- **Execution:** 5 facet được cập nhật theo các tập con khác nhau, nên có thể vừa TrackingMain vừa TrackBackup; `invalidate()` xoá cả latch fail-closed (F19).
- **FAST-LIO:** không có bảng chuyển trạng thái, status bị đổi từ 5 nơi (F27).
- **Runtime:** trạng thái hoàn thành nằm ở 5 cờ, được xoá tay ở hơn 10 chỗ; `validateRetainedCommand` quyết định qua khoảng 10 bool rồi một chuỗi if/else 11 nhánh.
- **Khác:** `MappingWorker` dùng 7 bool, `MissionProgress` khoảng 9 optional/bool, A* 5 bool giải mã từ bitmask.

### M2. Không có chủ sở hữu duy nhất cho một quyết định
*Owner nêu: "phân nhiệm vụ không rõ ràng", "bố trí lớp không tốt".*

- **Reset epoch localization:** 3 callback cảm biến khác nhau đều có thể kích hoạt, mỗi lần là khoảng 25 lệnh ghi.
- **"Cùng goal":** có 3–4 định nghĩa không tương đương.
- **Trajectory start time:** có hai nơi cùng ghi.
- **Gate sensing-horizon:** đọc `solve_state_` của lần solve trước, trong khi request đã cập nhật `robot_state_` (F28).
- **Callback mapping:** dài khoảng 550 dòng, nằm trong constructor, và tự quyết định giữ hay thu hồi command.

### M3. Kết quả nhiều nghĩa, dẫn tới fail-open
*Owner nêu: "nhiều trạng thái xung đột", "hiểu nhầm".*

- Route-regression gate **mở** khi route hỏng hoặc thiếu (F25).
- Optimizer nominal chấp nhận kết quả đã bị huỷ hoặc quá deadline (F31).
- Optimizer BACKUP trả `-1.0` khi cancel, nên bị coi là thành công (F30).
- Quy tắc hoãn anchor ở terminal STOP không bao giờ kích hoạt (F18).
- `optimize()` trả về một số lẫn nghĩa: cost cộng mã trạng thái, hoặc 1.0, hoặc 0.0.
- Cặp `applicable`/`valid` mang ba nghĩa khác nhau.

### M4. Log không truy được nguyên nhân
*Owner nêu: "log không rõ ràng để debug nguyên nhân".*

- **Tổng quan:** 346 issue logging, nhiều nhất trong mọi loại.
- **Return im lặng:** khoảng 35 đường thoát trong `runCycle` return mà không có log.
- **Diagnostic sai chỗ:** được publish trước mọi gate, nên không bao giờ nói được vì sao dừng.
- **External mode:**
  - LIO chuyển valid→invalid mà không có log;
  - mọi deactivate đều bị gắn nhãn OPERATOR_TAKEOVER (F15).
- **Lý do bị ghi đè:**
  - FAST-LIO ghi đè `reason` ở mỗi chuyển trạng thái;
  - cancel/deadline bị log thành lỗi corridor (F30).
- **Kênh log sai:** planner log ra `stdout`, và 16 hook visualization đều rỗng.

### M5. Param chưa chuẩn hoá
*Owner nêu: "param chưa chuẩn hóa gây ra hành vi không đồng nhất các lớp".*

- **Trùng lặp và đa nghĩa:**
  - cùng một tunable có hai bản sao ở hai đơn vị (s và ns);
  - bản ns còn kiêm 4 nghĩa: freshness, tuổi world, cửa sổ export, lease.
- **YAML bị bỏ qua:**
  - giá trị `exp_traj` trong YAML bị ghi đè (F24);
  - `local_window_m` là param nhưng bị khoá đúng 20.0.
- **Ràng buộc che giấu sai sót:**
  - external mode khai báo 4 param nhưng ném exception nếu chúng khác literal;
  - tolerance route-regression lấy từ `RouteProgressConfig{}` mặc định, không theo mission.
- **Default lệch nhau giữa các lớp:**
  - `grav` mặc định 1.0 trong khi chỗ khác dùng 9.81 (F26);
  - correction-age 250 ms trong code nhưng 0.50 s trong ROS.

### M6. Timeline và đồng hồ không nhất quán
*Owner nêu: "timeline không chuẩn", "budget chưa khảo sát".*

- **Đồng hồ:**
  - mỗi cycle runtime gọi `now()` khoảng 9 lần, xen lẫn steady clock;
  - LiDAR timeout dùng thời điểm nhận theo `steady_clock`, trong khi phần còn lại dùng sensor time.
- **Budget:**
  - thời gian chờ khoá bị tính vào budget solve;
  - p50/p95/p99 chỉ đếm các cycle chạy tới cuối.
- **Giám sát LIO:** `superviseLidarTimeout` không bao giờ chạy khi IMU vẫn đến, nên LIO không tự chuyển sang LOST (F22).
- **Odometry 50 Hz:** nhảy bậc ở mỗi correction 10 Hz (F23).
- **Bridge sang EKF2:**
  - gửi reset counter lấy từ luồng health chậm (F14);
  - không xử lý được bước reset kép của PX4 (F13).

### M7. Đồng thời và khoá
*Owner nêu: "chia process không tốt".*

- **Publish dưới khoá:**
  - `publishCommand` publish ROS trong khi giữ mọi owner lock;
  - external mode publish setpoint khi giữ `trajectory_mutex_`.
- **Callback dưới mutex:**
  - execution chạy callback transport dưới mutex của store;
  - mapping store chạy certificate của execution dưới khoá publish.
- **Data race:** `gi_.new_goal` và `planner_previous_exp_` được bảo vệ bằng hai mutex khác nhau ở hai luồng.
- **Không tự phục hồi:**
  - một exception làm planning worker chết vĩnh viễn;
  - một scan rỗng làm mapping khoá vĩnh viễn (F20).

### M8. Xuyên tầng
*Owner nêu: "lỗi xuyên tầng".*

- Planner tự suspend command và ghi anchor của mission.
- World model hard-code thân X500/Mid360.
- `isGoalSegmentTraversable` hard-code kAllowUnknown, bỏ qua policy của mission.
- External mode suy ra "đang bay" từ z của LIO.

### M9. Code test, thử nghiệm và code chết trong đường sản phẩm
*Owner nêu: "làm quá phức tạp chỉ tạo thêm gánh nặng".*

- Khoảng 60 dòng predicate failure-injection nằm trong `runCycle`; ngoài ra có param `inject_*` trong node sản phẩm.
- `tracking_experiment_` làm thay đổi quyết định an toàn ngay trong đường sản phẩm.
- Code chết:
  - khoảng 1000 dòng quickhull chỉ được test dùng;
  - nhánh frontier của A* không thể tới;
  - `ReferencePointConverter` không được dùng.

## 4. Hệ quả cho thiết kế lại (đầu vào cho O9)

Các nguyên tắc dưới đây suy ra trực tiếp từ M1–M9. Chúng chưa được owner chốt.

1. **State machine tường minh, một writer.** Mỗi thành phần (LIO lifecycle, execution, external mode, mission progress) có một `enum` trạng thái, một bảng chuyển trạng thái và một hàm `transition(event) → (state, effects)` duy nhất. Không còn cờ bool rời cho trạng thái. (M1, M2)
2. **Kết quả có kiểu, fail-closed theo mặc định.** Mọi quyết định trả về `Result<T, Reason>`, với `Reason` là enum đầy đủ. Không dùng số hay bool để mã hoá nhiều nghĩa. Thiếu dữ liệu nghĩa là bị từ chối. (M3)
3. **Mỗi quyết định ghi một bản ghi sự kiện có cấu trúc** gồm event, state trước/sau, reason và các giá trị dẫn tới quyết định. Bản ghi được ghi ở điểm quyết định, không ghi trước gate. Thay `cout`/`printf` bằng một logger duy nhất. (M4)
4. **Một schema config có kiểu và đơn vị**, được validate một lần khi khởi động, có một nguồn duy nhất cho mỗi giá trị. Không có default ẩn khác YAML. (M5)
5. **Mỗi cycle chụp một mốc thời gian**, với clock domain tường minh: sensor, ROS, steady. Budget được đo không tính thời gian chờ khoá. (M6)
6. **Không publish hay gọi callback khi đang giữ khoá.** Ranh giới thread phải tường minh, và mọi lỗi worker đều có đường phục hồi. (M7)
7. **Ranh giới lớp do kiểu dữ liệu ép buộc**, không do quy ước. (M8)
8. **Code test và thử nghiệm nằm ngoài binary sản phẩm**, hoặc sau một compile flag. (M9)

## 5. Hạn chế

- 4 TU có lỗi biên dịch dưới clang (xung đột tên `vec_E`, `using` trùng), nên clang-tidy chỉ phân tích được một phần các TU này.
- Phát hiện về concurrency được rút ra từ đọc code tĩnh, chưa chạy ThreadSanitizer.
- Budget và latency chưa được đo runtime; cần SITL kèm tracing.
- Code ngoài hàm (khai báo lớp, biến toàn cục) do `inventory.py` phủ, không đi qua manifest.

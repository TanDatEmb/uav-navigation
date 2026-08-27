# Timing contract

`Timestamp` and `Duration` carry signed integer nanoseconds. Absolute timestamps
must never be represented as `float` or `double`; a local duration may be
converted to seconds only at the calculation boundary. `ClockDomain` values may
not be mixed without an explicit synchronization model.

Only `RosTimeConverter` converts a ROS header timestamp to the core type. A
LiDAR adapter documents whether its header is scan start, end, or another event;
it must not assume. `LidarScan` has explicit start/end times and each point's
`relative_time_ns`. IMU time is the measurement time, never callback time.

For real scans, point time is `scan_start + relative_time_ns`; synchronization
must bracket the scan interval with IMU and reject gaps/regressions. For Gazebo
simultaneous scans all relative times are zero and deskew is reported as bypassed,
not fabricated from point index. `auto` timing belongs to offline development
only; production configurations are explicit.

## Chọn nguồn time của hệ thống

`use_sim_time` là selector duy nhất ở boundary ROS:

- SITL và replay đặt `use_sim_time=true`. Các node dùng ROS `/clock`, và
  `/clock` phải được kiểm tra là đã nhận, dương, đơn điệu và còn fresh trước khi
  coi runtime là đang chạy theo simulation time.
- Hardware realtime đặt `use_sim_time=false`. Các node dùng system ROS clock;
  không được tự chuyển sang simulation time khi `/clock` xuất hiện hoặc khi
  timesync mất.

`timing.clock_domain` không chọn clock. Nó chỉ mô tả domain của timestamp đi
  vào estimator (`simulation_time`, `sensor_time`, `ros_time`, ...). Không được
  so sánh trực tiếp hai domain khác nhau.

`runtime.streams.simulation_clock` vẫn có thể xuất hiện trong monitor vì đó là
  stream evidence của `/clock`, không phải parameter điều khiển. Cờ
  `xrce_synchronized` đã bỏ vì không tạo ra mapping có thể kiểm chứng.

PX4 `VehicleOdometry` có timestamp nội bộ theo PX4 boot time. Với đường ROS 2
qua uXRCE-DDS, việc có áp dụng `session.time_offset` hay không là contract của
PX4 client, không phải của product bridge. Khi transport time-sync được bật,
PX4 sở hữu việc áp dụng offset lúc serialize/deserialize sau khi đồng bộ hội tụ;
product không được áp dụng thêm một offset cục bộ. `TimesyncStatus` chỉ là bằng
chứng transport (source, estimated offset, round-trip time), không phải một
parameter enable thứ hai.

Hai deployment contract là khác nhau và không được trộn:

- SITL: `use_sim_time=true`, `/clock` của Gazebo là source duy nhất. Harness đặt
  `UXRCE_DDS_SYNCT=0` để không đưa wall-clock DDS vào simulation epoch; bridge
  dùng identity trong ROS simulation epoch. `/clock` phải dương, đơn điệu và
  fresh; nếu không, bridge/runtime fail-closed.
- Realtime hardware: `use_sim_time=false`, ROS system clock là source duy nhất
  và `UXRCE_DDS_SYNCT=1` (mặc định của PX4 hoặc cấu hình tương đương) phải được
  xác nhận trên firmware. External odometry chỉ nhận source `ros_time` hoặc
  `system_time`, rồi gửi timestamp lấy từ ROS clock cho PX4. Không tự cộng
  `estimated_offset`; nếu DDS time-sync không hội tụ hoặc source là
  `sensor_time`, đường dữ liệu phải bị chặn.

Vì PX4 có thể dùng các topic message definition khác nhau theo firmware, việc
kiểm chứng realtime phải ghi lại `TimesyncStatus.source_protocol`,
`estimated_offset`, `round_trip_time` và độ lệch giữa timestamp VehicleOdometry
đã nhận với ROS clock. Không dùng một boolean local để thay thế evidence này.

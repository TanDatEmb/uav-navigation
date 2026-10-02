# W3-T1b — brake overshoot

## Verdict

`NOT_MEASURED`: chưa có PX4 `.ulg`/`.ulog` trong session input, nên chưa thể
tính tốc độ vào phanh, gia tốc lệnh/thực, sai số bám, lag hoặc cruise anchor
error. Không có kết luận ngưỡng và không có đề xuất tuning.

## Input audit

- Input root: `/home/letandat/Dev/uav-navigation-w3-c1/.artifacts/`.
- Có 20 `report.json` của các session external-mode.
- Có rosbag `.db3`, `metadata.yaml` và log ROS/PX4; không có file `.ulg` hoặc
  `.ulog`.
- `pyulog` đã hiện diện trong môi trường, nhưng không có ulog để đọc.

## Scope decision

Theo W3-T1b, khi thiếu ulog phải dừng phân tích. Vì vậy branch này không tạo
`brake_events.csv`, `cruise_tracking.csv`, tool hoặc test giả tạo dữ liệu.
Khi có ulog thật, chạy lại audit này trên cả session cũ và session C1 sạch;
chỉ báo cáo các ô có `n >= 5`.

## Verification

    find /home/letandat/Dev/uav-navigation-w3-c1/.artifacts -type f \\
      \( -iname '*.ulg' -o -iname '*.ulog' \) -print

Kết quả: không có output.

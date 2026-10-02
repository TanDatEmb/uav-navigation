# W3-T1b — brake overshoot

## Verdict

`NOT_MEASURED`: chưa có PX4 `.ulg`/`.ulog` trong session input, nên chưa thể
tính tốc độ vào phanh, gia tốc lệnh/thực, sai số bám, lag hoặc cruise anchor
error. Không có kết luận ngưỡng và không có đề xuất tuning.

## Input audit

- Input roots audited: the retained legacy baseline artifacts and the clean C1
  cohort under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`.
- The clean C1 cohort contains 48 complete session bundles, all bound to
  navigation SHA `c435a4f19da0455cd538cb7ae111c0ba00e2e644`.
- The retained sessions contain `report.json`, ROS bags (`.db3`),
  `metadata.yaml`, and ROS/PX4 logs, but no `.ulg` or `.ulog` file.
- `pyulog` is available in the system interpreter; the missing input, not the
  parser, is the blocker.

## Scope decision

Theo W3-T1b, khi thiếu ulog phải dừng phân tích. Vì vậy branch này không tạo
`brake_events.csv`, `cruise_tracking.csv`, tool hoặc test giả tạo dữ liệu.
Khi có ulog thật, chạy lại audit này trên cả session cũ và session C1 sạch;
chỉ báo cáo các ô có `n >= 5`.

## Verification

    rg --files /home/letandat/Dev/uav-navigation/.artifacts \\
      /home/letandat/uavnav-w3-c1-baseline-20261001 \\
      /home/letandat/uavnav-w3-c1-part2-golden-20261002 \\
      | rg -i '\.(ulg|ulog)$'

Kết quả: không có output; `pyulog` import được bằng `/usr/bin/python3`.

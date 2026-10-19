# Sổ các lần chạy SITL của S1c

Mục đích: mỗi `run:<id>` trong [TRACEABILITY.md](../TRACEABILITY.md) phải tra được ở đây hoặc trong một báo cáo phân tích
cùng thư mục (`arch_check.py` kiểm điều này). Artifact thật nằm local ở `log/sitl/<id>/` (không vào Git): `summary.json`,
bản ghi JSONL, event log, log PX4. Chỉ ghi các con số đã đọc từ `summary.json` hoặc từ bản ghi của lần chạy. Sổ của S1b:
[2026-10-s1b-sitl-runs.md](2026-10-s1b-sitl-runs.md).

| Run | Lệnh | Commit | Kết quả | Ghi chú |
|---|---|---|---|---|
| `20261014T061438Z-gps-hover-yaw1.2` | `make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2` | `a693bcf` | PASS | S1c-T1 lần 1, `EKF2_EV_CTRL=15`. Heading: la bàn (`mag_hdg` rồi `mag_3d`), EV yaw không bao giờ được fuse. Độ trải `yaw_T` 0.075°, `yaw_T` 19.53°. Drift 0.028 m. Xem [báo cáo EV đầy đủ](2026-10-s1c-ev-full.md) |
| `20261014T062206Z-no_gps-hover-yaw1.2` | `make sitl-hover PROFILE=no_gps SPAWN_YAW_RAD=1.2` | `a693bcf` | PASS | S1c-T1 lần 2. Heading: la bàn rồi EV yaw từ 9.76 s (reset −20.47°, trước VALID). Độ trải `yaw_T` 0.053°, `yaw_T` 0.000°. Drift 0.007 m |
| `20261014T062427Z-gps-hover-yaw1.2-gnssout10` | `make sitl-hover PROFILE=gps SPAWN_YAW_RAD=1.2 GNSS_OUTAGE_S=10` (tuỳ chọn đã gỡ ở `52fbf46`) | `a693bcf` | FAIL (bỏ khỏi quyết định) | S1c-T1 lần 3. PX4 bỏ qua lệnh tiêm lỗi GNSS (không ack), GNSS vẫn được fuse suốt cửa sổ, nên vế lần 3 của quy tắc plan không được đánh giá và lần này bị bỏ khỏi quyết định. Ngoài ra `T` đi VALID→INVALID (STALE) ở 32.90 s rồi INIT→VALID ở 37.53 s do chuỗi `NO_PX4_SAMPLE` dài hơn 1 s (DV2, nguyên nhân còn mở), không liên quan GNSS hay EV |

Ghi chú: mã thời gian là UTC; mốc giây trong cột ghi chú là thời gian `/clock` của lần chạy. Bộ chấm là
`tools/uavnav/sitl_checks.py` ở commit trong cột Commit (luật F34 theo D42). Từ `52fbf46` luật F34 tách cửa sổ ở mỗi
reset heading và mỗi output không VALID; chấm lại offline hai lần PASS bằng luật đó vẫn PASS (độ trải 0.075° và 0.053°).

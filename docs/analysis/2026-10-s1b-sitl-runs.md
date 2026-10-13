# Sổ các lần chạy SITL của S1b

Mục đích: mỗi `run:<id>` trong [TRACEABILITY.md](../TRACEABILITY.md) phải tra được ở đây hoặc trong một báo cáo phân tích
cùng thư mục (`arch_check.py` kiểm điều này). Artifact thật nằm local ở `log/sitl/<id>/` (không vào Git): `summary.json`,
bản ghi JSONL, event log, log PX4. Chỉ ghi các con số đã đọc từ `summary.json` của lần chạy.

| Run | Lệnh | Kết quả | Ghi chú |
|---|---|---|---|
| `20261013T151158Z-gps-hover` | `make sitl-hover PROFILE=gps` | FAIL | Chưa có profile Fast DDS: nhận 775 trong 913 cloud (khoảng 85%). `lio_flap_rate` 11/60 s, `lio_tracking` 16 lần vào TRACKING, `alignment_valid` 11 chu kỳ VALID→FROZEN→VALID, `ekf2_ev_pos` 84.5%. Nguyên nhân do harness (xem [báo cáo F34](2026-10-s1b-f34-check.md) và commit `3abb661`) |
| `20261013T152418Z-gps-hover` | như trên | PASS | Có profile Fast DDS: 898 trong 898 cloud; `ekf2_ev_pos` 100%; drift 0.032 m; `/lio/state` 100 Hz; EV age p99 28 ms |
| `20261013T154709Z-gps-hover` | như trên | PASS | Lần 1 của [báo cáo F34](2026-10-s1b-f34-check.md); `yaw_T` dao động 0.076° |
| `20261013T154923Z-gps-hover-yaw1.2` | `… SPAWN_YAW_RAD=1.2` | PASS | `yaw_T` lệch −1.71° so với π/2 − yaw spawn |
| `20261013T155144Z-no_gps-hover` | `make sitl-hover PROFILE=no_gps` | PASS | `yaw_T` ≈ 0 (EKF2 lấy heading từ EV yaw) |
| `20261013T155429Z-no_gps-hover-yaw1.2` | `… SPAWN_YAW_RAD=1.2` | PASS | `yaw_T` ≈ 0 với yaw spawn 1.2 rad (D35) |
| `20261013T155805Z-gps-hover` | `… HOVER_MODE=hold` | PASS | Hold dự phòng; PX4 tự sang AUTO_LOITER sau cất cánh |
| `20261013T162711Z-gps-altitude_sweep` | `… PATTERN=altitude_sweep` | PASS (kiểm dữ liệu) | LIO LOST ở độ cao thật 4.76 m; xem [báo cáo hiệu chỉnh](2026-10-s1b-degeneracy-calibration.md). Đây cũng là lần duy nhất ép đường lỗi: health không còn TRACKING nên EV dừng và `T` đi VALID→FROZEN→INVALID |
| `20261013T163717Z-gps-hover` | `make sitl-hover PROFILE=gps` | PASS | Chạy lại sau khi sửa driver dùng chung ở Task 13 |
| `20261013T232211Z-gps-hover` | như trên | PASS | Hover cuối của S1b, sau `make sanitize` và `gate.sh all` |

Ghi chú: mã thời gian là UTC. Các lần chạy sau ngày 2026-10-13 dùng bộ kiểm đã siết ở `6832cbf`; sau khi D35 đưa kỳ vọng
F34 vào bộ kiểm, các run F34 phải được chấm lại và vẫn PASS (xem commit tương ứng).

# Dữ liệu của KB

| File | Nội dung |
|---|---|
| `findings.csv` | 223 finding, sắp theo mức độ. Các cột: `id, area, severity, category, confidence, location, excerpt (nguyên văn trên 7e0b850), claim, failure_scenario, dup_of, direction, verify, architect_check` |
| `coverage_matrix.csv` | 771 file của repo: vùng, LOC, độ sâu đã review, các câu Q1–Q12 đã trả lời, ghi chú |
| `repro/<vùng>/` | Script tái hiện độc lập do reviewer viết. Chúng chỉ mô phỏng lại logic đã trích, không link với code product. Khi giao việc sửa, chuyển chúng thành test RED trong repo |

Các script repro:

| Vùng | Script | Tái hiện finding |
|---|---|---|
| R1 | `repro_rebind_roles.py` | R1-07 |
| R1 | `check_backup_braking.py` | kiểm toán stop polynomial (kết quả: đúng) |
| R2 | `asin_domain.py` | R2-14 |
| R2 | `pm_alloc_repro.py` | R2-17 |
| R2 | `simplify_sfc_loop.py` | R2-24 |
| R2 | `yaw_stop_check.py` | kiểm công thức yaw stop (kết quả: đúng) |
| R4 | `imu_saturation.cpp` | R4-01 |
| R4 | `gap_overlap.cpp` | R4-02 |
| R5 | `repro_jump_dt_small.cpp` | R5-08 |
| R5 | `repro_tracking_envelope.cpp` | R5-16 |
| R5 | `repro_bridge_quat_sign.cpp` | R5-03 |
| R5 | `repro_seconds_to_ros_time.py` | R5-01 |
| R6 | `repro_math.py` | R6-01, R6-02 |
| R7 | `pct_repro.py` | R7-23 |

Giá trị `verify`:
- `OK`: excerpt khớp nguyên văn trong khoảng dòng được cite (±8 dòng).
- `OK_MULTI_LOCATION`: finding cite nhiều chỗ; excerpt khớp nguyên văn trong các file được cite.
- `OK_LINE_FIXED`: số dòng được kiến trúc sư sửa lại.

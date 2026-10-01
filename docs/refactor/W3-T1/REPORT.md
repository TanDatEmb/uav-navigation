# W3-T1 REPORT — C1 failure triage (read-only, lớp L)

Dữ liệu: 20 session của C1 (18 M1 + 2 M2) trên scene `legacy`, một máy, một cohort. Không SITL mới, không đổi product code, không đề xuất ngưỡng. Số đếm, không phải phân phối. Tái lập: `python3 tools/refactor/triage_sessions.py <session dirs> --csv out.csv --json out.json`. Dữ liệu chi tiết: `triage_runs.csv`, `triage_aggregate_m1.json`, `findings_T1.csv`, `OPEN_QUESTIONS.md`.

## Commit
| SHA | message |
|---|---|
| d8c52de | feat(tooling): W3-T1 read-only session triage tool |
| eb561bf | docs(W3-T1): triage data, findings T1-01..T1-07 and open questions |

## Kết luận
1. **Chuỗi nhân quả chung, 16/16 run dừng (CONFIRMED).** Renewal solve hỏng đúng lúc cầu path-relative MAIN không khả dụng và sai số neo > 0.25 m → runtime phát phanh khẩn → xe vượt điểm cuối lệnh phanh 0.70–0.88 m → vượt envelope 0.75 m → `safetyStopNavigation`. Cổng cuối: adapter `TRACKING_ENVELOPE` 14/16; runtime 2/16 (run 10 `STOPPED_HOLD` 0.753 vs 0.750 m; run 14 `PlanFromRest` timeout 5 s, thời điểm dừng chỉ là ước lượng).
   - Solve hỏng có neo > 0.25 m và `phase_execution_bridge_usable`=0: phanh khẩn 70/70. Cầu khả dụng: phanh 0/33.
2. **ADR-020 G-1/G-2 không được số liệu ủng hộ.** `kWorldAdvanced` 8/451 solve (1,8%), 0/16 mở đầu chuỗi hỏng cuối. `execution_boundary_rejection`=0 ở 451 bản ghi (G-2 CONDITIONAL: trường chỉ là giá trị chốt cuối). Điều kiện ADR-020 §5.2 không thoả.
3. **Nhãn `outcome` của C1 không phải nguyên nhân (CONFIRMED).** `NO_EXECUTION_AUTHORITY` chỉ xuất hiện sau khi `planner_failed` đã chốt.
4. **Mở đầu chuỗi hỏng (16 run dừng):** dynamics 9, known-free 3, deadline 80 ms 3, certificate route-regression 1, world advance 0. Tracking/anchor là yếu tố khuếch đại và cổng kết thúc. Nhãn `-7` → `commit_recertification/world_changed` là mislabel (`replan_contract.hpp:101-102`).
5. **Estimator bị loại (CONFIRMED ở mức cờ chẩn đoán):** 18/18 LIO `TRACKING`, `observability_rejection`=0, `px4.estimator_fault_events`=0.
6. **Q3 (hồi quy so với `7e0b850`): NOT_EVALUABLE** — không có session mang SHA đó.
7. **Q4 (run 4, 5 khác 16 run kia): CONDITIONAL, n=2.** Không có ngưỡng tốc độ tách sạch; FAST không phân biệt (FAST 7/9 dừng, SAFE 9/9 dừng).
8. **OQ-1 của C1 là nhầm trường (CONFIRMED):** `deaff86e` là `provenance.external_px4.git_head` (PX4). Nav là `provenance.manifest.source.git_head`=`24ec0fc` nhưng `git_dirty=true`, `file_count=939` (T1-07).

## Top-3 theo số run
| Hạng | Nguyên nhân | Finding | WP đề xuất |
|---|---|---|---|
| 1 | phanh/BACKUP bị xe vượt ≥ envelope 0.75 m | T1-02 | T1b đo SITL (lớp L); T1c quyết định thiết kế (kiến trúc sư) |
| 2 | solve hỏng khi cầu MAIN không khả dụng và neo > 0.25 m | T1-03 | T1a observability; T1c |
| 3 | mở đầu bằng solve hỏng (dynamics/known-free/deadline/certificate) | T1-05, T1-04 | T1d phát lại bằng trace C2 (K3) |

Hệ quả cho thứ tự WP (đề xuất, kiến trúc sư quyết định): T1 không chỉ ra G-1/G-2 nên không cần WP sửa ADR-020 §5.2 trước F1; nhưng F1/F2 thừa hưởng T1-02/T1-03, nên T1a/T1b nên đi trước hoặc cùng F1/F2.

## Giới hạn / NOT_MEASURED
Một scene, một máy, một cohort. Neo/phase trong trace chỉ có ở nhánh validate-giữ-lại và bị carry-over. Lý do cầu bị từ chối, anchor error steady-state, telemetry controller PX4, quá đà cuối phanh của lần phanh sống sót: NOT_MEASURED. Hai session M2 `MISSION_TIMEOUT`: NOT_ANALYSED.

## Gate
static=PASS; python=30/30 + 422/422 OK (skipped=2, Python 3.12.3); ros=NOT_RUN (không đổi `src/`, `config/`). Sửa C1 phần 2: `baseline_summary.py` đọc nhầm `git_head` (T1-07) và dùng nhãn outcome làm cột nguyên nhân (T1-01).

## Deviation
(1) Q3 NOT_EVALUABLE. (2) Tool đọc `planner_trace` vì trace P4-0 chưa có. (3) Báo cáo này do coordinator ghi lại từ kết quả của agent T1 (harness chặn sub-agent ghi file `.md` báo cáo); reviewer nên đối chiếu với `findings_T1.csv`.

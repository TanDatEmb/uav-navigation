# WP-P0.2 — Baseline SITL: corpus MCAP, phân phối đo và golden snapshot

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** đo đạc + tooling. KHÔNG đổi code product. **Phụ thuộc:** WP-D0; nên có WP-P0.1 để có build Release chuẩn. **Môi trường:** máy dev có ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic (máy dùng để chạy `make run`). Máy này về sau là **máy baseline**, và mọi so sánh hiệu năng đều phải chạy trên nó (ADR-016).

## Vì sao cần
Mọi phase P3–P6 phải chứng minh "không xấu hơn baseline". SITL hiện không deterministic (wall timer, 5 thread), nên baseline phải là **phân phối** qua nhiều run, cộng với **golden corpus ở mức component** (nominal planner snapshot) để so bit-level cho planner.

## Việc cần làm
### 1. Chuyển bag sang MCAP (tooling, commit riêng)
- `tools/runtime/runner.py:1246`: `--storage sqlite3` → tham số `--bag-storage {mcap,sqlite3}`, mặc định `mcap`.
- Các tool đọc bag đang hard-code `storage_id="sqlite3"` (`tools/runtime/extract_e5_px4_layers.py:29,67,110`, và các chỗ khác nếu có): tự nhận storage từ `metadata.yaml`.
- Test: tạo một bag mcap nhỏ bằng `ros2 bag record` trong 2 s trên topic giả, rồi đọc lại bằng tool.

### 2. Script tổng hợp `tools/refactor/baseline_summary.py` (mới, read-only với session)
- Input: danh sách thư mục session `.artifacts/runtime/external-mode-check-*`.
- Output: `baseline_runs.csv`, mỗi run một dòng: `session, profile_or_scene, speed_mps, policy (SAFE|FAST), run_idx, outcome, runtime_verdict, c0_sw_status, c0_ifp_status, accepted_waypoints, expected_waypoints, mission_time_s, planning_p50/p95/p99/max_ms, planning_n, certificate_aggregate_max_ms, command_gap_max_ms, lease_rejection_count, tracking_gt_p95_m, min_clearance_m, collision_count, stale_events_{propagated_odometry,external_odometry,lidar,imu}, clock_gap_max_ms, rtf_mean, bag_path, bag_bytes`.
- Lấy dữ liệu từ `report.json` / `scenario.json` / `monitor.json` / `runtime.json`; không tính lại logic của judge. Field nào không có thì ghi `NOT_AVAILABLE`.
- **Lưu ý R-01:** `stale_events_*` phải lấy thẳng từ `monitor.json` (số raw trong active window), KHÔNG lấy từ reasons của runtime verdict. Lý do: verdict hiện che stall khi outcome là COMPLETE.
- Output thứ hai: `baseline_distribution.csv`, gom theo `(profile, speed, policy)`: tỉ lệ các outcome, p50/p95/p99/max của các metric số, n.

### 3. Chạy ma trận baseline
Build Release sạch ở `7e0b850` (cộng thêm commit tooling của bước 1–2, không có gì khác). Chạy **tuần tự**, mỗi run một ROS domain/xrce port, không chạy song song.
- **M1 (bắt buộc), ma trận frozen 5 m/s:** 2WP `long_three_pillars_speed`, 5WP `long_three_pillars`, 9WP `long_three_pillars_multiwaypoint` × {SAFE, FAST} × 3 run = 18 run. Mẫu lệnh ở `docs/reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md:184-195`. Cách chọn SAFE/FAST phải xác định cho đúng cách report đó đã chạy (policy UNKNOWN của mission / `unknown_policy`). **Nếu không xác định chắc chắn được thì DỪNG và hỏi.**
- **M2 (bắt buộc), scene deterministic:** `sanity_open`, `structured_corner`, `structured_obstacle` (theo `config/runtime/planning_stability_qualification.yaml`) × {1, 3, 5} m/s × 5 run = 45 run.
- **M3 (tuỳ chọn, nếu còn thời gian):** như M2 nhưng 10 run, theo đúng yêu cầu của qualification file.
- Mỗi run giữ nguyên toàn bộ session (bag, report, log). Không xoá run fail, không chạy lại để "đẹp" số liệu. Run lỗi hạ tầng (PX4 không lên, `/clock` chết) vẫn giữ lại, ghi `infrastructure_invalid=true` và chạy bù một run, nhưng cả hai đều phải có trong CSV.

### 4. Golden planner snapshot
Trên 3 run M1 (2WP/5WP/9WP, SAFE, run 1), bật `UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR` (xem `planner.cpp:581`, `nominal_trajectory_optimizer.cpp:49`). Sau đó chạy `replay_nominal_problem_snapshot` trên toàn bộ snapshot 2 lần và xác nhận output bit-identical giữa 2 lần. Lưu snapshot cùng output replay làm golden.

### 5. Manifest và lưu trữ
- `artifacts/baseline_20260928/manifest.json`: commit SHA, output của `tools/runtime/planner_baseline.py` (hash config), phiên bản PX4 (commit), Gazebo, ROS, `uname -a`, CPU model, số core, RAM, governor, RTF trung bình, danh sách session.
- Bag lớn KHÔNG commit vào git. Lưu ở `~/uav_baseline/20260928/` (hoặc nơi người giao việc chỉ định). Trong repo chỉ commit `manifest.json`, `baseline_runs.csv`, `baseline_distribution.csv`, `BASELINE.md` và checksum sha256 của từng bag.
- `BASELINE.md`: bảng phân phối, kèm nhận xét **mô tả thuần**. Không kết luận nguyên nhân, không đề xuất tuning.

## Ngoài phạm vi
Sửa bất kỳ lỗi nào quan sát được (chỉ ghi nhận). Đổi ngưỡng. Chạy ở tốc độ 6–8 m/s.

## Nghiệm thu
- `baseline_runs.csv` có ≥ 63 run hợp lệ về hạ tầng (18 + 45), mỗi run trỏ tới một bag tồn tại với sha256 khớp.
- Replay golden bit-identical 2/2.
- `baseline_summary.py` có unit test trên 2 session fixture nhỏ (dùng bản rút gọn của session thật).
- Diff không đụng `src/`.

---

## HỢP ĐỒNG CHUNG (bắt buộc, áp dụng cho mọi work package)

**Repo:** `github.com/TanDatEmb/uav-navigation`. ROS 2 Jazzy, FAST-LIO, PX4 External Mode, beta chỉ SITL.
**Baseline:** `main @ 7e0b850`. Tạo branch `refactor/<WP-ID>` từ đúng commit này. Các nhánh khác (`codex/*`, `feat/*`) chỉ để đọc tham khảo: KHÔNG merge, KHÔNG cherry-pick.

**Đọc trước khi làm:**
1. `AGENTS.md`.
2. `docs/refactor/ARCHITECTURE_REVIEW.md`: kiến trúc hiện tại, V1–V7, RC1–RC6, tên module đích, 8 quy tắc cứng, các phase.
3. `docs/refactor/adr/ADR-013..016`.
4. `docs/refactor/risk_register_20260928.md` (R-01…R-11).
5. Chỉ khi WP đụng estimation/mapping/planning/control/PX4/threshold: đọc thêm `docs/safety/runtime_safety_current.md`.

**Ràng buộc không thương lượng:**
- KHÔNG đổi giá trị ngưỡng, deadline, lease, budget, UNKNOWN policy, tolerance, trừ khi WP ghi rõ là được phép.
- Refactor và thay đổi hành vi không bao giờ nằm chung một commit.
- Mọi khẳng định trong deliverable phải có `file:line` trên commit baseline. Khi phân loại thì dùng CONFIRMED (đã tái hiện bằng test/trace), CONDITIONAL (đường code có thật nhưng chưa chứng minh được là tới được), SPECULATIVE.
- Không suy đoán hành vi runtime khi chưa đo. Số đo nào chưa có thì ghi `NOT_MEASURED`, tuyệt đối không bịa.
- Không sửa code product nếu WP là loại phân tích (read-only).
- Khi prompt mâu thuẫn với code hoặc với tài liệu an toàn: DỪNG phần đó, ghi vào `docs/refactor/<WP-ID>/OPEN_QUESTIONS.md` (câu hỏi, bằng chứng, các phương án), làm tiếp phần còn lại, không tự quyết.

**Nơi đặt kết quả:** `docs/refactor/<WP-ID>/`. Bắt buộc có `REPORT.md` gồm:
1. Tóm tắt 5–10 dòng.
2. Danh sách deliverable kèm đường dẫn.
3. Lệnh verify đã chạy, kèm output thật (exit code, số test pass/fail).
4. Những chỗ lệch khỏi prompt và lý do.
5. Open questions.
6. Commit SHA của từng commit.

Báo cáo viết tiếng Việt; identifier, tên file và tên cột giữ tiếng Anh.

**Hoàn tất:** push branch `refactor/<WP-ID>`, mở PR vào `main` ở trạng thái **draft**, không tự merge. Tổng công trình sư sẽ review.

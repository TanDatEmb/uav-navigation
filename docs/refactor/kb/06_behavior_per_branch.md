# KB-06: Hành vi hệ thống ở từng nhánh

Bảng branch → outcome chi tiết cho từng hàm có trong các file sau, mục `## Behavior per branch`:

| Lớp | File | Hàm được phân tích |
|---|---|---|
| L3 | `areas/R1_layer.md` | plan, planInitial, planSuccessor, generateExpTraj, generateBackup, authorizeAndStage, export, validator, A*, corridor, resolveGoal, emergency |
| L3 | `areas/R2_layer.md` | Piece, Trajectory, costFunctional, optimize, BackupTrajOpt, YawTrajOpt, SimplifySFC, CmdTraj, facade |
| L4 | `areas/R3_layer.md` | onRegisteredScan, process_mapping, onPropagatedOdometry, applyValidatedGoal, onModeStatus, commitPlannerCandidate, currentPlanningKey, runCycle, validateRetainedCommand, publishCommand, mission |
| L1 | `areas/R4_layer.md` | synchronizeNext, processNext, processInternal, recoverFromDiscontinuity, predict, correct, prior, propagator, worker, output publisher |
| L5 | `areas/R5_layer.md` | updateSetpoint, onNavigationCommand, velocity-only, EV `on_lio`, jump continuity, ingress `on_odometry`, ResetCompensator, ExecutionAuthority tryCommit / activate / world revoke, CommandSampler, tracking envelope |
| L2 | `areas/R6_layer.md` | MappingActor::process, validateObservation, updateProbMap, raycast, probabilisticMapFromCache, sliding, worker, store, change-region, classify, segment traversable |
| L6 | `areas/R7_layer.md` | resolve scene / profile / collision, config witness, wait, ground truth, finish, mission acceptance, sim report, tracking, qualification |

Tài liệu này hợp nhất ở **mức hệ thống**: mỗi loại sự cố thì hệ thống làm gì, sau bao lâu, và còn hở ở đâu.

## Ma trận phản ứng sự cố (hiện trạng)
| # | Sự cố | Phát hiện ở | Hành vi của hệ thống | Độ trễ phát hiện đến phản ứng | Chỗ hở (finding) |
|---|---|---|---|---|---|
| B1 | LIO registration fail một scan | pipeline correct | Rollback; Tracking → Degraded; propagator vô hiệu hoá; EV gate đóng; adapter `failNavigation` | tới 0.2 s (lease adapter), vì health DEGRADED phát từ đường scan bị adapter bỏ | R4-06 |
| B2 | Mất LiDAR kéo dài | wall timeout | 0.2 s → Degraded, 1.0 s → Lost; sau đó giống B1 | theo **wall clock** | R4-22 |
| B3 | Có IMU trong hơn 41 s mà không có LiDAR | buffer | Mọi IMU bị từ chối, scan chờ mãi; estimator **kẹt vĩnh viễn** | vô hạn | R4-01 (S2) |
| B4 | Predict fail trước lần correct đầu tiên | pipeline | Kẹt ở INITIALIZING vĩnh viễn; mission không bao giờ bắt đầu | vô hạn | R4-12 (S2) |
| B5 | Pose LIO nhảy, cùng generation, dt bình thường | EV continuity | JUMP_DETECTED → latch → dừng gửi EV cho PX4 | 1 mẫu | — |
| B6 | Pose nhảy kèm gap trên 0.5 s, hoặc frame invalid | EV continuity | Reseed ở trạng thái trusted → jump **được gửi** sang PX4 | — | R-02 (S1) |
| B7 | Pose nhảy khi dt < 100 µs, hoặc dt ≤ 0 với sequence mới | EV continuity | Reseed ở trạng thái trusted mà không so hình học → jump **được gửi** sang PX4 | — | **R5-08 (S1)** |
| B8 | Heading LIO lệch heading EKF2 | không ai kiểm | Mọi setpoint PVA bị xoay theo độ lệch; ở 30 m, 10° lệch cho sai vị trí khoảng 5 m | không phát hiện | **R5-17 (S1)**, chỉ đúng trong SITL |
| B9 | World stale (hơn 0.5 s) | runtime planning và command | Cancel solve, suspend exposure, **ngừng gửi command**; adapter thấy command stale 0.1 s → safety stop | khoảng 0.1–0.2 s | — |
| B10 | Odometry stale | runtime và adapter | Runtime giữ nguyên command, không solve; adapter lease 0.2 s → `failNavigation` → Hold | 0.2 s | freshness không đồng nhất (R3-03, R7-08) |
| B11 | Solve fail hoặc timeout | planner → policy | Giữ command đã certify (HG-023); dùng BACKUP khi reserve MAIN cạn | theo reserve (khoảng 0.6 s) | Lý do reject bị gộp thành `kWorldChanged` (R1-40) |
| B12 | SimplifySFC không hội tụ | optimizer | **Treo planning worker**, bộ nhớ tăng không giới hạn; emergency và recert (chạy chung worker) bị chặn | vô hạn | **R2-24 (S2)**, N7 |
| B13 | World mới làm bundle active không còn an toàn | mapping revalidate | Revoke + fail-closed, hoặc kích hoạt BACKUP, hoặc emergency | 1 chu kỳ mapping (20–57 ms) | — |
| B14 | Tracking vượt tube | `validateRetainedCommand` | Emergency one-shot (chỉ từ MAIN) hoặc fail-closed | tới 100 ms (chu kỳ planning) + solve emergency | projected bound thiếu tuổi state (R3-10); state emergency lệch v·age (R3-09) |
| B15 | Chặng STOP cuối với lease cũ 50 ms ở 3 m/s | retained validation | **Fail-closed sai** → PX4 Hold dù vẫn trong tube | — | **R3-11 (S2)** |
| B16 | Heading rebind bị reject trong khi successor đang stage | thread command | Successor của worker bị xoá; handoff thất bại; MAIN reserve tiếp tục cạn | — | **R3-01 (S2)** |
| B17 | Command anchor lệch 0.75 m dọc + 0.75 m ngang | adapter envelope | **Được chấp nhận**: tổng 1.06 m | — | **R5-16 (S2)** |
| B18 | Reset xy/z của PX4 khi đang bay | adapter | Bỏ phần căn tịnh tiến, chờ đứng yên (\|v\| < 0.15) mới căn lại; command PVA kế tiếp safety stop | ngay lập tức | R5-18 |
| B19 | Mapping gặp BELOW_GROUND / ABOVE_CEILING | actor | Actor poison → worker FATAL → world stale mãi → B9 kéo dài | vĩnh viễn | R6-04 (plane đang tắt trong product) |
| B20 | Mission complete | runtime → adapter | Receipt → giữ vị trí → handover Hold → scenario COMPLETE | ≤ `hold_handover_timeout_s` | — |
| B21 | Không có command trong 5 s kể từ lúc airborne | adapter | Giữ đứng yên, rồi safety stop | 5 s | literal được pin (V6) |
| B22 | Process mapping exception | runtime | `abort()` cả process navigation_runtime | ngay lập tức | chủ ý (fail-stop) |
| B23 | Estimator exception | fast_lio | Worker fail; node vẫn sống nhưng **không còn output**; downstream thấy stale | 0.2 s | không có restart |

## Nhận xét
1. **Hướng lỗi của phần lớn nhánh là an toàn** (dừng, Hold). Chỗ hở còn lại tập trung ở ba nhóm:
   - **Biên estimator → PX4** (B6, B7, B8): lỗi đi thẳng vào EKF2 hoặc setpoint, không có lớp chặn sau.
   - **Ngõ cụt không có đường thoát** (B3, B4, B12, B19, B23): hệ thống dừng vĩnh viễn, không có recovery.
   - **Quyết định sai do đại lượng lệch thời điểm** (B14, B15, B17): fail-closed sai, hoặc chấp nhận sai.
2. Không có sự cố nào dẫn tới việc runtime gửi command sai **mà có thể tránh ở lớp adapter**. Lý do: adapter chỉ kiểm envelope và độ tươi, không có world model. Đây là thiết kế đúng (ADR-014), với điều kiện certificate ở nav_core phải độc lập.

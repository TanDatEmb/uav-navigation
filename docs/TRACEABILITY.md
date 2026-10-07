# Bảng truy vết (rebuild v2)

File này trả lời câu hỏi "repo đã đi tới đâu". Mỗi dòng nối một yêu cầu với:
- mục thiết kế hiện thực nó, trong [SYSTEM_DESIGN.md](architecture/SYSTEM_DESIGN.md);
- work package (WP) thực hiện nó;
- trạng thái hiện tại;
- bằng chứng.

**Trạng thái:** `todo`, `doing`, `done`, `deferred`.

**Bằng chứng:** tên test, commit hoặc file event log. Thiếu bằng chứng thì chưa được ghi `done`.

Cột WP sẽ được điền khi có implementation plan.

## Vấn đề (P\*) → lời giải

| Yêu cầu | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|
| P1 Hai profile bay (UNKNOWN theo profile) | §0, §5.3 | S3 | | todo | |
| P2 Hai profile định vị | §0, §4.1 | S1 | | todo | |
| P3 Lệch LIO–PX4 | §4.1 (FRD, `T`), §4.2 | S1, S2 | S1a-T10 | doing | `Alignment.ConvergesToTrueOffset`, `Alignment.RejectsGnssGlitchJump`, `Alignment.TransitionTableIsTheSpecDiagram`, `AlignmentFuzz.EveryStateChangeIsAListedEdge` (ước lượng `T` thuần; cần sửa theo D30/O12 ở S1b; SITL ở S1b/S2) |
| P4 Kiểu setpoint (A′: P/V/A qua `T`) | §4.2 | S2 | | todo | |
| P5 LIO mất rồi khởi động lại | §3.1, §3.4, §2.2 | S4 | | todo | |
| P6 Không GPS mà LIO fail → bàn giao có Reason | §2.3, §4.2 | S4 | | todo | |
| P7 Trạng thái 50–100 Hz với LiDAR 10 Hz | §3.3 | S1 | S1a-T7, S1a-T8 | doing | `OutputPredictor.CorrectionConvergesWithoutJump`, `OutputPredictor.CorrectionMatchedByTimestampNotOldest`, `OutputPredictor.AttitudeTrackingDoesNotOvershootAtScanRate` (predictor xong, nghĩa τ chốt ở D30; tách thread theo D30/O14 và phát 100 Hz thuộc S1b) |
| P8 Phạm vi SITL, PX4 1.17 | §0 | S0 | S0-T1 | done | commit `f618909` (gỡ code main-only khỏi nhánh; gate build/test `uavnav_core` và `uavnav_interfaces`); pin PX4 v1.17 ghi ở `src/external/README.md` |
| P9 Chất lượng cấu trúc (M1–M9) | §2, §6, AGENTS.md §2 | mọi lát | | todo | |
| P10 Chống chuyển nhánh liên tục | §2.1, §2.4, §7.3 | S3, S5 | | todo | |

## Lỗi đã kiểm chứng trên `main` (F\*) → không được tái diễn

| Lỗi | Mục spec | Lát | WP | Trạng thái | Bằng chứng (test chống tái diễn) |
|---|---|---|---|---|---|
| F13 Reset counter PX4 là tổng | §4.1 | S1 | S1a-T10 | done | `Alignment.AppliesDoubleResetDeltas`, `Alignment.ZResetStepOfThreeAppliesDeltaZ`, `Alignment.ResetCounterWrapIsAReset` (reset heading theo phép đổi hệ quy chiếu G, D30/O11) |
| F14 Reset counter EV theo epoch của mẫu | §4.1, §3.4 | S1 | S1a-T9 | doing | `EvEncoder.UsesSampleEpochAsResetCounter` (chỉ chứng minh ở mức bộ mã hoá; node điền `EvInput.epoch` từ epoch của mẫu là việc S1b, như F34) |
| F15 "Đang bay" theo z LIO, deactivate sai nhãn | §4.2 | S2 | | todo | |
| F18 Quy tắc chết về logic | §2.3 (không còn quy tắc này) | S3 | | todo | |
| F20 Mapping bị poison vĩnh viễn | §5.2 | S3 | | todo | |
| F21 Hai WorldView / hai DDA | §5.2 | S3 | | todo | |
| F22 LIO không tự chuyển LOST khi chỉ LiDAR mất | §3.1 | S1 | S1a-T5, S1a-T8 | done | `LioLifecycle.LidarGapOnImuTicksAloneReachesLost`, `LioEstimator.ImuOnlyGapReachesLost`, `LioEstimator.ScanAheadOfImuDoesNotHideALidarGap`, `LioEstimator.NormalScansAreAcceptedAfterAScanAheadOfImu` (lỗ hổng scan đóng dấu thời gian vượt IMU đã sửa ở `a5fa24c`) |
| F23 Output 50 Hz nhảy bậc | §3.3 | S1 | S1a-T7 | doing | `OutputPredictor.CorrectionConvergesWithoutJump` (giữ kẹp 0.03 s của PX4 theo D30/O13; phát 100 Hz thuộc S1b) |
| F24, F26 Param bị ghi đè hoặc default lệch | §6.2 | S0, S3 | | todo | |
| F25 Route gate fail-open | §5.3 | S3 | | todo | |
| F27 LIO nhảy Lost→Tracking không xác nhận | §3.1 | S1 | S1a-T5 | done | `LioLifecycle.LostNeverJumpsToTracking`, `LioLifecycleTable.NoEdgeFromLostToTracking` |
| F28 Hai nguồn trạng thái cho một quyết định | §5.3 | S3 | | todo | |
| F29 Heading rebind bỏ qua certificate | §5.3 (không có đường này) | S3 | | todo | |
| F30, F31 Cancel/deadline bị coi là thành công | §5.3 | S3 | | todo | |
| F34 Nhãn NED cho EV | §4.1 | S1 | S1a-T9 | doing | `EvEncoder.LabelsFrdNotNed`, `EvEncoder.PositionAndVelocityAreFlippedIntoFrd` (kiểm tra SITL thuộc S1b) |

## Hạ tầng

| Hạng mục | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|
| Thay `tools/gate.sh`, ledger validator và test của `main` bằng gate tối giản | §7.2 | S0 | S0-T1 | done | `tools/uavnav/gate.sh all` |
| Event log + script KPI | §6.1 | S0, S5 | S0-T4, S0-T5, S0-T8 | doing | `test_event_recorder`, `test_jsonl_sink`, `test_events` (phần script KPI thuộc S5) |
| Config ba tầng | §6.2 | S0 | S0-T6, S1a-T5, S1a-T6, S1a-T7, S1a-T8, S1a-T10 | doing | `test_config` (tầng b: ParamValues); `LioConfig.BetaValuesLoad`, `LioConfig.PerKeyBoundsAreEnforced`, `LioLimits.ScanPointCap`, `AlignmentConfig.BetaValuesLoad`, `AlignmentConfig.TierAInvariantsHoldForEveryLoadableConfig` (struct có kiểu cho LIO và alignment; các package khác chưa làm) |
| Kiểu thời gian | §6.3 | S0 | S0-T3 | done | `test_time` |
| Message v2 | §6.5 | S0 | S0-T7 | done | commit `5ad488e`, `d8ef690`; build `uavnav_interfaces` |
| Gate beta | §7.3 | S5 | | todo | |

## Việc code theo quyết định thiết kế mới

| Hạng mục | Quyết định | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|---|
| `AlignmentEstimator` đo residual, cổng nhảy và bước lọc yaw tại vị trí xe; bỏ bộ giới hạn tốc độ riêng; test hồi quy R ≈ 300 m, nhiễu yaw 5 mrad | D30 (O12) | §4.1 | S1b | | todo | |
| Tách `LioEstimator` thành frontend (ingest: predictor, lifecycle, kiểm gap, phát message) và backend (estimator: IKFoM, map, ICP, suy biến) nối bằng hàng đợi bất biến; hằng số thread OpenMP tầng (a) | D30 (O14) | §3.5 | S1b | | todo | |
| `make test` dọn kết quả test cũ của package trong `build/` trước khi chạy | D30 | §7.2 | S1b | | todo | |
| Đánh giá lại τ của output predictor từ output tracking error trong log SITL | D30 (O13) | §3.3 | S1b | | todo | |

## Lệch thiết kế đang mở

Khi phát hiện lệch: thêm mục O\* trong [DECISIONS.md](architecture/DECISIONS.md) và một dòng ở đây. Có ≥ 3 dòng thì dừng lại sửa thiết kế (§7.4).

Không có lệch đang mở. O11–O14 đã được chốt trong D30 (2026-10-07); quy tắc dừng của §7.4 được gỡ.

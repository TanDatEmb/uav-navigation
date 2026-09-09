# Kế hoạch đóng lỗi mission và tốc độ — 2026-09-09

Mục tiêu: hoàn thành mission qua toàn bộ waypoint trên từng map, với các mức
requested speed 3/5/8/12 m/s và adaptive tracking bật. Cap 12 m/s là giới hạn
cấu hình, không phải kết quả đo hoặc cam kết mọi đoạn ngắn đều đạt 12 m/s.

## Phân công

- **Root review:** `01a08075-5876-76c1-ba9f-8432a1ec74be` — đối chiếu source,
  artifact, phản biện và kiểm chứng kết quả; đang bàn giao sửa cap/horizon.
- **Task A:** `01a083d6-eaf2-7770-9988-04965ae8c29f` — fix/commit WIP
  diagnostic/replay, tích hợp cap/horizon, sau đó tracking terminal.
- **Task B:** `01a07215-a562-7a02-b359-70d35135c5d0` — yaw và tests riêng ngay;
  đóng deadline/seed sau khi nhận lại ownership nominal planner; chạy SITL.

Task A đang sở hữu nominal optimizer/planner/facade/runtime identity và replay.
Task B không sửa trùng các file đó khi làm yaw; integration cần planner.cpp
phải bàn giao patch cho A. Root giữ config speed/horizon và tests đến khi báo
xong. Các task phối hợp ledger và index trước commit; không stage toàn bộ WIP.

## Backlog và tiêu chí đóng

| ID | Vấn đề và trạng thái kiểm chứng | Việc thực hiện | Owner / điều kiện đóng |
|---|---|---|---|
| V1 | Shared MAIN cap còn 3 m/s; physical cap và GPS-off profile đã 12. Root đã sửa source shared cap lên 12. | Giữ request thấp; request 5 phải effective 5, request 12 effective 12; lưu requested/configured/effective/measured riêng. | Root → A; config/runner regression, commit riêng, Release manifest và xác minh generated config của run mới. |
| V2 | Nâng cap lộ constructor reject: visibility reserve dùng MAIN A/J 2/4, cần 49.4 m tại 12 m/s, vượt cap 23 m. Root đã sửa reserve theo BACKUP A/J 12/30. | BACKUP stopping reserve là 18.8 m; giữ map/cap và certificate quỹ đạo phanh thật, không coi công thức reserve là chứng nhận candidate. | Root → A; regression tốc độ 1/3/5/7/12 và kiểm map/budget/physical contracts. |
| D1 | Chín nhóm findings WIP snapshot/replay chưa có bằng chứng đã đóng toàn bộ. Baseline compile không phải regression pass. | Bounded writer/lifecycle, bỏ hot-path log dài, snapshot không lẫn request, capture đủ failure, identity đúng, config round trip, production certificate parity, test damping đúng, test-only dependency; tách MISSING fix. | A đang thực hiện; regression trước/sau tương ứng từng lỗi, commits tooling/observability riêng. |
| P1 | Gia hạn MAIN+BACKUP bị vượt budget, không có successor và phải dừng khi command cũ hết hạn. | Trace thời gian từng stage theo cùng generation; giữ quỹ thời gian cho yaw/BACKUP/certificate/staging; phân biệt mandatory feasibility với optional refinement. Tối ưu stage có latency tail và số lần làm lại. | B sau handoff nominal; A hỗ trợ evidence. Đóng khi successor được commit/activate kịp trong repeat SITL, không chỉ optimizer return success. |
| P2 | Seed/candidate fail corridor hoặc dynamics; giảm derivative scale chưa chứng minh recovery. | Replay đúng input, immutable boundary PVAJ và corridor; kiểm coupling duration/junction derivatives, feasibility retry và objective; tìm regression có candidate khả thi. Không suy ra bài toán vô nghiệm chỉ từ một seed fail. | B sau D1; correctness fix riêng với exact regression, rồi map/speed repeats. |
| Y1 | Yaw đổi muộn so với chuyển đoạn ngắn; heading đoạn trước có thể xuất hiện ở đoạn sau. | Yaw reference phải cùng thời gian/progress với position trajectory và đúng route generation; test short segment, multi-turn, wraparound, near-zero speed, continuity và yaw-rate/acceleration. | B làm ngay trên yaw files/tests riêng; integration qua A nếu đụng planner.cpp. Đóng bằng trajectory regression và synchronized SITL yaw/velocity/waypoint trace. |
| T1 | Adaptive bật nhưng moving MAIN có terminal_stop bị loại ở experimental_tracking.hpp và path_relative_tracking.hpp. | Áp allowance cho moving terminal MAIN theo pha; chuyển đúng sang endpoint settle/hold. Đồng bộ runtime/PX4; giữ BACKUP/EMERGENCY, freshness, path-clear và mission acceptance. | A sau D1; tests moving/decelerating/stopped terminal, hai phía cho cùng verdict, rồi final approach SITL. |
| M1 | Mission tới vùng waypoint cuối nhưng chưa COMPLETE; artifact đã kiểm chỉ đạt 8/9. | Xác định first blocker tại terminal: tracking, solve renewal, lease hay không đạt position/speed/dwell. Sửa trajectory/transition gây lỗi thay vì duyệt waypoint chỉ vì đã từng đi qua gần nó. | A+B; đủ 9/9 và COMPLETE thật trong repeat, báo riêng những lần collision-free nhưng không hoàn thành. |
| E1 | GPS-off chỉ là phương án cô lập planner; GPS+LIO đồng bộ chưa được chứng minh khép kín. | Đối chiếu LIO/PX4/GT theo cùng timestamp và frame, reset/innovation/quality; xử lý global position/height anchoring cùng local velocity/position, suy giảm LiDAR khi bay cao, và chuyển nguồn có kiểm chứng. | B thực hiện nhóm evidence/design riêng sau yaw và stabilization; root review trước tích hợp. Không quy mọi brake cho estimator chỉ từ GPS enable flag. |
| R1 | Evidence còn thiếu: benchmark_metrics có zero rolling records trong khi report.json cùng artifact có 67 complete records. | Trace parser/schema và missing-data semantics; báo NOT_RECORDED thay vì kết luận zero hoạt động; giữ raw timeline và identity. | A triage sau D1, giao patch tooling riêng; fixture cùng run cho thống kê nhất quán. |

## Bằng chứng hiện tại

Đã kiểm trực tiếp artifact
`.artifacts/runtime/external-mode-gui-20260909T013257-525294/report.json`:

- `verdict=BLOCKED`, accepted waypoint indices 0..7, cần 0..8;
  mission chưa COMPLETE và kết thúc `PAUSED_SAFETY_STOP`.
- Adaptive `enabled=true`, `config_mismatch=false`, base 0.2 m,
  alpha 0.05 s, beta 0.15 s, `suppress_braking=false`.
- Execution maximum velocity 2.941849 m/s; run dùng shared cap 3 m/s.
- Planning total p50 80.228 ms, p95 94.553 ms, max 100.926 ms.
  Nominal optimization p95 76.580 ms; backup frontend p95 23.981 ms.
  Đây là percentile độc lập, không cộng chúng thành thời gian một solve.
- Rolling trace có 67 complete records, certified-seed dynamics failure 63,
  corridor failure 3, seed được dùng 0; 244 duration retries, 55 build-valid.
  Đây là bộ đếm stage/attempt, không phải 244 mission failures.

## Trình tự tích hợp và SITL

1. Commit sửa cap/horizon sau focused tests; A tiếp tục D1 và B làm Y1 song song.
2. D1 xong thì bàn giao ownership nominal cho B đóng P1/P2; A đóng T1/M1.
   Việc sửa tooling không được đánh dấu thay cho hoàn thành flight stability.
3. Build Release và manifest thống nhất sau integration. Chạy GPS-off EV,
   adaptive bật, các map và speed 3/5/8/12 với artifact riêng từng tổ hợp.
   Repeated representative runs phải chứng minh completion, không chỉ một run.
4. Báo tốc độ thực đo theo đoạn đủ dài; các đoạn ngắn/corner có thể không đạt
   cap vì A/J hoặc geometry. Báo rõ requested speed chưa đạt thay vì gọi PASS
   tốc độ 12 từ một run chỉ có cap 12.
5. GPS-on là campaign riêng cho E1, không trộn kết quả với GPS-off. Khả năng
   không chuyển Hold khi GPS-off là giới hạn user đã chấp nhận cho test,
   không xếp là bug cần fix. Cũng không chuyển mission failure thành PASS.

Mỗi báo cáo task cần commit SHA, files/contract thay đổi, test thực chạy,
artifact map/profile/route/speed, lỗi còn lại và bước tiếp theo có owner.

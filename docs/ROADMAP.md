# Lộ trình refactor từ baseline mới

Ngày 2026-10-05. Roadmap này bắt đầu lại từ source/test/contract của baseline,
không kế thừa DONE counts hoặc thứ tự checkpoint cũ. [Thiết kế](architecture/SYSTEM_DESIGN.md)
định nghĩa ownership; [safety contract](safety/runtime_safety_current.md) quyết định
safety authority. Một writer, một task đang đổi behavior, root worktree duy nhất.

## Nguyên tắc nghiệm thu

Mỗi task bắt đầu bằng counterexample có source/runtime provenance, RED→GREEN,
review ba cấp (ownership, source/units/evidence, adversarial/tails) và commit
behavior tách observability/docs. Không tune hard gate từ một run. Component
PASS không đóng qualification. Missing evidence giữ NOT_MEASURED/NOT_EVALUABLE.
Không mở planner optimization trước khi infrastructure và runtime ownership rõ.

## Các mốc

| Mốc | Vấn đề và đầu ra | Phụ thuộc | Nghiệm thu / điều kiện dừng |
|---|---|---|---|
| R0 — Baseline reproducible | Build identity, full-package tests, configuration/dependency inventory, replayable failure corpus; triage mọi branch-only safety fix từ backup | Baseline mới | Reproduce clean build và đủ static/Python/ROS gates; counterexample fail-closed còn lỗi phải có task cụ thể. Dừng khi snapshot/source/build không bind hoặc backup không phục hồi được |
| R1 — Evidence infrastructure | Đo /clock sim-vs-wall, stale streams, Gazebo host contention, cleanup và ownership; chọn frozen scenario matrix từ qualification config hiện hành | R0 | Giữ mọi failure; không có sample/infrastructure-invalid thì không đánh giá planner. Không tăng tốc để bù dữ liệu thiếu |
| R2 — Physical/frame budget | Đo achieved speed, tracking ngang/dọc, frame drift/bias/reset/yaw, reaction và executed braking support trên SITL + recorded data | R1 | Per-speed distributions và coverage hợp lệ; capability PX4/thrust/tilt/ramp tách planner nominal. Dừng speed escalation khi frame/capability chưa đủ evidence; không lấy 10 m/s requested làm observed |
| R3 — Safe mission liveness | Tái hiện retained MAIN → forward successor, route regression, nominal dynamics và measured progress; sửa nguyên nhân với fixture thật | R1 và các bound cần dùng từ R2 | Valid successor, đầy đủ world/route/dynamics/lease/BACKUP; repeated mission results theo config qualification. Dừng nếu chỉ tăng complete bằng bypass/retune hoặc thiếu exact artifact topology |
| R4 — Structural extraction | Characterize events/predicates/effects; tách independent certifier, data bundle và execution reducer từng seam; đóng dependency allow-list | R0 + corpus R3 | Baseline/shadow parity trên positive và adversarial failures; không đổi ownership/lease/world/UNKNOWN policy. Bất đồng planner-pass/certifier-fail phải giải trước switch |
| R5 — Frame adapter và profile | Quyết định T3 dựa trên R2; immutable transform witness/derivatives/reset recertification; cross-process config witness nếu cần | R2 + certificate interface R4 | Firmware/topic provenance, transformed P/V/A parity, drift/reset/stale negative tests, full certificate và repeated distributions; pure helper không đủ product authority |
| R6 — Qualification | Freeze Release matrix, seeds, speed bands và representative recorded data; publish evidence manifest | R1–R5 và debt liên quan đóng bằng evidence | Theo qualification config hiện hành, không đặt gate mới trong cleanup. Mission completion, safety, tracking và latency tails phải xét riêng. Hardware HG-011 không tự được gỡ |

## Cách mở task

R0 là công việc đầu tiên: từ `commit-disposition.csv` trong local backup, xét
các DEFER có sửa product safety trước docs-only. Pin counterexample và so với
source baseline; chỉ chọn patch/hunk không còn present hoặc equivalent. Một
branch không được merge nguyên chỉ để dọn tên nhánh.

Mỗi task ghi một issue/brief: owner, problem, source/config/artifact identity,
contract/invariant, exact negative control, expected behavior, verification và
rollback condition. Chỉ một review request cho đầu ra thực, không tạo chuỗi
open/close/SHA checkpoint thay cho source progress. Reference evidence phải
replayable; tài liệu cũ trong backup là historical input, không verdict mới.

## Trạng thái khởi đầu

R0 READY; R1–R6 NOT_STARTED. Safe-forward liveness, frame/capability distributions,
T3 authority, reducer/certifier parity và hardware visibility là OPEN. Không
chuyển các mốc thành DONE bằng việc tạo baseline một commit.

`config/runtime/planning_stability_qualification.yaml` giữ nguyên; thay đổi
threshold hoặc distribution matrix cần task behavior riêng theo safety contract.

R0 phải tái hiện các debt đã review: reset LIO publication fence và topic-prior
rearm (923cdc1), deadline/cancellation của interior duration retry (a98d131,
8ab34d6), U1 frame witness identity/time/basis, U2 source ordering và bounded
horizon, host telemetry single-writer và RTF source freshness. Các tính năng
hoãn không được tự mở lại chỉ vì helper/component test PASS.

R0 cũng phải xử lý độ nhạy tải của `PlannerFacade.CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary`:
planner dùng steady-clock hard deadline và test chạy bằng wall time. Trên cây
`d55c0a5`, một lần gate dưới tải SITL của owner fail test này (không candidate),
trong khi 36/36 lần chạy lại pass; khác biệt source chỉ là whitespace. Cần fixture
deadline xác định hoặc đo phân bố, không nới gate.

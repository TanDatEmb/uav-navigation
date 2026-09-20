# Audit giả thuyết nghẽn H0–H7

Đây là audit độc lập, read-only với production: không sửa source/config/interface/dependency pin/safety gate/existing test; không điều khiển ROS/PX4/SITL. Test, script, report và SVG mới chỉ nằm trong thư mục output này.

Phân biệt baseline A (dirty-worktree source snapshot ở artifact commit f2bd3f46f9936d622377ea4761f733f988273b66) và checkout B ban đầu (HEAD 9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2, branch codex/close-proven-findings). Snapshot source của A chỉ được đọc trong baseline/source_snapshot của artifact worktree; A/B hash khớp 596/596 manifest paths lúc audit bắt đầu. B có 34 status entries, 0 staged bytes và 1,778,436 unstaged bytes. Trong lúc audit, checkout chuyển sang C: HEAD 8eaab3d33db36e9e636ad4005ed91b8f055f4d68; 15 manifest paths khác A. Kết quả A không được gộp với C; chi tiết ở validation/final_checkout_drift.json.

Audit được commit riêng trên branch `codex/audit-claim-validation`, dựa trên C. Frozen source A có thể lấy từ branch `codex/as-is-architecture-audit-20260920` tại commit f2bd3f46f9936d622377ea4761f733f988273b66, dưới `artifacts/architecture_as_is/20260920T-as-is-local/baseline/source_snapshot/`. Script hỗ trợ `AUDIT_SOURCE_SNAPSHOT` để chỉ rõ vị trí snapshot. Build/install tree và binary cục bộ không được commit; raw test/build logs, compile database, link command và kết quả audit được giữ lại.

## Điểm bắt đầu

- Mở index.html để xem kết quả và diagram.
- AUDIT_VERDICT.md: verdict, counterevidence và giới hạn.
- claims.json + evidence/refs.jsonl: claim register và line/hash evidence.
- evidence/: H0 profile, H3 lock timeline, H4 timer owner, H7 funnel.
- tables/scenarios.md: sequence có điều kiện và mức bằng chứng.
- validation/commands.md: build/test command, provenance, exit code, raw logs.

## Regenerate/check

- python3 tools/generate_refs.py — hash và excerpt source refs từ A snapshot/pinned dependency.
- bash tools/render_diagrams.sh — render DOT thành SVG với Graphviz.
- python3 tools/verify_claim_artifacts.py — fail-closed check ID/ref/hash/range, A manifest và link offline; đồng thời ghi nhận source/worktree drift hiện tại. Lần chạy cuối báo DRIFT_DETECTED vì checkout đã chuyển commit sau baseline. Kiểm tra này không thay thế semantic review hoặc kiểm tra hình ảnh bằng mắt.
- bash tools/run_h0_loader_truth_table.sh; bash tools/run_h1_mission_counterexample.sh; python3 tools/run_h5_speed_grid_oracle.py — rerun standalone audit fixtures sau isolated builds.
- Các GTest filters và isolated colcon build ở validation/commands.md. `tools/run_focused_tests.sh` cần isolated build/install tạo theo lệnh đó trước.

Artifact được gắn nhãn PARTIAL_AS_IS: không có target workload trace/binary/profile match, nên không xếp hạng bottleneck thực tế. NOT_REPRODUCED không được đổi thành REFUTED.

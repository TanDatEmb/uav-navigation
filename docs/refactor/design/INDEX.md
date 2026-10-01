# Target design baseline — index (2026-09-28, baseline `main @ 7e0b850`)

| File | Nội dung | Dùng cho phase |
|---|---|---|
| `../adr/ADR-017-target-design-baseline.md` | **Quyết định chờ duyệt**: D1–D9; finding N1–N11; mục cần đo M1–M7; WP mới P4-0, P4-1 | tất cả |
| `A1_module_ownership_map.md` | Class/symbol → package đích; đồ thị dependency và CMake gate | P1–P6 |
| `A2_icd_current_and_target.md` | Topic map; quyết định theo từng field msg; msg evidence; lịch chuyển đổi | P1, P2, P3, P6 |
| `A3_pipeline_timing_target.md` | Thread và timing hiện tại; lane đích; sequence; budget; clock; nhánh lỗi | P4, P0.2 |
| `A4_reducer_design.md` | Trạng thái oracle A4-R1; sum type ExecutionState; event/effect/transition; map 45 hàm `planner_fsm` và 327 `RT-*` | P4-0, P4-1, P4 |
| `A5_contract_specs.md` | `nav_certifier`, CertificateRecord, bundle pure data, SafetyProfile, định nghĩa `.msg` evidence | P1, P2, P3 |
| `../../../tools/check_citations.py` | Kiểm mọi `file:line` trỏ vào baseline tồn tại | CI docs |

Authority: khi mâu thuẫn với output các WP-A* R1 thì tài liệu này thắng. Mọi giá trị số đều là giá trị đang chạy; không có tune.

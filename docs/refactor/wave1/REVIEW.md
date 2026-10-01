# Review đợt 1 — tổng công trình sư (2026-09-28)

Phương pháp: fetch cả 8 branch `refactor/WP-*`, đối chiếu deliverable với prompt, tự chạy lại các checker và test chạy được không cần ROS, kiểm conflict khi merge.

## Verdict
| WP | PR | Verdict | Kiến trúc sư đã tự verify | Việc tiếp theo |
|---|---|---|---|---|
| D0 | #3 | **APPROVE** | 6/6 file giống nguyên văn nguồn (diff); ledger PASS | Merge #1 |
| P0.1 | #10 | **APPROVE** | Python 462 test OK (3 skip) trên clone sạch; py3.11 `test_html_report` OK; `check_ci_contract` PASS; job CI có khai báo | Merge #2. Theo dõi lần chạy hosted đầu tiên: container `ros:jazzy-ros-base` + `actions/checkout` submodules (nếu thiếu `git` trong image thì thêm bước cài git trước checkout) |
| H1 | #4 | **APPROVE** | 7/7 test mới pass; repro R-01 gốc giờ báo violation cho cả outcome COMPLETE; bằng chứng RED trước fix có trong REPORT; ledger PASS. 1 fail còn lại của full suite là R-04, P0.1 đã sửa | Merge #3, sau P0.1 |
| A1 | #9 | **REVISE → A1-R1** | `coverage_check` PASS | Phần phủ đạt, ngữ nghĩa không dùng được (responsibility sinh theo khuôn; `prod_callers` sai, MissionController=26) |
| A2 | — | **GIAO LẠI** | — | Không có branch |
| A3 | #6 | **ACCEPT + A3-R1** | ledger PASS, `diff --check` sạch, 0 file `src/` | F-01/F-10 có giá trị; thiếu 5 kịch bản; chưa khai thác số đo có sẵn |
| A4 | #5 | **REVISE → A4-R1** | `check_tables` PASS | 32 rule cho 74 return / 163 nhánh; guard viết văn xuôi. Chưa dùng làm oracle P4 được |
| A5 | #7 | **ACCEPT + A5-R1 (format)** | — | Nội dung tốt; chỉ cần chuẩn hoá theo schema chung |
| A6 | #8 | **REVISE → A6-R1** | — | Thiếu 0.15 m/s ×6 (mục bắt buộc đầu tiên) và khoảng 15 mục khác; `pinned` bị hiểu sai (51/70) |

Merge tuần tự D0 → P0.1 → H1: đã kiểm bằng `merge-tree`, không conflict. Trên cây gộp, Python suite OK, 10 static guard PASS, ledger PASS. Các PR A* chỉ merge sau khi R1 được duyệt, để tránh đưa bảng phân tích chưa đạt vào `main`.

## Lỗi của chính prompt đợt 1 (kiến trúc sư nhận)
- Tiêu chí nghiệm thu của A1/A6 chỉ kiểm phủ cơ học, nên deliverable rỗng ngữ nghĩa vẫn qua. Các prompt R1 đã thêm oracle và checker ngữ nghĩa.
- Yêu cầu A5 "theo format A4" trong khi hai WP chạy song song. Đã thay bằng `SCHEMA_decision_table.md` dùng chung.
- Các con số trong prompt: "58 method" là sai (đúng là 38 definition); "38 hàm `planner_fsm`" là sai (đúng là 40). Các agent đã ghi nhận đúng.

## Quyết định chờ chủ dự án
**HG-001 lệch với code (A3 F-01, A6):**
- Ledger: A* 40 / 80 ms, solve 180 ms, future-state lead 200 ms.
- Code và YAML đang chạy: A* 30 / 60 ms (`planner.yaml:199,203`), solve 80 ms (`planning_timing.hpp:11`).
- Toàn bộ evidence SITL gần đây đều chạy với 30/60/80.

Đề xuất: `nav_safety_profile` (P1) lấy **giá trị đang chạy** 30/60/80 làm AS-IS, và sửa HG-001 trong ledger bằng một commit docs riêng. Commit đó ghi lineage (180 ms là scope "SUPER absolute" hay đã superseded) và không đổi code. Theo AGENTS.md, đây là quyết định về ngưỡng nên cần chủ dự án xác nhận.

## CI bị khoá billing: chính sách merge tạm thời
Khi hosted CI chưa chạy được: chỉ merge một PR khi `make ci-local` đã chạy **trên đúng head SHA của PR** và PASS, log được dán vào PR, và kiến trúc sư đã approve. Khi billing được mở lại: chạy lại hosted CI cho `main` và ghi kết quả vào `docs/refactor/WP-P0.1/CI.md`.

## Giao việc tiếp
1. Merge D0 → P0.1 → H1 theo chính sách trên.
2. Giao song song: A1-R1 (máy ROS), A2 (giao lại), A3-R1, A4-R1 (kèm `SCHEMA_decision_table.md`), A5-R1 (sau khi A4-R1 đã commit schema, hoặc đính kèm file schema), A6-R1.
3. Giao **P0.3** sau khi P0.1 đã merge; prompt gốc giữ nguyên.
4. Giao **P0.2** sau khi D0/P0.1/H1 đã merge. Thêm addendum `P0.2-ADDENDUM.md`.
5. Spec và prompt đợt 2 (P1 SafetyProfile, P2 evidence, P3 certifier) chỉ ra sau khi A1-R1, A2, A4-R1, A6-R1 đạt.

# WP-D1: Đưa bộ tài liệu thiết kế đích và knowledge base vào repo (docs-only)

**Loại:** docs. Không chạm `src/`, `config/` hay `tools/` (trừ `tools/refactor/check_citations.py`).
**Phụ thuộc:** các PR #3, #10, #4 đã merge.
**Input:** file `uav_nav_knowledge_base_20260929.zip` do kiến trúc sư gửi kèm, đã bao gồm ADR-017 rev 1.

## Việc cần làm
1. Giải nén vào gốc repo. Kết quả phải đúng cấu trúc sau:
   - `docs/refactor/adr/ADR-017-target-design-baseline.md`, bản rev 1 có §6 Amendment 1.
   - `docs/refactor/design/{INDEX,A1..A5}.md`.
   - `docs/refactor/kb/`: các file 00–10, `areas/R1–R7_layer.md`, `data/{findings.csv, coverage_matrix.csv, README.md}`, `data/repro/**`.
   - `tools/refactor/check_citations.py`. Chuyển file từ `tools/check_citations.py` trong zip sang đường dẫn này.
2. Các file `ADR-013..016`, `ARCHITECTURE_REVIEW.md` và `risk_register_20260928.md` đã có trên main (qua D0). **Không ghi đè.** Chạy `diff` giữa bản trong zip và bản trên main, dán kết quả vào REPORT. Nếu khác nhau thì giữ bản trên main và ghi vào OPEN_QUESTIONS.
3. Các script trong `kb/data/repro/` là dữ liệu tái hiện:
   - Không đưa chúng vào CI.
   - Thêm `docs/refactor/kb/data/repro/README.md`, ghi một dòng: "không build/chạy trong CI; mỗi WP sửa finding tương ứng sẽ chuyển thành test".
4. Chạy `python3 tools/refactor/check_citations.py . docs/refactor`. Kết quả mong đợi: `out_of_range=0`. Nếu có dòng lệch, ghi vào REPORT; không tự sửa nội dung tài liệu.
5. Kiểm link nội bộ: mọi đường dẫn trong backtick có dạng `kb/...`, `design/...`, `adr/...`, `areas/...`, `data/...` phải tồn tại. Viết kiểm này thành script nhỏ `tools/refactor/check_doc_links.py` và chạy nó.

## Ngoài phạm vi
- Không sửa nội dung KB hay design. Lỗi phát hiện được thì ghi OPEN_QUESTIONS.
- Không sửa `docs/architecture/*`. Doc drift D-01..D-05 thuộc P0.3 I5.

## Nghiệm thu
- `git diff --name-only origin/main` chỉ gồm `docs/refactor/**` và `tools/refactor/**`.
- Gate static PASS (xem COMMON_CONTRACT_v2).
- `check_citations.py` cho `out_of_range=0`; `check_doc_links.py` cho 0 link hỏng.

(Áp dụng `COMMON_CONTRACT_v2.md`.)

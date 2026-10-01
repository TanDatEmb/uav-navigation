# Refactor program

Thư mục này lưu bộ tài liệu kiến trúc và các deliverable của chương trình refactor. Baseline: `main @ 7e0b850`. Thay đổi hành vi, ngưỡng và code sản phẩm phải tuân theo các phase và ADR tương ứng.

## Ghi chú baseline (2026-10-01)
Repo được dựng lại thành một commit gốc duy nhất; lịch sử và dữ liệu sinh ra trước đó đã bị gỡ. Nội dung kế hoạch trong `docs/refactor/` được giữ nguyên, chỉ chỉnh để khớp trạng thái mới:
- **Đã gỡ khỏi repo:** `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, `docs/runtime_validation.md`, `docs/refactor_program/`, `docs/wave1_review/` (trùng với `docs/refactor/`), `docs/safety/runtime_safety_index.md`, `docs/safety/archive/`, `.github/workflows/ci.yml` và nhiều script phân tích one-off trong `tools/runtime/`.
- **Trích dẫn bằng chứng đã gỡ** (số đo, đường dẫn session, báo cáo) vẫn giữ nguyên như nguồn gốc lịch sử; chúng không còn kiểm chứng lại được và không được dùng làm bằng chứng qualification.
- **Các nhãn `DEC-*`, `HG-*`, `TB-*`** là nhãn legacy; bản ghi duy nhất còn lại là `docs/safety/runtime_safety_current.md`.
- **Mô tả CI / `make ci-local`** (WP-P0.1, risk register) là trạng thái trước baseline: workflow GitHub đã bị gỡ.
- **Báo cáo WP đã merge trước đây** (ví dụ WP-H1 sửa `tools/runtime/report.py`) mô tả công việc không nằm trong cây code của baseline này; coi là thiết kế/kế hoạch cần triển khai.
- **Đường dẫn tuyệt đối** tới worktree cũ được thay bằng `<repo>` hoặc đường dẫn tương đối. Số dòng trích dẫn trong `runtime_safety_current.md` và `tools/data.py` đã được ánh xạ lại.
- Kế hoạch và script dọn repo cũ (`wave2/CLEANUP_PLAN_*`, `wave2/*cleanup*.sh`) đã được thực hiện khi dựng baseline và được gỡ.

## Đọc theo thứ tự
1. `kb/00_INDEX.md`: knowledge base của hệ thống hiện tại (kiến trúc, lớp, process, luồng, cây trạng thái, hành vi theo nhánh, 223 lỗi, 51 quy tắc, mục tiêu, ma trận phủ).
2. `adr/ADR-013..017`: quyết định kiến trúc. ADR-017 rev 1 là baseline thiết kế đích.
3. `design/INDEX.md`: spec A1–A5 (module map, ICD, pipeline/timing, reducer, contract).
4. `ARCHITECTURE_REVIEW.md`, `risk_register_20260928.md`: review ban đầu (RC1–RC6, V1–V7, R-01..R-11).

## Work package
- **Wave 1:** `wave1/` (prompt, bản sửa R1, review). Deliverable nằm ở `WP-D0`, `WP-A1`–`WP-A6`, `WP-P0.1`, `WP-H1`.
- **Wave 2:** `wave2/` (`REVIEW.md`, `COMMON_CONTRACT_v2.md`, `prompts/`).

Deliverable của mỗi work package đặt tại `docs/refactor/<WP-ID>/`, bắt buộc có `REPORT.md`. Nếu còn câu hỏi chưa giải quyết thì có thêm `OPEN_QUESTIONS.md`. ADR của chương trình tạm đặt ở `docs/refactor/adr/`, và chỉ chuyển sang `docs/adr/` khi phase tương ứng đóng.

## Kiểm tra tài liệu
- `python3 tools/refactor/check_citations.py . docs/refactor`: mọi `file:line` phải nằm trong phạm vi file hiện có trong working tree (số dòng của `runtime_safety_current.md` và `tools/data.py` đã được ánh xạ lại sau baseline 2026-10-01).

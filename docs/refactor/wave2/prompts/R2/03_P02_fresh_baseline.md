# R2-03: P0.2 — baseline SITL mới trên `2543b0b4` (PR #18)

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Đọc:** `WP-P0.2.md`, `P0.2-ADDENDUM.md`, `P0.2-ADDENDUM-2.md`, `OWNER_DECISIONS_20260930.md` (P02-OQ01, P02-OQ02), contract v2.1 (baseline P0.2 giữ `2543b0b4`).
**Branch:** `refactor/WP-P0.2` (đã có). Tooling/report giữ trên branch; **product build lấy đúng `2543b0b4`** (worktree riêng, `repo_dirty=false`).

## Vì sao phải chạy lại
Session lịch sử không replay được: PX4 binary `c19…` không còn, và trải 2 binary cohort. Chúng giữ lại làm tham khảo, không vào baseline.

## Việc
1. **Build product** tại `2543b0b4` (worktree sạch). Ghi manifest: nav build hash, **PX4 binary SHA-256 + copy binary** vào `artifacts/baseline_<date>/px4_bin/` (hoặc đường dẫn lưu trữ cố định ghi trong manifest, kèm checksum). Thiếu binary = session không hợp lệ.
2. **Phân loại SAFE/FAST (P02-OQ01):** đọc policy BACKUP **hiệu lực** trong session (`backup_evidence_experiment`), không suy từ tốc độ. Thêm hàm này vào `baseline_summary.py` kèm test; session thiếu field → `UNCLASSIFIED`, không đoán.
3. **Ma trận:**
   - M1: frozen 5 m/s, 18 run (theo WP-P0.2), label SAFE/FAST theo bước 2.
   - M2: 2 scene runner hỗ trợ × 1/3/5 m/s × 5 run = 30 run. `structured_corner` bỏ (P02-OQ02), ghi `partial: 15 cell không có scene`.
   - Golden replay 2/2: chọn 2 run SAFE của M1, replay bit-identical theo WP-P0.2.
   - Không xoá run FAIL; không thay đổi số run; không tune.
4. **Số đo ADR-M3..M6** theo Addendum 2 §3; ADR-M1/M2/M5 thiếu field → `NOT_MEASURED`.
5. **Tự đủ để rejudge** (Addendum 2 §4): chạy `tools/runtime/rejudge_all.py` (J1) trên bản copy của 2 session; nếu J1 chưa merge dùng `report.py --session`.
6. **Máy dùng chung:** sau khi xong M2 scene mà P1 dùng cho A/B (xem `04_P1A_finalize.md`), báo agent P1 để dùng 3 run đó làm nhánh A. Ghi rõ scene, tốc độ, tracking mode (`off`) để P1 lặp lại đúng.
7. Cập nhật REPORT/`baseline_runs.csv`/`baseline_distribution.csv`, push, PR #18 → Ready khi đủ nghiệm thu.

## Nghiệm thu
- M1 18/18, M2 30/30 (2 scene), golden 2/2, tất cả cùng một binary cohort, `infrastructure_invalid` được báo rõ theo từng run.
- Distribution p50/p95/p99/max cho ADR-M3/M4/M6.
- Bảng limitation còn lại (nếu có) với lý do.

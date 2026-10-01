# R2-02: Rebase WP-J1 (PR #15)

**Bắt đầu khi:** #16 đã merge. **Branch:** `refactor/WP-J1`. Không đổi logic.

## Việc
1. `git fetch origin && git rebase origin/main`. Conflict dự kiến duy nhất: `docs/safety/runtime_safety_current.md` (ledger). Giữ mọi entry của cả hai phía theo thứ tự merge (H2, P0.3, rồi J1). Không sửa nội dung entry.
2. `python3 tools/validate_runtime_safety_ledger.py` PASS.
3. Gate v2 static + python (Python 3.12) trên head mới; J1 không chạm `src/` nên không cần ros.
4. Chạy lại `tools/runtime/rejudge_all.py` trên 16 session như lần trước; xác nhận vẫn 0 verdict đổi (dán bảng).
5. Cập nhật REPORT (head SHA, gate log, range-diff), push `--force-with-lease`, PR #15 → **Ready for review**.

## Giữ nguyên
- R7-17, R7-25, R7-26 OPEN; R7-27 NOT_FIXED (sẽ làm ở `05_J1b_policy.md`, PR riêng).
- Q-TRK default vẫn `relaxed` trong PR này (đổi ở J1b, commit behavior riêng).

Không tự merge.

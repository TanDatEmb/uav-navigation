# R2-05: WP-J1b — policy judge (R7-27, Q-TRK, nhãn tracking)

**Bắt đầu khi:** #15 đã merge. **Branch mới:** `refactor/WP-J1b` từ `origin/main`. Python-only (`tools/runtime/**`); không chạm `src/`, `config/`.
**Đọc:** `WP-J1.md`, `docs/refactor/WP-J1/{REPORT,OPEN_QUESTIONS}.md`, `OWNER_DECISIONS_20260930.md` (R7-27, Q-TRK, R7-25/26, Q-XTRK).

Mỗi mục là **một commit behavior** riêng: RED trước, ledger entry (Authority = `owner decision 2026-09-30 <ID>: <nội dung>`, xem README mục 0), validate ledger.

| Mục | Việc | Finding |
|---|---|---|
| J1b.1 | `evaluation_window` áp cho error statistics (position/velocity/acceleration) **và** coverage (`evaluation.py:1910-1934,2159-2161` trên baseline, kiểm lại). Một hàm chọn cửa sổ duy nhất dùng cho cả hai | R7-27 |
| J1b.2 | Runner default tracking mode `relaxed` → `off`. Giữ cảnh báo metadata khi mode ≠ `off` | Q-TRK |
| J1b.3 | Mọi số liệu tracking (kể cả cross-track) trong `report.json`/HTML gắn `authority: diagnostic`, `frame_status: diagnostic_frame_unverified` (R7-25/26 vẫn OPEN). Không được góp vào verdict — thêm test chứng minh verdict không đổi khi thay tracking stats bằng giá trị cực đoan | R7-25, R7-26, Q-XTRK |

## Rejudge
- Chạy `rejudge_all.py` trên toàn bộ session lịch sử + session P0.2 mới (nếu đã có). Bảng: session → verdict trước/sau → finding ID giải thích.
- Run nào PASS → FAIL (hoặc ngược lại) đưa vào debt trong ledger, không giấu (REVIEW §5).

## Cấm
- Không chuyển đổi frame trong Python; không mirror default waypoint C++ (R7-17 thuộc P2).
- Không đổi ngưỡng.

## Nghiệm thu
Gate v2 static + python PASS; 3 cặp RED→GREEN; bảng rejudge; PR draft → ready.

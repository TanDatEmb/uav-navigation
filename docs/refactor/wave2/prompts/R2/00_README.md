# Wave 2 — vòng R2: prompt triển khai (tổng công trình sư, 2026-09-30)

**Vị trí tài liệu (quan trọng):** bộ tài liệu điều phối nằm trực tiếp trong thư mục dự án trên máy, **không nằm trong git/`main`**:
- `docs/refactor/wave2/DISPOSITION_R1.md`
- `docs/refactor/wave2/prompts/R2/*.md` (README này, `OWNER_DECISIONS_20260930.md`, prompt 01–07)
- `docs/refactor/wave2/COMMON_CONTRACT_v2.md` + delta v2.1 dưới đây

Agent làm việc trong worktree riêng (tạo từ `origin/main`) nhưng **đọc các file trên bằng đường dẫn tuyệt đối**. Không copy, không commit, không sửa chúng; không tạo PR cho tài liệu điều phối.

## Trạng thái vào R2
- `main` = `74054ac` (đã merge #14 H2). Còn mở: #16 P0.3, #15 J1, #17 P1 PR-A, #18 P0.2.
- Branch protection của `main`: không có required check. Merge do **chủ dự án** bấm (merge commit, không squash). Agent KHÔNG tự merge.

## COMMON_CONTRACT v2.1 (delta, áp dụng cho mọi prompt R2)
0. **Trích dẫn quyết định chủ dự án:** vì `OWNER_DECISIONS_20260930.md` không nằm trong git, mọi ledger entry / REPORT / OPEN_QUESTIONS ghi Authority dạng tự đủ: `owner decision 2026-09-30 <ID>: <nội dung quyết định, 1 dòng>`. Không để link tới file này trong repo.
1. **Baseline:** `origin/main` tại lúc bắt đầu (ghi SHA). Kiểm `git log --first-parent 7e0b850..<baseline> -- src config` và dán vào REPORT: mọi commit ở đó phải là merge của PR wave 2 đã duyệt (#14, #16, #15, #17…). Có commit lạ → DỪNG, hỏi. (Thay cho yêu cầu "diff 7e0b850 rỗng", vốn chỉ đúng trước H2.) Riêng P0.2 giữ baseline `2543b0b4`.
2. **Gate ros (G1):** CTest chạy với `--packages-skip px4_ros2_cpp` (hoặc `-E integration_tests`); log `px4_ros2_cpp` chỉ đính kèm, không chặn.
3. **Rebase:** chỉ `git rebase origin/main` trên branch của chính WP. Conflict ledger (`docs/safety/runtime_safety_current.md`): giữ mọi entry của cả hai phía, theo thứ tự thời gian merge. Sau rebase **chạy lại gate trên head mới** và cập nhật SHA trong REPORT/PR body. Force-push chỉ bằng `--force-with-lease` trên branch của WP.
4. **Máy SITL dùng chung:** P0.2-R2 và P1A-R2 dùng cùng một máy; chạy tuần tự, không chạy song song hai stack.

## Thứ tự và phụ thuộc
| Bước | Prompt | Điều kiện bắt đầu | Máy | Kết thúc bằng |
|---|---|---|---|---|
| 1 | `01_P03_finalize.md` | ngay | ROS Jazzy | #16 ready-for-review → **chủ dự án merge** |
| 2 | `02_J1_rebase.md` | sau khi #16 merge | Python 3.12 | #15 ready → **merge** |
| 3 | `03_P02_fresh_baseline.md` | ngay (song song bước 1–2) | SITL | #18 ready → **merge** |
| 4 | `04_P1A_finalize.md` | sau khi #16 merge; phần A/B cần máy SITL rảnh (sau/đan xen bước 3) | ROS + SITL | #17 ready → **merge** |
| 5 | `05_J1b_policy.md` | sau khi #15 merge | Python 3.12 | PR mới |
| 6 | `06_P1B.md` | sau khi #17 merge | ROS + SITL | PR mới |
| 7 | `07_P2A.md` | sau khi #17 merge (song song bước 6) | ROS Jazzy | PR mới |
| 8 | P2 PR-B (`WP-P2.md` §P2.6) | sau P2 PR-A merge + #18 merge | SITL | wave 3 song song |

## Điều kiện để kiến trúc sư mở wave 3 (P3 certifier shadow, P4-0 trace, H3, P2b)
- Đã merge: #16, #15, #17, #18, J1b, P2 PR-A.
- P1 PR-B: PR mở, gate v2 PASS (merge có thể song song wave 3).
- P0.2: `baseline_distribution.csv` trên `2543b0b4` với M1 18/18 và M2 30/30 (2 scene), golden replay 2/2.
- Mỗi REPORT có bảng finding → trạng thái → commit.

Khi đủ, báo kiến trúc sư: danh sách PR đã merge + SHA `main` hiện tại.

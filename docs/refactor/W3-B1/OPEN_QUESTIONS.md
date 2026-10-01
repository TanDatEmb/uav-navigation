# W3-B1 — OPEN_QUESTIONS

## OQ-01 — guard dependency direction chưa có trên baseline

- Câu hỏi: có cần rebase/re-run W3-B1 sau khi W3-A2 merge để chạy
  `tools/check_dependency_direction.py` và `tools/gate.sh all` không?
- Bằng chứng: trên `origin/main=432dc94630fbc76ca670138228f2f616f6840bb0`,
  `tools/check_dependency_direction.py` chưa tồn tại; lệnh trả `No such file or directory`.
- Phương án: giữ nguyên write-set hiện tại; sau khi A2 merge, rebase branch này,
  chạy guard/gate trên đúng head mới và cập nhật REPORT.
- Trạng thái: OPEN; không phải code blocker của move diff.

## OQ-02 — màu diagnostic của `Trajectory::printProfile`

- Câu hỏi: chấp nhận compatibility alias cục bộ `color_text::{RESET,GREEN}`
  trong implementation đã move, hay muốn tách helper màu thành một WP riêng?
- Bằng chứng: implementation cũ dùng hai symbol này nhưng chỉ nhận được qua
  include chuyển tiếp `type_utils.hpp` → `navigation_math/type_utils.hpp`;
  target `navigation_planning::polynomial` không được phép thêm `rog_map_vendor`.
  Alias mới giữ nguyên byte ANSI và không đổi logic/threshold.
- Phương án hiện tại: giữ alias cục bộ trong `src/polynomial/trajectory.cpp`,
  không thêm ROS/msg hoặc `rog_map_vendor` vào `navigation_planning`.
- Trạng thái: OPEN để kiến trúc sư xác nhận; build syntax của TU đã PASS.

## OQ-03 — evidence build/CTest

- Câu hỏi: khi nào máy dùng chung giải phóng build lock để chạy gate ROS trên
  W3-B1?
- Bằng chứng: `make build` trả `ERROR: cannot start build: canonical build/runtime lock is held
  (pid=17616, mode=exclusive-build)`; PID thuộc build của worktree R2.
- Phương án: không kill tiến trình ngoài phạm vi; chạy lại `make build`, CTest package
  bị ảnh hưởng và reverse dependency ngay khi lock rảnh.
- Trạng thái: NOT_MEASURED.

## OQ-04 — lineage legacy

- Câu hỏi: có cần cung cấp object/tag chứa `7e0b850` để hoàn tất lệnh lineage trong contract?
- Bằng chứng: `git log --first-parent 7e0b850..432dc946 -- src config` trả
  `fatal: bad revision '7e0b850..432dc946'` trên clone hiện tại.
- Phương án: không phục hồi SHA legacy; giữ `NOT_MEASURED` theo COMMON_CONTRACT_v3.
- Trạng thái: OPEN/NOT_MEASURED.

# W3-B3 — OPEN QUESTIONS

## Q-B3-001 — prerequisite commits are absent on B3

- Câu hỏi: W3-B3 yêu cầu A1 đã merge và chạy song song sau B1; `origin/main` tại SHA `432dc94630fbc76ca670138228f2f616f6840bb0` không chứa A1 hoặc B1.
- Bằng chứng: `git log --first-parent 7e0b850..HEAD -- src config` trả `fatal: bad revision '7e0b850..HEAD'`; `origin/main` đã re-baseline và không còn object `7e0b850`.
- Xử lý: làm phần MOVE độc lập trên code B3 hiện có; không sửa B1 polynomial files, B6 adapter, H3 hoặc A2 tooling. Mọi include/target mismatch được ghi trong REPORT.
- Trạng thái: CONDITIONAL — owner cần rebase/đối chiếu khi A1/B1 merge.

## Q-B3-002 — `test_mission_progress` còn phụ thuộc shell integration

- Câu hỏi: test hiện tại có nên được tách lại trước khi đưa hẳn vào package `navigation_mission` không?
- Bằng chứng: `test_mission_progress.cpp` gọi `navigation_runtime::makeMissionGoal`, `navigation_execution::ExecutionAuthority`, `CommandSampler` và runtime identity helper; đưa nguyên file vào `navigation_mission` sẽ tạo test-only dependency ngược về shell hoặc buộc rewrite test.
- Xử lý: giữ test trong `navigation_runtime` nhưng link implementation mới của `navigation_mission`; production library và header đã chuyển. Không đổi hành vi test.
- Trạng thái: PARTIAL — cần owner/kiến trúc sư quyết định split test ở WP sau nếu nghiệm thu yêu cầu test physically nằm trong package mission.

## Q-B3-003 — provenance guard for uninitialized submodules belongs to A2

- Câu hỏi: A2 có thể thêm guard rõ ràng khi `source_fingerprint()` gặp submodule chưa init không?
- Bằng chứng: trước init, `git submodule status --recursive` hiển thị `-86d8239... src/external/px4_msgs` và `-4a3370f... src/external/px4_ros2_interface_lib`; `make build` lặp tại `build_provenance.py:126`. Sau `git submodule update --init --recursive`, cùng build chạy PASS.
- Xử lý: W3-B3 không sửa tooling; đã init submodule và xác nhận `PARALLEL_WORKERS=2 MAKE_JOBS=2 make build` tạo authoritative manifest. Guard/runtime error + test được giao cho A2 PR #2.
- Trạng thái: RESOLVED for B3 build; OPEN only as A2 follow-up.

## Q-B3-004 — full product build dependency initialization

- Câu hỏi: baseline này cần initialize/build `px4_msgs` ở worktree hay đổi dependency staging trước khi chạy full product gate?
- Bằng chứng: lỗi `px4_msgsConfig.cmake` chỉ xuất hiện khi submodule chưa init; sau init, `PARALLEL_WORKERS=2 MAKE_JOBS=2 make build` hoàn tất 23/23 package.
- Xử lý: không sửa PX4/B6/A2 trong W3-B3; chạy đúng `git submodule update --init --recursive` theo quy trình.
- Trạng thái: RESOLVED.

## Q-B3-005 — review rebase prerequisite is not present on remote main

- Câu hỏi: có thể rebase B3 lên `main` sau #7 khi owner merge #7/A1 vào `origin/main` không?
- Bằng chứng: sau `git fetch origin`, `origin/main` vẫn ở `432dc94630fbc76ca670138228f2f616f6840bb0`; object `00c0478` tồn tại nhưng `git merge-base --is-ancestor 00c0478 origin/main` trả false. Remote B3 đã được force-push bằng lease sau khi amend comment.
- Xử lý: không rebase lên nhánh chưa chứa #7; chỉ áp dụng các sửa §3 độc lập, giữ branch `refactor/W3-B3`, chờ owner merge prerequisite rồi rebase riêng theo quy trình.
- Trạng thái: OPEN — owner cần cập nhật `main`/ra lệnh rebase tiếp theo trước khi B3 được xem là merge-ready.

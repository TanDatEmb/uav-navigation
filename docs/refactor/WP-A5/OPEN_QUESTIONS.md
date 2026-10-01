# WP-A5 — Open questions

## OQ-01 — Shared schema root chưa có trong các ref WP-A4 hiện có

- Câu hỏi: shared schema root có được publish ở WP-A4-R1 hay chỉ được truyền
  qua task prompt?
- Bằng chứng: `git ls-tree -r --name-only origin/refactor/WP-A4` không trả về
  `docs/refactor/SCHEMA_decision_table.md`; task prompt cung cấp đầy đủ schema.
  D0 documents vẫn chỉ được đọc từ `origin/refactor/WP-D0` tại commit
  `6e531de29bcfe8efd7ba62f29b8ca4ff136fd6e4`, không merge hoặc cherry-pick.
- Xử lý trong WP-A5: dùng nguyên schema trong prompt làm nguồn chuẩn cho R1,
  không tự thêm shared schema root vào branch WP-A5; đây là dependency quản trị,
  không phải thay đổi hành vi.

## OQ-02 — Chính sách recovery của external odometry bridge khi epoch quay về 1

- Câu hỏi: `PropagatedOdometry.localization_epoch` có được phép quay về 1 sau
  restart hay phải có producer instance/generation mới?
- Bằng chứng: `strictly_newer_source_identity()` chỉ cho epoch mới hơn hoặc
  sequence lớn hơn trong cùng epoch (`frame_generation_policy.hpp:13-20`).
  External bridge gọi nó trước mọi gate (`px4_external_odometry_bridge_node.cpp:215-226`),
  trong khi `frame_generation_after_source_restart()` chỉ được dùng bởi
  executable bridge còn lại (`px4_odometry_bridge_node.cpp:307-330`).
- Phương án A: epoch restart phải được nâng thành public generation mới ở
  producer trước khi gửi sample.
- Phương án B: bridge nhận diện restart bằng source instance identity riêng.
- Xử lý trong WP-A5: không tự quyết định; ghi nhận hiện trạng và gap bảo mật
  continuity trong `bridge_continuity.md`.

## OQ-03 — Emergency velocity-only handoff

- Câu hỏi: emergency command có được phép đi qua velocity-only boundary không?
- Bằng chứng: `publishVelocityOnlySetpoint()` tính
  `certified_emergency` cho `ROLE_EMERGENCY/STATUS_BRAKING` nhưng ngay sau đó
  yêu cầu `STATUS_READY` (`navigation_mode_node.cpp:1702-1716`), nên emergency
  BRAKING không thể tới `adapt()` theo đường này.
- Phương án A: coi đây là chủ ý, emergency velocity-only luôn handover Hold.
- Phương án B: tách điều kiện status cho emergency trong WP hành vi riêng.
- Xử lý trong WP-A5: không đổi code; phân loại là gap thiết kế/coverage,
  không tuyên bố runtime reachability ngoài source logic.

## OQ-04 — Kiểm thử continuity sau gap hoặc source invalid

- Câu hỏi: sau một gap lớn hơn `max_dt` hoặc một frame có
  `source_valid=false`, có cần giữ trusted anchor cũ và yêu cầu generation mới
  trước khi cho phép một frame mới làm baseline hay không?
- Bằng chứng: `geometric_jump_continuity.cpp:130-135` reseed frame hiện tại
  với `continuity_trusted=false`; nhánh `95-100` sau đó tin frame kế tiếp mà
  không so delta với anchor cũ. Gate ở `external_odometry_gate.cpp:18-24`
  có thể mở khi các input còn lại hợp lệ.
- Xử lý trong WP-A5: không sửa code; ghi nhận đây là finding CONFIRMED ở mức
  component và CONDITIONAL ở mức tích hợp. Test hiện có chưa bao phủ chuỗi
  gap/source-invalid -> jump -> recovery.

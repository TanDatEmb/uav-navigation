# WP-A2 — giao lại

Giao lại **nguyên văn** prompt `prompts/WP-A2.md` của gói đợt 1, cho một agent mới. Thêm các điều sau vào cuối prompt:

1. Branch `refactor/WP-A2` phải được push **ngay sau commit đầu tiên**, kể cả khi chưa xong, và mở draft PR ngay lúc đó. Mục đích là để trạng thái luôn nhìn thấy được. Lần trước WP này không để lại branch nào.
2. Nếu phải dừng giữa chừng: commit phần đã làm, ghi `REPORT.md` phần "đã làm / chưa làm / vì sao dừng", rồi push.
3. Format YAML phải parse được bằng `yaml.safe_load`. Thêm `docs/refactor/WP-A2/validate_icd.py`, kiểm:
   - (a) mọi publisher/subscriber trong `src/` (trừ test/external/vendor) có mặt trong `icd_topics.yaml`;
   - (b) cả 12 msg `navigation_contracts` có đủ field (so với file `.msg`);
   - (c) mọi `DiagnosticStatus.name` được parse trong `tools/runtime/*.py` và trong `src/` có mặt trong `diagnostic_channels.yaml`.
4. Thêm deliverable `evidence_candidates.md`. Với mỗi `DiagnosticStatus.name` mà judge hoặc gate đang dùng, liệt kê các key thực sự được đọc (`file:line` phía consumer) cùng kiểu dữ liệu. Đây là đầu vào trực tiếp để thiết kế `nav_evidence_msgs` (ADR-015).

# Knowledge base hệ thống: uav-navigation (`main @ 7e0b850`, 2026-09-29)

Bộ tài liệu này gom toàn bộ tri thức về hệ thống hiện tại. Nó là nền cho bước tiếp theo: cải thiện kiến trúc và chốt kiến trúc chuẩn mới (ADR-017 cùng các bổ sung D10–D14 và H2).

## Checklist: trạng thái và nơi trả lời
| # | Hạng mục checklist | Trạng thái | Tài liệu |
|---|---|---|---|
| 1 | Toàn bộ kiến trúc dự án | ✅ | `01_architecture.md` |
| 2 | Các lớp | ✅ (L0–L6) | `01` §3, `02_layers_responsibilities.md` |
| 3 | Các process | ✅ (6 process product, ≥8 hạ tầng/qualification; thread, timer, lock, clock) | `01` §2, `03_processes_threads.md` |
| 4 | Luồng hoạt động | ✅ F1–F13 | `04_flows.md` (đích: `design/A3_*`) |
| 5 | Cây trạng thái hệ thống | ✅ S0–S9 + bất biến chéo X1–X9 | `05_state_tree.md` |
| 6 | Hành vi ở từng nhánh | ✅ ma trận sự cố B1–B23 + bảng branch theo hàm (7 vùng) | `06_behavior_per_branch.md`, `areas/*` |
| 7 | Chức năng, nhiệm vụ, nhu cầu thông tin mỗi lớp | ✅ | `02_layers_responsibilities.md` |
| 8 | Điểm nghẽn | ✅ 12 finding + bottleneck theo thread | `07` §2, `03` §5 |
| 9 | Điểm xung đột | ✅ 51 finding | `07` §3, §10 |
| 10 | Lỗi toán học | ✅ 29 finding (kèm danh sách phần toán **đã xác nhận đúng**) | `07` §4, `01` §4 |
| 11 | Lỗi hiệu năng | ✅ 21 finding | `07` §5 |
| 12 | Lỗi logic, quy tắc lập trình | ✅ 49 LOGIC + 46 CODING_RULE + 15 CONFIG | `07` §6–§8 |
| 13 | Quy tắc chung toàn dự án | ✅ 51 quy tắc có cách kiểm, kèm nhóm chống over-engineering | `08_project_rules.md` |
| 14 | File và đối tượng đã duyệt | ✅ 771 file × độ sâu × câu hỏi Q1–Q12 | `10_coverage.md`, `data/coverage_matrix.csv` |
| 15 | Mục tiêu cuối cùng | ✅ G1–G7 kiểm được, rà lại ADR-017, phân bổ finding theo phase | `09_goal_and_target_direction.md` |

## Thứ tự đọc
- **Người mới:** 01 → 02 → 04 → 05.
- **Người thiết kế:** 06 → 07 §1 và §10 → 09 → `design/` → ADR-017.
- **Agent triển khai:** 08 (quy tắc) + `data/findings.csv` (theo ID) + `areas/R*_layer.md`, là báo cáo gốc của reviewer theo vùng.

## Phương pháp và giới hạn
- **Phân tích tĩnh** trên `7e0b850`. Code được chia thành 7 vùng (R1–R7), mỗi vùng có một reviewer riêng với cùng brief và cùng luật evidence.
- **Kiểm chứng:**
  - Script kiểm cho 223/223 finding rằng file tồn tại, dòng nằm trong phạm vi, và excerpt khớp nguyên văn.
  - 5 finding bị cite sai dòng đã được sửa.
  - Kiến trúc sư tự đọc lại code cho 2 S1 và các S2 tiêu biểu (R4-06, R3-11, R7-25), kèm N6 từ đợt trước.
  - Các reviewer có chạy repro độc lập cho một số finding (R4-01, R4-02, R5-08, R5-16, R6-02, R2-14, R2-17, R2-24, R7-10, R7-23, R7-24). File repro nằm trong scratch; cần đưa vào repo khi giao việc sửa.
- **Không làm:**
  - Không chạy SITL.
  - Không chạy TSan.
  - Không build C++ đầy đủ (thiếu boost, pcl).
  - Không đọc sâu nội bộ vendor (chỉ xem ở mức interface).
  - Mọi kết luận về runtime có nhãn `PLAUSIBLE` nếu chưa có repro.
- **Không tune ngưỡng nào.** Mọi chỗ "chọn giá trị" đều được ghi là quyết định của chủ dự án.

## Liên kết với các tài liệu trước
- `ARCHITECTURE_REVIEW.md` (RC1–RC6, V1–V7), `risk_register_20260928.md` (R-01..R-11).
- `design/A1`–`A5`, `adr/ADR-013`..`ADR-017`.
- Review này bổ sung **RC7** (đại lượng lệch thời điểm hoặc lệch frame) và **RC8** (latch không có đường thoát). Hai root cause này được ghi trong `01` §5.

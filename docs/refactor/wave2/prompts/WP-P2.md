# WP-P2: Evidence typed + decoder sinh tự động + judge v2 chạy song song (RC4, V7)

**Loại:** additive, tức dual emit (không tắt `DiagnosticArray`, không đổi control).

Chia 2 PR:
- **PR-A (P2.1–P2.5):** làm được ngay sau khi P1 PR-A merge. P2.3 cần hash của profile.
- **PR-B (P2.6):** cần bag MCAP từ P0.2.

**Spec:** ADR-015; ADR-017 D8 và §6.2 (chỉ các key judge dùng cho verdict; package tên `navigation_evidence`); `design/A2_icd_current_and_target.md` §4; `design/A5_contract_specs.md` §4.
**Input:** `docs/refactor/WP-A2/{diagnostic_channels.yaml, evidence_candidates.md, icd_msgs.yaml}`.
**Finding:** R-05, R7-31, V5 (chỉ phần bridge đọc generation qua evidence, ghi chú cho P6), V7.

## P2.1: inventory key judge đang đọc (bắt buộc làm trước)
- Viết script `docs/refactor/WP-P2/judge_key_inventory.py`, quét AST của `tools/runtime/{evaluation,report,external_mode_scenario,monitor,html_report,flight_review_report}.py`.
- Output: `judge_key_inventory.csv` với các cột `channel, key, reader file:line, dùng cho verdict (Y/N), kiểu, enum ordinal nếu có`.
- Đối chiếu với `WP-A2/diagnostic_channels.yaml`. Key có trong inventory mà không có trong A2 thì ghi rõ.
- **Quy tắc chọn:** chỉ key có `dùng cho verdict = Y` mới được đưa vào msg typed. Các key khác vẫn ở lại `DiagnosticArray`, dạng diagnostic thuần.

## P2.2: package `src/contracts/navigation_evidence` (chỉ chứa msg)
- `EvidenceHeader.msg`, đúng như `A2 §4`: stamp nguồn, `producer_id`, `producer_instance_id`, `sequence`, `clock_domain`, `schema_version`, `safety_profile_hash`.
- Các họ msg chỉ phủ key verdict. Thứ tự ưu tiên:
  1. `LifecycleEvent`
  2. `RetainedDecisionEvidence`
  3. `WorldTransactionEvidence`
  4. `ExecutionEvidence`
  5. `CommandRejectionEvidence`
  6. `SetpointInputEvidence`
  7. `EstimatorEvidence`
  8. `ConfigWitness`
  9. `DropCounter`

  `CertificateEvidence` chỉ khai báo msg; P3 mới publish.
- **Mỗi enum** có hằng số tường minh. Producer phải có `static_assert` rằng hằng số msg bằng giá trị enum C++. Ví dụ `RetainedDecisionDisposition`, `CandidateSource`, `RetainedValidationPurpose`, `PlannerResultDisposition`: đây là nguyên nhân gốc của R-05 và R7-31. Trước hết phải gán giá trị tường minh cho các enum C++ đó (refactor, giá trị giữ **đúng ordinal hiện tại**).

## P2.3: dual emit (additive)
- Producer publish msg typed **cùng thời điểm và cùng dữ liệu** với `DiagnosticArray` hiện có. Không đổi thứ tự hay điều kiện phát.
- QoS: reliable, keep_last 1000, volatile. Riêng `ConfigWitness` dùng transient_local, depth 1, phát lúc start với `safety_profile_hash` lấy từ P1.
- `sequence` tăng +1 cho mỗi producer.
- **Không** dựng thêm chuỗi hay cấp phát trên đường 50 Hz ngoài phần tối thiểu để điền msg. Đo thời gian thêm trên tick command nếu có harness; không có thì ghi `NOT_MEASURED`.

## P2.4: ghi evidence
- Runner (`RUNTIME_EVIDENCE_TOPICS`, `runner.py:132-160`) thêm các topic evidence mới.
- rosbag2 dùng storage MCAP (ADR-015). Kiểm plugin có sẵn trong Jazzy; không có thì DỪNG và ghi OPEN_QUESTIONS.

## P2.5: decoder sinh tự động
- `tools/runtime/evidence_v2/typestore.py`: đăng ký type từ các file `.msg` trong repo bằng `rosbags`. Pin version của `rosbags`.
- API `iter_evidence(bag_path, msg_type) → typed records`.
- Test round-trip: serialize bằng `rosbags`, rồi decode. Không cần ROS runtime.
- Guard mới `tools/check_judge_no_string_keys.py`: dùng cho `evidence_v2/` và các reducer v2. Cấm đọc `KeyValue`/`DiagnosticArray` trong các module đó.

## P2.6 (PR-B): judge v2 chạy song song và parity
- Viết lại **chỉ** các reducer ra verdict: lifecycle, retained decision, world transaction, C0_SW. Chúng đọc từ `evidence_v2`.
- `report.json` thêm block `evaluation_v2`. **Verdict chính vẫn lấy từ v1.**
- Bag của P0.2 (baseline) **không** chứa msg typed. Vì vậy parity cần một lượt chạy mới trên build có PR-A: ma trận frozen 5 m/s của P0.2 (18 run), cùng máy và cùng quy trình. Các bag này đồng thời cho thấy dual emit không làm lệch phân phối so với baseline P0.2; so bằng `baseline_distribution.csv`.
- Parity: chạy trên mọi bag của lượt chạy đó. Mọi trục phải cho `v1 == v2`. Mỗi lệch được phân tích thành "lỗi v1" (dẫn chiếu finding) hoặc "lỗi v2" (sửa v2). Không nới điều kiện so sánh.
- Chỉ khi parity đạt 100% (hoặc mọi lệch đã được giải thích là lỗi v1 kèm ledger) mới mở WP P2-switch để chuyển verdict chính sang v2 và tắt các key diagnostic đã được thay.

## Ngoài phạm vi
- D6 (một đường mission) và D2(c) (bỏ echo acceptance, bỏ `/mission_complete`) là behavior. Chúng thuộc WP P2b riêng.
- Không đổi QoS của topic control.

## Nghiệm thu
- **PR-A:**
  - gate v2 cả ba mức;
  - `static_assert` phủ mọi enum qua biên (liệt kê trong REPORT);
  - test round-trip decoder;
  - một bag SITL (nếu có) chứa msg typed với sequence không gap; không có SITL thì dùng bag tổng hợp từ test node.
- **PR-B:** bảng parity theo từng bag và từng trục.

(Áp dụng `COMMON_CONTRACT_v2.md`.)

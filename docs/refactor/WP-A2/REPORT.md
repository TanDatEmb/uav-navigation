# WP-A2 — ICD hiện trạng

## 1. Tóm tắt

1. Đã lập inventory AS-IS cho topic, message field, diagnostics, parameter và TF trên `main@7e0b850`.
2. Không sửa `src/`; mọi anchor đều trỏ về source/config/runner của baseline.
3. `icd_msgs.yaml` bao phủ 12 message `navigation_contracts` và 22 kiểu `px4_msgs` được product/library/observer chạm tới; bốn message lớn có tổng 41/65/32/33 declaration.
4. QoS ghi theo code path thực tế; các endpoint PX4/ros_gz ngoài clone được đánh dấu `NOT_STATICALLY_DECLARED`.
5. Các rate không có phép đo bag/runtime được ghi `NOT_MEASURED`.
6. Đã đánh dấu mapping QoS lệch helper/thực thi, topology mission có điều kiện, SITL-only visibility publisher và diagnostics chuỗi dùng làm safety gate.
7. `icd_params.yaml` có 169 entries, gồm 12 `navigation_runtime.inject_*`, fault-injection gate SITL và các YAML-only config blocks.
8. Đã thêm validator YAML/source và danh sách evidence candidates cho ADR-015.

## 2. Deliverables

- [icd_topics.yaml](icd_topics.yaml)
- [icd_msgs.yaml](icd_msgs.yaml)
- [diagnostic_channels.yaml](diagnostic_channels.yaml)
- [icd_params.yaml](icd_params.yaml)
- [tf_frames.md](tf_frames.md)
- [findings.md](findings.md)
- [evidence_candidates.md](evidence_candidates.md)
- [validate_icd.py](validate_icd.py)
- [extract_check.py](extract_check.py)
- [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md)
- [REPORT.md](REPORT.md)

## 3. Lệnh verify và output thật

Chạy trong `<repo>`:

```text
$ python3 docs/refactor/WP-A2/validate_icd.py
PASS: topic create-call coverage, 12 navigation_contracts messages, and 17 diagnostic channels
exit code: 0

$ python3 docs/refactor/WP-A2/extract_check.py
extract_check: no missing interfaces (46 create_* sites)
exit code: 0

$ python3 - <<'PY'  # yaml.safe_load trên bốn YAML deliverable
diagnostic_channels.yaml: OK dict 3
icd_msgs.yaml: OK list 34
icd_params.yaml: OK list 169
icd_topics.yaml: OK list 46
exit code: 0
PY

$ python3 - <<'PY'  # inventory counts
params=169 fault_injection=18 yaml_only_orphan=7
exit code: 0
PY

$ python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
exit code: 0

$ git diff --check
exit code: 0
```

Không chạy build/SITL/runtime vì đây là phân tích read-only và không có số đo
rate/QoS negotiated trong clone.

## 4. Chỗ lệch khỏi prompt và lý do

- `px4_msgs` nằm trong submodule/dependency ngoài source message contract; ICD
  ghi 22 schema product/library/observer chạm tới và field không đọc là
  `unused: true` khi có đủ schema anchor, không tự nhận đây là toàn bộ upstream.
- `ARCHITECTURE_REVIEW.md`, ADR-013..016 và risk register được yêu cầu đọc
  nhưng không tồn tại trên commit baseline `7e0b850`; bản untracked tương ứng
  trong clone D0 hiện tại chỉ được dùng làm ngữ cảnh, không được coi là source
  provenance của ICD. Chi tiết ở `OPEN_QUESTIONS.md`.
- QoS của firmware PX4 và `ros_gz_parameter_bridge` không được source clone
  công bố; dùng `NOT_STATICALLY_DECLARED`, không suy đoán.
- `navigation_planning/planner` được giữ như alias parser/tooling nhưng không
  có publisher product tại baseline.
- `rate` chưa đo bằng bag nên giữ `NOT_MEASURED`, kể cả khi timer/config nêu
  50/20/2 Hz.

## 5. Open questions

Xem [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md): QoS offered thực của PX4/DDS,
phạm vi schema upstream cần đưa vào ICD đích, và lifecycle contract của các
topic mission tạo có điều kiện.

## 6. Commit SHA

- Commit đầu tiên đã push: `9f1e0abf` (`docs: add WP-A2 current ICD`).
- Commit evidence/params follow-up đã push: `5a6cf820` (`docs: complete WP-A2 evidence candidates`).
- Commit hoàn thiện REPORT: được ghi trong verify cuối bằng `git log
  --oneline -- docs/refactor/WP-A2`; không rewrite history.

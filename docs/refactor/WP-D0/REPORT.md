# WP-D0 — Đưa tài liệu kiến trúc vào repo

## Tóm tắt

WP-D0 đã đưa nguyên văn bộ tài liệu kiến trúc vào `docs/refactor/`.

Branch được tạo từ baseline `main @ 7e0b850`.

Hai ADR được giữ trong `docs/refactor/adr/`, không đặt vào `docs/adr/`.

README thư mục mô tả mục đích, danh sách WP và quy ước deliverable.

Không có thay đổi code sản phẩm hoặc thay đổi safety contract.

Các file nguồn và file đích đã được kiểm tra bằng `cmp` và SHA-256.

Validator safety ledger và kiểm tra whitespace đều PASS.

## Deliverables

- `docs/refactor/README.md`
- `docs/refactor/ARCHITECTURE_REVIEW.md`
- `docs/refactor/risk_register_20260928.md`
- `docs/refactor/adr/ADR-013-nav-core-process-topology.md`
- `docs/refactor/adr/ADR-014-independent-certifier.md`
- `docs/refactor/adr/ADR-015-typed-evidence-rosbag.md`
- `docs/refactor/adr/ADR-016-beta-scope-sitl.md`

## Verification

```text
$ cmp <each input file> <corresponding docs/refactor file>
exit code: 0; output: none

$ python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
exit code: 0; tests pass/fail: N/A (structural validator, không báo cáo test count)

$ git diff --check
exit code: 0; output: none
```

SHA-256 của 6 file copy trùng nguồn:

```text
ARCHITECTURE_REVIEW.md  2fd11e7451063f62bc6286c0acccde08ac2eef5dec4f8b943889851120ce8da7
risk_register_20260928.md  26303067862d2d3b12bd6eebea7f9bab8b3d7064b6aaa1ebc710f0dcf4afc173
ADR-013  1d2477e789f765694e2ed6a1801882cdd6e1ae1d57b8a78825a632daf48a5098
ADR-014  475b4e067b34ce7f371107cd61118b1728329431965e434170d070d96fa7be5d
ADR-015  797b04c9d09b7f92e3101af36021e20c9f68017b7828a9fbb8e60b568f575906
ADR-016  29c2ac241eb0b74cbf3b81cc3e628e4477654238b960470d89a84c065d492129
```

## Lệch khỏi prompt

- Thư mục input `docs/refactor_program/` (trùng với `docs/refactor/`, đã gỡ ở baseline 2026-10-01) được giữ nguyên và không đưa vào
  commit; chỉ các file được yêu cầu trong `docs/refactor/` được stage.
- WIP có sẵn ngoài phạm vi vẫn chưa stage và chưa thay đổi.

## Open questions

Không có. Validator không yêu cầu bổ sung thư mục mới vào safety ledger.

## Commit SHA

- Baseline: `7e0b8508781f68ecbd18d15d40129e019108f3e7`
- Commit chứa deliverables: `132f1ca0`

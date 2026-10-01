# WP-A6 — Kiểm kê ngưỡng và hằng số

## Tóm tắt

1. WP-A6 là phân tích read-only trên baseline `main @ 7e0b850` và branch
   `refactor/WP-A6`; không sửa `src/` hoặc `config/`.
2. `constants.csv` có 119 dòng semantic, mỗi dòng gom một ý nghĩa và giữ
   occurrence `file:line|kind[value]`, authority, `all_equal`, `yaml_loadable`
   và `pinned`.
3. Inventory bao phủ các nhóm `timing`, `freshness`, `lease`, `envelope`,
   `geometry`, `tolerance_numeric`, `retry`, `queue`, `evidence`, `judge`.
4. Các điểm lệch được ghi thành scenario cụ thể trong `inconsistencies.md`;
   không có giá trị nào được tune hoặc đổi trong WP này.
5. `safety_profile_draft.yaml` là profile AS-IS, resolve YAML override trước
   C++ default; các authority chưa resolve giữ `CONFLICT` và trạng thái
   qualification là `NOT_EVALUABLE`.
6. `scan.py` tái tạo raw candidate stream 6.292 occurrence; stream này rộng
   hơn semantic inventory và không tự chứng minh reachability/runtime.
7. Mọi HG có số trong `runtime_safety_current.md` đều có `hg_id` trong CSV,
   gồm `HG-002;HG-013` trên cùng corridor-plane tolerance row.
8. Safety-ledger validator và các kiểm tra cấu trúc cuối đều PASS; chưa chạy
   SITL/PX4 hoặc phân phối recorded-sensor vì đây là deliverable read-only.
9. R1 bổ sung phân loại toàn phần trên 13 file quyết định: 1.229 candidate,
   phủ 100%, 47/47 witness bắt buộc và mọi `INCLUDED` trỏ về `constants.csv`.

## Deliverables

- [constants.csv](constants.csv): semantic inventory và provenance.
- [inconsistencies.md](inconsistencies.md): các dòng có conflict, mixed
  YAML/literal authority hoặc parameter bị pin, kèm scenario tác động.
- [safety_profile_draft.yaml](safety_profile_draft.yaml): profile draft theo
  group, giữ giá trị AS-IS và `CONFLICT` khi chưa có owner/effective value.
- [scan.py](scan.py): scanner tái tạo raw occurrence list trong scope WP-A6.
- [literal_classification.csv](literal_classification.csv): phân loại từng
  numeric literal trong các biểu thức quyết định của R1.
- [check_constants.py](check_constants.py): checker read-only (có `--write`
  để tái tạo classification), kiểm tra witness, coverage và tên INCLUDED.
- [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md): các câu hỏi provenance/authority
  chưa đủ bằng chứng để tự quyết.
- [REPORT.md](REPORT.md): báo cáo này.

## Phương pháp scan và lọc false positive

`scan.py` quét C/C++ trong `src/` sau khi loại path có component
`test`, `tests`, `external`, `vendor`, `third_party` hoặc kết thúc bằng
`_vendor`; YAML trong `config/runtime/**/*.yaml` và `src/*/*/config/*.yaml`;
Python trong `tools/runtime/*.py`; và các dòng bảng HG trong
`docs/safety/runtime_safety_current.md`.

Các regex chính được định nghĩa trực tiếp trong `scan.py`:

- `NUMBER = (?<![A-Za-z0-9_.])[-+]?(?:\d[\d']*(?:\.\d[\d']*)?|\.\d+)(?:[eE][-+]?\d+)?[fFlLuU]*(?![A-Za-z0-9_.])`.
- `SEMANTIC` nhận các token vật lý/safety như `age`, `anchor`, `accel`,
  `jerk`, `speed`, `velocity`, `radius`, `tolerance`, `budget`, `limit`,
  `timeout`, `lease`, `fresh`, `stale`, `retry`, `queue`, `horizon`,
  `clearance`, `resolution`, `voxel`, `goal`, `corridor`, `thrust`,
  `visibility`, `watchdog`, `sample`, `rate`, `duration`, `distance`,
  `capacity`, `count`, `size`, `iterations` và `association`.
- `PYTHON_SEMANTIC` nhận các token judge/harness như `threshold`, `stale`,
  `residual`, `collision`, `acceptance_`, `stop_enter`, `deadline` và các
  tên gate tương đương.
- `COMPARISON = (?:[<>]=?|==|!=|std::abs|std::max|std::min|clamp)` và
  `HG_LINE = ^\\| (HG-\\d{3}) \\|.*`.

False-positive handling có hai lớp. Scanner chỉ phát candidate: nó bỏ comment
`//`/`#`, deduplicate multiline `declare_parameter`/`LoadParam`, và giữ YAML
numeric keys/vectors để review. Khi gom thành semantic row, các literal thuần
index/size, unit conversion, diagnostic formatting, enum/bitmask, test-only
và epsilon roundoff dưới `1e-6` không được tạo thành safety key riêng; epsilon
được gom vào `tolerance_numeric`. Các trường hợp chưa chắc thuộc safety owner
được giữ trong CSV với note/`NOT_EVALUABLE`, không bị âm thầm bỏ qua.

### R1 changes

- Giữ nguyên 119 semantic rows; bổ sung các row 0.15 m/s theo từng authority,
  bridge envelope/continuity, adapter airborne gate, visibility 500 ms, các age
  200/500 ms, PVAJ roundoff, command-clock tolerance và judge/harness gates.
- `pinned=true` chỉ còn 4 row có witness `navigation_mode_node.cpp:255-258`;
  đó là các ROS parameter bị reject khi khác literal.
- `check_constants.py` quét đúng 13 file trong prompt, loại comment và
  string/char literal trước khi áp regex số; trigger là comparison,
  `min/max/clamp` hoặc default parameter. Output phân loại là:
  `INCLUDED=20`, `EXCLUDED:numeric_epsilon<=1e-6=806`,
  `EXCLUDED:test_only=248`, `EXCLUDED:index/size=144`,
  `EXCLUDED:enum_or_bitmask=7`, `EXCLUDED:diagnostic_format=4`.
- Prompt ghi declaration `px4_external_odometry_bridge_node.cpp:56`, còn
  numeric default nằm ở `:57`; checker nhận cả declaration anchor `56` và
  literal witness `57`, không bịa thêm occurrence.

## Verification

Các lệnh dưới đây chạy tại
`<repo>`.

    $ python3 docs/refactor/WP-A6/scan.py --csv /tmp/wp-a6-scan-final.csv
    scan exit code: 0
    $ wc -l /tmp/wp-a6-scan-final.csv
    6293 /tmp/wp-a6-scan-final.csv
    # 6292 raw candidates + header
    constexpr 231
    doc 28
    literal 4132
    param_default 149
    python 787
    yaml 965

    $ python3 -m py_compile docs/refactor/WP-A6/scan.py
    py_compile exit code: 0

    $ python3 -m py_compile docs/refactor/WP-A6/check_constants.py
    py_compile exit code: 0

    $ python3 docs/refactor/WP-A6/check_constants.py
    target_files=13
    constants_rows=119
    literal_candidates=1229 classification_rows=1229
    included=20 excluded=1209
    required_file_line_occurrences=47/47
    literal_coverage=100%
    status=PASS

    $ python3 - <<'PY'
    # CSV/YAML structural check: row count, 11 columns, allowed groups,
    # PVAJ anchors and HG coverage.
    # PY
    CSV/YAML structural checks: PASS (119 rows, 11 fields, 0 missing anchors)
    HG coverage: PASS (HG-001,HG-002,HG-004,HG-005,HG-006,HG-007,HG-008,HG-009,HG-010,HG-011,HG-013,HG-027,HG-034)

    $ python3 tools/validate_runtime_safety_ledger.py
    runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
    ledger validator exit code: 0

    $ git diff --check 7e0b8508781f68ecbd18d15d40129e019108f3e7 HEAD
    git diff --check exit code: 0

    $ git diff --name-only 7e0b8508781f68ecbd18d15d40129e019108f3e7 HEAD -- src config
    # empty output
    product/config path check exit code: 0

    $ git status --short --branch
    ## (detached HEAD 6aed0ded before R1 artifact commit)

## Deviations / provenance

- `ARCHITECTURE_REVIEW.md`, ADR-013..016 và risk register không tồn tại trên
  baseline; chúng chỉ được đọc read-only từ `refactor/WP-D0` làm context, không
  copy, merge hoặc cherry-pick vào branch này. Chi tiết ở `OPEN_QUESTIONS.md`.
- `constants.csv` là semantic aggregation; scanner raw candidate stream không
  phải proof rằng mọi occurrence reachable ở runtime.
- `literal_classification.csv` là static source classification, không phải
  chứng cứ runtime reachability; các `test_only`/epsilon/index exclusions vẫn
  cần owner review khi P1 tạo profile.
- Không chạy SITL/PX4 hay recorded-data distribution. Vì vậy mọi nhận định về
  effective runtime qualification, latency distribution hoặc flight safety
  vẫn `NOT_EVALUABLE` khi thiếu manifest/witness tương ứng.

## Open questions

Xem [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md). Các điểm chính là authority/units
của HG-001, effective units của thrust, manifest hiệu lực SIM/dataset/judge,
phạm vi owner của epsilon dưới `1e-6`, và line anchor 56/57 của bridge.

## Commit SHA

- `954ecae1`: initial WP-A6 artifacts.
- `b600398b`: provenance update.
- `e0c7f4b0`: complete semantic inventory, inconsistencies, profile và open questions.
- `7debfbbf`: report-carrier commit trước lần chỉnh report cuối.
- `26dea65a`: R1 artifact commit, thêm `check_constants.py`,
  `literal_classification.csv` và cập nhật open questions.
- Commit report-carrier kế tiếp được tạo sau dòng SHA này; SHA HEAD sau push
  được xác nhận trong handoff để tránh tự tham chiếu trong chính `REPORT.md`.

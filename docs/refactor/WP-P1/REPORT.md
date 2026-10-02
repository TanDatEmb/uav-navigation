# WP-P1 PR-A — báo cáo nghiệm thu

## Kết luận

**Đề xuất giữ draft PR cho P1.1–P1.3; không tuyên bố nghiệm thu A6 hoặc mission qualification PASS.** Release build/manifest/CTest và run witness cuối được thực hiện trên exact source HEAD `e6dd53db9d8c2e00e77772c23d95b7be7dab87bb` (`repo_dirty=false`). Các kiểm tra nhẹ được chạy lại trên PR/report HEAD `316ce661fbdc8ee6d9c239ee24206588cec097b1`: 10 guard, codegen, ledger, compileall, Python 3.12.3 discovery (474 tests, 2 skipped) và diff-check đều PASS. Run ghi đủ năm startup witness thật ở tracking `off`; cả năm có hash `994102588801b7fd`, key count 53/10/9/6/4 và `mismatches=[]`. Scenario runner trả `FAIL`, evaluation `NOT_EVALUABLE`: mission timeout, không observed External Mode handover, waypoint 4 không được nhận (chỉ có `[0,1,2,3]`). Đây là bằng chứng cho profile startup của năm process, không phải qualification evidence cho mission. Bản sửa tiếp theo chỉ thay REPORT; không làm thay đổi mã nguồn/build inputs. Build/manifest/CTest/witness không được nhận là chạy trên docs-only SHA.

Oracle A6 giữ ba hàng owner-blocked: `maximum_thrust_n`, `minimum_thrust_n`, `max_yaw_acceleration_rad_s2`. Exact checker output là `semantic_rows=119 represented=114 excluded=2 open_questions=3`, `failures=1`, cùng đúng ba tên trên (exit 1). Không đổi giá trị hiệu lực hoặc oracle. Draft PR chỉ trình bày phần độc lập đã làm, nêu blocker và chờ owner; không merge.

## Cây, baseline và KB

- Branch `refactor/WP-P1`, baseline `2543b0b4bc469c709038b4db9e332b6b9d1de11f`; implementation commit `6c050804af733f6a7772ebcf1c2cb7bf51cca685`; exact tested source HEAD `e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`. Báo cáo wave-2 contract audit yêu cầu cập nhật này vì nội dung cũ trỏ tới `d5ab1ae8`. Artifact run exact-head có `metadata.json.repo_commit=e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`, `repo_dirty=false` tại `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260930T064849-78862` .
- Bắt buộc trước sửa: `git diff 7e0b850 2543b0b4bc469c709038b4db9e332b6b9d1de11f -- src config` → không có output, exit 0.
- Áp dụng KB-08 C1–C5, K3, Q1 theo bảng dưới. P1 PR-A chỉ thêm profile/codegen/loader/witness/docs; không bắt consumer lấy giá trị từ profile và không enforce hash/missing keys (P1.4/PR-B).
- Q-HG001 dùng các giá trị đã được chủ dự án chốt. Q-ENV, Q-VOX, Q-UNK, Q-TRK, Q-XTRK vẫn OPEN; không tự diễn giải.
- `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp` trùng vùng sửa với H2/PR #14. Sau khi H2 merge phải rebase, giữ cả hai thay đổi; không cherry-pick.

## Thực hiện

- Thêm profile YAML hiện trạng, schema/oracle, Python/C++ codegen và package `nav_safety_profile`; profile ghi 114/119 semantic rows, có 3 hàng OPEN_QUESTION và 2 hàng excluded.
- Năm process khai báo `safety_profile.path`, nạp profile và phát witness startup chỉ khi load thành công. Load failure phát marker rõ ràng; runner giữ `status=unavailable`, không tạo hash/witness giả. Effective values được đọc từ tham số/member đang chạy, named constant hoặc planner YAML thực tế.
- Runner gom record theo logger/process vào `metadata.runtime_configuration.safety_profile`; hash mismatch chỉ là evidence, không đổi quyết định arm. Thêm parser/unit tests, static codegen check, guard CI và đưa package profile vào product test list.
- Cập nhật HG-001 trong `docs/safety/runtime_safety_current.md` theo prompt.
- Chẩn đoán witness phát hiện lỗi alias trong `yaml-cpp`: copy `YAML::Node` rồi gán `node = node[key]` làm biến đổi root node dùng chung. Probe nhỏ tái hiện `root_has_planner=0` sau khi đọc path `planner/vehicle_radius_m`; lỗi runtime cụ thể là `invalid node; first invalid key: "planner"` ở `effective_value_extraction`. Sửa tối thiểu bằng `reset(node[key])` ở hai helper traversal (`add_yaml_value`, `add_yaml_pair`), giữ fail-closed và log stage/detail. Không đổi profile/effective values.

## Xác minh trên exact source HEAD e6dd53d

Build/manifest/CTest và witness dưới đây chạy trên `e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`, trước docs-only follow-up của báo cáo. Các gate nhẹ được chạy lại trên PR/report HEAD `316ce661fbdc8ee6d9c239ee24206588cec097b1` (10 guard, codegen, ledger, compileall, Python 3.12.3 discovery 474/2 skipped, diff-check: PASS); chúng không thay thế build/runtime evidence exact e6. Log nguyên bản nằm tại `/home/letandat/.cache/uav-navigation/wp-p1-release-log/`: `p1-e6dd53db-static-python.log`, `p1-e6dd53db-a6-oracle.log`, `p1-e6dd53db-release-build.log`, `p1-e6dd53db-release-ctest.log`, `p1-e6dd53db-profile-witness.log`. Runtime artifact đầy đủ ở `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260930T064849-78862`; metadata bind commit e6, clean repo, tracking off và PX4 1.17. Các log build/test được giữ ngoài repo để không làm biến đổi input fingerprint.

| Gate | Kết quả |
|---|---|
| Baseline `git diff 7e0b850 <baseline> -- src config` | PASS; output rỗng |
| 10 `tools/check_*.py` guards | PASS |
| `python3 tools/safety_profile/gen.py --check` | PASS |
| `python3 tools/validate_runtime_safety_ledger.py` | PASS; current=500, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1 |
| `git diff --check` | PASS |
| Python 3.12.3 `py_compile` + discovery `tools/runtime/tests` | PASS; 474 tests, 2 skipped |
| `python3 tools/safety_profile/check_against_a6.py` | BLOCKED_BY_OWNER; exit 1, `semantic_rows=119 represented=114 excluded=2 open_questions=3`, `failures=1`; unresolved: `max_yaw_acceleration_rad_s2`, `maximum_thrust_n`, `minimum_thrust_n` |
| Canonical ROS Jazzy Release build | PASS; 24 packages finished, `--packages-up-to` dependency closure for the four consumer packages and reverse dependencies |
| Product CTest | PASS; 17 packages, 19 tests, 0 failures (16.49 s) |
| Release manifest | PASS trên source HEAD `e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`; 319 artifacts; source SHA-256 `5016d147775af5c28bd89643fda648ff0fc8fc43cdb7b5d6c616bd03dff0ac4a`; manifest SHA-256 `c3fbd1bc715d22c639a4075a5eeade02b4189aa2df454350abd5b4805b7eb5d3`. |
| PX4 | PASS; SITL source/binary provenance is PX4 v1.17 commit `deaff86ee335dd697677bcfc2415a23878e1b895`; version/source/binary match P0.2 provenance. |
| Real five-process witness records | 5/5 startup records PASS về mặt profile; run `external-mode-check-20260930T064849-78862`, tracking mode `off`. Runner verdict `FAIL`, mission qualification `NOT_EVALUABLE`; chi tiết bên dưới. |

Witness records from the actual session:

| Process | Keys | Hash | Mismatches |
|---|---:|---|---|
| `navigation_runtime` — `SAFETY_PROFILE_WITNESS hash=994102588801b7fd keys=53 mismatches=[]` | 53 | `994102588801b7fd` | `[]` |
| `fast_lio` — `SAFETY_PROFILE_WITNESS hash=994102588801b7fd keys=10 mismatches=[]` | 10 | `994102588801b7fd` | `[]` |
| `px4_navigation_external_mode` — `SAFETY_PROFILE_WITNESS hash=994102588801b7fd keys=9 mismatches=[]` | 9 | `994102588801b7fd` | `[]` |
| `px4_external_odometry_bridge` — `SAFETY_PROFILE_WITNESS hash=994102588801b7fd keys=6 mismatches=[]` | 6 | `994102588801b7fd` | `[]` |
| `px4_odometry_bridge` ingress — `SAFETY_PROFILE_WITNESS hash=994102588801b7fd keys=4 mismatches=[]` | 4 | `994102588801b7fd` | `[]` |

Each record is in this run's `logs/{mapping,lio,external_mode,px4_ingress}.log`; `metadata.json` binds `repo_commit=e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`, `repo_dirty=false`, tracking mode `off`, P1 executable/install and PX4 `deaff86ee335dd697677bcfc2415a23878e1b895` (v1.17). The profile records are also indexed in `metadata.runtime_configuration.safety_profile`. Build source fingerprint and manifest SHA are recorded above and in the build logs. The shared artifact directory under the main checkout is intentional (`_shared_artifact_root()` uses the Git common directory). All runtime processes exited after the runner finished. This run is the final runtime evidence for the tested source HEAD e6; it is not claimed to bind a later docs-only report commit.

Runner verdict details on the latest run: `FAIL` because mission timed out, mission completion and External Mode handover were not observed, accepted waypoints were `[0,1,2,3]` instead of `[0,1,2,3,4]`, and terminal outcome was `MISSION_TIMEOUT`. Evaluation is `NOT_EVALUABLE`; the evidence pipeline also reports incomplete capture / PX4 input trace sequence gap and unavailable policy dimensions. A prior exact-code run separately reported lidar timestamp/freshness/validity violation, `CAPTURE_NOT_FINALIZED`, and `REFERENCE_LINEAGE_MISMATCH`. Do not treat either session as mission qualification PASS.

Earlier attempts are preserved and not counted as successful witness evidence: one default `relaxed` attempt was stopped as NOT_EVALUABLE; an explicit `off` attempt exposed the yaml-cpp alias bug and was recorded NOT_EVALUABLE. Both are diagnostics, not qualification evidence. One initial build also exposed a stale ignored generated `build/px4_msgs/.../px4_msgs` directory (only `__init__.py`, Sep 29 timestamp, where the build expected a symlink). The four raw build logs were preserved with their original bytes at `/home/letandat/.cache/uav-navigation/wp-p1-release-log/p1-build-failure-evidence/`; the failure evidence was captured before removing that exact generated directory. A later build using an external install root compiled but could not write the manifest because artifact paths must be relative to repository root. The original ignored `install/` tree was preserved intact at `install-pre-wp-p1/`; `install` now aliases the verified ignored `install-wp-p1-release/` tree, allowing canonical manifest validation. These were build-environment corrections; no tracked source changes were made for them.

## KB-08 rules

| Rule | Áp dụng | Trạng thái |
|---|---|---|
| C1 | Loader/witness bổ sung; chưa chuyển consumer sang lấy giá trị từ profile, chưa enforce thiếu-key/hash. | Partial theo PR-A; enforcement thuộc PR-B |
| C2 | Estimator override giữa sim/dataset khai báo `overlay: allowed`; key còn lại `forbidden`. Nhóm epsilon số học và fault-injection duration ngoài profile theo prompt. | Áp dụng |
| C3 | Năm process phát startup hash/effective-value witness; runner gắn record theo process. Mismatch là dữ liệu, load lỗi là unavailable. | Áp dụng; không arm gate |
| C4 | Radius sum và minimum main reserve khai báo derived formula, tính trong loader/codegen. | Áp dụng |
| C5 | `safety_profile.path` chỉ chọn nguồn profile; không thêm ngưỡng điều khiển. | Áp dụng |
| K3 | Epsilon số học giữ named source ngoài profile; `anchor_pvaj_roundoff_tolerances` giữ theo prompt/HG-027. | Áp dụng |
| Q1 | Không đổi effective values hoặc product behavior; không bắt đầu P1.4. | Áp dụng |

## Finding → status → commit

| Finding | Status | Commit |
|---|---|---|
| A6 I-01..I-17 | PARTIAL — checker reports 114/119 represented, 2 excluded, 3 owner-open rows; five profile startup records PASS, but mission runner is FAIL/NOT_EVALUABLE and oracle is unresolved. | `6c050804` |
| R1-13, R1-36, R2-16, R3-24 | NOT_FIXED trong PR-A; consumer/source consolidation là P1.4/PR-B. | Ngoài scope |
| R4-11, R4-13 | NOT_FIXED trong PR-A; defaults/threshold unification là P1.4/PR-B. | Ngoài scope |
| R6-07, R6-20 | NOT_FIXED trong PR-A; mapping plane/config required thuộc P1.4e/PR-B. | Ngoài scope |
| R-08, R7-02, R7-08 | NOT_FIXED trong PR-A; P1.4f/WP sở hữu và câu hỏi ADR vẫn chờ owner. | Ngoài scope |
| V6 | NOT_FIXED trong PR-A; adapter pinned literal removal thuộc P1.4b/PR-B. | Ngoài scope |
| P1.2 witness loader | FIXED — reset traversal avoids mutating aliased YAML root; five exact-commit startup records match the profile with no mismatch. Final session metadata is selected by `repo_commit` matching the final HEAD. | `6c050804` |

## Còn lại và câu hỏi mở

1. Owner quyết định semantics/unit và A6 mapping cho hai thrust rows; owner xác nhận A6 yaw acceleration (`0.3`) hay effective planner YAML (`2.0`). Không tự chọn hoặc sửa oracle.
2. Giữ Q-ENV, Q-VOX, Q-UNK, Q-TRK, Q-XTRK của ADR-017 §6.3 OPEN.
3. Runtime session dùng làm bằng chứng là `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260930T064849-78862`, metadata `repo_commit=e6dd53db9d8c2e00e77772c23d95b7be7dab87bb`, clean tree. Runner verdict là `FAIL` / evaluation `NOT_EVALUABLE`; giữ tách biệt khỏi 5/5 profile startup records.
4. Draft PR #17 đã mở cho P1.1–P1.3; không merge. Trên report HEAD `316ce661fbdc8ee6d9c239ee24206588cec097b1`, 10 guard, codegen, ledger, compileall, Python 3.12.3 discovery (474 tests, 2 skipped) và diff-check đều PASS. Bản sửa báo cáo này tiếp tục chỉ là docs-only; không rerun SITL/build theo contract audit. Các oracle rows vẫn owner-blocked, mission run không qualification-PASS. Sau H2 merge, rebase theo ghi chú overlap.

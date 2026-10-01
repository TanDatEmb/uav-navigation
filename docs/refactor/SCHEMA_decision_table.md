# Schema chung cho bảng quyết định

Schema này là hợp đồng dữ liệu cho WP-A4, WP-A5 và các reducer spec tiếp
theo. WP-A4-R1 là revision đầu tiên commit schema vào repository; các WP sau
chỉ đọc và dùng cùng tên cột.

## `state_vars.csv`

`var_id` là tên snake_case duy nhất trong component. `cpp_name` giữ nguyên tên
nguồn để truy nguyên. `writers` và `readers` dùng `file:line;…`; `partition`
phải là một trong các phân vùng đã thống nhất (`execution`, `mission`,
`planning_policy`, `ingress`, `world`, `setpoint_guard`, `continuity`,
`diagnostics_only`, `fault_injection`).

Cột bắt buộc:

```text
var_id,component,cpp_name,type,owner_class,writers,readers,threads,
guarding_lock,reset_on,partition
```

## `events.csv`

`event_id` là `UPPER_SNAKE`, duy nhất. `payload_fields` là danh sách field
phân tách bằng `;`; guard chỉ được tham chiếu các field này qua
`payload.<field>`.

```text
event_id,component,source,thread,payload_fields,notes
```

## `decision_table.csv` hoặc `decision_table_<function>.csv`

Nếu tổng số rule vượt 400, mỗi function phải có file riêng theo mẫu
`decision_table_<function>.csv`; checker gộp tất cả các file này. Nếu không
vượt 400, dùng `decision_table.csv`.

```text
rule_id,event,guard,effects,state_updates,source,exit_site,
predicates_used,covered_by_test,suspect
```

- `rule_id` có dạng `<COMP>-<nnn>`; `source` là `file:start-end` của nhánh.
- `guard` là biểu thức boolean, chỉ dùng `var_id`, `payload.<field>`, hằng
  số, và `pred:<name>(…)` với `<name>` có trong `predicates.csv`. Toán tử được
  phép: `&& || ! == != < <= > >=` và ngoặc.
- `effects` là danh sách có thứ tự, phân tách bằng `;`, chỉ dùng vocabulary
  đóng bên dưới. `state_updates` là `var_id := <expr>` phân tách bằng `;`,
  hoặc `-`.
- `exit_site` là `file:line` của `return`/điểm kết thúc nhánh, hoặc `-` cho
  rule effect-only.
- `covered_by_test` ghi binary/test thật, `predicate_only:<…>`, hoặc `NONE`.

## `predicates.csv`

```text
name,file:line,signature,pure,semantics,used_by_rules
```

`used_by_rules` là danh sách `rule_id` phân tách bằng `;`, giúp checker phát
hiện predicate được khai báo nhưng không gắn vào oracle.

## Vocabulary effects đóng

```text
PUBLISH_COMMAND, PUBLISH_HOLD_SETPOINT, PUBLISH_VELOCITY_HOLD,
PUBLISH_TRAJECTORY_SETPOINT, PUBLISH_STATUS, PUBLISH_MISSION_PROGRESS,
PUBLISH_ADMISSION, PUBLISH_REJECTION, PUBLISH_EXTERNAL_ODOMETRY,
SUBMIT_SOLVE, CANCEL_SOLVE, COMMIT_CANDIDATE, STAGE_CANDIDATE,
DISCARD_CANDIDATE, ACTIVATE_STAGED, ACTIVATE_BACKUP,
EMERGENCY_BRAKE_PREPARE, EMERGENCY_BRAKE_COMMIT, RECERTIFY_RETAINED,
SUSPEND_COMMAND, RESUME_COMMAND, FAIL_CLOSED, REQUEST_PX4_HOLD,
LATCH_FAILURE, CLEAR_LATCH, ADVANCE_WAYPOINT, COMPLETE_MISSION,
ACCEPT_GOAL, REJECT_GOAL, RESET_EPOCH, ACCEPT_OBSERVATION,
REJECT_OBSERVATION, UPDATE_STATE_STORE, RESEED_CONTINUITY, LATCH_JUMP,
DROP_INPUT, EMIT_DIAGNOSTIC, EMIT_EVIDENCE, LOG, NO_OP
```

Muốn thêm effect phải ghi vào `OPEN_QUESTIONS.md`; không mở rộng vocabulary
ngầm trong CSV.

## Quy tắc phủ source

Checker phải parse source baseline, không nhận denominator do người viết bảng
truyền vào:

1. Mỗi `return` trong các function thuộc scope phải xuất hiện ở ít nhất một
   `exit_site`.
2. Mỗi side-effect call site phải nằm trong `source` của rule có effect tương
   ứng. Scope tối thiểu gồm `failClosedLocked(`, `planning_worker_->`,
   `->publish(`, `commitPlannerCandidate(`, `discardCommandCandidate(`,
   `cancelActive(`, `commitEmergencyBrake(`, `tryLatch(` và
   `publishCommand(`; worker phụ trợ trong cùng function cũng được kiểm tra.
3. Identifier lạ trong guard là lỗi; effect ngoài vocabulary là lỗi.

Đây là inventory tĩnh để làm oracle cho reducer, không phải runtime/PX4
qualification evidence.

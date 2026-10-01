# Schema chung cho bảng quyết định (WP-A4, WP-A5 và mọi reducer spec về sau)

File này nằm ở `docs/refactor/SCHEMA_decision_table.md`. WP-A4-R1 là WP commit nó. Các WP khác chỉ đọc.

## 1. `state_vars.csv`
`var_id` (snake_case, duy nhất trong component), `component` (runtime|adapter|bridge), `cpp_name`, `type`, `owner_class`, `writers` (file:line;…), `readers` (file:line;…), `threads`, `guarding_lock`, `reset_on` (epoch_reset|new_goal|mode_exit|activation|never|…), `partition` (execution|mission|planning_policy|ingress|world|setpoint_guard|continuity|diagnostics_only|fault_injection).

## 2. `events.csv`
`event_id` (UPPER_SNAKE), `component`, `source` (file:line), `thread`, `payload_fields` (tên field;…), `notes`.

## 3. `decision_table.csv`
| cột | quy tắc |
|---|---|
| `rule_id` | `<COMP>-<nnn>`, ví dụ `RT-001`, `AD-001`, `BR-001` |
| `event` | một `event_id` có trong `events.csv` |
| `guard` | biểu thức boolean chỉ dùng: `var_id` có trong `state_vars.csv`, `payload.<field>`, hằng số, `pred:<name>(…)` với `<name>` có trong `predicates.csv`. Toán tử: `&& \|\| ! == != < <= > >=`. Cấm viết văn xuôi. |
| `effects` | danh sách có thứ tự, chỉ lấy từ bộ từ vựng đóng ở §5, phân tách bằng `;` |
| `state_updates` | `var_id := <expr>` phân tách bằng `;`, hoặc `-` nếu không có |
| `source` | `file:start-end` của nhánh |
| `exit_site` | `file:line` của câu `return` / câu kết thúc nhánh mà rule mô tả, hoặc `-` |
| `predicates_used` | tên predicate |
| `covered_by_test` | `<test_binary>:<TestSuite.TestName>` \| `predicate_only:<…>` \| `NONE` |
| `suspect` | trống, hoặc mô tả ngắn + verdict |

## 4. `predicates.csv`
`name`, `file:line`, `signature`, `pure` (true/false), `semantics` (1 câu), `used_by_rules`.

## 5. Bộ từ vựng effect (đóng; muốn thêm phải ghi vào OPEN_QUESTIONS)
`PUBLISH_COMMAND, PUBLISH_HOLD_SETPOINT, PUBLISH_VELOCITY_HOLD, PUBLISH_TRAJECTORY_SETPOINT, PUBLISH_STATUS, PUBLISH_MISSION_PROGRESS, PUBLISH_ADMISSION, PUBLISH_REJECTION, PUBLISH_EXTERNAL_ODOMETRY, SUBMIT_SOLVE, CANCEL_SOLVE, COMMIT_CANDIDATE, STAGE_CANDIDATE, DISCARD_CANDIDATE, ACTIVATE_STAGED, ACTIVATE_BACKUP, EMERGENCY_BRAKE_PREPARE, EMERGENCY_BRAKE_COMMIT, RECERTIFY_RETAINED, SUSPEND_COMMAND, RESUME_COMMAND, FAIL_CLOSED, REQUEST_PX4_HOLD, LATCH_FAILURE, CLEAR_LATCH, ADVANCE_WAYPOINT, COMPLETE_MISSION, ACCEPT_GOAL, REJECT_GOAL, RESET_EPOCH, ACCEPT_OBSERVATION, REJECT_OBSERVATION, UPDATE_STATE_STORE, RESEED_CONTINUITY, LATCH_JUMP, DROP_INPUT, EMIT_DIAGNOSTIC, EMIT_EVIDENCE, LOG, NO_OP`.

## 6. Quy tắc phủ (checker phải kiểm)
- Mỗi câu `return` nằm trong các hàm thuộc phạm vi phải xuất hiện đúng ở ≥1 `exit_site`.
- Mỗi call site có tác dụng phụ (`failClosedLocked(`, `planning_worker_->`, `->publish(`, `commitPlannerCandidate(`, `discardCommandCandidate(`, `cancelActive(`, `commitEmergencyBrake(`, `tryLatch(`, `publishCommand(`; với adapter/bridge là các lệnh publish và các hàm latch/reseed) phải nằm trong `source` của ≥1 rule có effect tương ứng.
- Checker tự quét source để lấy danh sách return/call site. Không được hard-code con số.
